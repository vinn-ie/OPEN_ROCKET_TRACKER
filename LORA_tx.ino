#include <RadioLib.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MS5x.h>

// -----------------------------------------------------------------------------
// Configure Radio
// -----------------------------------------------------------------------------
STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] = { PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
  {STM32WLx::MODE_RX,    {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {LOW,  HIGH}},
  END_OF_MODE_TABLE,
};

// -----------------------------------------------------------------------------
// Constants & Config
// -----------------------------------------------------------------------------
#define TX_PACKET_SIZE 25
#define TX_INTERVAL    500       // Transmit interval in ms (2 Hz)
bool debugEnabled    = true;     // Set to false to disable debug prints

// -----------------------------------------------------------------------------
// Sensor Instances
// -----------------------------------------------------------------------------
SFE_UBLOX_GNSS myGNSS;
MS5x barometer(&Wire);

// I2C pins
const int SDAPin = PA15;
const int SCLPin = PB15;

// -----------------------------------------------------------------------------
// Data Packet Structure
// -----------------------------------------------------------------------------
union DataPacket {
  struct {
    int32_t latitude;    
    int32_t longitude;   
    int32_t altitude;    
    uint32_t timeMs;     
    int32_t pressure;    
    int16_t temperature; 
    uint8_t satellites;  
    uint8_t status;      
    uint8_t checksum;    
  } fields;
  uint8_t buffer[TX_PACKET_SIZE];
};

DataPacket txPacket;

// -----------------------------------------------------------------------------
// State and Flags
// -----------------------------------------------------------------------------
int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false;
uint32_t lastTransmitTime = 0;
bool transmissionInProgress = false;

// -----------------------------------------------------------------------------
// ISR for Transmission Complete
// -----------------------------------------------------------------------------
void setFlag(void) {
  transmittedFlag = true;
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  if (debugEnabled) {
    Serial.println(F("[Setup] Starting initialisation..."));
  }

  // I2C initialisation
  Wire.setSDA(SDAPin);
  Wire.setSCL(SCLPin);
  Wire.begin();

  // GPS initialisation
  if (debugEnabled) Serial.print(F("[GPS] Initialising ... "));
  if (myGNSS.begin()) {
    if (debugEnabled) Serial.println(F("success!"));
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.saveConfiguration();
  } else {
    if (debugEnabled) Serial.println(F("failed! Check wiring."));
    while (1) { delay(10); }
  }

  // Barometer initialisation
  if (debugEnabled) Serial.print(F("[Barometer] Initialising ... "));
  barometer.setI2Caddr(I2C_HIGH);
  barometer.setSamples(MS5xxx_CMD_ADC_2048);
  barometer.setDelay(1000);
  if (barometer.connect() > 0) {
    if (debugEnabled) Serial.println(F("failed! Check wiring."));
    while (1) { delay(10); }
  }
  if (debugEnabled) {
    Serial.println(F("success!"));
    barometer.checkUpdates();
  }

  // Radio switch configuration
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // Radio initialisation
  if (debugEnabled) Serial.print(F("[Radio] Initialising ... "));
  int state = radio.begin(922.6, 125.0, 12, 5);
  if (state == RADIOLIB_ERR_NONE) {
    if (debugEnabled) Serial.println(F("success!"));
  } else {
    if (debugEnabled) {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
    while (1) { delay(10); }
  }

  // Set TCXO voltage
  state = radio.setTCXO(1.7);
  if (state != RADIOLIB_ERR_NONE) {
    if (debugEnabled) {
      Serial.print(F("TCXO failed, code "));
      Serial.println(state);
    }
    while (1) { delay(10); }
  }

  // Additional radio configuration
  radio.setSyncWord(0x12);
  radio.setOutputPower(13);
  radio.setPreambleLength(8);

  // Attach interrupt
  radio.setDio1Action(setFlag);

  if (debugEnabled) Serial.println(F("Setup complete!\n"));
}

// -----------------------------------------------------------------------------
// Main Loop
// -----------------------------------------------------------------------------
void loop() {
  // Check if transmission is complete
  if(transmittedFlag) {
    transmittedFlag = false;
    transmissionInProgress = false;
    
    if (debugEnabled) {
      if (transmissionState == RADIOLIB_ERR_NONE) {
        Serial.println(F("Transmission finished successfully!"));
      } else {
        Serial.print(F("Transmission failed, code "));
        Serial.println(transmissionState);
      }
    }
    radio.finishTransmit();
  }

  // Time to start a new transmission?
  if(!transmissionInProgress && (millis() - lastTransmitTime >= TX_INTERVAL)) {
    // Get sensor readings
    long latitude = myGNSS.getLatitude();
    long longitude = myGNSS.getLongitude();
    long altitude = myGNSS.getAltitudeMSL();
    uint8_t sats = myGNSS.getSIV();

    // Barometer update
    barometer.checkUpdates();
    int32_t pressure = 0;
    int16_t temperature = 0;
    if (barometer.isReady()) {
      pressure = (int32_t)(barometer.GetPres() * 100);
      temperature = (int16_t)(barometer.GetTemp() * 100);
    }

    // Clear packet buffer
    memset(txPacket.buffer, 0, TX_PACKET_SIZE);

    // Fill packet fields
    txPacket.fields.latitude    = latitude;
    txPacket.fields.longitude   = longitude;
    txPacket.fields.altitude    = altitude;
    txPacket.fields.timeMs      = millis();
    txPacket.fields.pressure    = pressure;
    txPacket.fields.temperature = temperature;
    txPacket.fields.satellites  = sats;
    txPacket.fields.status      = 0x01;

    // Compute checksum
    uint8_t checksum = 0;
    for(size_t i = 0; i < TX_PACKET_SIZE - 1; i++) {
      checksum ^= txPacket.buffer[i];
    }
    txPacket.fields.checksum = checksum;

    // Debug info
    if (debugEnabled) {
      Serial.println(F("\n[Debug] Sensor Readings:"));
      Serial.print(F("GPS: "));
      Serial.print(latitude/10000000.0, 6);
      Serial.print(F("°, "));
      Serial.print(longitude/10000000.0, 6);
      Serial.print(F("°, "));
      Serial.print(altitude/1000.0, 2);
      Serial.println(F("m"));
      Serial.print(F("Satellites: "));
      Serial.println(sats);

      Serial.print(F("Pressure: "));
      Serial.print(pressure/100.0, 2);
      Serial.println(F(" hPa"));

      Serial.print(F("Temperature: "));
      Serial.print(temperature/100.0, 2);
      Serial.println(F("°C"));

      Serial.print(F("Checksum: 0x"));
      Serial.println(checksum, HEX);

      Serial.print(F("Starting transmission ... "));
    }

    // Start transmission
    transmissionState = radio.startTransmit(txPacket.buffer, TX_PACKET_SIZE);
    if(transmissionState == RADIOLIB_ERR_NONE) {
      transmissionInProgress = true;
      lastTransmitTime = millis();
    } else if (debugEnabled) {
      Serial.print(F("Failed to start, code "));
      Serial.println(transmissionState);
    }
  }
}
