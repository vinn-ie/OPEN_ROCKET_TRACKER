#include <RadioLib.h>

// Unique Device IDs defined from system memory
#define MASTER_UID  *((uint32_t*)0x1FFF7580)
#define SLAVE_UID   0x05370378

// Pin configuration for RF switch
static const uint32_t rfswitch_pins[] = {PA4, PA5, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
  {STM32WLx::MODE_RX,    {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {LOW,  HIGH}},
  END_OF_MODE_TABLE,
};

// Data union for easier management of different data types
union unionData {
  uint32_t  dataBuff32[3];
  float     dataBuff32F[3];
  int16_t   dataBuff16[6];
  uint8_t   dataBuff8[12];
};
union unionData ud;

// Communication status flags
bool receivedFlag = false;
bool transmittedFlag = false;

// Radio module instance
STM32WLx radio = new STM32WLx_Module();

void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(2000);

  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  int state = radio.begin(915.0, 125.0, 12, 5, 0x12, 13, 8, 1.7, 0);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Radio initialized successfully!");
  } else {
    Serial.print("Radio initialization failed, code: ");
    Serial.println(state);
  }

  radio.setDio1Action(setFlag);
  delay(2000);
}

void setFlag(void) {
  uint16_t irqstatus = radio.getIrqStatus();
  receivedFlag = irqstatus & RADIOLIB_SX126X_IRQ_RX_DONE;
  transmittedFlag = irqstatus & RADIOLIB_SX126X_IRQ_TX_DONE;
}

void loop() {
  transmitNMEAPacket();
  delay(10000); // Delay between transmissions
}

// Function to transmit a dummy NMEA packet and print it to serial
void transmitNMEAPacket() {
  String nmeaMessage = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

  // Start transmission
  Serial.println("Transmitting NMEA packet...");
  int state = radio.startTransmit(nmeaMessage.c_str());

  // Wait for transmission to complete
  while (!transmittedFlag) {
    // Include a small delay or idle processing here if necessary
  }

  // Print transmitted packet
  Serial.println(nmeaMessage);
  
  // Reset the flag
  transmittedFlag = false;

  // Check and print transmission status
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("NMEA packet transmission successful.");
  } else {
    Serial.print("Transmission failed, code: ");
    Serial.println(state);
  }
}
