#include <RadioLib.h>

// Unique Device IDs defined from system memory (if needed)
#define MASTER_UID  *((uint32_t*)0x1FFF7580)
#define SLAVE_UID   0x05370378

// Pin configuration for RF switch based on Wio-E5 pinout
static const uint32_t rfswitch_pins[] = {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
  {STM32WLx::MODE_RX,    {HIGH, HIGH, LOW}},
  {STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH}},
  {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},
  END_OF_MODE_TABLE,
};

// Data union for easier management of different data types (if needed)
union unionData {
  uint32_t  dataBuff32[3];
  float     dataBuff32F[3];
  int16_t   dataBuff16[6];
  uint8_t   dataBuff8[12];
};
union unionData ud;

// Communication status flags
volatile bool transmittedFlag = false;

// Radio module instance
STM32WLx radio = new STM32WLx_Module();

void setup() {
  Serial1.begin(115200);
  while(!Serial);
  delay(2000);

  // Set RF switch configuration
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // Initialize the radio
  Serial1.print("[STM32WL] Initializing ... ");
  int state = radio.begin(915.0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial1.println("success!");
  } else {
    Serial1.print("failed, code ");
    Serial1.println(state);
    while (true) { delay(10); }
  }

  // Set TCXO voltage (if required by your board)
  state = radio.setTCXO(1.7);
  if (state == RADIOLIB_ERR_NONE) {
    Serial1.println("TCXO set successfully!");
  } else {
    Serial1.print("TCXO set failed, code ");
    Serial1.println(state);
    while (true) { delay(10); }
  }

  // Set modulation parameters
  state = radio.setBandwidth(125.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial1.print("Failed to set bandwidth, code ");
    Serial1.println(state);
  }

  state = radio.setSpreadingFactor(12);
  if (state != RADIOLIB_ERR_NONE) {
    Serial1.print("Failed to set spreading factor, code ");
    Serial1.println(state);
  }

  state = radio.setCodingRate(5);
  if (state != RADIOLIB_ERR_NONE) {
    Serial1.print("Failed to set coding rate, code ");
    Serial1.println(state);
  }

  state = radio.setSyncWord(0x12);
  if (state != RADIOLIB_ERR_NONE) {
    Serial1.print("Failed to set sync word, code ");
    Serial1.println(state);
  }

  state = radio.setPreambleLength(13);
  if (state != RADIOLIB_ERR_NONE) {
    Serial1.print("Failed to set preamble length, code ");
    Serial1.println(state);
  }

  state = radio.setOutputPower(8); // Adjust power as needed
  if (state != RADIOLIB_ERR_NONE) {
    Serial1.print("Failed to set output power, code ");
    Serial1.println(state);
  }

  // Set the function that will be called when transmission is finished
  radio.setDio1Action(setFlag);

  delay(2000);
}

void setFlag(void) {
  // Transmission complete
  transmittedFlag = true;
}

void loop() {
  transmitNMEAPacket();
  delay(10000); // Delay between transmissions
}

// Function to transmit a dummy NMEA packet and print it to serial
void transmitNMEAPacket() {
  String nmeaMessage = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

  // Start transmission
  Serial1.println("Transmitting NMEA packet...");
  int state = radio.startTransmit(nmeaMessage.c_str());

  // Wait for transmission to complete
  while (!transmittedFlag) {
    // Include a small delay to prevent watchdog resets
    delay(1);
  }

  // Reset the flag
  transmittedFlag = false;

  // Finish transmission (disable transmitter, power down RF switch, etc.)
  radio.finishTransmit();

  // Print transmitted packet
  Serial1.println(nmeaMessage);

  // Check and print transmission status
  if (state == RADIOLIB_ERR_NONE) {
    Serial1.println("NMEA packet transmission successful.");
  } else {
    Serial1.print("Transmission failed, code: ");
    Serial1.println(state);
  }
}
