#include <RadioLib.h>

// Define Device UIDs
#define MASTER_UID  0x0535EE2F  // Sample Master device UID
#define SLAVE_UID   *((uint32_t*)0x1FFF7580)    // Use UID from system memory for Slave device UID

// Pin configuration for RF switch
static const uint32_t rfswitch_pins[] = {PA4, PA5, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
  {STM32WLx::MODE_RX,    {HIGH, LOW}},
  END_OF_MODE_TABLE,
};

// Communication data union
union unionData {
  uint32_t  dataBuff32[3];
  float     dataBuff32F[3];
  int16_t   dataBuff16[6];
  uint8_t   dataBuff8[100];
};
union unionData ud;

// Communication parameters
int rssi;               // Signal RSSI [dBm]
int snr;                // Signal SN ratio [dB]
uint32_t slaveuid = SLAVE_UID;    // Slave device UID
uint32_t masteruid;     // Master device UID
String rxdata = "";    // Received data packet string
bool receivedFlag = false;     // Flag that a packet was received

STM32WLx radio = new STM32WLx_Module();

void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(1000);
  Serial.println("**** Device Wake Up ****");

  // Radio initialization with specific settings
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  int state = radio.begin(915.0, 125.0, 12, 5, 0x12, 13, 8, 1.7, 0);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Radio initialized successfully!");
  } else {
    Serial.print("Radio initialization failed, error code: ");
    Serial.println(state);
  }

  radio.setDio1Action(setFlag);
}

// Callback function for packet received/transmitted
void setFlag(void) {
  uint16_t irqstatus = radio.getIrqStatus();
  receivedFlag = irqstatus & RADIOLIB_SX126X_IRQ_RX_DONE;
}


// Function to calculate and verify NMEA checksum
bool verifyChecksum(String nmea) {
  int asteriskIndex = nmea.indexOf('*');
  if (asteriskIndex < 1 || asteriskIndex + 2 >= nmea.length()) return false;

  String checksum = nmea.substring(asteriskIndex + 1);
  char calculatedChecksum = 0;
  for (int i = 1; i < asteriskIndex; i++) { // Start at 1 to skip '$'
    calculatedChecksum ^= nmea[i];
  }

  char hexChecksum[3];
  sprintf(hexChecksum, "%02X", calculatedChecksum);
  return checksum.equals(hexChecksum);
}

void loop() {
  // Start listening for packets
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    // Handle the error quietly or log as needed
    return;  // Exit if unable to start receiving
  }

  while (true) {
    // Continuously check if a packet has been received
    if (receivedFlag) {
      receivedFlag = false;  // Reset flag for next packet

      // Read received packet
      state = radio.readData(ud.dataBuff8, sizeof(ud.dataBuff8));
      if (state == RADIOLIB_ERR_NONE) {
        String receivedNMEAMessage = "";
        for (int i = 0; i < sizeof(ud.dataBuff8); i++) {
          if (ud.dataBuff8[i] == 0) break;  // Stop if null character is found
          receivedNMEAMessage += (char)ud.dataBuff8[i];
        }

        if (verifyChecksum(receivedNMEAMessage)) {
          Serial.println(receivedNMEAMessage);
        } else {
          // Error for log checksum failure
        }
      } else {
        // Error output
      }

      // Restart listening for new packet
      state = radio.startReceive();
      if (state != RADIOLIB_ERR_NONE) {
        // Optionally handle or log the failure to restart reception
        break;  // Exit loop if we fail to restart receiving
      }
    }
    delay(100);  // Maintain a delay to avoid overloading the CPU
  }
}
