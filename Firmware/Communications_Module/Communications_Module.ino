/*
 * Quick SPAC SRAD firmare for communicataions module
 * 
 * Jun.16.2024
 */

#include "CANPackets.h"
#include "pinouts.h"
#include "STM32_CAN.h" //https://github.com/pazi88/STM32_CAN
#include <Wire.h>
#include <RadioLib.h>
#include <SoftwareSerial.h>
#include <SerialFlash.h>
#include <SPI.h>
//#include "flashTable.h"


//TODO: Replace Defines
#define SERIAL_ENABLE true
#define SERIAL_BAUD 38400
#define CANBUS_BAUD 500000 //500kbps

//Buzzer Settings
const uint32_t BEEP_DELAY = 6000;
const uint32_t BEEP_LENGTH = 1000;
const uint32_t BEEP_FREQ = 1000;

const double FREQUENCY = 905.4;
const double BANDWIDTH = 31.25;
const int32_t SPREADING_RATE = 10;
const uint8_t CODING_RATE = 6;

#define NORMAL_TRANSMIT_GAP 100
#define BEACON_TRANSMIT_GAP 100 //15000 //15sec
uint32_t transmitGapTime = NORMAL_TRANSMIT_GAP;


#define POWER_DOWN_DELAY 6000 //600000
bool pendingPowerDown = false;
uint32_t powerDownInitTime = 0; //10min
bool inPowerDown = false;



// Software softSerial object
SoftwareSerial softSerial(USB_DM_PIN, USB_DP_PIN); // RX, TX

// CANBus objects
STM32_CAN can( CAN1, ALT ); //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_RX_msg;
static CAN_message_t CAN_TX_msg;

// Create FlashTable object
const uint8_t TABLE_NAME = 0;
const uint8_t TABLE_COLS = 3;
const uint32_t TABLE_SIZE = 204800; //4096000
//FlashTable table = FlashTable(TABLE_COLS, 16384, TABLE_SIZE, TABLE_NAME, 256);

SPIClass newSPI(RF_MOSI_PIN, RF_MISO_PIN, RF_SCK_PIN);

// RFM95 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
Module* mod;
RFM95* radio;
int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false; // flag to indicate that a packet was sent


void powerDown() {
  transmitGapTime = BEACON_TRANSMIT_GAP;
}//powerDown


void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
  digitalWrite(STATUS_LED_PIN, LOW);
}//setFlag()

void radioInit() {
// initialize RFM95 with default settings
  Serial.print(F("[RFM95] Initializing ... "));
  radio->reset();
  delay(100);
  int state = radio->begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(500);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(500);
    }
  }


  //radio->forceLDRO(true);

  if (radio->setFrequency(FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }
  if (radio->setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }

  // set spreading factor to 10
  if (radio->setSpreadingFactor(SPREADING_RATE) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  // set coding rate to 6
  if (radio->setCodingRate(CODING_RATE) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

  // set output power to 17 dBm (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission duty cycle MUST NOT exceed 1%
  if (radio->setOutputPower(17) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Selected output power is invalid for this module!"));
    while (true);
  }
  //Enable LoRa CRC
  radio->setCRC(true);

}//radioInit()





// Empties all bytes from incoming serial buffer.
// Used by Debug mode
void emptySerialBuffer() {
  while (softSerial.available()) {softSerial.read();}
}//emptySerialBuffer()


void setup() {
  #if defined(SERIAL_ENABLE)
  softSerial.begin(SERIAL_BAUD);
  #endif
  
  // Set pinmodes
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Set up SPI
//  SPI.setSCLK(FLASH_SCK_PIN);
//  SPI.setMISO(FLASH_MISO_PIN);
//  SPI.setMOSI(FLASH_MOSI_PIN);

  // Init CANBUS
  can.begin(); //automatic retransmission can be done using arg "true"
  can.setBaudRate(CANBUS_BAUD);

  // Initialize Flash Chip
//  while (!SerialFlash.begin(FLASH_CS_PIN)) {
//    softSerial.println(F("Connecting to SPI Flash chip..."));
//    delay(250);
//    //toggleStatusLED();
//  }//while

  //digitalWrite(STATUS_LED_PIN, LOW);
  //delay(1000);

  // Initialize FlashTable object
  //table.init(&SerialFlash);

  //digitalWrite(STATUS_LED_PIN, HIGH);
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  newSPI.begin();
  mod = new Module(RF_CS_PIN, RF_DIO0_PIN, RF_RESET_PIN, RADIOLIB_NC, newSPI);
  radio = new RFM95(mod);
  radioInit();


  // set the function that will be called
  // when packet transmission is finished
  //radio->setDio0Action(setFlag, RISING);

  // STARTUP BEEP
  delay(BEEP_DELAY);
  tone(BUZZER_PIN, BEEP_FREQ);
  delay(BEEP_LENGTH);
  tone(BUZZER_PIN, 0);

  // Startup delay - Check to enter debug mode
  uint32_t startTime = millis();
  while (!softSerial.available() and millis()-startTime < 5000) {}
  
  if (softSerial.available()) {
    byte d = softSerial.read();
    emptySerialBuffer();
    if (d == 'D') {
      softSerial.println(F("Entered Debug Mode"));
      //debugMode();
      while (true) {}
    }//if
  }//if
  softSerial.println(F("Running Normally"));
  
}//setup()

uint32_t recvGPSLat = 0;
uint32_t recvGPSLon = 0;
uint8_t recvGPSSats = 0;
uint32_t recvAltitude = 0;

bool seenAltimeter = false;
bool seenSensors = false;
bool seenGPS = false;

#define PACKET_SIZE 14
uint8_t packet[PACKET_SIZE] = {};

void loop() {
  // put your main code here, to run repeatedly:

//  if (!inPowerDown && pendingPowerDown && millis()-powerDownInitTime > POWER_DOWN_DELAY) {
//    inPowerDown = true
//    powerDown();
//  }//if

  digitalWrite(STATUS_LED_PIN, HIGH);
  uint32_t strtTr = millis();
  //int state = radio->transmit("QRET RF TEST; THIS IS 30 BYTES");
  int state = radio->transmit(packet, PACKET_SIZE);
  uint32_t endTr = millis();
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(transmitGapTime);

  while (can.read(CAN_RX_msg)) {

    softSerial.print(F("CAN ID = "));
    softSerial.println(CAN_RX_msg.id, BIN);
    if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_LAT_CANID) {
      //CAN_RX_msg.buf[i]
      packet[0] = CAN_RX_msg.buf[0];
      packet[1] = CAN_RX_msg.buf[1];
      packet[2] = CAN_RX_msg.buf[2];
      packet[3] = CAN_RX_msg.buf[3];
      seenGPS = true;
      softSerial.println(F("Recv LAT"));
    } else if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_LON_CANID) {
      //CAN_RX_msg.buf[i]
      packet[4] = CAN_RX_msg.buf[0];
      packet[5] = CAN_RX_msg.buf[1];
      packet[6] = CAN_RX_msg.buf[2];
      packet[7] = CAN_RX_msg.buf[3];
      seenGPS = true;
      softSerial.println(F("Recv LON"));
    } else if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_NUMSAT_CANID) {
      //CAN_RX_msg.buf[i]
      packet[8] = CAN_RX_msg.buf[0];
      softSerial.println(F("Recv NUM SAT"));
      seenGPS = true;
    } else if (CAN_RX_msg.id == ALTIMETER_MOD_CANID+ALTITUDE_CANID) {
      //CAN_RX_msg.buf[i]
      packet[9] = CAN_RX_msg.buf[0];
      packet[10] = CAN_RX_msg.buf[1];
      packet[11] = CAN_RX_msg.buf[2];
      packet[12] = CAN_RX_msg.buf[3];
      softSerial.println(F("Recv ALTITUDE"));
    } else if (CAN_RX_msg.id == ALTIMETER_MOD_CANID+FLIGHT_STAGE_CANID) {
      //check if landed
      softSerial.println(F("Recv FLGHT STAGE"));
      seenAltimeter = true;
//      if (CAN_RX_msg.buf[0] >= 5) {
//        if (!pendingPowerDown) {
//          pendingPowerDown = true;
//          powerDownInitTime = millis();
//        }//if
//      }//if(landed)
    } else if (CAN_RX_msg.id == SENSOR_MOD_CANID+STATUS_CANID) {
      seenSensors = true;
    }

    packet[13] = seenGPS*1 + seenAltimeter*2 + seenSensors*4;

  }//while

  
}//loop()
