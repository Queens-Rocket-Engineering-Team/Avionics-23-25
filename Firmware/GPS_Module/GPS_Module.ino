/*
 * Quick SPAC SRAD firmare for GPS module
 * 
 * Jun.16.2024
 */

#include "CANPackets.h"
#include "pinouts.h"
#include "STM32_CAN.h" //https://github.com/pazi88/STM32_CAN
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SerialFlash.h>
#include <SPI.h>
#include "flashTable.h"

#define SERIAL_ENABLE true
#define SERIAL_BAUD 38400
#define CANBUS_BAUD 500000 //500kbps
#define BEEP_DELAY 4000
#define BEEP_FREQ 1000


// The TinyGPSPlus object
TinyGPSPlus gps;

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

const uint32_t GPS_SEND_INT = 1000; //how often to send GPS data over CANBUS
uint32_t lastGPSSend = 0;

void checkGPS() {
  //Called once a second

  uint8_t numSat = gps.satellites.value();
  int32_t latInt = gps.location.lat()*1000000;
  int32_t lngInt = gps.location.lng()*1000000;

  softSerial.print(F("NumSat="));
  softSerial.println(numSat);
  
  softSerial.print(F("Lat="));
  softSerial.print(latInt);
  softSerial.print(F(" Long="));
  softSerial.println(lngInt);

  CAN_TX_msg.id = GPS_MOD_CANID+GPS_LAT_CANID;
  CAN_TX_msg.len = 4;
  CAN_TX_msg.buf[0] = (latInt) & 0xFF;
  CAN_TX_msg.buf[1] = (latInt>>8) & 0xFF;
  CAN_TX_msg.buf[2] = (latInt>>16) & 0xFF;
  CAN_TX_msg.buf[3] = (latInt>>24) & 0xFF;
  can.write(CAN_TX_msg);

  CAN_TX_msg.id = GPS_MOD_CANID+GPS_LON_CANID;
  CAN_TX_msg.len = 4;
  CAN_TX_msg.buf[0] = (lngInt) & 0xFF;
  CAN_TX_msg.buf[1] = (lngInt>>8) & 0xFF;
  CAN_TX_msg.buf[2] = (lngInt>>16) & 0xFF;
  CAN_TX_msg.buf[3] = (lngInt>>24) & 0xFF;
  can.write(CAN_TX_msg);

  CAN_TX_msg.id = GPS_MOD_CANID+GPS_NUMSAT_CANID;
  CAN_TX_msg.len = 1;
  CAN_TX_msg.buf[0] =  numSat;
  can.write(CAN_TX_msg);

}//checkGPS()

void handleGPS() {
  byte error;
  // Request GPS data
  Wire.beginTransmission(GPS_ADDR);
  Wire.write(0xFF); //request data from data register
  Wire.endTransmission(false);
  // Read GPS data
  Wire.requestFrom(GPS_ADDR, 64);  // Arbitrarily read 64 bytes - LOOK INTO READING 0xFD & 0xFE

  // This sketch displays information every time a new sentence is correctly encoded.
  while (Wire.available()) {
    if (gps.encode(Wire.read())) {}
      //onGPSNew();
  }//while
  
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}//handleGPS()

void setup() {
  #if defined(SERIAL_ENABLE)
  softSerial.begin(SERIAL_BAUD);
  #endif
  
  // Set pinmodes
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Set up I2C
  Wire.setSCL(GPS_SCL_PIN);
  Wire.setSDA(GPS_SDA_PIN);
  Wire.begin();

  // Set up SPI
  SPI.setSCLK(FLASH_SCK_PIN);
  SPI.setMISO(FLASH_MISO_PIN);
  SPI.setMOSI(FLASH_MOSI_PIN);

  // Init CANBUS
  can.begin(); //automatic retransmission can be done using arg "true"
  can.setBaudRate(CANBUS_BAUD);

  // Initialize Flash Chip
  while (!SerialFlash.begin(FLASH_CS_PIN)) {
    softSerial.println(F("Connecting to SPI Flash chip..."));
    delay(250);
    //toggleStatusLED();
  }//while

  //digitalWrite(STATUS_LED_PIN, LOW);
  //delay(1000);

  // Initialize FlashTable object
  //table.init(&SerialFlash);

  //digitalWrite(STATUS_LED_PIN, HIGH);

  // STARTUP BEEP
  delay(BEEP_DELAY);
  tone(BUZZER_PIN, BEEP_FREQ);
  delay(1000);
  tone(BUZZER_PIN, 0);

  // Startup delay - Check to enter debug mode
  uint32_t startTime = millis();
  while (!softSerial.available() and millis()-startTime < 5000) {}
  
  if (softSerial.available()) {
    byte d = softSerial.read();
    emptySerialBuffer();
    if (d == 'D') {
      softSerial.println(F("Entered Debug Mode"));
      debugMode();
      while (true) {}
    }//if
  }//if
  softSerial.println(F("Running Normally"));
  
}//setup()



void loop() {
  // put your main code here, to run repeatedly:
  handleGPS();

  if (millis() - lastGPSSend > GPS_SEND_INT) {
    checkGPS();
    lastGPSSend = millis();
  }//if

}//loop()
