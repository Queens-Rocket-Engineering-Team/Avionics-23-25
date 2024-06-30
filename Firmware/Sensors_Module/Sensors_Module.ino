/*COTIJA Firmware - AIM Sensors Module

*/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <tone.h>
#include <flashTable.h>
#include <SerialFlash.h>
#include <SoftwareSerial.h>

#define USB_TX_PIN PA12 // D- pin
#define USB_RX_PIN PA11 // D+ pin
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define DB_LED_PIN PA15
#define BUZZER_PIN PA7
#define CAMERA_PWM_PIN PA11
#define STATUS_LED_PIN PA15

//flash
#define FLASH_CS_PIN PB12
#define FLASH_SCK_PIN PB13
#define FLASH_MISO_PIN PB14
#define FLASH_MOSI_PIN PB15
#define FLASH_RESET_PIN PA8

//buzzer
const uint32_t BEEP_DELAY = 6000;
const uint32_t BUZZER_TONE = 1000;
const uint32_t BUZZER_TONE_Q = 500;


//--- FLASH SETTINGS
uint16_t flashDelay = 250; // How frequently the debug LED should be toggled
uint32_t lastFlash = 0; // Last millis() the debug LED was toggle at

const uint32_t LOG_INTERVAL = 50;
uint32_t lastLog = 0;

const uint8_t TABLE_NAME = 0;
const uint8_t TABLE_COLS = 5;
const uint32_t TABLE_SIZE = 16776960; //16776960

//--- DATALOGGING SETTINGS

FlashTable flash = FlashTable(TABLE_COLS, 16384, TABLE_SIZE, TABLE_NAME, 256); 
Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

//GLOBAL VARIABLES


void setup() {

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(DB_LED_PIN, OUTPUT);
  pinMode(CAMERA_PWM_PIN, OUTPUT);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  SPI.setSCLK(PB13);
  SPI.setMISO(PB14);
  SPI.setMOSI(PB15);

  Serial.begin(115200);

  //LED FLASHES TO ALLOW FOR SERIAL OPEN
  for (int i=0 ; i <5 ; i++){
    digitalWrite(DB_LED_PIN,HIGH);
    delay(250);
    digitalWrite(DB_LED_PIN,LOW);
    delay(250);
  }

  while (!bme.begin(0x77)) {
    digitalWrite(DB_LED_PIN,HIGH);
    delay(100);
    digitalWrite(DB_LED_PIN,LOW);
    delay(100);
  }

//Set up flash device
while (!SerialFlash.begin(FLASH_CS_PIN)) {
    delay(250);
    //toggleStatusLED();
}//while


delay(BUZZER_TONE);
Serial.println("STARTING FLASH");
for (int i=0; i<3; i++) {
  tone(BUZZER_PIN, BUZZER_TONE_Q);
  delay(100);
  noTone(BUZZER_PIN);
  delay(100);
 }
 // Initialize FlashTable object
  flash.init(&SerialFlash, &Serial);

// STARTUP BEEP
tone(BUZZER_PIN, BUZZER_TONE);
delay(1000);
noTone(BUZZER_PIN);
Serial.println("STARTED");


Serial.print("Preparing BME680...");
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Serial.println("DONE");
  Serial.println("");

  Serial.println("ENTER D FOR DEBUG");
  digitalWrite(DB_LED_PIN,HIGH);
  
  uint32_t startTime = millis();
  while (!Serial.available() and millis()-startTime < 3000) {}
  
  if (Serial.available()) {
    byte d = Serial.read();
    emptySerialBuffer();
    Serial.println("Entered Debug Mode");
    debugMode();
    while (true) {}
  }//if

  digitalWrite(DB_LED_PIN,LOW);
}//start



/*---\/---\/---\/---\/---*\
      |  DEBUG MENU |    
\*---/\---/\---/\---/\---*/

// Empties all bytes from incoming serial buffer.
// Used by Debug mode
void emptySerialBuffer() {
  while (Serial.available()) {Serial.read();}
}//emptySerialBuffer()

// Called when Debug mode is activated;
// all normal board functions will cease.
// Only purpose is to respond to serial
// commands.
void debugMode() {

  // Status LED static "ON"
  digitalWrite(STATUS_LED_PIN, HIGH);

  while (true) {

    // Empty buffer
    emptySerialBuffer();

    // Wait for command
    while (!Serial.available()) {}
    uint8_t cmd = Serial.read();

    if (cmd == 'I') {
      // "Identify" command; return board name
      Serial.println(F("CRAB"));
    }//if
    if (cmd == 'F') {
      // "FlashInfo" command; return flash usage stats
      Serial.print(flash.getMaxSize());
      Serial.print(F(","));
      Serial.println(flash.getCurSize());   
    }//if
    if (cmd == 'D') {
      // "DumpFlash" command; dump all flash contents via serial
      flash.beginDataDump(&Serial);
    }//if
    if (cmd == 'E') {
      // "EraseFlash" command; completely erase contents of flash.
      // Should be restarted afterwards
      Serial.println(F("Erasing Flash"));
      SerialFlash.eraseAll();
      while (SerialFlash.ready() == false) {}
      //flash.init(&SerialFlash);
      Serial.println(F("Complete"));
    }//if
    if (cmd == 'Q') {
        // QUERY SENSORS
        
          //Read bme 680
        bme.performReading();
        
        Serial.println("--BME 680--");
        Serial.print("PRESSURE [Pa]: ");
        Serial.println(bme.pressure);
        Serial.print("TEMPERATURE [K]: ");
        Serial.println(bme.temperature+273);
        Serial.print("HUMIDITY [%]: ");
        Serial.println(bme.humidity);
        Serial.print("GAS RESISTANCE [Ohm]: ");
        Serial.println(bme.gas_resistance);
    }//if

  }//while

}//debugMode()

void loop() {

  if (millis() - lastLog > LOG_INTERVAL) {

    lastLog = millis();

    bme.performReading();
    uint32_t dataArr[5] = {0,0,0,0,0};
    dataArr[0] = millis();
    dataArr[1] = bme.pressure;
    dataArr[2] = bme.temperature+273;
    dataArr[3] = bme.humidity*100;
    dataArr[4] = bme.gas_resistance*100;

    //write to FLASH
    flash.writeRow(dataArr);
  }//if

  if (millis() > 5400000) {
    //turn off camera after 1.5hrs
    digitalWrite(CAMERA_PWM_PIN, LOW);
    Serial.println("CAM OFF");
  }
  else if (millis() > 15000) {
    //turn on camera after 15s
    digitalWrite(CAMERA_PWM_PIN, HIGH);
    Serial.println("CAM ON");
  }

}//loop()
