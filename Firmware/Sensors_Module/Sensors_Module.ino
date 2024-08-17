/*COTIJA Firmware - AIM Sensors Module
 * Authors: Kennan Bays, Joachim Blohm, Brent Naumann
*/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <tone.h>
#include <flashTable.h>
#include <SerialFlash.h>
#include <SoftwareSerial.h>
#include "pinouts.h"
#include <CANpackets.h>
#include "STM32_CAN.h"

//buzzer
// TODO: Update the buzzer settnigs for ALL modules
const uint32_t BEEP_DELAY = 6000;
const uint32_t BUZZER_TONE = 1000;
const uint32_t BUZZER_TONE_Q = 500;

const uint32_t CANBUS_DELAY = 1000;
uint32_t canbusLastSend; = 0;


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
STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_TX_msg ;

void setup() {
  // Configure pinmodes
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(CAMERA_POWER_PIN, OUTPUT);

  // Configure I2C bus
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  // Configure SPI
  SPI.setSCLK(PB13);
  SPI.setMISO(PB14);
  SPI.setMOSI(PB15);
  // Start USB debugging
  Serial.begin(115200);

  //LED FLASHES TO ALLOW FOR SERIAL OPEN
  for (int i=0 ; i <5 ; i++){
    digitalWrite(STATUS_LED_PIN,HIGH);
    delay(250);
    digitalWrite(STATUS_LED_PIN,LOW);
    delay(250);
  }//for


  //Set up flash device
  while (!SerialFlash.begin(FLASH_CS_PIN)) {
      delay(250);
      //toggleStatusLED();
  }//while


  // Initialize FlashTable object
  delay(BUZZER_TONE);
  Serial.println("STARTING FLASH");
  for (int i=0; i<3; i++) {
    tone(BUZZER_PIN, BUZZER_TONE_Q);
    delay(100);
    noTone(BUZZER_PIN);
    delay(100);
  }//for
  flash.init(&SerialFlash, &Serial);

  // STARTUP BEEP
  tone(BUZZER_PIN, BUZZER_TONE);
  delay(1000);
  noTone(BUZZER_PIN);
  Serial.println("STARTED");

  //Connect to BME680
  Serial.print("Preparing BME680...");
  while (!bme.begin(BME680_ADDR)) {
    digitalWrite(STATUS_LED_PIN,HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN,LOW);
    delay(100);
  }//while
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  Serial.println("BME680 Ready");

  Serial.println("ENTER D FOR DEBUG");
  digitalWrite(STATUS_LED_PIN,HIGH);
  
  uint32_t startTime = millis();
  while (!Serial.available() and millis()-startTime < 3000) {}

  // Prompt for entering debug mode
  if (Serial.available()) {
    byte d = Serial.read();
    emptySerialBuffer();
    Serial.println("Entered Debug Mode");
    debugMode();
    while (true) {}
  }//if

  digitalWrite(STATUS_LED_PIN,LOW);

  //Canbus Setup
  //Start CANBUS
  canb.begin(); //automatic retransmission can be done using arg "true"
  canb.setBaudRate(500000); //500kbps
}//setup()


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
  // Check if should log BME & other data
  if (millis() - lastLog > LOG_INTERVAL) {

    lastLog = millis();

    // TODO: Are all of these proper UINTs? Do we need to convert signed ints or floats?
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

  // Check if camera power should be given
  // TODO: Make this a bit more efficient? Only trigger at a certain time?
  if (millis() > 5400000) {
    //turn off camera after 1.5hrs
    digitalWrite(CAMERA_POWER_PIN, LOW);
    Serial.println("CAM OFF");
  } else if (millis() > 15000) {
    //turn on camera after 15s
    digitalWrite(CAMERA_POWER_PIN, HIGH);
    Serial.println("CAM ON");
  }//if

  if(millis() - canbusLastSend > CANBUS_DELAY){
    canbusLastSend = millis();
    sendCANstatus();
  }//if


}//loop()


// ---CANBUS 
void sendCANstatus(){

    //Build the CANBUS message
    CAN_TX_msg.id = (SENSOR_MOD_CANID+STATUS_CANID);
    CAN_TX_msg.len = 1;

    CAN_TX_msg.buf[0] = 1;

    canb.write(CAN_TX_msg);     //send
    return;
}
