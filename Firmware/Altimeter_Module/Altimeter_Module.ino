/*
QRET SRAD Avionics module - AIM
Altimeter Module Firmware
    V 1.0

Sensors used:
 - ms5607 : Pressure and Temperature
 - mpu6050 : Gyroscope
 - qma6100P : accelerometer

Written by:
    Brent Naumann
    Tristan Alderson
    Kennan Bays
Flash capability proveded by: 
    Kennan Bays

*/
//Libraries to include
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SimpleKalmanFilter.h>
#include <MS5xxx.h>
#include <Adafruit_MPU6050.h>
#include <QMA6100P.h>
#include <SerialFlash.h>
#include <SPI.h>
#include "flashTable.h"
#include <tone.h>
#include <CANpackets.h>
#include "STM32_CAN.h"
#include "pinouts.h"


// TODO: RE-WORK BUZZER FOR ALL MODULES
#define BUZZER_TONE 1000
#define BUZZER_TONE_Q 500


//--- Pin Definitions
#define USB_TX_PIN USB_DP_PIN // D+ pin
#define USB_RX_PIN USB_DM_PIN // D- pin

#define SERIAL_BAUD_RATE 38400
//#define SERIAL_BAUD_RATE 9600

//--- FILTER SETTINGS
#define M_UNCERT    15        //Uncertainty of taken measurements (varience)
#define EST_UNCERT  M_UNCERT  //Uncertainty of Kalman estimation (changes over time)
#define NOISE_SENS  0.008    //The filters sensitivity to unexpected changes. Range: 0.001 ->1
#define LAPSE_RATE -0.0065   //[K/m] change in temperature over height


//--- FLASH SETTINGS
uint16_t flashDelay = 250; // How frequently the debug LED should be toggled
uint32_t lastFlash = 0; // Last millis() the debug LED was toggle at

const uint8_t TABLE_NAME = 0;
const uint8_t TABLE_COLS = 11;
const uint32_t TABLE_SIZE = 16646144;
// IMPORTANT!!!; AT LEAST 2 BLOCK OF SPACE MUST BE RESERVED FOR FILE SYSTEM
// 16MiB = 16777216B, 2x 64KiB blocks = 131072B
// 16MiB - 128KiB = 16646144B

//--- CANBUS DATA SETTINGS
#define CANBUS_DATAINT 500 //[ms] interval bewteen each CANBUS message send.

//--- PREFLIGHT SETTINGS
#define PRE_DATAINT 50  //[ms] interval bewteen each log to FLASH.

//--- LAUNCH SETTINGS
#define LAUNCH_THRESHOLD  25  // [m/s]-mpu ; [g]-qma. Acceleration threshold to declare launch
#define LAUNCH_GRACE      75 // [ms] Time for measurements to be above threshold before Launch declared

//--- ASCENT SETTINGS
#define ASC_DATAINT 20 //  [ms] interval bewteen each log to FLASH.
#define APO_GRACE   500 // [ms] Time for measurements to be above threshold before Apogee is declared

//--- DESCENT SETTINGS
#define DEC_DATAINT 20 //  [ms] interval bewteen each log to FLASH.
#define LAND_THRESHOLD -0.3//[m/s] Velocity Threshold to declare land
#define LAND_GRACE  2000 // [ms] Time for measurements to be above threshold before Landing is declared
const uint16_t MAIN_DEPLOY_THRESHOLD = 1500/3.281;

//--- LAND SETTINGS
#define LAND_DATAINT 100  //[ms] interval bewteen each log to FLASH.



// Global objects to declare
SoftwareSerial softSerial(USB_RX_PIN, USB_TX_PIN); 
//#define softSerial Serial              //For debugging when not using CANBUS

SimpleKalmanFilter kalmanFilter(M_UNCERT,EST_UNCERT,NOISE_SENS); 
MS5xxx ms5607(&Wire);
Adafruit_MPU6050 mpu6050;
QMA6100P qma6100;

FlashTable flash = FlashTable(TABLE_COLS, 16384, TABLE_SIZE, TABLE_NAME, 256);

STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_TX_msg ;

//--- Global Variables
//sensors

outputData qmaData;          //For qma 6100
sensors_event_t a, g, temp;  // for mpu6050

float Po;
float To;

//state machine
uint8_t STATE = 0;  //State flag for state machine


//Pre-Launch
unsigned long potentialLaunchStart; //saves time of possible launch start
bool potentialLaunch = 0;

//Ascent

unsigned long potentialApoStart;
bool potentialApo = 0;

//decent
unsigned long potentialLandStart;
bool potentialLand = 0;

//ascent-decent shared
float vel;  //Decent velocity
float prev_alt;  //velocity from previous measurement
unsigned long curTime;
unsigned long prevTime;

//data logging and CANBUS
unsigned long lastLog; // Time of last data log
unsigned long lastSend; // Time of last CANBUS send

// ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---

//      SETUP

// ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---

void setup(){

  // Configure pinmodes
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(FIRE_MAIN_PIN,OUTPUT);
  pinMode(FIRE_DROGUE_PIN,OUTPUT);

  // Configure SPI
  SPI.setSCLK(PB13);
  SPI.setMISO(PB14);
  SPI.setMOSI(PB15);

  //Configure I2C PINS
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  //Start CANBUS
  canb.begin(); //automatic retransmission can be done using arg "true"
  canb.setBaudRate(500000); //500kbps

  //Start Serial
  softSerial.begin(SERIAL_BAUD_RATE);
    
  digitalWrite(STATUS_LED_PIN,LOW);

//    for (int i=0 ; i<5 ; i++){
//      digitalWrite(STATUS_LED_PIN,HIGH);
//      delay(250);
//      digitalWrite(STATUS_LED_PIN,LOW);
//      delay(750);
//    }

  softSerial.println("Connecting to required devices...");
            
  /*------------------*\
  |    FLASH SETUP    |
  \*------------------*/
  softSerial.print("SEARCHING: SPI Flash chip");
  while (!SerialFlash.begin(FLASH_CS_PIN)) {
    delay(250);
    //toggleStatusLED();
  }//while
  softSerial.println("  - CONNECTED");
  
  // Initialize FlashTable object
  for (int i=0; i<3; i++) {
  tone(BUZZER_PIN, BUZZER_TONE_Q);
  delay(100);
  noTone(BUZZER_PIN);
  delay(100);
  }
  flash.init(&SerialFlash, &softSerial);
  
  /*------------------*\
  |    MS5607 SETUP    |
  \*------------------*/
  softSerial.print("SEARCHING: MS5607");      
  ms5607.connect(); 
  
  while(ms5607.connect()>0) {  //Wait for ms5607 to connect
    delay(10);   
  }//while
  softSerial.println("  - CONNECTED");      //ms5607 connected
  ms5607.ReadProm();                    //Read the calibration coefficients
  ms5607.setOversampling(0x0);

  /*-------------------*\
  |    MPU6050 SETUP    |
  \*-------------------*/  
    softSerial.print("SEARCHING: MPU6050");

    while(!mpu6050.begin(MPU6050_ADDR)) {
      delay(10);
    }//while()
    softSerial.println("  - CONNECTED");    //ms6050 connected

    mpu6050.setAccelerometerRange(MPU6050_RANGE_16_G); 
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /*--------------------*\
  |    QMA6100P SETUP    |
  \*--------------------*/
  float qmaOffsetX = -1.495;
  float qmaOffsetY = -0.485;
  float qmaOffsetZ = 0.825;

  softSerial.print("SEARCHING: QMA6100P");
  
  while(!qma6100.begin()) {       //Wait for qma6100P to connect
      delay(10);  
  }
  softSerial.println("  - CONNECTED");    //qma6100P connected
  
  qma6100.softwareReset();
  qma6100.setRange(SFE_QMA6100P_RANGE32G);
  qma6100.setOffset(qmaOffsetX,qmaOffsetY,qmaOffsetZ);

  // STARTUP BEEP
  tone(BUZZER_PIN, BUZZER_TONE);
  delay(1000);
  noTone(BUZZER_PIN);

  // Get base measurements
  softSerial.println("Aquiring base Pressure...");
  Po = altimeterBasePres(500);
  softSerial.print("Base Pressure: ");
  softSerial.println(Po);
  
  softSerial.println("Aquiring base Temperature...");
  To = altimeterBaseTemp(500);
  softSerial.print("Base Temperature: ");
  softSerial.println(To);   
  
  softSerial.println("");
  softSerial.println("All sensors set up and configured.");
  softSerial.println("");

  // Startup delay - Check to enter debug mode
  softSerial.println("[MDE] SEND SERIAL TO ENTER DEBUG");
  uint32_t startTime = millis();
  while (!softSerial.available() and millis()-startTime < 4000) {}
  
  if (softSerial.available()) {
    byte d = softSerial.read();
    emptySerialBuffer();
    softSerial.println("[MDE] Entered Debug Mode");
    debugMode();
    while (true) {}
  }//if

  softSerial.println("Beginning PreFlight...");

  qma6100.enableAccel();  //Start accelerometer reading

  digitalWrite(STATUS_LED_PIN, HIGH);

}//setup()

// ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---

//      LOOP

// ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---

void loop(){
  /*---------|----------*\
      |SENSOR READING|    
  \*---------|----------*/
  
  //read pressure/temp
  ms5607.Readout();     //Read the data from the ms5607, which gets stored in the ms5xxx object
  
  float P = ms5607.GetPres();              //Get variables for pressure and temperature
  float T = ms5607.GetTemp()/100 + 273;

  //Kalman Filtering on pressure
  float P_filter = kalmanFilter.updateEstimate(P);

  //Find Altitude from filtered Pressure
  float alt = altitudeFind(P_filter,Po,To);

  //read gyro
  mpu6050.getEvent(&a, &g, &temp);

  //read accelerometer
  qma6100.getAccelData(&qmaData);
  qma6100.offsetValues(qmaData.xData, qmaData.yData, qmaData.zData);

  //--- CANBUS DATA SENDING
  if (millis() - lastSend >= CANBUS_DATAINT){
    //send Temperature, altitude, and Flight stage
    //sendCANtemp(T);
    sendCANaltitude(alt);
    sendCANstage(STATE);
    lastSend = millis();
  }//if

  /*---\/---\/---\/---\/---*\
       |  STATE MACHINE |    
  \*---/\---/\---/\---/\---*/

  switch (STATE){
    case 0:   //Pre-Flight Stage  
      // Data logging
      if( millis() - lastLog >= PRE_DATAINT){
        logDataToFlash(P,P_filter,T,&a,&g,&qmaData);
        lastLog = millis();
      }//if
      //Perform Launch Checks
      if(detectLaunch()){
          STATE = 1;  
          digitalWrite(STATUS_LED_PIN,LOW);
      }//if
      break;
      
    case 1:   //Ascending
      //logging data 
      if( millis() - lastLog >= ASC_DATAINT){
        logDataToFlash(P,P_filter,T,&a,&g,&qmaData);
        lastLog = millis();
      }//if
       
      //Perform Apogee check
      if(detectApogee(alt)){
        STATE = 3;
        digitalWrite(FIRE_DROGUE_PIN,HIGH);
        softSerial.println(F("FIRING DROGUE"));
      }//if
      
      //updating prevs
      prevTime = curTime;
      prev_alt = alt;
      break;

    case 3:   //Descending with drogue
      //logging data
      if( millis() - lastLog >= DEC_DATAINT){
        logDataToFlash(P,P_filter,T,&a,&g,&qmaData);
        lastLog = millis();
      }//if

      //fire main parachute if necessary
      if(alt<= MAIN_DEPLOY_THRESHOLD){
        digitalWrite(FIRE_MAIN_PIN, HIGH);
        softSerial.println(F("FIRING MAIN"));
        STATE = 4;
      }//if
      break;
      
    case 4:   //descending with main
      //logging data
      if( millis() - lastLog >= DEC_DATAINT){
        logDataToFlash(P,P_filter,T,&a,&g,&qmaData);
        lastLog = millis();
      }//if
      
      //perform Landing check
      if(detectLand(alt)){
        STATE = 5;
        softSerial.println("LANDED");
      }//if
      //updating prevs
      prevTime = curTime;
      prev_alt = alt;
      break;
      
    case 5:   // Landed
      //logging data
      if( millis() - lastLog >= LAND_DATAINT){
        logDataToFlash(P,P_filter,T,&a,&g,&qmaData);
        lastLog = millis();
      }//if
      break;

  }//switch
}//loop()

/*---\/---\/---\/---\/---*\
     |  DATA LOGGING |    
\*---/\---/\---/\---/\---*/

void logDataToFlash( float pressure,float pressure_filter,float temp,sensors_event_t *a,
                    sensors_event_t *g,outputData *qmaData){
  uint32_t dataArr[11] = {0,0,0,0,0,0,0,0,0,0,0};

  dataArr[0] = millis();
  dataArr[1] = uint32_t(pressure);
  dataArr[2] = uint32_t(temp*1000);
  // accelleration data (from mpu or qma)
  // dataArr[4] = static_cast<uint32_t>(a->acceleration.x*1e6);
  // dataArr[5] = static_cast<uint32_t>(a->acceleration.y*1e6);
  dataArr[3] = flash.unsignify(a->acceleration.z*10000);

  dataArr[4] = flash.unsignify(qmaData->xData*10000);
  dataArr[5] = flash.unsignify(qmaData->yData*10000);
  dataArr[6] = flash.unsignify(qmaData->zData*10000);
  
  //gyro data
  dataArr[7] = flash.unsignify(g->gyro.x*10000);
  dataArr[8] = flash.unsignify(g->gyro.y*10000);
  dataArr[9] = flash.unsignify(g->gyro.z*10000);

  //state
  dataArr[10] = STATE;

  //write to FLASH
  flash.writeRow(dataArr);

}//logDataToFlash()

/*---\/---\/---\/---\/---*\
     |  EVENT CHECKS |    
\*---/\---/\---/\---/\---*/
bool detectLaunch(){
  if (a.acceleration.z > LAUNCH_THRESHOLD){ //check if sensor reading is higher than 
    
    if (!potentialLaunch) {
      //if first measurement, begin launch countdown
      potentialLaunch = 1;
      potentialLaunchStart = millis();
    } else {
      //if not first, check time since first
      if (millis() - potentialLaunchStart >= LAUNCH_GRACE){
        //THEN LAUNCH IS ON
        return true;
      }//if
    }//if
    
  } else {
    potentialLaunch = 0;
  }//if
  return false;
}//end detectLaunch


bool detectApogee(float alt){
  curTime = millis();
  float dh = (alt - prev_alt)/(curTime-prevTime) * 1000;

  vel = 0.9*vel + 0.1*dh;
   
  if (vel < -0.1) {    //check if velocity is negative (with some error margin)
    if (!potentialApo) {
      //if first measurement, begin apogee countdown
      potentialApo = 1;
      potentialApoStart = millis();
    } else {
      //if not first, check time since first
      if (millis() - potentialApoStart >= APO_GRACE){
        //THEN WE HAVE HIT APOGEE
        return true;
      }//if
    }//if
    
  } else {
    potentialApo = 0;    
  }//if
  
  return false;    
}//detectApogee()

bool detectLand(float alt) {
  curTime = millis() ;
  float dh = (alt - prev_alt)/(curTime-prevTime) *1000;
  
  vel = 0.9*vel + 0.1*dh;
  
  if (vel > LAND_THRESHOLD && vel <= LAND_THRESHOLD) {  //check if velocity is less than -0.5 [m/s]
    
    if (!potentialLand) {
      //if first measurement, begin Land countdown
      potentialLand = 1;
      potentialLandStart = millis();
      
    } else {
      //if not first, check time since first
      if (millis() - potentialLandStart >= LAND_GRACE){
        //THEN WE HAVE LANDED
        return true;      
      }//if
    }//if
  
  } else {
    potentialLand = 0;
  }//if

  return false;  
}//detectLand()

/*---\/---\/---\/---\/---*\
      |  DEBUG MENU |    
\*---/\---/\---/\---/\---*/

/*
 * Empties all bytes from incoming Serial buffer.
 * Used by Debug mode
 */
void emptySerialBuffer() {
  while (softSerial.available()) {softSerial.read();}
}//emptySerialBuffer()

/*
 * Called when Debug mode is activated;
 * all normal board functions will cease.
 * Only purpose is to respond to serial
 * commands.
 */
void debugMode() {

  // Status LED static "ON"
  digitalWrite(STATUS_LED_PIN, HIGH);

  while (true) {

    // Empty buffer
    emptySerialBuffer();

    // Wait for command
    while (!softSerial.available()) {}
    uint8_t cmd = softSerial.read();

    if (cmd == 'I') {
      // "Identify" command; return board name
      softSerial.println(F("[MDE] OKA_ALT"));
    }//if
    if (cmd == 'F') {
      // "FlashInfo" command; return flash usage stats
      softSerial.print(F("[MDE] "));
      softSerial.print(flash.getMaxSize());
      softSerial.print(F(","));
      softSerial.println(flash.getCurSize());   
    }//if
    if (cmd == 'D') {
      // "DumpFlash" command; dump all flash contents via serial
      flash.beginDataDump(&softSerial);
    }//if
    if (cmd == 'E') {
      // "EraseFlash" command; completely erase contents of flash.
      // Should be restarted afterwards
      softSerial.println(F("[MDE] Erasing Flash"));
      SerialFlash.eraseAll();
      while (SerialFlash.ready() == false) {}
      //flash.init(&SerialFlash);
      softSerial.println(F("[MDE] Complete"));
    }//if
    if (cmd == 'Q') {
        // QUERY SENSORS
        qma6100.enableAccel();
          //Read pressure/temperature
        ms5607.Readout();
        softSerial.println("[MDE] --MS5607--");
        softSerial.print(F("[MDE] Temperature (0.01C): "));
        softSerial.println(ms5607.GetTemp()/100 + 273);
        softSerial.print(F("[MDE] Pressure (Pa): "));
        softSerial.println(ms5607.GetPres());

        //read gyro
      sensors_event_t a, g, temp;
      mpu6050.getEvent(&a, &g, &temp);
      softSerial.println("[MDE] --MPU6050--");
      softSerial.print("[MDE] X Rotation (rad/s): ");
      softSerial.println(g.gyro.x);
      softSerial.print("[MDE] Y Rotation (rad/s): ");
      softSerial.println(g.gyro.y);
      softSerial.print("[MDE] Z Rotation (rad/s): ");
      softSerial.println(g.gyro.z);
      
        //read accelerometer
      outputData qmaData;
      qma6100.getAccelData(&qmaData);
      qma6100.offsetValues(qmaData.xData, qmaData.yData, qmaData.zData);

      softSerial.println("[MDE] --QMA6100--");
      softSerial.print("[MDE] X Acceleration (g): ");
      softSerial.println(qmaData.xData);
      softSerial.print("[MDE] Y Acceleration (g): ");
      softSerial.println(qmaData.yData);
      softSerial.print("[MDE] Z Acceleration (g): ");
      softSerial.println(qmaData.zData);   
    }//if

  }//while

}//debugMode()

/*
 * Updates the status LED; should be run in loop() 
 */
void handleStatusFlash() {
  if (millis() - lastFlash > flashDelay) {
    lastFlash = millis();
    // toggle LED
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }//if
}//handleStatusFlash()

/*---\/---\/---\/---\/---*\
    | ALTIMETER FUNC |    
\*---/\---/\---/\---/\---*/
float altitudeFind(float P, float Po, float To){
  float alt = (To/LAPSE_RATE)*( pow(P/Po,0.19026627) - 1 );
  return alt;
}//altitudeFind()

/*
 * Finding Po and To
 */
float altimeterBaseTemp(int sample){
  float sum = 0;

  for (int i=0 ; i<sample ; i++){
    ms5607.Readout();
    float Ti = ms5607.GetTemp()/100 + 273;
    sum +=Ti;
  }//for
  float To = sum/sample;

  return To;
}//altimeterBaseTemp()

float altimeterBasePres(int sample){
  float sum = 0;
  
  for (int i=0 ; i<sample ; i++){
    ms5607.Readout();
    float Pi = ms5607.GetPres();
    sum +=Pi;
  }//for
  float Po = sum/sample;

  return Po;
}//altimeterBasePres()

/*---\/---\/---\/---\/---*\
    | CANBUS FUNCTIONS |    
\*---/\---/\---/\---/\---*/

/*
 * Sends current altitude over CANBUS
 */
void sendCANaltitude(float alt){

  //Adjust altitude into uin32_t 
  //pushes decimals to first 8 bits
  uint32_t altint = (alt*100);
  
  //Start shifting bits (b0 = LSB)
  uint8_t b0 = altint & 0xFF;
  uint8_t b1 = altint>>8 & 0xFF;
  uint8_t b2 = altint>>16 & 0xFF;
  uint8_t b3 = altint>>24 & 0xFF;
  
  //Build the CANBUS message
  CAN_TX_msg.id = (ALTIMETER_MOD_CANID+ALTITUDE_CANID);
  CAN_TX_msg.len = 4;
  
  CAN_TX_msg.buf[0] = b0;
  CAN_TX_msg.buf[1] = b1 ;
  CAN_TX_msg.buf[2] = b2 ;
  CAN_TX_msg.buf[3] = b3 ;
  
  canb.write(CAN_TX_msg);     //send
}//sendCANaltitude()

/*
 * Sends the current STAGE over the CANBUS
 */
void sendCANstage(int stage){
  //Build the CANBUS message
  CAN_TX_msg.id = (ALTIMETER_MOD_CANID+FLIGHT_STAGE_CANID);
  CAN_TX_msg.len = 1;
  
  CAN_TX_msg.buf[0] = (stage)&0xFF;
  
  canb.write(CAN_TX_msg);     //send
}//sendCANstage()

/*
 * Sends the current temperature over CANBUS
 */
void sendCANtemp(float temp){
  //pushes decimals to first 8 bits
  uint32_t tempint = (temp*100);
  
  //Start shifting bits (b0 = LSB)
  uint8_t b0 = tempint & 0xFF;
  uint8_t b1 = tempint>>8 & 0xFF;
  uint8_t b2 = tempint>>16 & 0xFF;
  uint8_t b3 = tempint>>24 & 0xFF;
  
  //Build the CANBUS message
  CAN_TX_msg.id = (ALTIMETER_MOD_CANID+TEMP_CANID);
  CAN_TX_msg.len = 4;
  
  CAN_TX_msg.buf[0] = b0;
  CAN_TX_msg.buf[1] = b1;
  CAN_TX_msg.buf[2] = b2;
  CAN_TX_msg.buf[3] = b3;
  
  canb.write(CAN_TX_msg);     //send
}//sendCANtemp()
