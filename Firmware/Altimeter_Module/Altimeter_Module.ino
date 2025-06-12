/*
Authors: Brent Naumann, Tristan Alderson, Kennan Bays, Caelan Donovan
Env: Arduino 1.8.10, STM32duino 2.7.1
Updated: Jun.12.2025
Purpose: QRET SRAD Avionics module - AIM Altimeter Module Firmware V 2.0 (STINGER)


Sensors used:
 - ms5611 : Pressure and Temperature
 - mpu6050 : Gyroscope
 - kx134 : accelerometer (SIKE)



*/
//Libraries to include
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_MPU6050.h>
#include <SparkFun_KX13X.h> 
#include <SerialFlash.h>
#include <SPI.h>
#include "flashTable.h"
#include <tone.h>
#include <CANpackets.h>
#include "STM32_CAN.h"
#include "pinouts.h"
#include <HardwareSerial.h>
#include "MS5611.h" // https://github.com/RobTillaart/MS5611


// TODO: RE-WORK BUZZER FOR ALL MODULES
#define BUZZER_TONE 1000
#define BUZZER_TONE_Q 500
const uint32_t BEEP_FREQ = 1000;

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
#define DROGUE_DATAINT 20 //  [ms] interval between each log to FLASH.
#define MAIN_DATAINTFAST 500 // [ms] interval between each log to FLASH - slowed rate before 5000 ms
#define MAIN_DATAINTSLOW 1000 // [ms] interval between each log to FLASH - slowed rate after 5000 ms
#define LAND_THRESHOLD -0.3//[m/s] Velocity Threshold to declare land
#define LAND_GRACE  2000 // [ms] Time for measurements to be above threshold before Landing is declared
const uint16_t MAIN_DEPLOY_THRESHOLD = 1500/3.281;

//--- LAND SETTINGS
#define LAND_DATAINT 1000  //[ms] interval between each log to FLASH.
#define LAND_SLOW_DATAINT 2000 //[ms] interval between each log to FLASH.



// Global objects to declare
HardwareSerial usb(USB_RX_PIN, USB_TX_PIN);
//#define usb Serial              //For debugging when not using CANBUS

SimpleKalmanFilter kalmanFilter(M_UNCERT,EST_UNCERT,NOISE_SENS); 
MS5611 ms5611(MS5611_ADDR, &Wire);


Adafruit_MPU6050 mpu6050;
SparkFun_KX134 kxAccel;

FlashTable flash = FlashTable(TABLE_COLS, 16384, TABLE_SIZE, TABLE_NAME, 256);

STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_TX_msg ;

//--- Global Variables
//sensors

outputData kxData;          //For kx134
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

//extra timers
int mainTimer = 0;
int landingTimer =0;

// ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---

//      SETUP

// ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---   ---

void setup(){

  // Configure pinmodes
  pinMode(BUZZER_A_PIN, OUTPUT);
  pinMode(BUZZER_B_PIN, OUTPUT);
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
  usb.begin(SERIAL_BAUD_RATE);
  delay(1000);

  usb.print("BUZZER REACHED");      

  initBuzzer();
  setBuzzerFreq(BEEP_FREQ);

  // // STARTUP BEEP
  startBuzzer();
  delay(1000);
  stopBuzzer();
    
  digitalWrite(STATUS_LED_PIN,LOW);

//    for (int i=0 ; i<5 ; i++){
//      digitalWrite(STATUS_LED_PIN,HIGH);
//      delay(250);
//      digitalWrite(STATUS_LED_PIN,LOW);
//      delay(750);
//    }

  usb.println("Connecting to required devices...");
            
  /*------------------*\
  |    FLASH SETUP    |
  \*------------------*/
  usb.print("SEARCHING: SPI Flash chip");
  while (!SerialFlash.begin(FLASH_CS_PIN)) {
    delay(250);
    //toggleStatusLED();
  }//while
  usb.println("  - CONNECTED");
      SerialFlash.eraseAll();
      while (SerialFlash.ready() == false) {}  
  // Initialize FlashTable object
  for (int i=0; i<3; i++) {
  // tone(BUZZER_A_PIN, BUZZER_TONE_Q);
  delay(100);
  noTone(BUZZER_A_PIN);
  delay(100);
  }
  flash.init(&SerialFlash, &usb);


  /*------------------*\
  |    MS5611 SETUP    |
  \*------------------*/
  usb.print("SEARCHING: MS5611");      
  while (!ms5611.begin()) {
    delay(10);
  }
  usb.println("  - CONNECTED"); //MS5611 connected

  // Set oversampling
  ms5611.setOversampling(OSR_ULTRA_LOW);

  /*-------------------*\
  |    MPU6050 SETUP    |
  \*-------------------*/  
    usb.print("SEARCHING: MPU6050");

    while(!mpu6050.begin(MPU6050_ADDR)) {
      delay(10);
    }//while()
    usb.println("  - CONNECTED");    //ms6050 connected

    mpu6050.setAccelerometerRange(MPU6050_RANGE_16_G); 
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);



  /*--------------------*\
  |    KX134 SETUP    |
  \*--------------------*/
  //float qmaOffsetX = -1.495;
  //float qmaOffsetY = -0.485;
  //float qmaOffsetZ = 0.825;

  // usb.print("SEARCHING: KX134");

  // while(!kxAccel.begin(KX134_ADDR)) {       // Wait for kx134 to connect
  //     delay(10);  
  //     for (byte address = 1; address < 127; ++address) {
  //       Wire.beginTransmission(address);
  //       byte error = Wire.endTransmission();

  //       if (error == 0) {
  //         Serial.print("Found device at 0x");
  //         Serial.println(address, HEX);
  //       }
  //     }
  // }
  // usb.println("  - CONNECTED");

  // usb.println("Resetting KX134...");
  // kxAccel.softwareReset();
  // delay(100); // Wait for reset to complete

  // usb.println("Setting range...");
  // kxAccel.setRange(SFE_KX134_RANGE32G);
  // usb.println("KX134 configured.");

  // Get base measurements
  usb.println("Aquiring base Pressure...");
  Po = altimeterBasePres(500);
  usb.print("Base Pressure: ");
  usb.println(Po);
  
  usb.println("Aquiring base Temperature...");
  To = altimeterBaseTemp(500);
  usb.print("Base Temperature: ");
  usb.println(To);   
  
  usb.println("");
  usb.println("All sensors set up and configured.");
  usb.println("");

  // Startup delay - Check to enter debug mode
  usb.println("[MDE] SEND SERIAL TO ENTER DEBUG");
  uint32_t startTime = millis();
  while (!usb.available() and millis()-startTime < 4000) {}
  
  if (usb.available()) {
    byte d = usb.read();
    emptySerialBuffer();
    usb.println("[MDE] Entered Debug Mode");
    debugMode();
    while (true) {}
  }//if

  usb.println("Beginning PreFlight...");

  // kxAccel.enableAccel();  //Start accelerometer reading

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
  ms5611.read();     //Read the data from the ms5611, which gets stored in object

  //Get variables for pressure and temperature
  float P = ms5611.getPressurePascal();  // Pascals
  float T = ms5611.getTemperature(); // Celcius

  //Kalman Filtering on pressure
  float P_filter = kalmanFilter.updateEstimate(P);

  //Find Altitude from filtered Pressure
  float alt = altitudeFind(P_filter,Po,To);

  // TODO: FIX IN THE FUTURE
  // Override the inaccurate altitude system using raw pressure reading (im assuming its in pascals)
  alt = (288.15d/0.0065d) * (1-pow(P_filter/101325.0f, 0.190284f));

  //read gyro
  mpu6050.getEvent(&a, &g, &temp);

  //read accelerometer
  //kxAccel.getAccelData(&kxData);
  //kxAccel.offsetValues(kxData.xData, kxData.yData, kxData.zData);

  //--- CANBUS DATA SENDING
  if (millis() - lastSend >= CANBUS_DATAINT){
    //send Temperature, altitude, and Flight stage
    //sendCANtemp(T);
    usb.print("Pres(Pa): ");
    usb.print(P_filter, 2);
    usb.print(", Temp(C): ");
    usb.print(T);
    usb.print(", Alt(m): ");
    usb.println(alt);
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
        logDataToFlash(P,P_filter,T,&a,&g);
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
        logDataToFlash(P,P_filter,T,&a,&g);
        lastLog = millis();
      }//if
       
      //Perform Apogee check
      if(detectApogee(alt)){
        STATE = 3;
        digitalWrite(FIRE_DROGUE_PIN,HIGH);
        usb.println(F("FIRING DROGUE"));
      }//if
      
      //updating prevs
      prevTime = curTime;
      prev_alt = alt;
      break;

    case 3:   //Descending with drogue
      //logging data
      if( millis() - lastLog >= DROGUE_DATAINT){
        logDataToFlash(P,P_filter,T,&a,&g);
        lastLog = millis();
      }//if

      //fire main parachute if necessary
      if(alt<= MAIN_DEPLOY_THRESHOLD){
        digitalWrite(FIRE_MAIN_PIN, HIGH);
        usb.println(F("FIRING MAIN"));
        STATE = 4;
        mainTimer = millis();
      }//if
      break;
      
    case 4:   //descending with main
      //logging data
      if(millis() - mainTimer <= 5000){
        if(millis() - lastLog >= MAIN_DATAINTFAST){
          logDataToFlash(P,P_filter,T,&a,&g);
          lastLog = millis();
        }//if
      }//if
      else if(millis() - mainTimer > 5000){
        if(millis() - lastLog >= MAIN_DATAINTSLOW){
          logDataToFlash(P,P_filter,T,&a,&g);
          lastLog = millis();
        }//if
      }//if
      
      //perform Landing check
      if(detectLand(alt)){
        landingTimer = millis();
        STATE = 5;
        usb.println("LANDED");
      }//if
      //updating prevs
      prevTime = curTime;
      prev_alt = alt;
      break;
      
    case 5:   // Landed
      //logging data
      if (millis() - landingTimer <= 1800000){
        if( millis() - lastLog >= LAND_DATAINT){
          logDataToFlash(P,P_filter,T,&a,&g);
          lastLog = millis();

        }//if
      }
      //slower logging rate after 30 mins to save power
      else if (millis() - landingTimer > 1800000 && millis() - landingTimer < 3600000){
        if( millis() - lastLog >= LAND_SLOW_DATAINT){
          logDataToFlash(P,P_filter,T,&a,&g);
          lastLog = millis();

        }//if
      }
      //power save mode after 60 mins
      else if (millis() - landingTimer > 36000000){
        
      }
      break;

  }//switch
}//loop()

/*---\/---\/---\/---\/---*\
     |  DATA LOGGING |    
\*---/\---/\---/\---/\---*/

void logDataToFlash( float pressure,float pressure_filter,float temp,sensors_event_t *a, sensors_event_t *g){
  uint32_t dataArr[8] = {0,0,0,0,0,0,0,0};

  dataArr[0] = millis();
  dataArr[1] = uint32_t(pressure);
  dataArr[2] = uint32_t(temp*1000);
  // accelleration data (from mpu or qma)
  // dataArr[4] = static_cast<uint32_t>(a->acceleration.x*1e6);
  // dataArr[5] = static_cast<uint32_t>(a->acceleration.y*1e6);
  dataArr[3] = flash.unsignify(a->acceleration.z*10000);
  
  //gyro data
  dataArr[4] = flash.unsignify(g->gyro.x*10000);
  dataArr[5] = flash.unsignify(g->gyro.y*10000);
  dataArr[6] = flash.unsignify(g->gyro.z*10000);

  // dataArr[7] = flash.unsignify(altitudeFind(pressure_filter, altimeterBasePres(500),altimeterBaseTemp(500)));

  //state
  dataArr[7] = STATE;

  //write to FLASH
  bool success = flash.writeRow(dataArr);
  if (success) {
    //usb.println("[MDE] Data logged successfully");
  } else {
      usb.println("[MDE] Flash write failed!");
  }

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
  while (usb.available()) {usb.read();}
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
    while (!usb.available()) {}
    uint8_t cmd = usb.read();

    if (cmd == 'I') {
      // "Identify" command; return board name
      usb.println(F("[MDE] STINGER_ALT"));
    }//if
    if (cmd == 'F') {
      // "FlashInfo" command; return flash usage stats
      usb.print(F("[MDE] "));
      usb.print(flash.getMaxSize());
      usb.print(F(","));
      usb.println(flash.getCurSize());   
    }//if
    if (cmd == 'D') {
      // "DumpFlash" command; dump all flash contents via serial
      flash.beginDataDump(&usb);
    }//if
    if (cmd == 'L') {
      // LOG DATA
      //Get variables for pressure and temperature
      float P = ms5611.getPressurePascal(); //pascals
      float T = ms5611.getTemperature(); //celcius
      float P_filter = kalmanFilter.updateEstimate(P);
      mpu6050.getEvent(&a, &g, &temp);
      logDataToFlash(P,P_filter,T,&a,&g);
    }  
    if (cmd == 'E') {
      // "EraseFlash" command; completely erase contents of flash.
      // Should be restarted afterwards
      usb.println(F("[MDE] Erasing Flash"));
      SerialFlash.eraseAll();
      while (SerialFlash.ready() == false) {}
      //flash.init(&SerialFlash);
      usb.println(F("[MDE] Complete"));
    }//if
    if (cmd == 'Q') {
        // QUERY SENSORS
        // kxAccel.enableAccel();
          //Read pressure/temperature
        ms5611.read();
        usb.println("[MDE] --MS5611--");
        usb.print(F("[MDE] Temperature (1.00C): "));
        usb.println(ms5611.getTemperature());
        usb.print(F("[MDE] Pressure (Pa): "));
        usb.println(ms5611.getPressurePascal());

        //read gyro
      sensors_event_t a, g, temp;
      mpu6050.getEvent(&a, &g, &temp);
      usb.println("[MDE] --MPU6050--");
      usb.print("[MDE] X Rotation (rad/s): ");
      usb.println(g.gyro.x);
      usb.print("[MDE] Y Rotation (rad/s): ");
      usb.println(g.gyro.y);
      usb.print("[MDE] Z Rotation (rad/s): ");
      usb.println(g.gyro.z);
      
        //read accelerometer
      // outputData kxData;
      //kxAccel.getAccelData(&kxData);
      //kxAccel.offsetValues(kxData.xData, kxData.yData, kxData.zData);

      // usb.println("[MDE] --kxAccel--");
      // usb.print("[MDE] X Acceleration (g): ");
      // usb.println(kxData.xData);
      // usb.print("[MDE] Y Acceleration (g): ");
      // usb.println(kxData.yData);
      // usb.print("[MDE] Z Acceleration (g): ");
      // usb.println(kxData.zData);   
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
    ms5611.read();
    float Ti = ms5611.getTemperature(); //celcius
    sum +=Ti;
  }//for
  float To = sum/sample;

  return To;
}//altimeterBaseTemp()

float altimeterBasePres(int sample){
  float sum = 0;
  
  for (int i=0 ; i<sample ; i++){
    ms5611.read();
    float Pi = ms5611.getPressurePascal(); //pascals
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
  int32_t altint = (alt);
  
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
