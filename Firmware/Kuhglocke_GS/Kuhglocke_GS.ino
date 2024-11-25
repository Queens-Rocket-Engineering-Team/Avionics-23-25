/*
 * Author: Kennan Bays (Kenneract)
 * Created: Aug.2.2024
 * Updated: Aug.20.2024
 * Purpose: Testing firmware for the Kuhglocke ground station
 * Hardware: QRET Kuhglocke ground station V1.0 (ESP32-S3-N16R2; 16MiB Flash (QSPI), 2MiB PSRAM (QSPI))
 * 
 * Suggested Configration:
 * - Board: ESP32S3 Dev Module
 * - Upload Speed: 921600
 * - USB Mode: Hardware CDC and JTAG [IMPORTANT]
 * - USB CDC On Boot: Disabled
 * - USB Firmware MSC On Boot: Disabled
 * - USB DFU On Boot: Disabled
 * - Upload Mode: UART0 / Hardware CDC [IMPORTANT]
 * - CPU Frequency: 240MHz (WiFi)
 * - Flash Mode: QIO 80MHz
 * - Flash Size: 16MB (128Mb) [IMPORTANT]
 * - Partition Scheme: 8M with SPIFFS (3MB APP/1.5MB SPIFFS) [IMPORTANT]
 * - Core Debug Level: None
 * - PSRAM: QSPI PSRAM [IMPORTANT]
 * - Arduino Runs On: Core 1 [IMPORTANT]
 * - Events Run On: Core 1 [IMPORTANT]
 * - Erase All Flash Before Sketch Upload: Disabled
 * 
 * Use the ESP32FS.jar plugin for uploading data to SPIFFS
 * 
 * Onboard Peripherals
 *  - Temperature Sensor
 *  - MicroSD Card
 *  - GPS
 *  - LoRa Radio
 *  - Speaker
 *  - I2C ADC
 *    - Battery Sense
 *    - Current Sense
 *  - RGB LEDs
 *  - Batt voltage
 *  - Current sense
 *  
 */


#include "pinouts.h"
#include <Wire.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <RadioLib.h>
#include <GxEPD2_BW.h> // https://github.com/ZinggJM/GxEPD2
#include "GxEPD2_display_selection_new_style.h" //TODO: Investigate how much of this is nessesary to include.
#include "SD_MMC.h"
#include <NAU7802_2CH.h> //https://github.com/Kenneract/NAU7802_Arduino_2CH
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "MedianFilterLib.h" //https://github.com/luisllamasbinaburo/Arduino-MedianFilter
#include <math.h>

#define EARTH_RADIUS_FEET 20925524.9

const String FIRMWARE_VERSION = "Aug.20.2024, V1.0.2A";

// ================================================================================================
// ================================= \/ GLOBAL CONSTANTS \/ =======================================
// ================================================================================================

uint8_t NUM_RGB_LEDS = 5;
uint32_t USB_BAUD = 115200;
uint32_t GPS_BAUD = 9600; //supports up to 115200?
uint32_t EPD_SPI_CLOCK = 1000000; //1MHz
uint32_t EPD_BAUD = 115200; //TODO: what does this baud actually do electrically?
uint32_t RFM_SPI_CLOCK = 2000000; //2MHz
uint32_t I2C_SPEED = 100000; //100kHz

const char* AP_SSID     = "QRET Kuhglocke";
const char* AP_PASSWORD = "BessieRocks";
const uint8_t WIFI_CHANNEL = 10;
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_7dBm; //WIFI_POWER_19_5dBm

const double BATT_VDIV = 3.003579098067;
const uint16_t SENSOR_SAMPLE_PERIOD = 250; //How often (ms) to sample onboard sensors (temp, batt volt, current, etc)

const uint8_t DEFAULT_LED_BRIGHTNESS = 15; //0-255
const uint16_t IND_RF_FLASH_TIME = 130; //Duration (ms) of Ping LED flash time
const uint16_t IND_BATTERY_FLASH_GAP = 750; //How long (ms) between LED flashes when battery is charging

const uint16_t RFM_CONNECTED_TIMEOUT = 2000; //Wait period after a ping to consider radio disconnected
const uint8_t RFM_PACKET_SIZE = 14; //Expectd num bytes in rocket packet

const uint32_t LOCAL_GPS_LOG_RATE = 1000;

const uint32_t ALT_CORE_STACKS_SIZE = 10000; //Bytes allocated to second core's stack

const uint16_t EPD_UPDATE_INT = 800; //How frequently to update values on EPD (partial update)


// ================================================================================================
// ================================= \/ GLOBAL VARIABLES \/ =======================================
// ================================================================================================

double freqOpts[] = {902.0, 905.4, 928.0};
uint8_t numFreqOpts = 3;
double bandwidthOpts[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
uint8_t numBandwidthOpts = 10;
int32_t spreadOpts[] = {7, 8, 9, 10, 11, 12};
uint8_t numSpreadOpts = 6;
int32_t codingOpts[] = {5, 6, 7, 8};
uint8_t numCodingOpts = 4;
int32_t freqCorrectionOpts[] = {10000, 8000, 6000, 4000, 2000, 0, -2000, -4000, -6000, -8000, -10000}; //Hz
uint8_t numFreqCorrectionOpts = 11;

uint8_t freqSelected = 1;
uint8_t bandwidthSelected = 4;
uint8_t spreadSelected = 3;
uint8_t codingSelected = 1;
uint8_t freqCorrectionSelected = 5;

// Incoming RFM95 packet data
volatile uint32_t rfmLastRFReceived = 0; // last millis() when RF was received
volatile int16_t rfmLastRSSI = 0;
volatile float rfmLastSNR = 0;
volatile int32_t rfmLastFreqErr = 0;
byte rfmLastPacket[RFM_PACKET_SIZE]; // Actual last packet data
volatile bool rfmLastPacketValid = false;

// Decoded rocket packet data
volatile int32_t rocketGPSLat = 0;
volatile int32_t rocketGPSLon = 0;
volatile uint8_t rocketGPSSats = 0;
volatile int32_t rocketAltitude = 0;
volatile uint8_t rocketStatus = 0;


uint32_t lastLocalGPSLog = 0; //When we last logged local GPS to SD

int32_t curFreqOffset = 0;


uint32_t lastEPDUpdate = millis(); // How long its been since a partial EPD update was done



// (anything used set by Core0 must be volatile)
// (assignment is mostly atomic operation, so no mutex for simple stuff)
volatile uint32_t core0FreeStack = 0;
volatile uint32_t core0LoopTime = 0; //TODO: SWITCH TO AVG FILTER
volatile uint32_t maxCore0LoopTime = 0;
uint32_t maxCore1LoopTime = 0; // Longest recorded loop() execution time

volatile bool rfmReceivedFlag = false; //Flag for if packet received by RFM95W

uint32_t lastSensorRead = 0; //millis() when onboard sensors were last read
bool nauChannel = 0; //Which channel is currently to be sampled

bool usbDataOnly = false; //If USB is set to "data only" mode

uint32_t indRFFlashStart = 0; //time the ping LED started flashing
bool indBattFlashState = true; //on
uint32_t indBattLastToggle = 0; //last time battery indicator flashed

uint16_t logFileNumber = 0; //Number of current log file (assuming logging is active)



// ================================================================================================
// ================================== \/ GLOBAL OBJECTS \/ ========================================
// ================================================================================================

// Second core task handle
TaskHandle_t Core0Task;

// Sensor median filters
MedianFilter<uint32_t> vccVoltFilter(8);
MedianFilter<uint32_t> battVoltFilter(8);
MedianFilter<uint32_t> sysCurrentFilter(8); //TODO: CHANGE THIS TO AN AVG FILTER
MedianFilter<uint32_t> ambTempFilter(8);

// Core loop median filter TODO: CHANGE THIS TO AN AVG FILTER
MedianFilter<uint16_t> core1LoopFilter(8);

// The TinyGPSPlus object
TinyGPSPlus gps;

// UART1 Serial Object
HardwareSerial gpsSerial(1);

// RGB LEDs object
Adafruit_NeoPixel rgbLEDs(NUM_RGB_LEDS, ARGB_DATA_PIN, NEO_GRB + NEO_KHZ800);

// E-Paper Display Objects (pick class/driver)
SPIClass epdSPI(FSPI);
GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT> display(GxEPD2_213_B74(EINK_CS_PIN, EINK_DC_PIN, EINK_RESET_PIN, EINK_BUSY_PIN));

// RFM95 Objects
SPIClass rfmSPI(HSPI);
SPISettings rfmSPISettings(RFM_SPI_CLOCK, MSBFIRST, SPI_MODE0);
RFM95 radio = new Module(RF_CS_PIN, RF_DIO0_PIN, RF_RESET_PIN, RF_DIO1_PIN, rfmSPI, rfmSPISettings);

// NAU7802 ADC object
NAU7802 nau;

// AsyncWebServer object on port 80
AsyncWebServer webServer(80);

// Global log file handle
File logFile;


// ================================================================================================
// ===================================== \/ FUNCTIONS \/ ==========================================
// ================================================================================================

/*
 * Returns the estimated PSU voltage
 * in mV. NOTE: Not nessesarily
 * accurate!
 */
uint16_t getPSUVoltage() {
  return vccVoltFilter.GetFiltered();
}//getPSUVoltage()

/*
 * Returns the battery voltage in mV
 */
uint16_t getBatteryVoltage() {
  // Acquire raw values
  uint16_t vRef = getPSUVoltage()/2; //full-scale range is 0.5*VREF
  uint32_t battCount = battVoltFilter.GetFiltered();
  // Calculate battery voltage
  return uint16_t(BATT_VDIV * vRef * (battCount / 8388607.0));
}//getBatteryVoltage()

/*
 * Returns the system current consumption
 * in mA
 */
uint16_t getSystemCurrent() {
  // Acquire raw values
  uint16_t vRef = getPSUVoltage()/2; //full-scale range is 0.5*VREF
  uint32_t currCount = sysCurrentFilter.GetFiltered();
  // Calculate system current
  uint16_t currVolt = (vRef * (currCount / 8388607.0));
  return (currVolt-250)*1.25; //Simplified formula, see ACS70331 datasheet
}//getSystemCurrent()

/*
 * Returns the reading from the onboard 
 * temperature sensor (in 0.01C increments)
 */
int16_t getAmbTemperature() {
  return ambTempFilter.GetFiltered();
}//getAmbTemperature()


uint8_t voltToPercent(uint16_t mv) {
  // A rough converstion that converts the OCV of
  // a 1S Li-Ion (INR) battery to a SoC percentage.
  
  if (mv>=4160) { return 100; }
  else if (mv>=4110) { return 96; }
  else if (mv>=4080) { return 92; }
  else if (mv>=4050) { return 88; }
  else if (mv>=4010) { return 84; }
  else if (mv>=3970) { return 80; }
  else if (mv>=3920) { return 76; }
  else if (mv>=3880) { return 72; }
  else if (mv>=3840) { return 68; }
  else if (mv>=3800) { return 64; }
  else if (mv>=3750) { return 60; }
  else if (mv>=3720) { return 56; }
  else if (mv>=3690) { return 52; }
  else if (mv>=3660) { return 48; }
  else if (mv>=3630) { return 44; }
  else if (mv>=3620) { return 40; }
  else if (mv>=3590) { return 36; }
  else if (mv>=3570) { return 32; }
  else if (mv>=3540) { return 28; }
  else if (mv>=3500) { return 24; }
  else if (mv>=3460) { return 20; }
  else if (mv>=3430) { return 16; }
  else if (mv>=3330) { return 12; }
  else if (mv>=3220) { return 8; }
  else if (mv>=3100) { return 4; }
  else { return 0; }

}//voltToPercent


int16_t rawReadTempSensor() {
  // Reads the raw value from the temperature sensor.
  // For a filtered value, see getAmbTemperature
  // Returns temperature in dC (100 = 1.00C)
  // TODO: Simple math can only handle positive temps; see datasheet for proper method

  Wire.requestFrom(P3T1755_ADDR, 2);
  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  int16_t rawData = (b1 << 8) | b2;
  rawData = rawData >> 4;
  //Serial.print("TempRaw = ");
  //Serial.print(b1, BIN);
  //Serial.print(" ");
  //Serial.println(b2, BIN);
  return (rawData * 6.25);

  //1111111 11110000 (very hot from heat gun)
  //100100 11110000 (warm from heat gun)
  //11010 1010000 (basically room temperature)
}//rawReadTempSensor()


/*
 * Sets if the system can draw power from
 * the USB port, or if it should just be
 * used for data exchange.
 * WARNING: Setting Data-Only while only
 * being powered from USB will crash system
 */
void setUSBDataOnlyMode(bool state) {
  usbDataOnly = state;
  digitalWrite(DISABLE_5V_PIN, state);
}//setUSBDataOnlyMode()

/*
 * Sets the global brightness for all LED
 * indicators.
 */
void setLEDBrightness(uint8_t brightness) {
  rgbLEDs.setBrightness(brightness);
  rgbLEDs.show();
}//setLEDBrightness

void setRGB(byte index, byte r, byte g, byte b) {
  setRGB(index, r,g,b, true);
}//setRGB(byte, byte, byte, byte)

void setRGB(byte index, byte r, byte g, byte b, bool push) {
  rgbLEDs.setPixelColor(index, rgbLEDs.Color(r,g,b));
  if (push) {rgbLEDs.show();}
}//setRGB(byte, byte, byte, byte, bool)

void setRGB(byte r, byte g, byte b) {
  for (byte i=0; i<NUM_RGB_LEDS; i++) {
    setRGB(i, r,g,b);
  }//for
}//setRGB(byte, byte, byte)


/*
 * Converts a given byte array to a Hexademical String object
 */
String byteArrayToHexString(byte* byteArray, int length) {
  String hexString = "";
  for (int i = 0; i < length; i++) {
    if (byteArray[i] < 0x10) {
      hexString += "0"; // Add leading zero for single digit hex values
    }
    hexString += String(byteArray[i], HEX);
  }
  hexString.toUpperCase(); // Convert to uppercase if desired
  return hexString;
}//byteArrayToHexString()


/*
 * Assuming SD card is connected,
 * finds the next available ID for
 * a log file and generates it.
 * 
 * Fills in the global file & ID
 * variables.
 * 
 * Returns true if successful
 */
bool makeNextSDLog() {
  // Ensure SD card is present
  if (SD_MMC.cardType() == CARD_NONE) {
    return false;
  }//if

  // Ensure logs directory exists
  SD_MMC.mkdir("/logs");

  // Find next Log ID
  uint16_t id = 0;
  while (true) {
    if (SD_MMC.exists("/logs/Kuhglocke_Log" + String(id) + ".txt")) {
      id++;
    } else {
      break;
    }//if
  }//while
  logFileNumber = id;
  Serial.println("Selected log file number");

  // Create new log file
  logFile = SD_MMC.open("/logs/Kuhglocke_Log" + String(id) + ".txt", FILE_WRITE);
  if (!logFile) {
    return false;
    Serial.println("[WARN] Unable to open log file for writing");
  }//if

  return true;
}//makeNextSDLog()

/*
 * Given a string, writes it to the
 * ongoing log file. 
 *  
 * If no SD card is present, function
 * will silently & gracefully fail.
 */
bool writeToSDLog(String txt) {
  String output = "[" + String(millis()) + "] ";
  output += txt;
  bool r = logFile.println(output);
  logFile.flush();
  return r;
}//writeToSDLog()


/*
 * Determines the current charge status.
 * 0 = Charging
 * 1 = Fully Charged
 * 2 = Disabled (on battery)
 */
uint8_t getChargingStatus() {
  uint16_t chrgStat = analogRead(CHRG_STAT_PIN);
  if (chrgStat < 200) {
    // 0.00V = Charging
    return 0;
  } else if (chrgStat > 3800) {
    // 3.30V = Disabled
    return 2;
  }//if
  // 1.65V = Fully Charged
  return 1;
}//getChargingStatus



// this function is called when a complete packet
// is received by the RFM95 module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we got a packet, set the flag
  rfmReceivedFlag = true;
}//setFlag()

// Returns if we're connected to the rocket
bool isRFMConnected() {
  return (millis() - rfmLastRFReceived < RFM_CONNECTED_TIMEOUT);
}//isRFMConnected()


void rfmInit() {
  // Initialize RFM95 with default settings
  setRGB(2, 255,0,0, true); //set RF light to RED
  Serial.print(F("RFM95: Initializing ... "));
  radio.reset();
  delay(100);
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) {
      Serial.println("Fail");
      delay(1000);
    }//while
  }//if

  //radio.forceLDRO(true);

  // set the function that will be called
  // when new packet is received
  radio.setDio0Action(setFlag, RISING);

  double baseFreq = freqOpts[freqSelected];
  double freqOffset = (freqCorrectionOpts[freqCorrectionSelected]/1000000.0); //Hz -> MHz
  freqOffset = (curFreqOffset/1000000.0); //Hz -> MHz
  if (radio.setFrequency(baseFreq+freqOffset) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Frequency is invalid!"));
    while (true);
  }//if
  
  if (radio.setBandwidth(bandwidthOpts[bandwidthSelected]) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Bandwidth is invalid!"));
    while (true);
  }//if

  // set spreading factor to 10
  if (radio.setSpreadingFactor(spreadOpts[spreadSelected]) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Spreading factor is invalid!"));
    while (true);
  }//if

  // set coding rate to 6
  if (radio.setCodingRate(codingOpts[codingSelected]) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Coding rate is invalid!"));
    while (true);
  }//if

  // Start listening (async)
  radio.startReceive();

  Serial.println(F("RFM95 configuration success!"));
}//rfmInit()


/*
 * Initializes the NAU7802 ADC
 */
void initNAU7802() {

  if (!nau.begin(Wire, true)) {
      Serial.println("[ERROR] NAU7802 initialization error");
      while(true);
  }//if
  
  // Ensure correct settings are being used
  nau.setChannel(NAU7802_CHANNEL_1);
  nau.setPGACapEnable(false);
  nau.setBypassPGA(true);
  nau.setLDO(NAU7802_LDO_3V3); //so ref=3.3V
  nau.setSampleRate(NAU7802_SPS_40); //TODO: Does setting this to max speed up measurements? And does it greatly affect accuracy?

  // Calibrate Analog Front End (AFE) after major changes
  nau.calibrateAFE();

  Serial.println("NAU7802 setup & calibration complete");

  //TODO: Flush out readings (10x for loop that reads?)

}//initNAU7802()






void setup() {
  // Configure pinmodes
  pinMode(DISABLE_5V_PIN, OUTPUT);
  pinMode(DB_LED_PIN, OUTPUT);
  pinMode(GPS_RESET_PIN, OUTPUT);
  digitalWrite(GPS_RESET_PIN, HIGH); //TODO: NESSESARY??
  pinMode(MENU_BTNS_PIN, INPUT);
  pinMode(CHRG_STAT_PIN, INPUT);
  analogReadResolution(12);

  // Ensure USB mode is PWR+DATA by default
  setUSBDataOnlyMode(false);

  // Add WS2812B
  rgbLEDs.begin(); // initialize WS2812Bs
  setRGB(0,0,0);
  setLEDBrightness(DEFAULT_LED_BRIGHTNESS);
  // Set power LED to "loading"
  setRGB(0,128,32,0);

  // Start USB Serial
  Serial.begin(USB_BAUD);

  // Prepare SPI busses
  epdSPI.begin(EINK_SCK_PIN, EINK_MISO_PIN, EINK_MOSI_PIN, EINK_CS_PIN);
  rfmSPI.begin(RF_SCK_PIN, RF_MISO_PIN, RF_MOSI_PIN, RF_CS_PIN);
  pinMode(epdSPI.pinSS(), OUTPUT);
  pinMode(rfmSPI.pinSS(), OUTPUT);

  // Initialize E-Paper Display (EPD)
  display.init(EPD_BAUD, true, 2, false, epdSPI, SPISettings(EPD_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  display.setRotation(1);

  display.setTextColor(GxEPD_BLACK);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextSize(1);
    display.println("QRET Kuhglocke    FW=" + FIRMWARE_VERSION);
    display.setCursor(30,20);
    display.setTextSize(7);
    display.println("QRET");
    display.setTextSize(2);
    display.println("     Loading");

    display.setTextSize(1);
    display.setCursor(0,112);
    display.println("POWER  BATT    RADIO   SRADOK   GPS FIX");
    
  } while (display.nextPage());

  // Configure I2C Bus
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_SPEED);

  // Initialize MicroSD card (1-bit mode)
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[WARN] MicroSD Card Mount Failed");
    //while(true);
  } else {
    Serial.println("SD: Card Mount Success");
    makeNextSDLog();
    writeToSDLog("Kuhglocke SD card initialized");
  }//if (SD OK)

  // Initialize NAU7802 ADC
  initNAU7802();

  // Configure GPS UART1 Bus
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 

  // Initialize RFM95 Radio
  rfmInit();

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }//if

  // Launch WiFi AP
  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
  WiFi.setTxPower(WIFI_TX_POWER);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("WiFi AP IP address: ");
  Serial.println(IP);

  // Configure Async web server
  configWebServer();
  webServer.begin();

  
  // Launch background task for Core 0
  xTaskCreatePinnedToCore(
      loopAltCoreHandler,   // Task function
      "Core0Loop",          // Name of the task
      ALT_CORE_STACKS_SIZE, // Stack size
      NULL,                 // Task input parameter
      1,                    // Priority of the task
      &Core0Task,           // Task handle
      0                     // Core number
  );
  

  // Set power LED to "ready"
  setRGB(0,0,128,0); //Turn on power LED
  Serial.println("Kuhglocke Initialization Complete!");
  
}//setup()


void gpsDisplayInfoGrand() {
if (gps.location.isUpdated())
  {
    Serial.print("LOCATION   Fix Age=");
    Serial.print(gps.location.age());
    Serial.print("ms Raw Lat=");
    Serial.print(gps.location.rawLat().negative ? "-" : "+");
    Serial.print(gps.location.rawLat().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLat().billionths);
    Serial.print(" billionths],  Raw Long=");
    Serial.print(gps.location.rawLng().negative ? "-" : "+");
    Serial.print(gps.location.rawLng().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLng().billionths);
    Serial.print(" billionths],  Lat=");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Long=");
    Serial.println(gps.location.lng(), 6);
  }

  else if (gps.date.isUpdated())
  {
    Serial.print("DATE       Fix Age=");
    Serial.print(gps.date.age());
    Serial.print("ms Raw=");
    Serial.print(gps.date.value());
    Serial.print(" Year=");
    Serial.print(gps.date.year());
    Serial.print(" Month=");
    Serial.print(gps.date.month());
    Serial.print(" Day=");
    Serial.println(gps.date.day());
  }

  else if (gps.time.isUpdated())
  {
    Serial.print("TIME       Fix Age=");
    Serial.print(gps.time.age());
    Serial.print("ms Raw=");
    Serial.print(gps.time.value());
    Serial.print(" Hour=");
    Serial.print(gps.time.hour());
    Serial.print(" Minute=");
    Serial.print(gps.time.minute());
    Serial.print(" Second=");
    Serial.print(gps.time.second());
    Serial.print(" Hundredths=");
    Serial.println(gps.time.centisecond());
  }

  else if (gps.speed.isUpdated())
  {
    Serial.print("SPEED      Fix Age=");
    Serial.print(gps.speed.age());
    Serial.print("ms Raw=");
    Serial.print(gps.speed.value());
    Serial.print(" Knots=");
    Serial.print(gps.speed.knots());
    Serial.print(" MPH=");
    Serial.print(gps.speed.mph());
    Serial.print(" m/s=");
    Serial.print(gps.speed.mps());
    Serial.print(" km/h=");
    Serial.println(gps.speed.kmph());
  }

  else if (gps.course.isUpdated())
  {
    Serial.print("COURSE     Fix Age=");
    Serial.print(gps.course.age());
    Serial.print("ms Raw=");
    Serial.print(gps.course.value());
    Serial.print(" Deg=");
    Serial.println(gps.course.deg());
  }

  else if (gps.altitude.isUpdated())
  {
    Serial.print("ALTITUDE   Fix Age=");
    Serial.print(gps.altitude.age());
    Serial.print("ms Raw=");
    Serial.print(gps.altitude.value());
    Serial.print(" Meters=");
    Serial.print(gps.altitude.meters());
    Serial.print(" Miles=");
    Serial.print(gps.altitude.miles());
    Serial.print(" KM=");
    Serial.print(gps.altitude.kilometers());
    Serial.print(" Feet=");
    Serial.println(gps.altitude.feet());
  }

  else if (gps.satellites.isUpdated())
  {
    Serial.print("SATELLITES Fix Age=");
    Serial.print(gps.satellites.age());
    Serial.print("ms Value=");
    Serial.println(gps.satellites.value());
  }

  else if (gps.hdop.isUpdated())
  {
    Serial.print("HDOP       Fix Age=");
    Serial.print(gps.hdop.age());
    Serial.print("ms raw=");
    Serial.print(gps.hdop.value());
    Serial.print(" hdop=");
    Serial.println(gps.hdop.hdop());
  }

}//gpsDisplayInfoGrand()

/*
 * Handles receiving and processing GPS information.
 * To be run in loop()
 * 
 * TODO: Log local GPS data to SD too
 */
void handleGPS() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      //gpsDisplayInfoGrand();
    }//if
  }//while


  if (millis() - lastLocalGPSLog > LOCAL_GPS_LOG_RATE) {
    lastLocalGPSLog = millis();
    String resp = String("LocalGPS: ");
    resp += String(gps.satellites.age());
    resp += "," + String(gps.satellites.value());
    resp += "," + String(gps.location.lat(), 6);
    resp += "," + String(gps.location.lng(), 6);
    writeToSDLog(resp);
  }//if

  if (millis() > 50000 && gps.charsProcessed() < 10)
  {
    Serial.println("[ERROR] No response from onboard GPS");
  }//if
}//handleGPS()


/*
 * Handles updating indicator LEDs.
 * To be run in loop.
 */
void handleLEDs() {
  // Power LED remains constant

  // Battery/Charging indicator
  //0 = charging, otherwise not
  uint8_t chrgStatus = getChargingStatus();
  uint16_t battVolt = getBatteryVoltage();
  if (battVolt < 3500) {
    if (chrgStatus == 1) {
      //Full & Off
      setRGB(1, 0,255,0, false);
    } else {
      //Low
      setRGB(1, 255,0,0, false);
    }//if (chrgd)
  } else if (battVolt < 3700) {
    setRGB(1, 255,80,0, false);
  } else if (battVolt < 3900) {
    setRGB(1, 255,220,0, false);
  } else {
    setRGB(1, 0,255,0, false);
  }//if (batt volt to colour)

  if (chrgStatus == 0) {
    //Charging; flash LED
    if (indBattFlashState) {
      setRGB(1, 0,0,0, true);
    }//if (turn off)
    if (millis() - indBattLastToggle > IND_BATTERY_FLASH_GAP) {
      indBattLastToggle = millis();
      indBattFlashState = !indBattFlashState;
    }//if (flash time)
  }//if (charging)


  // RF Indicator
    // Default OFF, Green if connected
    // Flash Blue every received ping
  if (isRFMConnected()) {
    if (millis() - indRFFlashStart < IND_RF_FLASH_TIME) {
      setRGB(2, 255,255,255); // Ping flash
    } else {
      setRGB(2, 0,255,0); // Default
    }//if (flash RF led)
  } else {
    setRGB(2, 0,0,0, true);
  }//if

  // ONBOARD GPS INDICATOR TESTING (TODO: REMOVE)
  //if (gps.satellites.value() > 3) {
  //  setRGB(3, 0,128,128); //GPS FIX ACHIEVED
  //} else {
  //  setRGB(3, 32,0,0);
  //}

  // SRAD OKAY LIGHT
  if (isRFMConnected()) {
    if (rocketStatus == 0b111) {
      setRGB(3, 255,255,255);
    } else {
      setRGB(3, 0,0,0);
    }//if    
  } else {
      setRGB(3, 0,0,0);
  }//if
  

  // Rocket GPS fix
  if (isRFMConnected()) {
    if (rocketGPSSats > 3) {
      setRGB(4, 0,255,0, true);
    } else {
      setRGB(4, 0,0,0, true);
    }//if    
  } else {
      setRGB(4, 0,0,0, true);
    }//if
  
}//handleLEDs()



uint16_t esimtateVREF(uint8_t adcPin, uint8_t samples) {
  /*
   * Estimates the VREF voltage with no extra hardware.
   * 
   * Given an ADC pin that has a VOLTAGE BELOW VREF (3.3V)
   * and ABOVE ZERO, attempts to back-calculate the VREF
   * voltage using analogRead, analogReadMilliVolts, and a
   * correction function (accounts for inaccuracies in
   * analogReadMillivolts & the non-linearity of the ADC).
   * Blocking function. Assumes ADC resolution is 12 bits.
   * 
   * Calibration will vary between ESP32 modules.
   * 
   * To calibrate this function, measure various analog voltages
   * using both analogReadMilliVolts() and an external multimeter,
   * recording them in a table. Calculate (dmmVolt/adcVolt) for each
   * entry to get a correction factor. Plotting these factors against 
   * the adcReadMilliVolts values should produce a strong 3rd-order
   * polynomial trend. Get an equation for this curve and evaluate it
   * for N steps over the 12-bit range (e.g. 0,409,819,...,3686,4095).
   * Fill these values into the if-statemtn in calibration stage.
   */
  // Get an average reading from the ADC pin
  uint32_t mvAvg = 0;
  uint32_t countAvg = 0;
  for (uint8_t i=0; i<samples; i++) {
      mvAvg += analogReadMilliVolts(adcPin);
      countAvg += analogRead(adcPin);
  }//for
  mvAvg /= samples;
  countAvg /= samples;
  
  // Ensure ADC pin is 0>x>3.3
  if (countAvg > 4090 or countAvg < 5) {
    return 0;
  }//if (not enough data)
  
  // Back-calculate VREF
  uint16_t backVolt = mvAvg/(countAvg/4095.0);

  // Apply calibrated correction factor
  if (mvAvg < 128) {backVolt *= 0.884;}
  else if (mvAvg < 256) {backVolt *= 0.902;}
  else if (mvAvg < 384) {backVolt *= 0.917;}
  else if (mvAvg < 512) {backVolt *= 0.930;}
  else if (mvAvg < 768) {backVolt *= 0.940;}
  else if (mvAvg < 1024) {backVolt *= 0.956;}
  else if (mvAvg < 1280) {backVolt *= 0.966;}
  else if (mvAvg < 1536) {backVolt *= 0.972;}
  else if (mvAvg < 1792) {backVolt *= 0.976;}
  else if (mvAvg < 2048) {backVolt *= 0.980;}
  else if (mvAvg < 2303) {backVolt *= 0.987;}
  else if (mvAvg < 2559) {backVolt *= 0.999;}
  else if (mvAvg < 2815) {backVolt *= 1.016;}
  else if (mvAvg < 3071) {backVolt *= 1.042;}
  else if (mvAvg < 3327) {backVolt *= 1.078;}
  else if (mvAvg < 3583) {backVolt *= 1.127;}
  else if (mvAvg < 3839) {backVolt *= 1.189;}
  else {backVolt *= 1.269;}

  return backVolt;
}//esimtateVREF()

// Converts degrees to radians
double degToRad(double degs) {
    return degs * (M_PI / 180.0);
}//degToRad()

/*
 * Using the onboard GPS & latest rocket GPS coordinates,
 * returns a distance (meters) to the rocket. Expensive
 * calculation, so use sparingly.
 * 
 * Returns -1 if could not eval distance.
 */
int32_t getDistanceToRocket() {
  // Ensure we have enough data
  if (gps.location.lat() == 0 || gps.location.lng() == 0 || gps.location.age() > 5000) {
    return -1;
  }//if
  if (rocketGPSLat == 0 or rocketGPSLon == 0) {
    return -1;
  }//if

  // Convert latitude and longitude from degrees to radians
  double lat1Rad = degToRad(gps.location.lat());
  double lon1Rad = degToRad(gps.location.lng());
  double lat2Rad = degToRad(rocketGPSLat/1000000.0);
  double lon2Rad = degToRad(rocketGPSLon/1000000.0);
  
  // Calculate the differences between the points
  double dLat = lat2Rad - lat1Rad;
  double dLon = lon2Rad - lon1Rad;
  
  // Apply the Haversine formula
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1Rad) * cos(lat2Rad) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  // Calculate the distance in feet
  double distance = EARTH_RADIUS_FEET * c;
  
  return distance/3.281;
  
}//getDistanceToRocket()


/*
 * Handles reading various power sensors,
 * such as battery voltage, system current,
 * charge status, and USB status.
 * To be run in loop.
 * 
 * TODO: Optimize by reading other sensors
 * while NAU7802 isn't available
 */
void handleReadSensors() {

  if (millis() - lastSensorRead > SENSOR_SAMPLE_PERIOD) {
    lastSensorRead = millis();

    // Read NAU channel & then switch to other
    // (CH1 & CH2 get read in alternating cycles)
    // (gives time for MUX to stabilize)
    while (!nau.available()) {}
    int32_t nauRaw = nau.getReading();
    if (nauRaw < 0) {nauRaw = 0;}
    if (nauChannel == 0) {
      // Just read battery; switch to Current
      nauChannel = 1;
      nau.setChannel(NAU7802_CHANNEL_2);
      battVoltFilter.AddValue(nauRaw);
    } else {
      // Just read current; switch to battery
      nauChannel = 0;
      nau.setChannel(NAU7802_CHANNEL_1);
      sysCurrentFilter.AddValue(nauRaw);
    }//if

    // Attempt to read VCC
    vccVoltFilter.AddValue(3300); //Assume 3.3V
    /*
    uint16_t estVREF = esimtateVREF(CHRG_STAT_PIN, 5);
    if (estVREF == 0) {
      estVREF = esimtateVREF(MENU_BTNS_PIN, 5);
    }//if
    if (estVREF > 0) {
      vccVoltFilter.AddValue(estVREF);
    } else {
      vccVoltFilter.AddValue(3300); //Assume 3.3V if can't calibrate
    }//if
    */

    ambTempFilter.AddValue(rawReadTempSensor());

    //read core voltage
    //read temp sensor
    //read nau7802
    //parse data & load into global vars
    //parse CHRG status & make into integer?
    //parse USB status & make into integer?
    
  }//if (sample)
}//handleReadSensors


// Triggers the RF indicator LED to flash (async)
// To be called whenever an RF ping is received
void indRFDoFlash() {
  indRFFlashStart = millis();
}//indRFDoFlash


/*
 * Quick method to write data to the EPD using a partial update.
 * 
 * This is only meant to update VALUES, not an entire screen. If
 * you're changing menus, you need to do a full refresh and change
 * the global menu variable.
 * 
 * TAKES ABOUT 500ms (BLOCKING)
 */
void updateEPD() {
  uint16_t x = 0; // X coordinate of the update area
  uint16_t y = 15; // Y coordinate of the update area
  uint16_t w = 250; // Width of the update area
  uint16_t h = 91; // Height of the update area
  display.setPartialWindow(x, y, w, h);
  display.firstPage();
  do
  {
    display.setTextSize(2);
    display.fillScreen(GxEPD_WHITE); // Clear the partial window
    
    
    display.setCursor(x, y);
    display.print("LAT:");
    display.println(rocketGPSLat/1000000.0, 6);
    //display.print(gps.location.lat(), 6);
    //display.setCursor(x, y + 18);
    display.print("LON:");
    display.println(rocketGPSLon/1000000.0, 6);
    //display.print(gps.location.lng(), 6);

    //display.setCursor(x, y + 16*2);
    display.print("ALT:");
    //display.print(gps.altitude.feet(), 0);
    display.print(rocketAltitude, 0);
    display.println("ft");

    //display.setCursor(x, y + 16*3);
    display.print("AGE:");
    display.print((millis()-rfmLastRFReceived)/1000.0, 1);
    display.println("s");

    display.setCursor(x+160, y + 16*3);
    //display.print("D=");
    int32_t dist = getDistanceToRocket();
    if (dist > 0) {
      display.print(dist);
    } else {
      display.print("????");
    }//if
    display.print("m");

    display.setCursor(x+160, y + 16*2);
    //display.print("Q=");
    display.print(rfmLastSNR,1);
    display.print("dB");

    //Update display at bottom of screen
    display.setTextSize(1);
    bool localGPS = (gps.location.age() < 3000);
    display.setCursor(x+5,98);
    if (localGPS) {
      display.print("+GPS");
    }
    uint16_t battVolt = getBatteryVoltage();
    display.setCursor(x+42,98);
    display.print(voltToPercent(battVolt));
    display.print("%");
    display.setCursor(x+85,98);
    display.print(rfmLastSNR, 1);
    display.print("dB");
    display.setCursor(x+150,98);
    display.print(rocketStatus, BIN);
    display.setCursor(x+210,98);
    display.print(rocketGPSSats);

  }
  while (display.nextPage());
}//updateEPD()


void printTimeToEPD() {
  // Quick method that uses partial refresh to write millis() to screen. Takes about 500ms (blocking)
  uint16_t x = 0; // X coordinate of the update area
  uint16_t y = 20; // Y coordinate of the update area
  uint16_t w = 100; // Width of the update area
  uint16_t h = 30; // Height of the update area
  display.setPartialWindow(x, y, w, h);
  display.firstPage();
  do
  {
    display.setTextSize(1);
    display.fillScreen(GxEPD_WHITE); // Clear the partial window
    display.setCursor(x, y + 20); // Set cursor position within the partial window
    display.print(millis());
  }
  while (display.nextPage());
}//printTimeToEPD()


/*
 * Function called when a packet is received
 * by the RFM95W radio
 * 
 * TODO: Add writeToSDLog(""); logging to this function!!
 */
void onRFMReceive() {
  // Flash ping indicator
  indRFDoFlash();

  // Read data
  byte rfmPayload[RFM_PACKET_SIZE];
  int state = radio.readData(rfmPayload, RFM_PACKET_SIZE);

  // Clone data to global array
  for (uint8_t i=0; i<RFM_PACKET_SIZE; i++) {
    rfmLastPacket[i] = rfmPayload[i];
  }//for

  //byteArrayToHexString(rfmPayload, RFM_PACKET_SIZE)
  //writeToSDLog("")

  // Print debug
  Serial.print("RFM95W Packet: ");
  for (int i=0; i<RFM_PACKET_SIZE; i++) {
    Serial.print(rfmPayload[i],HEX);
  }//for
  Serial.println();
  Serial.print("RFM95W Stats: RSSI=");
  Serial.print(radio.getRSSI());
  Serial.print("dBm, SNR=");
  Serial.print(radio.getSNR());
  Serial.print("dB, FreqErr=");
  Serial.print(radio.getFrequencyError());
  Serial.println("Hz");

  // Ensure packet valid
  if (state != RADIOLIB_ERR_NONE) {
    rfmLastPacketValid = false;
    Serial.print("[WARN] RFM95W Packet error: ");
    if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      Serial.println("RX Timeout");
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("CRC Error");
    } else {
      Serial.print("Unknown, code");
      Serial.println(state);
    }//if (failure mode)
    return;
  }//if (invalid packet)

  // Process packet data
  rfmLastPacketValid = true;
  rocketGPSLat = (rfmPayload[3]<<24) + (rfmPayload[2]<<16) + (rfmPayload[1]<<8) + rfmPayload[0];
  rocketGPSLon = (rfmPayload[7]<<24) + (rfmPayload[6]<<16) + (rfmPayload[5]<<8) + rfmPayload[4];
  rocketGPSSats = rfmPayload[8];
  rocketAltitude = (rfmPayload[12]<<24) + (rfmPayload[11]<<16) + (rfmPayload[10]<<8) + rfmPayload[9];
  rocketStatus = rfmPayload[13];

  // Record stats
  rfmLastRFReceived = millis();
  rfmLastRSSI = radio.getRSSI();
  rfmLastSNR = radio.getSNR();
  rfmLastFreqErr = radio.getFrequencyError();

  // Log to SD card
  String resp = String("RocketPacket: ");
  resp += String(millis()-rfmLastRFReceived);
  resp += "," + String(byteArrayToHexString(rfmLastPacket, RFM_PACKET_SIZE));
  resp += "," + String(rfmLastRSSI);
  resp += "," + String(rfmLastSNR);
  resp += "," + String(rfmLastFreqErr);
  resp += "," + ((rfmLastPacketValid) ? String("Yes") : String("NO"));
  writeToSDLog(resp);

  String resp2 = String("RocketData: ");
  resp2 += String(rocketGPSSats);
  resp2 += "," + String(rocketGPSLat/1000000.0, 6);
  resp2 += "," + String(rocketGPSLon/1000000.0, 6);
  resp2 += "," + String(rocketAltitude);
  resp2 += "," + String(rocketStatus, BIN);
  writeToSDLog(resp2);
 
}//onRFMReceive()


/*
 * Code to run repeatedly
 */
void loop() {
  // Note loop start time
  uint32_t loopStart = millis();
  
  //setRGB(4, 64,0,0);
  //delay(50);
  //setRGB(4, 0,0,0);
  //delay(1000);

  
  handleGPS();
  handleReadSensors();
  handleLEDs();
  //Serial.print("BTNS=");
  //Serial.println(analogRead(MENU_BTNS_PIN));
  if (analogRead(MENU_BTNS_PIN) > 40) {
    setLEDBrightness(255);
    //setRGB(4, 255,255,255);
    //writeToSDLog("This is an example log file line.");
  } else {
    setLEDBrightness(DEFAULT_LED_BRIGHTNESS);
    //setRGB(4, 0,0,0);
  }

  // Check for incoming RFM95 packets
  if (rfmReceivedFlag) {
    // Reset flag
    rfmReceivedFlag = false;
    onRFMReceive();
  }//if (rfm recv)

  // Note loop execution time
  uint16_t core1LoopTime = (millis()-loopStart);
  core1LoopFilter.AddValue(core1LoopTime);
  if (core1LoopTime > maxCore1LoopTime) {maxCore1LoopTime = core1LoopTime;}

  if (millis() - lastEPDUpdate > EPD_UPDATE_INT) {
    lastEPDUpdate = millis();
    //updateEPD();
  }

}//loop()


/*
 * Run as a task on Core0; runs the alternate loop
 * function indefinitely.
 */
void loopAltCoreHandler(void * pvParameters) {
  Serial.println("Starting background thread on Core0");
  while (true) {
    loopAltCore();
  }//while
}//loopAltCoreHandler()


/*
 * Like loop(), but runs on Core0 of the ESP32
 * 
 * Make sure you use a MUTEX when exchanging
 * lots of info with Core1.
 * 
 * NOTE: STACK SIZE IS LIMITED HERE.
 */
void loopAltCore() {
  // Note loop start time
  uint32_t loopStart = millis();
  
//  Serial.print("AltCore Loop: ");
//  Serial.println(core0FreeStack);
//  delay(100);

  // Check to update EPD
  if (millis() - lastEPDUpdate > EPD_UPDATE_INT) {
    lastEPDUpdate = millis();
    updateEPD();
  }//if

  //for reading sensors or updating the EPD, and maybe playing audio

  // Note free stack space
  core0FreeStack = uxTaskGetStackHighWaterMark(NULL);
  // Note loop execution time
  core0LoopTime = (millis()-loopStart);
  if (core0LoopTime > maxCore0LoopTime) {maxCore0LoopTime = core0LoopTime;}
}//loopAltCore()
