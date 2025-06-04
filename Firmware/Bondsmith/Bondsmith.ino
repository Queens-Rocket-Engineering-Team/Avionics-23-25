/*
 * Authors: Kennan Bays
 * Created: Jun.3.2025
 * Updated: Jun.4.2025
 * Hardware: Bondsmith PCB Rev1.0 (STM32F103C8T6)
 * Environment: Arduino 1.8.10, STM32duino 2.7? (likely works with newer cores)
 * Purpose: Firmware for the Bondsmith Motherboard inside the QRET 2025 briefcase ground station.
 *    Firmware facilitates measuring rail voltages, battery voltages, current consumption, fan RPM,
 *    button states, lid open/close state, and PSU temperature, reporting them over Serial. Also can
 *    be send commands to change which battery is being used, change fan speed, toggle 12V peripheral
 *    pins (VRX, Aux, RGB) and change the colours of the RGB indicator LEDs.
 *
 * Upload Settings:
 * - Board Part Number: STM32F103C8Tx [!!!]
 * - USART Support: Enabled (Generic serial)
 * - USB Support: None
 * - Optimize: Smallest
 * - Upload Method: STM32CubeProgrammer
 * 
 * KNOWN HARDWARE PROBLEMS:
 * - The resistor dividers for the 5V and 12V rail monitoring need a bypass capacitor as the divider is too high impedance. Workaround was longer ADC sampling time.
 * - The "BATT1_EN" and "BATT2_EN" signals exposed for LED indicators is inverted (HIGH disables a battery), thus the LEDs are inverted. Swap the GND pin to 3.3V to fix.
  */


#include "pinouts.h"
#include <HardwareSerial.h>
#include <movingAvg.h>          // https://github.com/JChristensen/movingAvg
//TODO: Include STM32duino low power library to improve power consumption


// # # # # DEFINITIONS # # # # 
//#define FW_SIGNATURE "Jun.4.2025" // Firmware signature string. Update after major changes; allows end-user to check firmware version.
#define SERIAL_BAUD_RATE 115200

// # # # # CONSTANTS # # # #
// Internal STM32 V(ref) voltage (default 1200mV)
const uint16_t STM_INT_REF = 1206; //CAL'D (experimental)
// Voltage/Current sense resistor divider ratios (experimentally measure for better accuracy)
const float BATT1_SENSE_DIV = 8.5812; //CAL'D (15.12, 1.762)
const float BATT2_SENSE_DIV = 8.5231;  //CAL'D (15.12, 1.774)
const float RAIL5_SENSE_DIV = 1.9996; //CAL'D (5.091, 2.546)
const float RAIL12_SENSE_DIV = 4.6286;  //CAL'D (12.09, 2.612)
// Temperature sensor divider resistor (measure for better accuracy)
const uint32_t TEMP_SENSE_R1_OHMS = 10030; //CAL'D
const uint16_t SENSOR_MEASURE_INT = 50; // period (ms) for sensor measurements
const uint16_t REPORT_INTERVAL = 1000; // period (ms) for printing reports to rasppi
// Cooling fan management curve
const uint8_t FAN_HYSTERESIS = 3; //Hysteresis (deg C) for when to turn the fan down again after hitting a threshold
const uint8_t FAN_OFF_TO_LOW = 35;  // Deg C when the fan will kick on to low speed
const uint8_t FAN_LOW_TO_HIGH = 45; // Deg C when the fan will kick on to high speed
const uint8_t FAN_HIGH_TO_LOW = FAN_LOW_TO_HIGH-FAN_HYSTERESIS;
const uint8_t FAN_LOW_TO_OFF = FAN_OFF_TO_LOW-FAN_HYSTERESIS;
const uint8_t FAN_LOW_POWER = 190;  // Low fan power
const uint8_t FAN_HIGH_POWER = 255; // High fan power


// # # # # GLOBAL VARIABLES # # # #
uint32_t lastMeasureMillis = 0; // last millis() when measurements were taken
uint32_t lastReportMillis = 0; // last millis() when a pi report was made

uint8_t curFanPower = 0; // Current fan power (0-255)
volatile uint32_t fanTachPulses = 0; // Increments as fan tachometer pulses

// Most recent filtered sensor measurements
uint16_t lastRail3v3Volt = 0; //mV
uint16_t lastRail5Volt = 0; //mV
uint16_t lastRail12Volt = 0; //mV
uint32_t lastBatt1Volt = 0; //mV
uint32_t lastBatt2Volt = 0; //mV
uint16_t lastCurrent = 0; //mA
float lastPsuTemp = 0; //deg C
bool lastB1Down = false;
bool lastB2Down = false;
bool lastB3Down = false;
bool lastLidClosed = false;
uint16_t lastFanRPM = 0;


// # # # # GLOBAL OBJECTS # # # #
HardwareSerial pi(PI_RX_PIN, PI_TX_PIN);

movingAvg rail3v3RawFilter(20);
movingAvg rail5RawFilter(20);
movingAvg rail12RawFilter(20);
movingAvg batt1RawFilter(20);
movingAvg batt2RawFilter(20);
movingAvg currentRawFilter(20);


/*
 * Reads internal voltage reference and back-calculates
 * the system Vdd (VDDA) using a calibrated Vref consant.
 * Returns the average across several samples (MAX ~32k
 * samples supported). Probably works with many STM32F1xx
 * chips.
 * 
 * Inspired by the STM32duino Internal_channels example
 */
uint16_t measureVref(uint16_t samples) {
  // Take samples
  uint16_t result;
  uint32_t accum = 0;
  for (int i = 0; i < samples; i++) {
    result = analogRead(AVREF);
    accum += ((STM_INT_REF * 4096L) / result); // Back-calculate VDDA in mV
  }//for
  return accum/samples; //Average
}//measureVref(int)


// Called when fan tach pin pulses
void onFanTach() {
  fanTachPulses++;
}//onFanTach()


void setup() {
  // Configure pinmodes
  pinMode(BATT1_SENSE_PIN, INPUT);
  pinMode(BATT2_SENSE_PIN, INPUT);
  pinMode(CURR_SENSE_PIN, INPUT);
  pinMode(RAIL_5V_SENSE_PIN, INPUT);
  pinMode(PSU_TEMP_PIN, INPUT);
  pinMode(RAIL_12V_SENSE_PIN, INPUT);
  pinMode(CURR_SENSOR_PWR_PIN, OUTPUT);
  pinMode(BATT1_DISABLE_PIN, OUTPUT);
  pinMode(BATT2_DISABLE_PIN, OUTPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(FAN_TACH_PIN, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(RGB12_B_PIN, OUTPUT);
  pinMode(RGB12_G_PIN, OUTPUT);
  pinMode(RGB12_R_PIN, OUTPUT);
  pinMode(LID_SENSE_PIN, INPUT);
  pinMode(BUTT3_SENSE_PIN, INPUT);
  pinMode(BUTT2_SENSE_PIN, INPUT);
  pinMode(BUTT1_SENSE_PIN, INPUT);
  pinMode(AUX_PWR_ENABLE_PIN, OUTPUT);
  pinMode(RGB_DATA_PIN, OUTPUT);
  pinMode(VRX_PWR_ENABLE_PIN, OUTPUT);

  // Immediately switch to the higher-voltage battery (TODO: IMPLEMENT)
  digitalWrite(BATT1_DISABLE_PIN, LOW);
  digitalWrite(BATT2_DISABLE_PIN, LOW);

  // Config fan tach interrupt
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onFanTach, FALLING);

  // Turn on VRX and AUX ports
  digitalWrite(AUX_PWR_ENABLE_PIN, HIGH);
  digitalWrite(VRX_PWR_ENABLE_PIN, HIGH);

  // Turn fan on 100%
  setFanPower(128);

  // Turn on current sensor (TODO: Disable this for power saving in the future)
  digitalWrite(CURR_SENSOR_PWR_PIN, HIGH);
  
  // Flash debug LED to signal startup
  for (uint8_t i=0; i<8; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100); 
  }//for

  // Start Serial comms
  pi.begin(SERIAL_BAUD_RATE);
  pi.println("Bondsmith starting...");

  // Configure ADC
  analogReadResolution(12); //12-bit

  // Initialize moving average filters
  rail3v3RawFilter.begin();
  rail5RawFilter.begin();
  rail12RawFilter.begin();
  batt1RawFilter.begin();
  batt2RawFilter.begin();
  currentRawFilter.begin();

}//setup()

// Quick function to average multiple samples of an analog pin
uint32_t adcMultiSample(uint16_t pin, uint8_t qnt) {
  uint32_t sum = 0;
  for (uint8_t i=0; i<qnt; i++) {
    sum += analogRead(pin);
    delay(1);
  }//for()
  return sum/qnt;
}//adcMultiSample()


const uint16_t FAN_TACH_MEASURE_PERIOD = 250; // How often (ms) to compute fan RPM
uint32_t lastFanTachMeasureMillis = 0; //The last millis() when the fan tach was resolved


/*
 * Samples various onboard sensors
 */
void sampleSensors() {

  // Check fan tach
  if (millis() - lastFanTachMeasureMillis > FAN_TACH_MEASURE_PERIOD) {
    lastFanTachMeasureMillis = millis();
    // Cache current num pulses & reset (pause interrupts due to non-atomic operation on volatile)
    noInterrupts();
    uint32_t pulses = fanTachPulses;
    fanTachPulses = 0;
    interrupts();
    //pi.print("Pulses: ");
    //pi.println(pulses);
    // Extrapolate missed pulses based on duty cycle
    if (curFanPower!=0 && curFanPower!=255) {
      pulses = pulses / (curFanPower/255.0f);
    }
    //pi.print("Extrp Pulses: ");
    //pi.println(pulses);
    // Convert pulses to RPM
    pulses /= 2; //2 pulse per rotation
    lastFanRPM = (pulses / (FAN_TACH_MEASURE_PERIOD/1000.0f)); //Rev/sec
    lastFanRPM *= 60; //Rev/min
  }//if (measure fan tach)

  // Read 3.3V rail voltage (ADC VRef)
  uint16_t rail3v3 = rail3v3RawFilter.reading(measureVref(8));
  // Query analog sensors
  uint16_t batt1Raw = batt1RawFilter.reading(analogRead(BATT1_SENSE_PIN));
  uint16_t batt2Raw = batt2RawFilter.reading(analogRead(BATT2_SENSE_PIN));
  uint16_t currRaw = currentRawFilter.reading(analogRead(CURR_SENSE_PIN));
  uint16_t rail5Raw = rail5RawFilter.reading(analogRead(RAIL_5V_SENSE_PIN));
  uint16_t rail12Raw = rail12RawFilter.reading(analogRead(RAIL_12V_SENSE_PIN));
  uint16_t psuTempRaw = analogRead(PSU_TEMP_PIN);
  // Query lid sensor & buttons
  bool lidClosed = digitalRead(LID_SENSE_PIN)==LOW;
  bool b1Down = digitalRead(BUTT1_SENSE_PIN)==LOW;
  bool b2Down = digitalRead(BUTT2_SENSE_PIN)==LOW;
  bool b3Down = digitalRead(BUTT3_SENSE_PIN)==LOW;
  
  // Calculate voltages
  // TODO: Check if voltage calculation is being done right
  uint32_t batt1 = rail3v3*batt1Raw*BATT1_SENSE_DIV/4095;
  uint32_t batt2 = rail3v3*batt2Raw*BATT2_SENSE_DIV/4095;
  uint32_t rail5 = rail3v3*rail5Raw*RAIL5_SENSE_DIV/4095;
  uint32_t rail12 = rail3v3*rail12Raw*RAIL12_SENSE_DIV/4095;

  // Calculate current draw
  uint16_t currSenseVoltage = rail3v3*currRaw/4095;
  uint16_t current = (currSenseVoltage-1650)/0.132f;
  
  // Calculate PSU temperature (via Beta Equation, R0=10k, B25/85=3435K, T0=25C)
  uint16_t tempSenseVoltage = rail3v3*psuTempRaw/4095;
  uint32_t rTherm = (tempSenseVoltage * TEMP_SENSE_R1_OHMS) / (rail3v3-tempSenseVoltage);
  float psuTemp = 1.0/((1.0/(25+273.15)) + (1.0/3435.0) * log(rTherm / 10000.0f));
  psuTemp -= 273.15;

  lastRail3v3Volt = rail3v3;
  lastRail5Volt = rail5;
  lastRail12Volt = rail12;
  lastBatt1Volt = batt1;
  lastBatt2Volt = batt2;
  lastCurrent = current;
  lastPsuTemp = psuTemp;

  lastB1Down = b1Down;
  lastB2Down = b2Down;
  lastB3Down = b3Down;
  lastLidClosed = lidClosed;

}//sampleSensors()


/*
 * Prints the latest filtered sensor data to the rasppi
 */
void reportSensorData() {
  pi.println("");
  pi.print("3.3V Rail: ");
  pi.print(lastRail3v3Volt);
  pi.print("mV\n5V Rail: ");
  pi.print(lastRail5Volt);
  pi.print("mV\n12V Rail: ");
  pi.print(lastRail12Volt);
  pi.print("mV\nBatt1: ");
  pi.print(lastBatt1Volt);
  pi.print("mV\nBatt2: ");
  pi.print(lastBatt2Volt);
  pi.print("mV\nCurrent Draw: ");
  pi.print(lastCurrent);
  pi.print("mA\nPSU Temp: ");
  pi.print(lastPsuTemp, 1);
  pi.print("C\nFan Power: ");
  pi.print(curFanPower);
  pi.print("\nFan Speed: ~");
  pi.print(lastFanRPM);
  pi.println(" RPM");



  pi.print("B1=");
  pi.print(lastB1Down);
  pi.print(", B2=");
  pi.print(lastB2Down);
  pi.print(", B3=");
  pi.print(lastB3Down);
  pi.print(", LID=");
  pi.println(lastLidClosed ? "Closed" : "Open");
  
}//reportSensorData()




// Sets the current cooling fan(s) power
void setFanPower(uint8_t pwr) {
  curFanPower = pwr;
  analogWrite(FAN_PWM_PIN, pwr);
}//setFanPower()



void loop() {

  // Check if should measure sensors
  if (millis() - lastMeasureMillis > SENSOR_MEASURE_INT) {
    lastMeasureMillis = millis();
    sampleSensors();
  }//if (measure now)

  // Check if should report data to rasppi
  if (millis() - lastReportMillis > REPORT_INTERVAL) {
    lastReportMillis = millis();
    reportSensorData();
  
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
  }//if (report now)

  // Manage fan
  if (curFanPower == 0) {
    // FAN OFF; check to see if turn on
    if (lastPsuTemp > FAN_OFF_TO_LOW) {
      setFanPower(FAN_LOW_POWER);
    }//if (above low temp)
  } else if (curFanPower == FAN_LOW_POWER) {
    // FAN ON LOW; check if go off or higher
    if (lastPsuTemp > FAN_LOW_TO_HIGH) {
      setFanPower(FAN_HIGH_POWER);
    } else if (lastPsuTemp < FAN_LOW_TO_OFF) {
      setFanPower(0);
    }//if
  } else if (curFanPower == FAN_HIGH_POWER) {
    if (lastPsuTemp < FAN_HIGH_TO_LOW) {
      setFanPower(FAN_LOW_POWER);
    }//if
  } else if (curFanPower > 0) {
    setFanPower(0);
  }//if

}//loop()
