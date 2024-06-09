// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define USB_TX_PIN PA12 // D- pin
#define USB_RX_PIN PA11 // D+ pin
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define ACCEL_INT1 PB5
#define ACCEL_INT2 PB6
#define DB_LED_PIN PA15



SoftwareSerial softSerial(USB_RX_PIN, USB_TX_PIN);

Adafruit_MPU6050 mpu;

void setup(void) {
  softSerial.begin(38400);
  delay(2000);

  softSerial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  
  if (!mpu.begin()) {
    softSerial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  softSerial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  softSerial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    softSerial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    softSerial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    softSerial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    softSerial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  softSerial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    softSerial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    softSerial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    softSerial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    softSerial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  softSerial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    softSerial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    softSerial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    softSerial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    softSerial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    softSerial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    softSerial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    softSerial.println("5 Hz");
    break;
  }

  softSerial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  softSerial.print("Acceleration X: ");
  softSerial.print(a.acceleration.x);
  softSerial.print(", Y: ");
  softSerial.print(a.acceleration.y);
  softSerial.print(", Z: ");
  softSerial.print(a.acceleration.z);
  softSerial.println(" m/s^2");

  softSerial.print("Rotation X: ");
  softSerial.print(g.gyro.x);
  softSerial.print(", Y: ");
  softSerial.print(g.gyro.y);
  softSerial.print(", Z: ");
  softSerial.print(g.gyro.z);
  softSerial.println(" rad/s");

  softSerial.print("Temperature: ");
  softSerial.print(temp.temperature);
  softSerial.println(" degC");

  softSerial.println("");
  delay(500);
}