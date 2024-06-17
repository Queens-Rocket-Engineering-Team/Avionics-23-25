// Basic demo for accelerometer readings from Adafruit MPU6050



#define USB_TX_PIN PA12 // D- pin
#define USB_RX_PIN PA11 // D+ pin
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define ACCEL_INT1 PB5
#define ACCEL_INT2 PB6
#define DB_LED_PIN PA15

SoftwareSerial softSerial(USB_RX_PIN, USB_TX_PIN);

Adafruit_MPU6050 mpu;

bool potentialLaunch = false;
bool highGTime = 0;

void setup(void) {
  softSerial.begin(38400);
  delay(2000);

  softSerial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  
  if (!mpu.begin(0x69)) {
    softSerial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  softSerial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  softSerial.println("");
  delay(100);
  
}

void loop() {
  time = millis();

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (!potentialLaunch && a.acceleration.z < G_THRESHOLD) {
    potentialLaunch = true;
    highGTime = millis();
  }

  if (potentialLaunch && millis() - highGTime > 50) {
    if (a.acceleration.z < G_THRESHOLD) {
      softSerial.print("LAUNCH");
    } else {
      potentialLaunch = false;
    }
  }


  /* Print out the values */
  // softSerial.print("Acceleration X: ");
  // softSerial.print(a.acceleration.x);
  // softSerial.print(", Y: ");
  // softSerial.print(a.acceleration.y);
  // softSerial.print(", Z: ");
  // softSerial.print(a.acceleration.z);
  // softSerial.println(" m/s^2");

  // softSerial.print("Rotation X: ");
  // softSerial.print(g.gyro.x);
  // softSerial.print(", Y: ");
  // softSerial.print(g.gyro.y);
  // softSerial.print(", Z: ");
  // softSerial.print(g.gyro.z);
  // softSerial.println(" rad/s");


  softSerial.println("");
  delay(10);
}