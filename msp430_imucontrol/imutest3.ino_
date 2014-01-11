#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#include "Servo.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"

// Pinout
#define LED RED_LED
#define MOTOR_TL P1_4
#define MOTOR_TR P1_5
#define MOTOR_BL P2_0
#define MOTOR_BR P2_1

Servo motorTL, motorTR, motorBL, motorBR; 
MPU6050 mpu;
HMC5883L mag;
MS5611 baro;

// offsets
int16_t gxo, gyo, gzo;
int16_t axo, ayo, azo;

// measured (and corrected) values
int16_t mx, my, mz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float t, p;

bool blinkState = false;

// ------------------ IMU helpers
void readSensorsIMU() {
  // Gyroscope & Accelerometer
  mpu.getMotion6(&ax, &ay, &az,  &gx, &gy, &gz);
  gx -= gxo; gy -= gyo; gz -= gzo;
  ax -= axo; ay -= ayo; az -= azo;
  
  // Magnetometer
  mag.getHeading(&mx, &my, &mz);
  
  // Barometer
  baro.startReadTemperature(MS5611_SAMPLES_256);
  delay(2);
  t = baro.finishReadTemperature();
  baro.startReadPressure(MS5611_SAMPLES_256);
  delay(2);
  p = baro.finishReadPressure();
}

void calibrateIMU() {
  // Calibration assumes that the quadrocopter is on a flat surface during startup
  Serial.println("Calibrating Sensors...");
  gxo = 0; gyo = 0; gzo = 0;
  readSensorsIMU();
  
  int64_t cnt = 256;
  int64_t gxOffset = 0; int64_t gyOffset = 0; int64_t gzOffset = 0;
  int64_t axOffset = 0; int64_t ayOffset = 0; int64_t azOffset = 0;
  for (int i = 0; i < cnt; i++) {
    readSensorsIMU();
    gxOffset += gx; gyOffset += gy; gzOffset += gz;
    axOffset += ax; ayOffset += ay; azOffset += az;
  }
  gxo = gxOffset / cnt; gyo = gyOffset / cnt; gzo = gzOffset / cnt;
  axo = axOffset / cnt; ayo = ayOffset / cnt; azo = azOffset / cnt;
}

// ------------------ Motor helpers
void setMotors(uint16_t tl, uint16_t tr, uint16_t bl, uint16_t br) { // values in permill
  motorTL.writeMicroseconds(tl + 1000L);
  motorTR.writeMicroseconds(tr + 1000L);
  motorBL.writeMicroseconds(bl + 1000L);
  motorBR.writeMicroseconds(br + 1000L);
}

// -----------------------------------
void setup() {
    // Pull SCL low to reset the bus
    pinMode(P1_6, OUTPUT);
    digitalWrite(P1_6, LOW);
    delay(5);
    
    // configure Arduino LED for
    pinMode(RED_LED, OUTPUT);
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);
    Serial.println();
    Serial.println("Initializing IMU...");
    mpu.reset();
    delay(50);
    mpu.initialize();
    mpu.setI2CBypassEnabled(true);
    mpu.setI2CMasterModeEnabled(false);
    mag.initialize();
    baro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");    
    Serial.println(baro.testConnection() ? "MS5611 connection successful" : "MS5611 connection failed");    
    for (int i = 0; i < 3; i++) loop(); // display uncalibrated values
    
    // Calibrate defaults
    calibrateIMU();
    
    // Initialize Servos
    Serial.println("Initializing motors...");
    motorTL.attach(MOTOR_TL); motorTR.attach(MOTOR_TR); motorBL.attach(MOTOR_BL); motorBR.attach(MOTOR_BR);
    setMotors(0, 0, 0, 0);
    delay(500);
    setMotors(100, 100, 100, 100); // 10%
    delay(500);
    setMotors(0, 0, 0, 0);
}

void loop() {
    readSensorsIMU();
    
    Serial.print("data\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
    Serial.print(t); Serial.print("\t");
    Serial.print(p); Serial.print("\t");
    Serial.println();
    
    // Handle input (only 1 char.tick!)
    if (Serial.available() > 0) {
      uint8_t cmd = Serial.read();
      if (cmd >= '0' && cmd <= '9') {
        uint16_t speed = (uint16_t)(cmd - '0') * 111;
        setMotors(speed, speed, speed, speed);
      } else if (cmd == ' ') setMotors(0, 0, 0, 0);
    }
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(RED_LED, blinkState);
}
