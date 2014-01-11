// Pinout
#define LED RED_LED
#define MOTOR_TL P1_5
#define MOTOR_TR P1_4
#define MOTOR_BL P2_0
#define MOTOR_BR P2_1
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

#define CMD_SIZE 5
#define DMP_PACKET_SIZE 42

// Components
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"

MPU6050 mpu;
Servo motorTL, motorTR, motorBL, motorBR; 

// Global State
bool ledState = false;
bool dmpInitialized = false;   // true if the DMP has been flashed
uint16_t fifoCount;            // count of all bytes currently in FIFO
uint8_t fifoBuffer[DMP_PACKET_SIZE];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ------------------ Motor helpers
void setMotors(uint8_t tl, uint8_t tr, uint8_t bl, uint8_t br) { // values in permill
  motorTL.writeMicroseconds((uint16_t)tl*3L + 1235L);
  motorTR.writeMicroseconds((uint16_t)tr*3L + 1235L);
  motorBL.writeMicroseconds((uint16_t)bl*3L + 1235L);
  motorBR.writeMicroseconds((uint16_t)br*3L + 1235L);
}

// -----------------------------------
void setup() {
    // Make sure RX has a pullup (we get random interference otherwise)
    pinMode(P1_1, INPUT_PULLUP);
    Serial.begin(115200);
    Serial.println();
    Serial.println("INIT MSP430 starting up..");
    
    // Setup LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    
    // Initialize Servos
    Serial.println("INIT Calibrating ESCs..");
    motorTL.attach(MOTOR_TL); motorTR.attach(MOTOR_TR); motorBL.attach(MOTOR_BL); motorBR.attach(MOTOR_BR);
    setMotors(254, 254, 254, 254);
    delay(5000);
    setMotors(0, 0, 0, 0);
    delay(2000);
    
    // All done
    Serial.println("RDY All ready!");
}

void handleIMU() {
  // IMU has not yet been initialized. Don't do anything
  if (!dmpInitialized) return;
  
  // Check if we have a new packet
  uint16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount < DMP_PACKET_SIZE) return; // not yet ready
  
  // Check for overflows
  uint8_t mpuIntStatus = mpu.getIntStatus();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.print(F("FIFO overflow! "));
    Serial.print(fifoCount);
    Serial.print(' ');
    Serial.println(mpuIntStatus);
  } else if (mpuIntStatus & 0x02) { // data ready
     mpu.getFIFOBytes(fifoBuffer, DMP_PACKET_SIZE);
     mpu.dmpGetQuaternion(&q, fifoBuffer);
     mpu.dmpGetGravity(&gravity, &q);
     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
     Serial.print("ypr\t");
     Serial.print(ypr[0] * 180/M_PI);
     Serial.print("\t");
     Serial.print(ypr[1] * 180/M_PI);
     Serial.print("\t");
     Serial.println(ypr[2] * 180/M_PI);
  }
}

void handleInput() {
  // Handle input. We assume a fixed command width of 5 byte, terminated with 0xFF (=6 byte total)
  if (Serial.available() < 6) return; // not yet ready
  uint8_t cmd[6];
  for (int i = 0; i < 5; i++) {
    uint8_t val = Serial.read();
    if (val == 0xFF) return; // illegal package here
    cmd[i] = val;
  }
  if (Serial.read() != 0xFF) return;
    
  // Process the command
  switch (cmd[0]) {
    case 0x00: // NOP
      break;
    case 0x01: // DMP/I2C init
      Serial.println("DMP init..");
      Wire.begin();
      dmpInitialized = mpu.testConnection();
      if (dmpInitialized) {
        mpu.resetFIFO();
        Serial.println("DMP ready");
      } else Serial.println("DMP error");
      break;
    case 0xF0: // Motor speed test
      // We have a valid motor speed...
      setMotors(cmd[1], cmd[2], cmd[3], cmd[4]);
      Serial.print('+');
      break;
  }
}

void loop() {
  handleIMU();
  handleInput();
  ledState = !ledState;
  digitalWrite(LED, ledState);
}
