// UART Commands:
//   [0x00] 0xXX   0xXX   0xXX   0xXX   0xFF    Who am I? Returns "MSP430"
//   [0x01] 0xXX   0xXX   0xXX   0xXX   0xFF    Read Yaw/Pitch/Roll
//   [0x02] 0xXX   0xXX   0xXX   0xXX   0xXX    Read Mode

//   [0x0A] 0xXX   0xXX   0xXX   0xXX   0xFF    Init DMP and I2C (Required for most other commands)
//   [0x0F] 0xB5   0x3A   0x79   0x00   0xFF    Reset controller

//   [0x10] 0xTL   0xTR   0xBL   0xBR   0xFF    Set Motor Speed directly

//   [0x20] 0xXX   0xXX   0xXX   0xXX   0xFF    Set Mode: Rate Control
//   [0x21] 0xXX   0xXX   0xXX   0xXX   0xFF    Print target Y/P/R rates
//   [0x22] float  ..................   0xFF    Set Rate control Yaw 
//   [0x23] float  ..................   0xFF    Set Rate control Pitch
//   [0x24] float  ..................   0xFF    Set Rate control Roll

// Pinout
#define LED RED_LED
#define MOTOR_TL P1_5
#define MOTOR_TR P1_4
#define MOTOR_BL P2_0
#define MOTOR_BR P2_1
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

#define CMD_SIZE 5
#define DMP_PACKET_SIZE 42

#define MODE_RAW     1
#define MODE_RATE    2
#define MODE_STABLE  3

// Components
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"

MPU6050 mpu;
Servo motorTL, motorTR, motorBL, motorBR; 

// Global State
#define __no_init    __attribute__ ((section (".noinit"))) 
#define RESET_MAGIC 0xAC19
__no_init uint16_t reset;
uint8_t tick = 0;
uint8_t mode = MODE_RAW;
bool ledState = false;
bool dmpInitialized = false;   // true if the DMP has been flashed
uint16_t fifoCount;            // count of all bytes currently in FIFO
uint8_t fifoBuffer[DMP_PACKET_SIZE];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];          // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// pid stuff
float targetYPR[3];

union BinaryFloat { 
      float num; 
      uint32_t numi; 
};

// ------------------ Motor helpers
void setMotors(uint8_t tl, uint8_t tr, uint8_t bl, uint8_t br) { // values in 0-2551
  motorTL.writeMicroseconds((uint16_t)tl*3L + 1235L);
  motorTR.writeMicroseconds((uint16_t)tr*3L + 1235L);
  motorBL.writeMicroseconds((uint16_t)bl*3L + 1235L);
  motorBR.writeMicroseconds((uint16_t)br*3L + 1235L);
}

// -----------------------------------
void setup() {
    bool wasReset = (reset == RESET_MAGIC);
    reset = 0;
    
    // Make sure RX has a pullup (we get random interference otherwise)
    pinMode(P1_1, INPUT_PULLUP);
    Serial.begin(115200);
    Serial.println();
    Serial.println("INIT MSP430 starting up..");
    
    // Setup LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    
    // Initialize Servos
    motorTL.attach(MOTOR_TL); motorTR.attach(MOTOR_TR); motorBL.attach(MOTOR_BL); motorBR.attach(MOTOR_BR);

    // Calibrate?
    if (!wasReset) {
      Serial.println("INIT Calibrating ESCs..");
      setMotors(254, 254, 254, 254);
      delay(5000);
    } else Serial.println("INIT Restart after reset");
    
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
    Serial.print(F("ERR FIFO overflow! "));
    Serial.print(fifoCount);
    Serial.print(' ');
    Serial.println(mpuIntStatus);
  } else if (mpuIntStatus & 0x02) { // data ready
     mpu.getFIFOBytes(fifoBuffer, DMP_PACKET_SIZE);
     mpu.dmpGetQuaternion(&q, fifoBuffer);
     mpu.dmpGetGravity(&gravity, &q);
     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
     ypr[0] *= 180/M_PI;
     ypr[1] *= 180/M_PI;
     ypr[2] *= 180/M_PI;
  }
}

void handleInput() {
  // Read input. We assume a fixed command width of 5 byte, terminated with 0xFF (=6 byte total)
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
    // Who am I?
    case 0x00:
      Serial.println("MSP430");
      break;
      
    // Read Yaw/Pitch/Roll
    case 0x01: 
      Serial.print("YPR ");
      for (int i = 0; i < 3; i++) {
        Serial.print(ypr[i]); Serial.print(i == 2 ? '\n' : ' ');      
      }
      break;
      
    // Read Mode
    case 0x02:
      Serial.print("MODE "); Serial.println(mode);
      break;
      

    // Init DMP & I2C
    case 0x0A:
      Serial.println("DMP init..");
      Wire.begin();
      dmpInitialized = mpu.testConnection();
      if (dmpInitialized) {
        mpu.resetFIFO();
        mpu.getIntStatus();
        Serial.println("DMP ready");
      } else Serial.println("IMU not found");
      break;
      
    //   [0x0F] 0xB5   0x3A   0x79   0x00   0xFF    Reset controller
    case 0x0F:
      if (cmd[1] != 0xB5 || cmd[2] != 0x3A || cmd[3] != 0x79 || cmd[4] != 0x00) {
        Serial.println("ERR wrong reset seq");
        break;
      }
      reset = RESET_MAGIC;
      Serial.println("RESET ");
      Serial.flush();
      WDTCTL = 0xFFFF; // Causes an Access Violation
      break;
      
    // Mode: Raw Motor Speed
    case 0x10:
      mode = MODE_RAW;
      setMotors(cmd[1], cmd[2], cmd[3], cmd[4]);
      break;
      
    //   [0x20] 0xXX   0xXX   0xXX   0xXX   0xFF    Set Mode: Rate Control
    case 0x20:
      mode = MODE_RATE;
      break;
    
    //   [0x21] 0xXX   0xXX   0xXX   0xXX   0xFF    Print target Y/P/R rates
    case 0x21:
      Serial.print("RATEYPR ");
      for (int i = 0; i < 3; i++) {
        Serial.print(targetYPR[i]); Serial.print(i == 2 ? '\n' : ' '); 
      }
      break;
      
    //   [0x22] float  ..................   0xFF    Set Rate control Yaw 
    //   [0x23] float  ..................   0xFF    Set Rate control Pitch
    //   [0x24] float  ..................   0xFF    Set Rate control Roll
    case 0x22: case 0x23: case 0x24:
      //targetYPR[cmd[0]-0x22] = *(float*)&cmd[1];
      BinaryFloat bf;
      bf.data
      targetYPR[cmd[0]-0x22] = *(float*)(&cmd+1);
      break;
  }
}

void loop() {
  handleIMU();
  handleInput();
  if (tick++ == 0) {
    ledState = !ledState;
    digitalWrite(LED, ledState);
  }
}
