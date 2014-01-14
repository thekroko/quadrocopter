// UART Commands:
//   [0x00] .........................    Who am I? Returns "MSP430"
//   [0x01] .........................    Read Yaw/Pitch/Roll
//   [0x02] .........................    Read Mode

//   [0x0A] .........................    Init DMP and I2C (Required for most other commands)
//   [0x0B] pid    val#  ............    Change PID register
//   [0x0C] .........4-float.........    Change PID value
//   [0x0D] .........................    Print PID values
//   [0x0F] 0xB5   0x3A   0x79   0x00    Reset controller

//   [0x10] tl     tr     bl     br      Set Motor Speed directly

//   [0x20] mode  addFlags ..........    Set Mode
//   [0x21] .........................    Print target Y/P/R rates
//   [0x22] .........4-float.........    Set Rate control Yaw 
//   [0x23] .........4-float.........    Set Rate control Pitch
//   [0x24] .........4-float.........    Set Rate control Roll
//   [0x25] .........4-float.........    Set Rate control Motor %
//   [0x26] bitmask                      Disable PID

// Pinout
#define LED RED_LED
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

#define DMP_PACKET_SIZE 42
#define DMP_BUFF_SIZE 16 // only the quaternion
#define RAD2DEG (180.0/M_PI)

// Components
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define STACK_MAGIC 0x3F
uint8_t* ramEnd; // this ends up in the far end of the ram, next to the stack. (but before our includes)
inline void checkStack(char c) {
  if (*ramEnd != STACK_MAGIC) {
    // memory has been corrupted by something
    Serial.print("!!!STACK OVERFLOW @ "); Serial.print(c); Serial.println("!!!");
    Serial.flush();
    while (1) ; // halt
  }
}

MPU6050 mpu;

// Global State
uint8_t tick;

#define RESET_MEASURE { TA1R = 0; }
void MEASURE(char* x) { 
  if (tick == 0) { 
    Serial.print(x); Serial.print(": us="); Serial.println((uint32_t)TA1R * 8L / clockCyclesPerMicrosecond()); 
    Serial.flush(); TA1R = 0;
  } 
}

uint8_t fifoBuffer[DMP_BUFF_SIZE];  // FIFO storage buffer

// orientation/motion vars
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// -----------------------------------
void setup() {    
    // Initialize all floats (printing crashes device otherwise)
    ramEnd = (uint8_t*)malloc(1);
    *ramEnd = STACK_MAGIC;
    for (int i = 0; i < 3; i++) ypr[i] = 0;

    // Make sure RX has a pullup (we get random interference otherwise)
    pinMode(P1_1, INPUT_PULLUP);
    Serial.begin(115200);
    Serial.println();
    Serial.println("MSP430 I2C Test. Press key.");
    
    // Setup LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    
    // Setup a cycle timer to measure performance
    TA1CTL = TASSEL_2 + MC_2 + ID_3; // SMCLK/8, count to 
    TA1CCTL0 = 0;
    checkStack('S');
    
    // Wait for mpu init
    bool init;
    do {
      while (!Serial.available()) ;
      while (Serial.available()) Serial.read();
      Wire.begin();
      init = mpu.testConnection();
      if (init) {
        mpu.resetFIFO();
        mpu.getIntStatus();
        Serial.println("DMP ready");
      } else Serial.println("IMU not found");
    } while (!init);
}

bool handleIMU() {
  // Check if we have a new packet
  uint16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount < DMP_PACKET_SIZE) return false; // not yet ready
  
  // Check for overflows
  uint8_t mpuIntStatus = mpu.getIntStatus();
  if ((mpuIntStatus & 0x10) || fifoCount >= 42*3 /* too many old frames... */) {
    // reset so we can continue cleanly3
    mpu.resetFIFO();
    Serial.println(F("FIFO!"));
    return false;
  } else if (mpuIntStatus & 0x02) { // data ready
     // WARNING: this assumes a fixed size of 42 bytes @ pak!
     // We do this so that we can efficiently flush our TWI buffer as things are happening
     mpu.getFIFOBytes(fifoBuffer, DMP_BUFF_SIZE /* 16 */);
     MEASURE("IMU-getBytes");
     Quaternion q;           // [w, x, y, z]         quaternion container
     VectorFloat gravity;    // [x, y, z]            gravity vector
     mpu.dmpGetQuaternion(&q, fifoBuffer);
     mpu.getFIFOBytes(fifoBuffer, 7); // discard
     MEASURE("IMU-getQuaternion");
     mpu.dmpGetGravity(&gravity, &q);
     mpu.getFIFOBytes(fifoBuffer, 7); // discard
     MEASURE("IMU-getGravity");
     checkStack('X');
     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // /!\ Deepest point in the stack
     checkStack('D');
     mpu.getFIFOBytes(fifoBuffer, 12); // discard
     MEASURE("IMU-getYPR");
     return true;
  }
}

void loop() {
  checkStack('M');
  RESET_MEASURE // reset timer
  
  if (handleIMU()) {
    if ((tick++) == 0) {
      Serial.print("YPR ");
      for (int i = 0; i < 3; i++) {
        Serial.print(ypr[i] * RAD2DEG); Serial.print(i == 2 ? '\n' : ' ');      
      }
    }
  } 
  MEASURE("Loop");
  
  // Give some status & frequency indicator
  if (tick == 0) {
    digitalWrite(LED, true);
  } else if (tick == 127) digitalWrite(LED, false);
}
