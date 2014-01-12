// UART Commands:
//   [0x00] .........................   0xFF    Who am I? Returns "MSP430"
//   [0x01] .........................   0xFF    Read Yaw/Pitch/Roll
//   [0x02] .........................   0xFF    Read Mode

//   [0x0A] .........................   0xFF    Init DMP and I2C (Required for most other commands)
//   [0x0B] pid    val#  ............   0xFF    Change PID register
//   [0x0C] .........4-float.........   0xFF    Change PID value
//   [0x0D] .........................   0xFF    Print PID values
//   [0x0F] 0xB5   0x3A   0x79   0x00   0xFF    Reset controller

//   [0x10] tl     tr     bl     br     0xFF    Set Motor Speed directly

//   [0x20] mode   ..................   0xFF    Set Mode
//   [0x21] .........................   0xFF    Print target Y/P/R rates
//   [0x22] .........4-float.........   0xFF    Set Rate control Yaw 
//   [0x23] .........4-float.........   0xFF    Set Rate control Pitch
//   [0x24] .........4-float.........   0xFF    Set Rate control Roll
//   [0x25] .........4-float.........   0xFF    Set Rate control Motor %

// Pinout
#define LED RED_LED
#define MOTOR_TL P1_5
#define MOTOR_TR P1_4
#define MOTOR_BL P2_0
#define MOTOR_BR P2_1
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

#define CMD_SIZE 5
#define DMP_PACKET_SIZE 21 // 2x that, but we can ignore the 2nd part

#define MODE_RAW     1
#define MODE_RATE    2
#define MODE_STABLE  3

// Components
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"
#include "pid.h"

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// pid stuff
#define R_YAW    0
#define R_PITCH  1
#define R_ROLL   2
float targetYPRS[4];
PID pids[3];
uint8_t selectedPIDRegister = 0;
uint8_t selectedPIDValue = 0;

// ------------------ Motor helpers
void setMotors(uint16_t tl, uint16_t tr, uint16_t bl, uint16_t br) { // values in 0-255*3L
  motorTL.writeMicroseconds((uint16_t)tl*3L + 1235L);
  motorTR.writeMicroseconds((uint16_t)tr*3L + 1235L);
  motorBL.writeMicroseconds((uint16_t)bl*3L + 1235L);
  motorBR.writeMicroseconds((uint16_t)br*3L + 1235L);
}

uint16_t makeSpeed(float speed) {
  if (speed >= 100.0) return 254*3L;
  if (speed <= 0.0) return 0;
  speed = speed * 254.0 * 3.0 / 100.0;
  return (uint16_t)speed;
}

// -----------------------------------
void setup() {
    bool wasReset = (reset == RESET_MAGIC);
    reset = 0;
    
    // Initialize all floats (printing crashes device otherwise)
    for (int i = 0; i < 3; i++) ypr[i] = i+1;
    for (int i = 0; i < 4; i++) targetYPRS[i] = i+1;
    memset(&pids, 0, sizeof(PID) * 3);
    
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

    // Calibrate? (TODO: Reenable)
    /*if (!wasReset) {
      Serial.println("INIT Calibrating ESCs..");
      setMotors(254*3L, 254*3L, 254*3L, 254*3L);
      delay(5000);
    } else Serial.println("INIT Restart after reset");*/
    
    setMotors(0, 0, 0, 0);
    delay(2000);
    
    // All done
    Serial.println("RDY All ready!");
}

inline void handleIMU() {
  // IMU has not yet been initialized. Don't do anything
  if (!dmpInitialized) return;
  
  // Check if we have a new packet
  uint16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount < 2*DMP_PACKET_SIZE) return; // not yet ready
  
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
     mpu.getFIFOBytes(fifoBuffer, DMP_PACKET_SIZE); // read the 2nd part of the packet
     ypr[0] *= 180/M_PI;
     ypr[1] *= 180/M_PI;
     ypr[2] *= 180/M_PI;
  }
}

inline void handlePIDs() {
  if (mode <= MODE_RAW) return;
  
  // Rate control
  float yawCmd = updatePID(pids[R_YAW], ypr[0], targetYPRS[0]);
  float pitchCmd = updatePID(pids[R_PITCH], ypr[1], targetYPRS[1]);
  float rollCmd = updatePID(pids[R_ROLL], ypr[2], targetYPRS[2]);
  float motorSpeed = targetYPRS[3];
  
  // Set motor speeds
  setMotors(
   /* TL */ makeSpeed(motorSpeed + yawCmd - rollCmd - pitchCmd),
   /* TR */ makeSpeed(motorSpeed - yawCmd + rollCmd - pitchCmd),
   /* BL */ makeSpeed(motorSpeed + yawCmd - rollCmd + pitchCmd),
   /* BR */ makeSpeed(motorSpeed - yawCmd + rollCmd + pitchCmd)
  );
}

inline void handleInput() {
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
    case 0x00: {
      Serial.println("MSP430");
      break;
    }
      
    // Read Yaw/Pitch/Roll
    case 0x01: {
      Serial.print("YPR ");
      for (int i = 0; i < 3; i++) {
        Serial.print(ypr[i]); Serial.print(i == 2 ? '\n' : ' ');      
      }
      break;
    }
    
    // Read Mode
    case 0x02: {
      printMode:
      Serial.print("MODE "); Serial.println(mode);
      break;
    }

    // Init DMP & I2C
    case 0x0A: {
      Serial.println("DMP init..");
      Wire.begin();
      dmpInitialized = mpu.testConnection();
      if (dmpInitialized) {
        mpu.resetFIFO();
        mpu.getIntStatus();
        Serial.println("DMP ready");
      } else Serial.println("IMU not found");
      break;
    }
      
     //   [0x0B] pid    val#  ............   0xFF    Change PID register
     case 0x0B: {
      selectedPIDRegister = cmd[1];
      selectedPIDValue = cmd[2];
      break;
    }
      
    //   [0x0C] float...,................   0xFF    Set PID value
    case 0x0C: {
      float* p = (float*)(&pids[selectedPIDRegister]);
      memcpy(&p[selectedPIDValue], &cmd[1], sizeof(float));
      break;
    }
      
    //   [0x0D] .........................   0xFF    Print PID values
    case 0x0D: {
      Serial.print("PIDDUMP ");
      Serial.print(selectedPIDRegister);
      Serial.print(':');
      PID pid = pids[selectedPIDRegister];
      /*float* p = (float*)(&pid) ;
      for (int i = 0; i < sizeof(PID)/sizeof(float) - 4; i++) {
        Serial.print(' ');
        Serial.print(p[i]);
      }*/
      Serial.print(' '); Serial.print(pid.kP);
      Serial.print(' '); Serial.print(pid.kI);
      Serial.print(' '); Serial.print(pid.kD);
      Serial.print(' '); Serial.print(pid.minI);
      Serial.print(' '); Serial.print(pid.maxI);
      Serial.print(' '); Serial.print(pid.minO);
      Serial.print(' '); Serial.print(pid.maxO);
      Serial.println();
      break;
    }
      
    //   [0x0F] 0xB5   0x3A   0x79   0x00   0xFF    Reset controller
    case 0x0F: {
      if (cmd[1] != 0xB5 || cmd[2] != 0x3A || cmd[3] != 0x79 || cmd[4] != 0x00) {
        Serial.println("ERR wrong reset seq");
        break;
      }
      reset = RESET_MAGIC;
      Serial.println("RESET ");
      Serial.flush();
      WDTCTL = 0xFFFF; // Causes an Access Violation
      break;
    }
      
    // Mode: Raw Motor Speed
    case 0x10: {
      mode = MODE_RAW;
      setMotors((uint16_t)cmd[1]*3L, (uint16_t)cmd[2]*3L, (uint16_t)cmd[3]*3L, (uint16_t)cmd[4]*3L);
      break;
    }
      
    //   [0x20] mode   0xXX   0xXX   0xXX   0xFF    Set Mode (fixed/rate/..)
    case 0x20: {
      mode = cmd[1];
      goto printMode;
      break;
    }
    
    //   [0x21] 0xXX   0xXX   0xXX   0xXX   0xFF    Print target Y/P/R rates
    case 0x21: {
      Serial.print("RATE_YPRS ");
      for (int i = 0; i < 4; i++) {
        Serial.print(targetYPRS[i]); Serial.print(i == 3 ? '\n' : ' '); 
      }
      break;
    }
      
    //   [0x22] float  ..................   0xFF    Set Rate control Yaw 
    //   [0x23] float  ..................   0xFF    Set Rate control Pitch
    //   [0x24] float  ..................   0xFF    Set Rate control Roll
    //   [0x25] float  ..................   0xFF    Set Rate control Motor speed
    case 0x22: case 0x23: case 0x24: case 0x25: {
      int destIndex = cmd[0] - 0x22;
      memcpy(&targetYPRS[destIndex], &cmd[1], sizeof(float));
      break;
    }
    
    default: {
      Serial.println("?");
    }
  }
}

void loop() {
  handleIMU();
  handlePIDs();
  handleInput();
  
  // Give some status & frequency indicator
  if (tick++ == 0) {
    ledState = !ledState;
    digitalWrite(LED, ledState);
  }
}
