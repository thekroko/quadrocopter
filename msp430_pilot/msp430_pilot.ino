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

#define RELEASE

// Pinout
#define LED RED_LED
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

#define CMD_SIZE 5
#define DMP_PACKET_SIZE 42
#define DMP_BUFF_SIZE 14 // only the quaternion

#if F_CPU != 16000000L
#error "WRONG F_CPU"
#endif

uint8_t mode; // statis in here @ high
#define MODE_RAW     0
#define MODE_RATE    1
#define MODE_STABLE  2
#define BIT_DMP (1 << 7)
#define BIT_LED (1 << 6)
#define BIT_PRINT_PID (1 << 5)
#define BITS_MODE (0b111)

// Components
#include "twi2.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "pid.h"
#include "esc.h"

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
#define __no_init    __attribute__ ((section (".noinit"))) 
#define RESET_MAGIC 0xAC
__no_init uint8_t reset;

#ifndef RELEASE
volatile uint16_t measure;
uint8_t tick;
#define RESET_MEASURE { measure = TA1R; }
// Measured maximum time period is limited by motor control frequency.
void MEASURE(char* x) { 
  if (tick == 0) { 
    uint16_t t = TA1R < measure ? (TA1CCR0 - measure + TA1R) : TA1R - measure;
    Serial.print(x); Serial.print(": us="); Serial.println(t / 2); 
    Serial.flush();
  } 
}
#else
#define RESET_MEASURE ;
#define MEASURE(x) ;
#endif

uint8_t fifoBuffer[DMP_BUFF_SIZE];  // FIFO storage buffer

// orientation/motion vars
float lastYpr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// pid stuff
#define RS_RATE   0
#define RS_STABLE 3
#define R_YAW    0
#define R_PITCH  1
#define R_ROLL   2
#define PIDS 6
#define RAD2DEG (180.0 / M_PI)

float targetYPRS[4];
PIDConfig pidC[PIDS];
PIDData pidD[PIDS];
uint8_t selectedPID;
uint8_t pidDisabled; // low: disable pids; high: disable motors

// ------------------ Motor helpers
void setMotors(uint16_t tl, uint16_t tr, uint16_t bl, uint16_t br) { // values in 0-255*3L
  if (pidDisabled & (1 << 7)) tl = 0;
  if (pidDisabled & (1 << 6)) tr = 0;
  if (pidDisabled & (1 << 5)) bl = 0;
  if (pidDisabled & (1 << 4)) br = 0;

  setESCs(tl, tr, bl, br);
}

uint16_t makeSpeed(float speed) {
  if (speed >= 100.0) return 1000;
  if (speed <= 0.0) return 0;
  return (uint16_t)(speed * 10.0);
}

// -----------------------------------
void setup() {
    bool wasReset = (reset == RESET_MAGIC);
    reset = 0;
    
    // Initialize all floats (printing crashes device otherwise)
    ramEnd = (uint8_t*)malloc(1);
    *ramEnd = STACK_MAGIC;
    for (int i = 0; i < 3; i++) ypr[i] = 0;
    for (int i = 0; i < 4; i++) targetYPRS[i] = 0;
    memset(&pidC, 0, sizeof(PIDConfig) * PIDS);
    memset(&pidD, 0, sizeof(PIDData) * PIDS);

    // Make sure RX has a pullup (we get random interference otherwise)
    pinMode(P1_1, INPUT_PULLUP);
    Serial.begin(115200); // TODO!
    Serial.println();
    Serial.println("INIT MSP430 starting up..");
    
    // Setup LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    
    // Initialize Servos
    initESCs();

    // Calibrate?
    if (!wasReset) {
      Serial.println("INIT Calibrating ESCs..");
      setESCs(1000, 1000, 1000, 1000);
      delay(5000);
    } else Serial.println("INIT Restart after reset");
    
    setMotors(0, 0, 0, 0);
    delay(10);
    
    // All done
    Serial.println("RDY All ready!");
    checkStack('S');
}

inline bool handleIMU() {
  if (!(mode & BIT_DMP)) return false;
  
  // Check if we have a new packet
  uint16_t fifoCount = mpu.getFIFOCount();;
  if (fifoCount < DMP_PACKET_SIZE) return false; // not yet ready
  
  // Check for overflows
  if (fifoCount >= DMP_PACKET_SIZE*5) {
    // Data is beginning to stack in our FIFO, and all the values out there are way outdated now ..
    mpu.resetFIFO(); // we have a massive overflow ..
    Serial.println(F("FIFO!"));
    return false;
  } else { // data ready
     // WARNING: this assumes a fixed size of 42 bytes @ pak!
     // We do this so that we can efficiently flush our TWI buffer as things are happening
     if (mpu.getFIFOBytes(fifoBuffer, DMP_BUFF_SIZE /* 16 */)) Serial.println("read failed");
     
     // Start discarding the remainder of the packet
     mpu.getFIFOBytes(0, DMP_PACKET_SIZE - DMP_BUFF_SIZE); // this should not block
     
     Quaternion q;           // [w, x, y, z]         quaternion container
     VectorFloat gravity;    // [x, y, z]            gravity vector
     mpu.dmpGetQuaternion(&q, fifoBuffer);
     mpu.dmpGetGravity(&gravity, &q);
     
     lastYpr[0] = ypr[0];
     lastYpr[1] = ypr[1];
     lastYpr[2] = ypr[2];
     
     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // /!\ Deepest point in the stack
     checkStack('D');
     twi_flush();
     return true;
  }
}

inline void handlePIDs() {
  // Stabilization PID
  float yawCmd = updatePID(pidC[R_YAW+RS_STABLE],   pidD[R_YAW+RS_STABLE],   ypr[0], targetYPRS[0]); // TL rotates right
  float pitchCmd = updatePID(pidC[R_PITCH+RS_STABLE], pidD[R_PITCH+RS_STABLE], ypr[1], targetYPRS[1]);
  float rollCmd =  updatePID(pidC[R_ROLL+RS_STABLE],  pidD[R_ROLL+RS_STABLE],  ypr[2], targetYPRS[2]);
  float motorSpeed = targetYPRS[3];

  if (pidDisabled & 0b0001) { // No Stabilization PID
    yawCmd = targetYPRS[0];
    pitchCmd = targetYPRS[1];
    rollCmd = targetYPRS[2];
  }
  
  // Rate PID (this won't run with 66 Hz ...)
  if (!(pidDisabled & 0b0010)) {
    yawCmd =   updatePID(pidC[R_YAW+RS_RATE],     pidD[R_YAW+RS_RATE],   ypr[0] - lastYpr[0], yawCmd);
    pitchCmd = updatePID(pidC[R_PITCH+RS_RATE],   pidD[R_PITCH+RS_RATE], ypr[1] - lastYpr[1], pitchCmd);
    rollCmd =  updatePID(pidC[R_ROLL+RS_RATE],    pidD[R_ROLL+RS_RATE],  ypr[2] - lastYpr[2], rollCmd);
  }
  
  if ((mode & BIT_PRINT_PID)) {
    mode &= ~BIT_PRINT_PID;
    Serial.print("YPR_ERR ");
    Serial.print(yawCmd); Serial.print(' ');
    Serial.print(pitchCmd); Serial.print(' ');
    Serial.println(rollCmd);    
  }
  
  // We do the calculation either way so that we have a constant timing
  if ((mode & BITS_MODE) <= MODE_RAW) return;
  
  // Set motor speeds
  setMotors(
   /* TL */ makeSpeed(motorSpeed - yawCmd - rollCmd - pitchCmd),
   /* TR */ makeSpeed(motorSpeed + yawCmd - rollCmd + pitchCmd),
   /* BL */ makeSpeed(motorSpeed + yawCmd + rollCmd - pitchCmd),
   /* BR */ makeSpeed(motorSpeed - yawCmd + rollCmd + pitchCmd)
  );
}

inline void handleInput() {
  // Read input. We assume a fixed command width of 5 byte, terminated with 0xFF (=6 byte total)
  if (Serial.available() < 6 || Serial.read() != 0xFF) return; // not yet ready
  uint8_t cmd[5];
  for (int i = 0; i < 5; i++) {
    uint8_t val = Serial.read();
    cmd[i] = val;
  }
    
  // Process the command
  switch (cmd[0]) {
    // Who am I?
    case 0x00: {
      Serial.println("MSP430 Pilot v0.1");
      break;
    }
      
    // Read Yaw/Pitch/Roll
    case 0x01: {
      Serial.print("YPR ");
      for (int i = 0; i < 3; i++) {
        Serial.print(ypr[i] * RAD2DEG); Serial.print(i == 2 ? '\n' : ' ');      
      }
      break;
    }
    
    // Read Mode
    case 0x02: {
      printMode:
      Serial.print("MODE "); Serial.print(mode & BITS_MODE);
      Serial.print(' '); Serial.println(mode >> 3);
      break;
    }

    // Init DMP & I2C
    case 0x0A: {
      Serial.println("DMP init..");
      mode &= ~BIT_DMP;
      mpu.initialize();
      bool init = mpu.testConnection();
      if (init) {
        mode |= BIT_DMP;
        Serial.print("DMP ready ");
        mpu.resetFIFO();
        Serial.println(mpu.getFIFOCount());
      } else Serial.println("IMU not found");
      break;
    }
      
     //   [0x0B] pidHIGH    val#LOW  ............   0xFF    Change PID register
     case 0x0B: {
      selectedPID = cmd[1];
      break;
    }
      
    //   [0x0C] float...,................   0xFF    Set PID value
    case 0x0C: {
      float* p = (float*)(&pidC[selectedPID >> 4]);
      memcpy(&p[selectedPID & 0b1111], &cmd[1], sizeof(float));
      Serial.println("PIDSET ");
      break;
    }
      
    //   [0x0D] .........................   0xFF    Print PID values
    case 0x0D: {
      Serial.print("PIDDUMP ");
      Serial.print(selectedPID >> 4);
      Serial.print(':');
      PIDConfig pid = pidC[selectedPID >> 4];
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
      mode &= ~BITS_MODE;
      mode |= MODE_RAW;
      setMotors((uint16_t)cmd[1]*4L, (uint16_t)cmd[2]*4L, (uint16_t)cmd[3]*4L, (uint16_t)cmd[4]*4L);
      break;
    }
      
    //   [0x20] mode   0xXX   0xXX   0xXX   0xFF    Set Mode (fixed/rate/..)
    case 0x20: {
      if (cmd[2] > 0) mode ^= cmd[2];
      else {
        mode &= ~BITS_MODE;
        mode |= cmd[1];
        goto printMode;
      }
      break;
    }
    
    //   [0x21] 0xXX   0xXX   0xXX   0xXX   0xFF    Print target Y/P/R rates
    case 0x21: {
      Serial.print("RATE_YPRS ");
      for (int i = 0; i < 4; i++) {
        Serial.print(targetYPRS[i] * (i == 3 ? 1 : RAD2DEG)); Serial.print(i == 3 ? "\r\n" : " "); 
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
    
    //   [0x26] bitmask                      Disable PID
    case 0x26: {
      pidDisabled = cmd[1];
      Serial.println("PIDMASK");
      break;
    }
    
    default: {
      Serial.println("?");
      break;
    }
  }
}


void loop() {
  // Main loop execution roughly takes 12ms (= 83Hz), but we are limited by the DMP sensor settings (66 or 100 Hz)
  RESET_MEASURE // reset timer

  if (handleIMU()) {
    handlePIDs();
  }
  else delay(8); // give it some regularity
  handleInput();
  MEASURE("Loop");
}
