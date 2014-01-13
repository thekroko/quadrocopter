#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

int setup() {
    // initialize device
    printf("Initializing MPU ...\n");
    mpu.initialize();

    // verify connection
    printf("Testing connection ...\n");
    if (!mpu.testConnection()) { printf("MPU6050 connection failed\n"); return 1; }

    // load and configure the DMP
    printf("Flashing DMP ...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        printf("DMP ready\n");
        printf("MPU6050 initialized!\n");
        return 0;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
        return 1;
    }
}

int main() {
    return setup();
}

