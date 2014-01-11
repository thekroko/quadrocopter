// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 6/18/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
#define _MPU6050_6AXIS_MOTIONAPPS20_H_

#include "I2Cdev.h"
#include "helper_3dmath.h"

// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#include "MPU6050.h"
//#include <avr/pgmspace.h>

/* Source is from the InvenSense MotionApps v2 demo code. Original source is
 * unavailable, unless you happen to be amazing as decompiling binary by
 * hand (in which case, please contact me, and I'm totally serious).
 *
 * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
 * DMP reverse-engineering he did to help make this bit of wizardry
 * possible.
 */

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).

//#define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

bool MPU6050::dmpPacketAvailable() {
    return getFIFOCount() >= dmpGetFIFOPacketSize();
}

uint8_t MPU6050::dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((int32_t)packet[0] << 24) + ((int32_t)packet[1] << 16) + ((int32_t)packet[2] << 8) + packet[3]);
    data[1] = (((int32_t)packet[4] << 24) + ((int32_t)packet[5] << 16) + ((int32_t)packet[6] << 8) + packet[7]);
    data[2] = (((int32_t)packet[8] << 24) + ((int32_t)packet[9] << 16) + ((int32_t)packet[10] << 8) + packet[11]);
    data[3] = (((int32_t)packet[12] << 24) + ((int32_t)packet[13] << 16) + ((int32_t)packet[14] << 8) + packet[15]);
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((int16_t)packet[0] << 8) + packet[1]);
    data[1] = (((int16_t)packet[4] << 8) + packet[5]);
    data[2] = (((int16_t)packet[8] << 8) + packet[9]);
    data[3] = (((int16_t)packet[12] << 8) + packet[13]);
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}
uint8_t MPU6050::dmpGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((int32_t)packet[16] << 24) + ((int32_t)packet[17] << 16) + ((int32_t)packet[18] << 8) + packet[19]);
    data[1] = (((int32_t)packet[20] << 24) + ((int32_t)packet[21] << 16) + ((int32_t)packet[22] << 8) + packet[23]);
    data[2] = (((int32_t)packet[24] << 24) + ((int32_t)packet[25] << 16) + ((int32_t)packet[26] << 8) + packet[27]);
    return 0;
}
uint8_t MPU6050::dmpGetGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((int16_t)packet[16] << 8) + packet[17];
    data[1] = ((int16_t)packet[20] << 8) + packet[21];
    data[2] = ((int16_t)packet[24] << 8) + packet[25];
    return 0;
}
// uint8_t MPU6050::dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t MPU6050::dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v -> x = vRaw -> x - gravity -> x*4096;
    v -> y = vRaw -> y - gravity -> y*4096;
    v -> z = vRaw -> z - gravity -> z*4096;
    return 0;
}
uint8_t MPU6050::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}
uint8_t MPU6050::dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t MPU6050::dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

uint8_t MPU6050::dmpProcessFIFOPacket(const unsigned char *dmpData) {
    return 0;
}
uint8_t MPU6050::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        getFIFOBytes(buf, dmpPacketSize);

        // process packet
        if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;
        
        // increment external process count variable, if supplied
        if (processed != 0) *processed++;
    }
    return 0;
}

uint16_t MPU6050::dmpGetFIFOPacketSize() {
    return 42;
}

#endif /* _MPU6050_6AXIS_MOTIONAPPS20_H_ */
