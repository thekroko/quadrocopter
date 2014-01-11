// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

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

#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"

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

// IMU helpers
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
    Serial.println("Initializing I2C devices...");
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
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(RED_LED, blinkState);
}
