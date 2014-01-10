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

int16_t mx, my, mz;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_mpu
//#define OUTPUT_BINARY_mpu

#define LED_PIN RED_LED
bool blinkState = false;

void setup() {
    delay(500); // programmer creates garbage otherwise
  
    // Pull SCL low to reset the bus
    pinMode(P1_6, OUTPUT);
    digitalWrite(P1_6, LOW);
    delay(1);
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
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

    for (int i = 0; i < 10; i++) loop();
    delay(5000);
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);
    baro.startReadTemperature(MS5611_SAMPLES_256);
    delay(1);
    float t = baro.finishReadTemperature();
    baro.startReadPressure(MS5611_SAMPLES_256);
    delay(1);
    float p = baro.finishReadPressure();


    #ifdef OUTPUT_READABLE_mpu
        // display tab-separated accel/gyro x/y/z values
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
    #endif

    #ifdef OUTPUT_BINARY_mpu
        //Serial.write((uint8_t)(ax & 0xFF));Serial.write((uint8_t)(ax >> 8)); 
        //Serial.write((uint8_t)(ay & 0xFF));Serial.write((uint8_t)(ay >> 8)); 
        //Serial.write((uint8_t)(az & 0xFF));Serial.write((uint8_t)(az >> 8)); 
        Serial.write((uint8_t)(gx & 0xFF));Serial.write((uint8_t)(gx >> 8)); 
        Serial.write((uint8_t)(gy & 0xFF));Serial.write((uint8_t)(gy >> 8)); 
        Serial.write((uint8_t)(gz & 0xFF));Serial.write((uint8_t)(gz >> 8)); 
        Serial.write('\n');
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
