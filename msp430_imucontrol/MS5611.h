// I2Cdev library collection - HMS5611 support
// 08/01/2014 by Matthias Linder <matthias@matthiaslinder.com>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//

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

#ifndef _MS5611_H_
#define _MS5611_H_

#include "I2Cdev.h"

#define MS5611_ADDRESS            0x77
#define MS5611_DEFAULT_ADDRESS    0x77

#define MS5611_RA_RESET           0x1E
#define MS5611_RA_CONVERT_D1      0x40
#define MS5611_RA_CONVERT_D2      0x50
#define MS5611_RA_ADC             0x00
#define MS5611_RA_READ_PROM       0xA0
#define MS5611_RA_NOP             0x01

#define MS5611_SAMPLES_256        0x00
#define MS5611_SAMPLES_512        0x02
#define MS5611_SAMPLES_1024       0x04
#define MS5611_SAMPLES_2048       0x06
#define MS5611_SAMPLES_4096       0x08

#define MS5611_PROM_ID            0 << 1
#define MS5611_PROM_C1            1 << 1
#define MS5611_PROM_C2            2 << 1
#define MS5611_PROM_C3            3 << 1
#define MS5611_PROM_C4            4 << 1
#define MS5611_PROM_C5            5 << 1
#define MS5611_PROM_C6            6 << 1
#define MS5611_PROM_CRC           7 << 1

class MS5611 {
    public: 
        MS5611();
        MS5611(uint8_t address);
        
        void initialize();
        bool testConnection();

	// ID_* registers
        void startReadTemperature(uint8_t samples);
        void startReadPressure(uint8_t samples);
	
        float finishReadTemperature();
        float finishReadPressure();

	uint16_t readPROM(uint8_t addr);

    private:
        uint8_t devAddr;
        uint8_t buffer[3];
	uint16_t coeff[7];
	int64_t SENS, OFF;
};

#endif /* _HMC5883L_H_ */
