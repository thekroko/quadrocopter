// I2Cdev library collection - MS5611 I2C device class

#include "MS5611.h"

/** Default constructor, uses default I2C address.
 * @see MS5611_DEFAULT_ADDRESS
 */
MS5611::MS5611() {
    devAddr = MS5611_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see MS5611_DEFAULT_ADDRESS
 * @see MS5611_ADDRESS
 */
MS5611::MS5611(uint8_t address) {
    devAddr = address;
}

/** Power on and prepare for general usage.
 * This will prepare the magnetometer with default settings.-
 */
void MS5611::initialize() {
    I2Cdev::writeByte(devAddr, MS5611_RA_RESET, MS5611_RA_NOP);
    delay(10);
    coeff[0] = readPROM(MS5611_PROM_ID);
    coeff[1] = readPROM(MS5611_PROM_C1);
    coeff[2] = readPROM(MS5611_PROM_C2);
    coeff[3] = readPROM(MS5611_PROM_C3);
    coeff[4] = readPROM(MS5611_PROM_C4);
    coeff[5] = readPROM(MS5611_PROM_C5);
    coeff[6] = readPROM(MS5611_PROM_C6);
    buffer[0] = 0;
    buffer[1] = 0;
    buffer[2] = 0;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MS5611::testConnection() {
  return (readPROM(MS5611_PROM_CRC) != 0);
}

/** Starts a temperature conversion with MS5611_SAMPLES_ samples */
#include <Wire.h>
void MS5611::startReadTemperature(uint8_t samples) {
  //I2Cdev::writeByte(devAddr, MS5611_RA_CONVERT_D2 | samples, MS5611_RA_NOP);
  Wire.beginTransmission(devAddr);
  Wire.write(MS5611_RA_CONVERT_D2 | samples);
  Wire.endTransmission();
}
/** Starts a pressure conversion with MS5611_SAMPLES_ samples */
void MS5611::startReadPressure(uint8_t samples) {
  //I2Cdev::writeByte(devAddr, MS5611_RA_CONVERT_D1 | samples, MS5611_RA_NOP);
  Wire.beginTransmission(devAddr);
  Wire.write(MS5611_RA_CONVERT_D1 | samples);
  Wire.endTransmission();
}

/** Finishes reading a temperature. Blocking. 100 = 1.0 degrees celsius */
float MS5611::finishReadTemperature(){
  int8_t cnt = I2Cdev::readBytes(devAddr, MS5611_RA_ADC,  3, &buffer[0]);
  if (cnt != 3) return -32767;
  uint32_t D2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
  int32_t dT = D2 - ((int32_t)coeff[5] << 8);
  int32_t TEMP = 2000 + ((dT * (int64_t)coeff[6]) >> 23);
  OFF  = ((int64_t)coeff[2] << 16) + ((dT * (int64_t)coeff[4]) >> 7);
  SENS = ((int64_t)coeff[1] << 15) + ((dT * (int64_t)coeff[3]) >> 8); 
  
  if (TEMP < 2000) { // if temperature lower than 20 Celsius
    int64_t T1 = (TEMP - 2000) * (TEMP - 2000);
    int64_t OFF1  = (5 * T1) >> 1;
    int64_t SENS1 = (5 * T1) >> 2;
 
    if(TEMP < -1500) { // if temperature lower than -15 Celsius
      T1 = (TEMP + 1500) * (TEMP + 1500);
      OFF1  += 7 * T1;
      SENS1 += (11 * T1) >> 1;
    } 
    OFF -= OFF1;
    SENS -= SENS1;
    TEMP -= ((int64_t)dT*dT) >> 31;
  }
  return (float)TEMP / 100.0f;
}
/** Finishes reading the pressure. Blocking. */
float MS5611::finishReadPressure(){
  int8_t cnt = I2Cdev::readBytes(devAddr, MS5611_RA_ADC,  3, &buffer[0]);
  if (cnt != 3) return -31267;
  uint32_t D1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2];
  return (((((int64_t)D1 * SENS ) >> 21) - OFF) >> 15) / 100.0f;
}

/** Reads a value from PROM */
uint16_t MS5611::readPROM(uint8_t addr) {
  uint16_t res = 0;
  I2Cdev::readWord(devAddr, MS5611_RA_READ_PROM | addr, &res);
  return res;
}
