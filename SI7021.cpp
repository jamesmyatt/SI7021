/*
Copyright 2014 Marcus Sorensen <marcus@electron14.com>

This program is licensed, please check with the copyright holder for terms

Updated: Jul 16, 2015: TomWS1: 
        eliminated Byte constants, 
        fixed 'sizeof' error in _command(), 
        added getTempAndRH() function to simplify calls for C & RH only
*/
#include "Arduino.h"
#include "SI7021.h"
#include <Wire.h>

#define I2C_ADDR 0x40

// I2C commands
#define RH_READ             0xE5 
#define TEMP_READ           0xE3 
#define POST_RH_TEMP_READ   0xE0 
#define RESET               0xFE 
#define USER1_READ          0xE7 
#define USER1_WRITE         0xE6 

// compound commands
byte SERIAL1_READ[]      ={ 0xFA, 0x0F };
byte SERIAL2_READ[]      ={ 0xFC, 0xC9 };

// Use integer operations for speed
#define CELSIUS_TO_FAHRENHEIT_HUNDRETHS(x) (9 * (long) x) / 5 + 3200

SI7021::SI7021() : _si_exists(false) {
}

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
bool SI7021::begin(int SDA, int SCL) {
    Wire.begin(SDA,SCL);
#else
bool SI7021::begin() {
    Wire.begin();
#endif
    Wire.beginTransmission(I2C_ADDR);
    if (Wire.endTransmission() == 0) {
        _si_exists = true;
    }
    return _si_exists;
}

bool SI7021::sensorExists() {
    return _si_exists;
}

int SI7021::getFahrenheitHundredths() {
    int c = getCelsiusHundredths();
    return CELSIUS_TO_FAHRENHEIT_HUNDRETHS(c);
}

int SI7021::getCelsiusHundredths() {
    return _getTemperature(false);
}

int SI7021::_getTemperature(bool fastConv) {
    long tempraw = _readMeasurement(fastConv ? POST_RH_TEMP_READ : TEMP_READ);
    return ((17572 * tempraw) >> 16) - 4685;
}

unsigned int SI7021::getHumidityPercent() {
    return getHumidityBasisPoints() / 100;
}

unsigned int SI7021::getHumidityBasisPoints() {
    long humraw = _readMeasurement(RH_READ);
    return ((12500 * humraw) >> 16) - 600;
}

uint16_t SI7021::_readMeasurement(uint8_t cmd) {
    // See page 16 of datasheet (Hold Master Mode)

    // Send command to device
    _writeReg(&cmd, sizeof cmd, false);  // No STOP

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
    // Delay for a while so WiFi can be serviced until measurement is guaranteed to be ready
    delay(25);
#endif

    // Read measurement from device
    if (Wire.requestFrom(I2C_ADDR, 2) < 2) {  // Read 2 bytes from I2C
        // Failed to read 2 bytes
        _si_exists = false;
        return 0;
}
    uint8_t msByte = Wire.read();
    uint8_t lsByte = Wire.read();
    uint16_t data = (uint16_t) msByte << 8 | lsByte;
    return data;
};

uint8_t SI7021::_writeReg(const uint8_t* reg, size_t reglen, uint8_t sendStop) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg, reglen);
    return Wire.endTransmission(sendStop);
}

uint8_t SI7021::_writeReg(const uint8_t* reg, size_t reglen) {
    return _writeReg(reg, reglen, true);
}

uint8_t SI7021::_readReg(uint8_t* reg, uint8_t reglen, uint8_t sendStop) {
    uint8_t read = Wire.requestFrom(I2C_ADDR, reglen, sendStop);
    for(uint8_t i = 0; i < reglen; i++) {
        reg[i] = Wire.read();
    }
    return read;
}

uint8_t SI7021::_readReg(uint8_t* reg, size_t reglen) {
    return _readReg(reg, reglen, true);
}

int SI7021::getSerialBytes(byte * buf) {
  byte serial[8];
  _writeReg(SERIAL1_READ, sizeof SERIAL1_READ);
  _readReg(serial, 8);
  
  //Page23 - https://www.silabs.com/Support%20Documents%2FTechnicalDocs%2FSi7021-A20.pdf
  buf[0] = serial[0]; //SNA_3
  buf[1] = serial[2]; //SNA_2
  buf[2] = serial[4]; //SNA_1
  buf[3] = serial[6]; //SNA_0

  _writeReg(SERIAL2_READ, sizeof SERIAL2_READ);
  _readReg(serial, 6);
  buf[4] = serial[0]; //SNB_3 - device ID byte
  buf[5] = serial[1]; //SNB_2
  buf[6] = serial[3]; //SNB_1
  buf[7] = serial[4]; //SNB_0
  return 1;
}

int SI7021::getDeviceId() {
  //0x0D=13=Si7013
  //0x14=20=Si7020
  //0x15=21=Si7021
  byte serial[8];
  getSerialBytes(serial);
  int id = serial[4];
  return id;
}

// 0x00 = 14 bit temp, 12 bit RH (default)
// 0x01 = 12 bit temp, 8 bit RH
// 0x80 = 13 bit temp, 10 bit RH
// 0x81 = 11 bit temp, 11 bit RH
void SI7021::setPrecision(byte setting) {
    // RES is bits 7 and 0 of user register 1
    
    // Read user register 1
    byte reg = USER1_READ;
    _writeReg(&reg, 1);
    _readReg(&reg, 1);

    // Update bits 7 and 0
    reg &= ~0x81;  // clear bits
    reg |= (setting & 0x81);  // set bits
    
    // Write user register 1
    byte userwrite[] = {USER1_WRITE, reg};
    _writeReg(userwrite, sizeof userwrite);
}

void SI7021::setHeater(bool on) {
    // HTRE is bit 2 of user register 1
    
    // Read user register 1
    byte reg = USER1_READ;
    _writeReg(&reg, 1);
    _readReg(&reg, 1);

    // Update bit 2 using Arduino function
    bitWrite(reg, 2, on);
    
    // Write user register 1
    byte userwrite[] = {USER1_WRITE, reg};
    _writeReg(userwrite, sizeof userwrite);
}

// get humidity, then get temperature reading from humidity measurement
struct si7021_env SI7021::getHumidityAndTemperature() {
    si7021_env ret;
    ret.humidityBasisPoints = getHumidityBasisPoints();
    ret.celsiusHundredths = _getTemperature(true);
    ret.fahrenheitHundredths = CELSIUS_TO_FAHRENHEIT_HUNDRETHS(ret.celsiusHundredths);
    return ret;
}

// get temperature (C only) and RH Percent
struct si7021_thc SI7021::getTempAndRH()
{
    si7021_thc ret;
    ret.humidityPercent = getHumidityPercent();
    ret.celsiusHundredths = _getTemperature(true);
    return ret;
}
