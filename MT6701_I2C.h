#ifndef MT6701_I2C_H
#define MT6701_I2C_H

#include <Arduino.h>
#include <Wire.h>

class MT6701_I2C {
public:
    MT6701_I2C(TwoWire& wireBus = Wire, uint8_t i2cAddr = 0x06) {
        _wire = &wireBus;
        _i2cAddr = i2cAddr;
    }

    void begin() {
        // Wire.begin() is handled in main setup
    }

    uint16_t readRawAngle() {
        _wire->beginTransmission(_i2cAddr);
        _wire->write(0x03); 
        if (_wire->endTransmission(false) != 0) {
            return 0; 
        }

        _wire->requestFrom((int)_i2cAddr, (int)2);
        
        if (_wire->available() >= 2) {
            uint8_t highByte = _wire->read();
            uint8_t lowByte = _wire->read();
            return (highByte << 6) | (lowByte >> 2);
        }
        
        return 0; 
    }

    float readAngleDegrees() {
        uint16_t raw = readRawAngle();
        return (raw * 360.0f) / 16384.0f;
    }

private:
    TwoWire* _wire;
    uint8_t _i2cAddr;
};

#endif