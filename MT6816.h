#ifndef MT6816_H
#define MT6816_H

#include <Arduino.h>
#include <SPI.h>
#include "driver/gpio.h"  // For fast CS toggling

class MT6816 {
public:
  MT6816(uint8_t csPin, SPIClass& spiBus) {
    _csPin = csPin;
    _spi = &spiBus;
  }

  void begin() {
    // Set up the CS pin using fast ESP-IDF commands
    gpio_set_direction((gpio_num_t)_csPin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)_csPin, 1);
  }

  uint16_t readRawAngle() {
    _spi->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));
    gpio_set_level((gpio_num_t)_csPin, 0);

    _spi->transfer(0x83);
    uint8_t highByte = _spi->transfer(0x84);
    uint8_t lowByte = _spi->transfer(0x00);

    gpio_set_level((gpio_num_t)_csPin, 1);
    _spi->endTransaction();

    if (lowByte & 0x02) return 0xFFFF;  // no magnet
    return (highByte << 6) | (lowByte >> 2);
  }

  float readAngleDegrees() {
    uint16_t raw = readRawAngle();
    return (raw * 360.0f) / 16384.0f;
  }

private:
  uint8_t _csPin;
  SPIClass* _spi;
  static const uint32_t SPI_FREQ = 8000000;  // Bumped to 8 MHz!
};

#endif