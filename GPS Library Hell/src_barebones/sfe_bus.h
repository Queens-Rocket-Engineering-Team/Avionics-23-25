#pragma once

#include <Arduino.h>
#include <Wire.h>


// The SfeI2C device defines behavior for I2C implementation based around the TwoWire class (Wire).
// This is Arduino specific.
class SfeI2C
{
public:
  SfeI2C(void);

  bool init(uint8_t address);

  bool init(TwoWire &wirePort, uint8_t address, bool bInit = false);

  // For I2C, ping the _address
  bool ping();

  // For I2C, read registers 0xFD and 0xFE. Return bytes available as uint16_t
  uint16_t available();

  // For I2C, push data to register 0xFF. Chunkify if necessary. Prevent single byte writes as these are illegal
  uint8_t writeBytes(uint8_t *data, uint8_t length) = 0;

  uint8_t readBytes(uint8_t *data, uint8_t length);

private:
  TwoWire *_i2cPort;
  uint8_t _address;
};