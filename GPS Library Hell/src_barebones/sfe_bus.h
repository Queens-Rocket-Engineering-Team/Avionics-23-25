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

    bool ping();

    uint16_t available();

    uint8_t writeBytes(uint8_t *data, uint8_t length);

    uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length)
    { (void)data; (void)readData; (void)length; return 0; }

    void startWriteReadByte(){};
    void writeReadByte(const uint8_t *data, uint8_t *readData){ (void)data; (void)readData; }
    void writeReadByte(const uint8_t data, uint8_t *readData){ (void)data; (void)readData; }
    void endWriteReadByte(){};

    uint8_t readBytes(uint8_t *data, uint8_t length);

  private:
    TwoWire *_i2cPort;
    uint8_t _address;
};