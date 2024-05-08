#include <Arduino.h>
#include "sfe_bus.h"

SfeI2C::SfeI2C(void) : _i2cPort{nullptr}, _address{0}
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C init()
  //
  // Methods to init/setup this device.
  // The caller can provide a Wire Port, or this class will use the default.
  // Always update the address in case the user has changed the I2C address - see Example9
  bool SfeI2C::init(TwoWire &wirePort, uint8_t address, bool bInit)
  {
    // if we don't have a wire port already
    if (!_i2cPort)
    {
      _i2cPort = &wirePort;

      if (bInit)
        _i2cPort->begin();
    }

    _address = address;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C init()
  //
  // Methods to init/setup this device.
  // The caller can provide a Wire Port, or this class will use the default.
  bool SfeI2C::init(uint8_t address)
  {
    return init(Wire, address);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // ping()
  //
  // Is a device connected?
  bool SfeI2C::ping()
  {

    if (!_i2cPort)
      return false;

    _i2cPort->beginTransmission(_address);
    return _i2cPort->endTransmission() == 0;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()
  //
  // Checks how many bytes are waiting in the GNSS's I2C buffer
  // It does this by reading registers 0xFD and 0xFE
  //
  // From the u-blox integration manual:
  // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
  //  address and thus allows any register to be read. The second "current address" form omits the
  //  register address. If this second form is used, then an address pointer in the receiver is used to
  //  determine which register to read. This address pointer will increment after each read unless it
  //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
  //  unaltered."

  uint16_t SfeI2C::available()
  {

    if (!_i2cPort)
      return false;

    // Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(0xFD);                               // 0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    uint8_t i2cError = _i2cPort->endTransmission(false); // Always send a restart command. Do not release the bus. ESP32 supports this.
    if (i2cError != 0)
    {
      return (0); // Sensor did not ACK
    }

    // Forcing requestFrom to use a restart would be unwise. If bytesAvailable is zero, we want to surrender the bus.
    uint16_t bytesReturned = _i2cPort->requestFrom(_address, static_cast<uint8_t>(2));
    if (bytesReturned != 2)
    {
      return (0); // Sensor did not return 2 bytes
    }
    else // if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    return (bytesAvailable);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

  uint8_t SfeI2C::writeBytes(uint8_t *dataToWrite, uint8_t length)
  {
    if (!_i2cPort)
      return 0;

    if (length == 0)
      return 0;

    _i2cPort->beginTransmission(_address);
    uint8_t written = _i2cPort->write((const uint8_t *)dataToWrite, length);
    if (_i2cPort->endTransmission() == 0)
      return written;

    return 0;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

  uint8_t SfeI2C::readBytes(uint8_t *data, uint8_t length)
  {
    if (!_i2cPort)
      return 0;

    if (length == 0)
      return 0;

    uint8_t bytesReturned = _i2cPort->requestFrom(_address, length);

    for (uint8_t i = 0; i < bytesReturned; i++)
      *data++ = _i2cPort->read();

    return bytesReturned;
  }