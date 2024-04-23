#pragma once

#include <Arduino.h>

#include <Wire.h>

#include "u-blox_GNSS.h"
#include "u-blox_external_typedefs.h"
#include "sfe_bus.h"


class SFE_UBLOX_GNSS : public DevUBLOXGNSS
{
public:
  SFE_UBLOX_GNSS() { _commType = COMM_TYPE_I2C; }

  ///////////////////////////////////////////////////////////////////////
  // begin()
  //
  // This method is called to initialize the SFE_UBLOX_GNSS library and connect to
  // the GNSS device. This method must be called before calling any other method
  // that interacts with the device.
  //
  // Begin will then return true if "signs of life" have been seen: reception of _any_ valid UBX packet or _any_ valid NMEA header.
  //
  // This method follows the standard startup pattern in SparkFun Arduino
  // libraries.
  //
  //  Parameter   Description
  //  ---------   ----------------------------
  //  wirePort    optional. The Wire port. If not provided, the default port is used
  //  address     optional. I2C Address. If not provided, the default address is used.
  //  retval      true on success, false on startup failure
  //
  // This methond is overridden, implementing two versions.
  //
  // Version 1:
  // User skips passing in an I2C object which then defaults to Wire.
  bool begin(uint8_t deviceAddress = kUBLOXGNSSDefaultAddress, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    // Setup  I2C object and pass into the superclass
    setCommunicationBus(_i2cBus);

    // Initialize the I2C buss class i.e. setup default Wire port
    _i2cBus.init(deviceAddress);

    // Initialize the system - return results
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }

  // Version 2:
  //  User passes in an I2C object and an address (optional).
  bool begin(TwoWire &wirePort, uint8_t deviceAddress = kUBLOXGNSSDefaultAddress, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    // Setup  I2C object and pass into the superclass
    setCommunicationBus(_i2cBus);

    // Give the I2C port provided by the user to the I2C bus class.
    _i2cBus.init(wirePort, deviceAddress);

    // Initialize the system - return results
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
  }

private:
  // I2C bus class
  SfeI2C _i2cBus;
};