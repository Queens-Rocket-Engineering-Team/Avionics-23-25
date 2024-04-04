#include <QRET.h>

bool DevUBLOXGNSS::setNavigationFrequency(uint8_t navFreq, uint8_t layer, uint16_t maxWait)
{
    if (navFreq == 0) // Return now if navFreq is zero
        return (false);

    if (navFreq > 40)
        navFreq = 40; // Limit navFreq to 40Hz so i2cPollingWait is set correctly

    // Adjust the I2C polling timeout based on update rate
    // Do this even if the sendCommand fails
    i2cPollingWaitNAV = 1000 / (((int)navFreq) * 4);                                                // This is the number of ms to wait between checks for new I2C data. Max is 250. Min is 6.
    i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

    uint16_t measurementRate = 1000 / navFreq;

    return setVal16(UBLOX_CFG_RATE_MEAS, measurementRate, layer, maxWait);
}

// Set the elapsed time between GNSS measurements in milliseconds, which defines the rate
bool DevUBLOXGNSS::setMeasurementRate(uint16_t rate, uint8_t layer, uint16_t maxWait)
{
  if (rate < 25) // "Measurement rate should be greater than or equal to 25 ms."
    rate = 25;

  // Adjust the I2C polling timeout based on update rate
  if (rate >= 1000)
    i2cPollingWaitNAV = 250;
  else
    i2cPollingWaitNAV = rate / 4;                                                                 // This is the number of ms to wait between checks for new I2C data
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

  return setVal16(UBLOX_CFG_RATE_MEAS, rate, layer, maxWait);
}

bool DevUBLOXGNSS::setNavigationRate(uint16_t rate, uint8_t layer, uint16_t maxWait)
{
  return setVal16(UBLOX_CFG_RATE_NAV, rate, layer, maxWait);
}

// Given a key, set a 16-bit value
// This function takes a full 32-bit key
// Default layer is RAM+BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::setVal16(uint32_t key, uint16_t value, uint8_t layer, uint16_t maxWait)
{
  uint8_t val[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};
  return (setValN(key, val, 2, layer, maxWait));
}

// Given a key, set a N-byte value
// This function takes a full 32-bit key
// Default layer is RAM+BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::setValN(uint32_t key, uint8_t *value, uint8_t N, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + N; // 4 byte header, 4 byte key ID, N bytes of value
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
  for (uint8_t i = 0; i < 4; i++)
    payloadCfg[i + 4] = key >> (8 * i); // Key

  // Load user's value
  for (uint8_t i = 0; i < N; i++)
    payloadCfg[i + 8] = *value++;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e DevUBLOXGNSS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly)
{
  if (!lock())
    return SFE_UBLOX_STATUS_FAIL;

  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial.print(F("\nSending: "));
    printPacket(outgoingUBX, true); // Always print payload
  }
#endif

  if (_commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX);
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial.println(F("Send I2C Command failed"));
      }
#endif
      unlock();
      return retVal;
    }
  }
  else if (_commType == COMM_TYPE_SERIAL)
  {
    sendSerialCommand(outgoingUBX);
  }
  else if (_commType == COMM_TYPE_SPI)
  {
    sendSpiCommand(outgoingUBX);
  }

  unlock();

  if (maxWait > 0)
  {
    // Depending on what we just sent, either we need to look for an ACK or not
    if ((outgoingUBX->cls == UBX_CLASS_CFG) || (expectACKonly == true))
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial.println(F("sendCommand: Waiting for ACK response"));
      }
#endif
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
    else
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial.println(F("sendCommand: Waiting for No ACK response"));
      }
#endif
      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
  }
  else
  {
    processSpiBuffer(&packetCfg, 0, 0); // Process any SPI data received during the sendSpiCommand - but only if not checking for a response
  }

  return retVal;
}

// Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
// This is called before we send a command message
void DevUBLOXGNSS::calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}