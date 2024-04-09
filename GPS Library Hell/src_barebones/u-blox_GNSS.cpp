#include <Arduino.h>
#include "u-blox_GNSS.h"

DevUBLOXGNSS::DevUBLOXGNSS(void)
{
  // Constructor
  if (debugPin >= 0)
  {
    pinMode((uint8_t)debugPin, OUTPUT);
    digitalWrite((uint8_t)debugPin, HIGH);
  }

  // _logNMEA.all = 0;                             // Default to passing no NMEA messages to the file buffer
  // _processNMEA.all = SFE_UBLOX_FILTER_NMEA_ALL; // Default to passing all NMEA messages to processNMEA
  // _logRTCM.all = 0;                             // Default to passing no RTCM messages to the file buffer

// #ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
//   rtcmInputStorage.init();
// #endif
}

DevUBLOXGNSS::~DevUBLOXGNSS(void)
{
  // Destructor

  end(); // Delete all allocated memory - excluding payloadCfg, payloadAuto and spiBuffer

  if (payloadCfg != nullptr)
  {
    delete[] payloadCfg; // Created with new[]
    payloadCfg = nullptr;
  }

  if (payloadAuto != nullptr)
  {
    delete[] payloadAuto; // Created with new[]
    payloadAuto = nullptr;
  }

  if (spiBuffer != nullptr)
  {
    delete[] spiBuffer; // Created with new[]
    spiBuffer = nullptr;
  }
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

// Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e DevUBLOXGNSS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly)
{
  if (!lock())
    return SFE_UBLOX_STATUS_FAIL;

  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet

// #ifndef SFE_UBLOX_REDUCED_PROG_MEM
//   if (_printDebug == true)
//   {
//     _debugSerial.print(F("\nSending: "));
//     printPacket(outgoingUBX, true); // Always print payload
//   }
// #endif

  if (_commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX); //// NEED TO GET sendI2cCommand FUNCTION STILL
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
// #ifndef SFE_UBLOX_REDUCED_PROG_MEM
//       if (_printDebug == true)
//       {
//         _debugSerial.println(F("Send I2C Command failed"));
//       }
// #endif
      unlock();
      return retVal;
    }
  }


  unlock();

  if (maxWait > 0)
  {
    // Depending on what we just sent, either we need to look for an ACK or not
    if ((outgoingUBX->cls == UBX_CLASS_CFG) || (expectACKonly == true))
    {
// #ifndef SFE_UBLOX_REDUCED_PROG_MEM
//       if (_printDebug == true)
//       {
//         _debugSerial.println(F("sendCommand: Waiting for ACK response"));
//       }
// #endif
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
    else
    {
// #ifndef SFE_UBLOX_REDUCED_PROG_MEM
//       if (_printDebug == true)
//       {
//         _debugSerial.println(F("sendCommand: Waiting for No ACK response"));
//       }
// #endif
      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
  }
  else
  {
    processSpiBuffer(&packetCfg, 0, 0); // Process any SPI data received during the sendSpiCommand - but only if not checking for a response
  }

  return retVal;
}

bool DevUBLOXGNSS::setMeasurementRate(uint16_t rate, uint8_t layer, uint16_t maxWait){
  if(rate < 25){
    rate = 25; // rate must be at least 25 ms
  }
  
  if(rate >= 1000){
    i2cPollingWaitNAV = 250;
  }
  else {
    i2cPollingWaitNAV = rate / 4;
  }

  // Set i2cPollingWait to the lower of NAV and HNR
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; 

  return setVal16(UBLOX_CFG_RATE_MEAS, rate, layer, maxWait);
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

// Given a key, set a 16-bit value
// This function takes a full 32-bit key
// Default layer is RAM+BBR
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool DevUBLOXGNSS::setVal16(uint32_t key, uint16_t value, uint8_t layer, uint16_t maxWait)
{
  uint8_t val[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};
  return (setValN(key, val, 2, layer, maxWait));
}

// Get the current latitude in degrees
// Returns a long representing the number of degrees *10^7
int32_t DevUBLOXGNSS::getLatitude(uin16_t maxWait) {
  if(packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;
  
  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.numSV == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.numSV = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.numSV);

}

// PRIVATE: Allocate RAM for packetUBXNAVPVT and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVPVT()
{
  packetUBXNAVPVT = new UBX_NAV_PVT_t; // Allocate RAM for the main struct
  if (packetUBXNAVPVT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial.println(F("initPacketUBXNAVPVT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVPVT->automaticFlags.flags.all = 0;
  packetUBXNAVPVT->callbackPointerPtr = nullptr;
  packetUBXNAVPVT->callbackData = nullptr;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0;
  packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
  return (true);
}

// ***** PVT automatic support

// Get the latest Position/Velocity/Time solution and fill all global variables
bool DevUBLOXGNSS::getPVT(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->automaticFlags.flags.bits.automatic && packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all;
  }
  else if (packetUBXNAVPVT->automaticFlags.flags.bits.automatic && !packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;
    // packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+packetCfgPayloadSize = 84 bytes Note:now hard-coded in processUBX

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
      return (true);

    return (false);
  }
}

