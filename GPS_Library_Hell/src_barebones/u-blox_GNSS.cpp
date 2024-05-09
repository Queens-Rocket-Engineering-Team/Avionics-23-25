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

}

// Stop all automatic message processing. Free all used RAM
void DevUBLOXGNSS::end(void)
{
  // Note: payloadCfg is not deleted

  // Note: payloadAuto is not deleted

  // Note: spiBuffer is not deleted

  if (ubxFileBuffer != nullptr) // Check if RAM has been allocated for the file buffer
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial.println(F("end: the file buffer has been deleted. You will need to call setFileBufferSize before .begin to create a new one."));
    }
#endif
    delete[] ubxFileBuffer; // Created with new[]
    ubxFileBuffer = nullptr;
    fileBufferSize = 0; // Reset file buffer size. User will have to call setFileBufferSize again
    fileBufferMaxAvail = 0;
  }

  if (rtcmBuffer != nullptr) // Check if RAM has been allocated for the RTCM buffer
  {
    delete[] rtcmBuffer; // Created with new[]
    rtcmBuffer = nullptr;
    rtcmBufferSize = 0; // Reset file buffer size. User will have to call setFileBufferSize again
  }

  if (cfgValgetValueSizes != nullptr)
  {
    delete[] cfgValgetValueSizes;
    cfgValgetValueSizes = nullptr;
  }

  if (moduleSWVersion != nullptr)
  {
    delete moduleSWVersion; // Created with new moduleSWVersion_t
    moduleSWVersion = nullptr;
  }

  if (currentGeofenceParams != nullptr)
  {
    delete currentGeofenceParams; // Created with new geofenceParams_t
    currentGeofenceParams = nullptr;
  }

  if (packetUBXNAVTIMELS != nullptr)
  {
    delete packetUBXNAVTIMELS; // Created with new UBX_NAV_TIMELS_t
    packetUBXNAVTIMELS = nullptr;
  }

  if (packetUBXNAVPOSECEF != nullptr)
  {
    if (packetUBXNAVPOSECEF->callbackData != nullptr)
    {
      delete packetUBXNAVPOSECEF->callbackData; // Created with new UBX_NAV_POSECEF_data_t
    }
    delete packetUBXNAVPOSECEF; // Created with new UBX_NAV_POSECEF_t
    packetUBXNAVPOSECEF = nullptr;
  }

  if (packetUBXNAVSTATUS != nullptr)
  {
    if (packetUBXNAVSTATUS->callbackData != nullptr)
    {
      delete packetUBXNAVSTATUS->callbackData;
    }
    delete packetUBXNAVSTATUS;
    packetUBXNAVSTATUS = nullptr;
  }

  if (packetUBXNAVDOP != nullptr)
  {
    if (packetUBXNAVDOP->callbackData != nullptr)
    {
      delete packetUBXNAVDOP->callbackData;
    }
    delete packetUBXNAVDOP;
    packetUBXNAVDOP = nullptr;
  }

  if (packetUBXNAVPVT != nullptr)
  {
    if (packetUBXNAVPVT->callbackData != nullptr)
    {
      delete packetUBXNAVPVT->callbackData;
    }
    delete packetUBXNAVPVT;
    packetUBXNAVPVT = nullptr;
  }

  if (packetUBXNAVATT != nullptr)
  {
    if (packetUBXNAVATT->callbackData != nullptr)
    {
      delete packetUBXNAVATT->callbackData;
    }
    delete packetUBXNAVATT;
    packetUBXNAVATT = nullptr;
  }

  if (packetUBXNAVODO != nullptr)
  {
    if (packetUBXNAVODO->callbackData != nullptr)
    {
      delete packetUBXNAVODO->callbackData;
    }
    delete packetUBXNAVODO;
    packetUBXNAVODO = nullptr;
  }

  if (packetUBXNAVVELECEF != nullptr)
  {
    if (packetUBXNAVVELECEF->callbackData != nullptr)
    {
      delete packetUBXNAVVELECEF->callbackData;
    }
    delete packetUBXNAVVELECEF;
    packetUBXNAVVELECEF = nullptr;
  }

  if (packetUBXNAVVELNED != nullptr)
  {
    if (packetUBXNAVVELNED->callbackData != nullptr)
    {
      delete packetUBXNAVVELNED->callbackData;
    }
    delete packetUBXNAVVELNED;
    packetUBXNAVVELNED = nullptr;
  }

  if (packetUBXNAVHPPOSECEF != nullptr)
  {
    if (packetUBXNAVHPPOSECEF->callbackData != nullptr)
    {
      delete packetUBXNAVHPPOSECEF->callbackData;
    }
    delete packetUBXNAVHPPOSECEF;
    packetUBXNAVHPPOSECEF = nullptr;
  }

  if (packetUBXNAVHPPOSLLH != nullptr)
  {
    if (packetUBXNAVHPPOSLLH->callbackData != nullptr)
    {
      delete packetUBXNAVHPPOSLLH->callbackData;
    }
    delete packetUBXNAVHPPOSLLH;
    packetUBXNAVHPPOSLLH = nullptr;
  }

  if (packetUBXNAVPVAT != nullptr)
  {
    if (packetUBXNAVPVAT->callbackData != nullptr)
    {
      delete packetUBXNAVPVAT->callbackData;
    }
    delete packetUBXNAVPVAT;
    packetUBXNAVPVAT = nullptr;
  }

  if (packetUBXNAVTIMEUTC != nullptr)
  {
    if (packetUBXNAVTIMEUTC->callbackData != nullptr)
    {
      delete packetUBXNAVTIMEUTC->callbackData;
    }
    delete packetUBXNAVTIMEUTC;
    packetUBXNAVTIMEUTC = nullptr;
  }

  if (packetUBXNAVCLOCK != nullptr)
  {
    if (packetUBXNAVCLOCK->callbackData != nullptr)
    {
      delete packetUBXNAVCLOCK->callbackData;
    }
    delete packetUBXNAVCLOCK;
    packetUBXNAVCLOCK = nullptr;
  }

  if (packetUBXNAVSVIN != nullptr)
  {
    if (packetUBXNAVSVIN->callbackData != nullptr)
    {
      delete packetUBXNAVSVIN->callbackData;
    }
    delete packetUBXNAVSVIN;
    packetUBXNAVSVIN = nullptr;
  }

  if (packetUBXNAVRELPOSNED != nullptr)
  {
    if (packetUBXNAVRELPOSNED->callbackData != nullptr)
    {
      delete packetUBXNAVRELPOSNED->callbackData;
    }
    delete packetUBXNAVRELPOSNED;
    packetUBXNAVRELPOSNED = nullptr;
  }

  if (packetUBXNAVAOPSTATUS != nullptr)
  {
    if (packetUBXNAVAOPSTATUS->callbackData != nullptr)
    {
      delete packetUBXNAVAOPSTATUS->callbackData;
    }
    delete packetUBXNAVAOPSTATUS;
    packetUBXNAVAOPSTATUS = nullptr;
  }

  if (packetUBXNAVEOE != nullptr)
  {
    if (packetUBXNAVEOE->callbackData != nullptr)
    {
      delete packetUBXNAVEOE->callbackData;
    }
    delete packetUBXNAVEOE;
    packetUBXNAVEOE = nullptr;
  }

#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  if (packetUBXNAVSAT != nullptr)
  {
    if (packetUBXNAVSAT->callbackData != nullptr)
    {
      delete packetUBXNAVSAT->callbackData;
    }
    delete packetUBXNAVSAT;
    packetUBXNAVSAT = nullptr;
  }

  if (packetUBXNAVSIG != nullptr)
  {
    if (packetUBXNAVSIG->callbackData != nullptr)
    {
      delete packetUBXNAVSIG->callbackData;
    }
    delete packetUBXNAVSIG;
    packetUBXNAVSIG = nullptr;
  }

  if (packetUBXRXMPMP != nullptr)
  {
    if (packetUBXRXMPMP->callbackData != nullptr)
    {
      delete packetUBXRXMPMP->callbackData;
    }
    delete packetUBXRXMPMP;
    packetUBXRXMPMP = nullptr;
  }

  if (packetUBXRXMPMPmessage != nullptr)
  {
    if (packetUBXRXMPMPmessage->callbackData != nullptr)
    {
      delete packetUBXRXMPMPmessage->callbackData;
    }
    delete packetUBXRXMPMPmessage;
    packetUBXRXMPMPmessage = nullptr;
  }

  if (packetUBXRXMQZSSL6message != nullptr)
  {
    if (packetUBXRXMQZSSL6message->callbackData != nullptr)
    {
      delete[] packetUBXRXMQZSSL6message->callbackData;
    }
    delete packetUBXRXMQZSSL6message;
    packetUBXRXMQZSSL6message = nullptr;
  }

  if (packetUBXRXMCOR != nullptr)
  {
    if (packetUBXRXMCOR->callbackData != nullptr)
    {
      delete packetUBXRXMCOR->callbackData;
    }
    delete packetUBXRXMCOR;
    packetUBXRXMCOR = nullptr;
  }

  if (packetUBXRXMSFRBX != nullptr)
  {
    if (packetUBXRXMSFRBX->callbackData != nullptr)
    {
      delete packetUBXRXMSFRBX->callbackData;
    }
    if (packetUBXRXMSFRBX->callbackMessageData != nullptr)
    {
      delete[] packetUBXRXMSFRBX->callbackMessageData;
    }
    delete packetUBXRXMSFRBX;
    packetUBXRXMSFRBX = nullptr;
  }

  if (packetUBXRXMRAWX != nullptr)
  {
    if (packetUBXRXMRAWX->callbackData != nullptr)
    {
      delete packetUBXRXMRAWX->callbackData;
    }
    delete packetUBXRXMRAWX;
    packetUBXRXMRAWX = nullptr;
  }

  if (packetUBXRXMMEASX != nullptr)
  {
    if (packetUBXRXMMEASX->callbackData != nullptr)
    {
      delete packetUBXRXMMEASX->callbackData;
    }
    delete packetUBXRXMMEASX;
    packetUBXRXMMEASX = nullptr;
  }
#endif

  if (packetUBXTIMTM2 != nullptr)
  {
    if (packetUBXTIMTM2->callbackData != nullptr)
    {
      delete packetUBXTIMTM2->callbackData;
    }
    delete packetUBXTIMTM2;
    packetUBXTIMTM2 = nullptr;
  }

  if (packetUBXTIMTP != nullptr)
  {
    if (packetUBXTIMTP->callbackData != nullptr)
    {
      delete packetUBXTIMTP->callbackData;
    }
    delete packetUBXTIMTP;
    packetUBXTIMTP = nullptr;
  }

  if (packetUBXMONHW != nullptr)
  {
    if (packetUBXMONHW->callbackData != nullptr)
    {
      delete packetUBXMONHW->callbackData;
    }
    delete packetUBXMONHW;
    packetUBXMONHW = nullptr;
  }

#ifndef SFE_UBLOX_DISABLE_ESF
  if (packetUBXESFALG != nullptr)
  {
    if (packetUBXESFALG->callbackData != nullptr)
    {
      delete packetUBXESFALG->callbackData;
    }
    delete packetUBXESFALG;
    packetUBXESFALG = nullptr;
  }

  if (packetUBXESFSTATUS != nullptr)
  {
    if (packetUBXESFSTATUS->callbackData != nullptr)
    {
      delete packetUBXESFSTATUS->callbackData;
    }
    delete packetUBXESFSTATUS;
    packetUBXESFSTATUS = nullptr;
  }

  if (packetUBXESFINS != nullptr)
  {
    if (packetUBXESFINS->callbackData != nullptr)
    {
      delete packetUBXESFINS->callbackData;
    }
    delete packetUBXESFINS;
    packetUBXESFINS = nullptr;
  }

  if (packetUBXESFMEAS != nullptr)
  {
    if (packetUBXESFMEAS->callbackData != nullptr)
    {
      delete[] packetUBXESFMEAS->callbackData;
    }
    delete packetUBXESFMEAS;
    packetUBXESFMEAS = nullptr;
  }

  if (packetUBXESFRAW != nullptr)
  {
    if (packetUBXESFRAW->callbackData != nullptr)
    {
      delete packetUBXESFRAW->callbackData;
    }
    delete packetUBXESFRAW;
    packetUBXESFRAW = nullptr;
  }
#endif

  if (packetUBXMGAACK != nullptr)
  {
    delete packetUBXMGAACK;
    packetUBXMGAACK = nullptr;
  }

  if (packetUBXMGADBD != nullptr)
  {
    delete packetUBXMGADBD;
    packetUBXMGADBD = nullptr;
  }

#ifndef SFE_UBLOX_DISABLE_HNR
  if (packetUBXHNRATT != nullptr)
  {
    if (packetUBXHNRATT->callbackData != nullptr)
    {
      delete packetUBXHNRATT->callbackData;
    }
    delete packetUBXHNRATT;
    packetUBXHNRATT = nullptr;
  }

  if (packetUBXHNRINS != nullptr)
  {
    if (packetUBXHNRINS->callbackData != nullptr)
    {
      delete packetUBXHNRINS->callbackData;
    }
    delete packetUBXHNRINS;
    packetUBXHNRINS = nullptr;
  }

  if (packetUBXHNRPVT != nullptr)
  {
    if (packetUBXHNRPVT->callbackData != nullptr)
    {
      delete packetUBXHNRPVT->callbackData;
    }
    delete packetUBXHNRPVT;
    packetUBXHNRPVT = nullptr;
  }
#endif

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  if (storageNMEAGPGGA != nullptr)
  {
    if (storageNMEAGPGGA->callbackCopy != nullptr)
    {
      delete storageNMEAGPGGA->callbackCopy;
    }
    delete storageNMEAGPGGA;
    storageNMEAGPGGA = nullptr;
  }

  if (storageNMEAGNGGA != nullptr)
  {
    if (storageNMEAGNGGA->callbackCopy != nullptr)
    {
      delete storageNMEAGNGGA->callbackCopy;
    }
    delete storageNMEAGNGGA;
    storageNMEAGNGGA = nullptr;
  }

  if (storageNMEAGPVTG != nullptr)
  {
    if (storageNMEAGPVTG->callbackCopy != nullptr)
    {
      delete storageNMEAGPVTG->callbackCopy;
    }
    delete storageNMEAGPVTG;
    storageNMEAGPVTG = nullptr;
  }

  if (storageNMEAGNVTG != nullptr)
  {
    if (storageNMEAGNVTG->callbackCopy != nullptr)
    {
      delete storageNMEAGNVTG->callbackCopy;
    }
    delete storageNMEAGNVTG;
    storageNMEAGNVTG = nullptr;
  }

  if (storageNMEAGPRMC != nullptr)
  {
    if (storageNMEAGPRMC->callbackCopy != nullptr)
    {
      delete storageNMEAGPRMC->callbackCopy;
    }
    delete storageNMEAGPRMC;
    storageNMEAGPRMC = nullptr;
  }

  if (storageNMEAGNRMC != nullptr)
  {
    if (storageNMEAGNRMC->callbackCopy != nullptr)
    {
      delete storageNMEAGNRMC->callbackCopy;
    }
    delete storageNMEAGNRMC;
    storageNMEAGNRMC = nullptr;
  }

  if (storageNMEAGPZDA != nullptr)
  {
    if (storageNMEAGPZDA->callbackCopy != nullptr)
    {
      delete storageNMEAGPZDA->callbackCopy;
    }
    delete storageNMEAGPZDA;
    storageNMEAGPZDA = nullptr;
  }

  if (storageNMEAGNZDA != nullptr)
  {
    if (storageNMEAGNZDA->callbackCopy != nullptr)
    {
      delete storageNMEAGNZDA->callbackCopy;
    }
    delete storageNMEAGNZDA;
    storageNMEAGNZDA = nullptr;
  }
#endif

  if (_storageNMEA != nullptr)
  {
    if (_storageNMEA->data != nullptr)
    {
      delete[] _storageNMEA->data;
    }
    delete _storageNMEA;
    _storageNMEA = nullptr;
  }

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  if (_storageRTCM != nullptr)
  {
    delete _storageRTCM;
    _storageRTCM = nullptr;
  }
#endif

  if (storageRTCM1005 != nullptr)
  {
    if (storageRTCM1005->callbackData != nullptr)
    {
      delete storageRTCM1005->callbackData;
    }
    delete storageRTCM1005;
    storageRTCM1005 = nullptr;
  }

  if (sfe_ublox_ubx_logging_list_head != nullptr)
  {
    while (sfe_ublox_ubx_logging_list_head->next != nullptr)
    {
      // Step through the list, find the tail
      sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_ptr_previous = sfe_ublox_ubx_logging_list_head;
      sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_head->next;
      while (sfe_ublox_ubx_logging_list_ptr->next != nullptr)
      {
        sfe_ublox_ubx_logging_list_ptr_previous = sfe_ublox_ubx_logging_list_ptr;
        sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_ptr->next;
      }
      // Delete the tail
      delete sfe_ublox_ubx_logging_list_ptr;
      sfe_ublox_ubx_logging_list_ptr_previous->next = nullptr;
    }
    // Finally, delete the head
    delete sfe_ublox_ubx_logging_list_head;
    sfe_ublox_ubx_logging_list_head = nullptr;
  }

  deleteLock(); // Delete the lock semaphore - if required
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

// Configure a port to output UBX, NMEA, RTCM3 or a combination thereof
bool DevUBLOXGNSS::setI2COutput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_I2COUTPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_I2COUTPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_I2COUTPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support RTCM3
  return result;
}

// Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e DevUBLOXGNSS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly)
{
  if (!lock())
    return SFE_UBLOX_STATUS_FAIL;

  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    Serial.print(F("\nSending: "));
    // printPacket(outgoingUBX, true); // Always print payload
#endif

  if (_commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX); //// NEED TO GET sendI2cCommand FUNCTION STILL
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    Serial.println(F("Send I2C Command failed"));
#endif
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
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      Serial.println(F("sendCommand: Waiting for ACK response"));
#endif
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
    else
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      Serial.println(F("sendCommand: Waiting for No ACK response"));
#endif
      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
  }

  return retVal;
}

// When messages from the class CFG are sent to the receiver, the receiver will send an "acknowledge"(UBX - ACK - ACK) or a
//"not acknowledge"(UBX-ACK-NAK) message back to the sender, depending on whether or not the message was processed correctly.
// Some messages from other classes also use the same acknowledgement mechanism.

// When we poll or get a setting, we will receive _both_ a config packet and an ACK
// If the poll or get request is not valid, we will receive _only_ a NACK

// If we are trying to get or poll a setting, then packetCfg.len will be 0 or 1 when the packetCfg is _sent_.
// If we poll the setting for a particular port using UBX-CFG-PRT then .len will be 1 initially
// For all other gets or polls, .len will be 0 initially
//(It would be possible for .len to be 2 _if_ we were using UBX-CFG-MSG to poll the settings for a particular message - but we don't use that (currently))

// If the get or poll _fails_, i.e. is NACK'd, then packetCfg.len could still be 0 or 1 after the NACK is received
// But if the get or poll is ACK'd, then packetCfg.len will have been updated by the incoming data and will always be at least 2

// If we are going to set the value for a setting, then packetCfg.len will be at least 3 when the packetCfg is _sent_.
//(UBX-CFG-MSG appears to have the shortest set length of 3 bytes)

// We need to think carefully about how interleaved PVT packets affect things.
// It is entirely possible that our packetCfg and packetAck were received successfully
// but while we are still in the "if (checkUblox() == true)" loop a PVT packet is processed
// or _starts_ to arrive (remember that Serial data can arrive very slowly).

// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got an ACK and a valid packetCfg (module is responding with register content)
// Returns SFE_UBLOX_STATUS_DATA_SENT if we got an ACK and no packetCfg (no valid packetCfg needed, module absorbs new register data)
// Returns SFE_UBLOX_STATUS_FAIL if something very bad happens (e.g. a double checksum failure)
// Returns SFE_UBLOX_STATUS_COMMAND_NACK if the packet was not-acknowledged (NACK)
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we had a checksum failure
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an ACK and a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e DevUBLOXGNSS::waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() < (startTime + (unsigned long)maxTime))
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {
      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("waitForACKResponse: valid data and valid ACK received after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
      }

      // We can be confident that the data packet (if we are going to get one) will always arrive
      // before the matching ACK. So if we sent a config packet which only produces an ACK
      // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
      // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
      // as these may have been changed by an automatic packet.
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("waitForACKResponse: no data and valid ACK after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
      }

      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by an automatic packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("waitForACKResponse: data being OVERWRITTEN after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
      // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
      else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("waitForACKResponse: CRC failed after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
      }

      // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
      // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
      // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
      // but outgoingUBX->cls and outgoingUBX->id would not match...
      // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
      // the packet was definitely NACK'd otherwise we are possibly just guessing...
      // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
      else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
      // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
      // If we were playing safe, we should return FAIL instead
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("waitForACKResponse: VALID data and INVALID ACK received after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
      // then we return a FAIL. This must be a double checksum failure?
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("waitForACKResponse: INVALID data and INVALID ACK received after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
      // then the ACK has not yet been received and we should keep waiting for it
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
      {
        //   Serial.print(F("waitForACKResponse: valid data after "));
        //   Serial.print(millis() - startTime);
        //   Serial.println(F(" msec. Waiting for ACK."));
      }

    } // checkUbloxInternal == true

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }           // while (millis() < (startTime + (unsigned long)maxTime))

  // We have timed out...
  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
  // even though we did not get an ACK
  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      Serial.print(F("waitForACKResponse: TIMEOUT with valid data after "));
      Serial.print(millis() - startTime);
      Serial.println(F(" msec. "));
#endif
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But no ACK!
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    Serial.print(F("waitForACKResponse: TIMEOUT after "));
    Serial.print(millis() - startTime);
    Serial.println(F(" msec."));
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

// For non-CFG queries no ACK is sent so we use this function
// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e DevUBLOXGNSS::waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {
      // If outgoingUBX->classAndIDmatch is VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          Serial.print(F("waitForNoACKResponse: valid data with CLS/ID match after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data!
      }

      // If the outgoingUBX->classAndIDmatch is VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("waitForNoACKResponse: data being OVERWRITTEN after "));
        Serial.print(millis() - startTime);
        Serial.println(F(" msec"));
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If outgoingUBX->classAndIDmatch is NOT_DEFINED
      // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        // if (_printDebug == true)
        // {
        //   Serial.print(F("waitForNoACKResponse: valid but UNWANTED data after "));
        //   Serial.print(millis() - startTime);
        //   Serial.print(F(" msec. Class: 0x"));
        //   Serial.print(outgoingUBX->cls, HEX);
        //   Serial.print(F(" ID: 0x"));
        //   Serial.print(outgoingUBX->id, HEX);
        // }
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
      else if (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
      {
        return (SFE_UBLOX_STATUS_CRC_FAIL); // We received invalid data
      }
    }

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    Serial.print(F("waitForNoACKResponse: TIMEOUT after "));
    Serial.print(millis() - startTime);
    Serial.println(F(" msec. No packet received."));
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

// PRIVATE: Called regularly to check for available bytes on the user' specified port
bool DevUBLOXGNSS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (!lock())
    return false;

  bool ok = false;
  ok = checkUbloxI2C(incomingUBX, requestedClass, requestedID);
  unlock();

  return ok;
}

// Polls I2C for data, passing any new bytes to process()
// Returns true if new bytes are available
bool DevUBLOXGNSS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (millis() - lastCheck >= i2cPollingWait)
  {
    // Get the number of bytes available from the module
    // From the u-blox integration manual:
    // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
    //  address and thus allows any register to be read. The second "current address" form omits the
    //  register address. If this second form is used, then an address pointer in the receiver is used to
    //  determine which register to read. This address pointer will increment after each read unless it
    //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
    //  unaltered."
    uint16_t bytesAvailable = available();

    if (bytesAvailable == 0)
    {
  // #ifndef SFE_UBLOX_REDUCED_PROG_MEM
  //     Serial.println(F("checkUbloxI2C: OK, zero bytes available"));
  // #endif
      lastCheck = millis(); // Put off checking to avoid I2C bus traffic
      return (false);
    }

    // Check for undocumented bit error. We found this doing logic scans.
    // This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
    // then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15))
    {
      // Clear the MSbit
      bytesAvailable &= ~((uint16_t)1 << 15);
    }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      Serial.print(F("checkUbloxI2C: "));
      Serial.print(bytesAvailable);
      Serial.println(F(" bytes available"));
#endif

    while (bytesAvailable)
    {
      // Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable; // 16-bit
      if (bytesToRead > i2cTransactionSize)  // Limit for i2cTransactionSize is 8-bit
        bytesToRead = i2cTransactionSize;

      // Here it would be desireable to use a restart where possible / supported, but only if there will be multiple reads.
      // However, if an individual requestFrom fails, we could end up leaving the bus hanging.
      // On balance, it is probably safest to not use restarts here.
      uint8_t buf[i2cTransactionSize];
      uint8_t bytesReturned = readBytes(buf, (uint8_t)bytesToRead);
      if ((uint16_t)bytesReturned == bytesToRead)
      {
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          process(buf[x], incomingUBX, requestedClass, requestedID); // Process this valid character
        }
      }
      else
      {
        // Something has gone very wrong. Sensor did not respond - or a bus error happened...
        if (_resetCurrentSentenceOnBusError)
          currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking for a new start char
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.println(F("checkUbloxI2C: bus error? bytesReturned != bytesToRead"));
#endif
        return (false);
      }

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} // end checkUbloxI2C()

// Return the ratio between the number of measurements and the number of navigation solutions. Unit is cycles
uint16_t DevUBLOXGNSS::getNavigationRate(uint8_t layer, uint16_t maxWait)
{
  uint16_t navigationRate;

  getNavigationRate(&navigationRate, layer, maxWait);

  return navigationRate;
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

// Get the current longitude in degrees
// Returns a long representing the number of degrees *10^-7
int32_t DevUBLOXGNSS::getLongitude(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lon == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lon = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.lon);
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

// Get the current altitude in mm according to ellipsoid model
int32_t DevUBLOXGNSS::getAltitude(uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.height == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.height = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.height);
}

// PRIVATE: Allocate RAM for packetUBXNAVPVT and initialize it
bool DevUBLOXGNSS::initPacketUBXNAVPVT()
{
  packetUBXNAVPVT = new UBX_NAV_PVT_t; // Allocate RAM for the main struct
  if (packetUBXNAVPVT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    Serial.println(F("initPacketUBXNAVPVT: RAM alloc failed!"));
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