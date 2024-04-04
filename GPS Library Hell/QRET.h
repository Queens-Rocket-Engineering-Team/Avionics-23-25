#include <QRET_typedefs.h>

class DevUBLOXGNSS
{

public:
  bool setNavigationFrequency(uint8_t navFreq, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the number of nav solutions sent per second
  bool setMeasurementRate(uint16_t rate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);       // Set the elapsed time between GNSS measurements in milliseconds, which defines the rate
  bool setNavigationRate(uint16_t rate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);        // Set the ratio between the number of measurements and the number of navigation solutions. Unit is cycles. Max is 127
  
  
  bool setVal16(uint32_t key, uint16_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 16-bit value at a given group/id/size location
  bool setValN(uint32_t key, uint8_t *value, uint8_t N, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Sets the N-byte value at a given group/id/size location

  // Send I2C/Serial/SPI commands to the module
  void calcChecksum(ubxPacket *msg);          
  sfe_ublox_status_e sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool expectACKonly = false); // Given a packet and payload, send everything including CRC bytes, return true if we got a response

protected:
  // Limit checking of new data to every X ms
  // If we are expecting an update every X Hz then we should check every quarter that amount of time
  // Otherwise we may block ourselves from seeing new data
  int8_t i2cPollingWait = 100;    // Default to 100ms. Adjusted when user calls setNavigationFrequency() or setHNRNavigationRate() or setMeasurementRate()
  uint8_t i2cPollingWaitNAV = 100; // We need to record the desired polling rate for standard nav messages
  uint8_t i2cPollingWaitHNR = 100; // and for HNR too so we can set i2cPollingWait to the lower of the two
  uint8_t *payloadCfg = nullptr;
  ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  // These lock / unlock functions can be used if you have multiple tasks writing to the bus.
  // The idea is that in a RTOS you override this class and the functions in which you take and give a mutex.
  virtual bool createLock(void) { return true; }
  virtual bool lock(void) { return true; }
  virtual void unlock(void) {}
  virtual void deleteLock(void) {}

  bool _printDebug = false;                      // Flag to print the serial commands we are sending to the Serial port for debug
  SparkFun_UBLOX_GNSS::SfePrint _debugSerial;    // The stream to send debug messages to if enabled
};