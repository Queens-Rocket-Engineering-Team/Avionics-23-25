#include <stdint.h>



/*
---------------------------------------- config keys ----------------------------------------
*/

// The following enum allows automatic identification of the Configuration Item data type.
// These are OR'd into the reserved bits in each Config Key ID.
// Based on an idea by Michael Ammann. Thank you @mazgch
const uint32_t UBX_CFG_U1 = 0x01002000;        // uint8_t
const uint32_t UBX_CFG_U2 = 0x01003000;        // uint16_t
const uint32_t UBX_CFG_E1 = 0x0100C000;        // 8-bit enum == uint8_t
const uint32_t UBX_CFG_SIZE_MASK = 0x0F00F000; // Bit mask

// CFG-RATE: Navigation and measurement rate configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_RATE_MEAS = UBX_CFG_U2 | 0x30210001;     // Nominal time between GNSS measurements
const uint32_t UBLOX_CFG_RATE_NAV = UBX_CFG_U2 | 0x30210002;      // Ratio of number of measurements to number of navigation solutions
const uint32_t UBLOX_CFG_RATE_TIMEREF = UBX_CFG_E1 | 0x20210003;  // Time system to which measurements are aligned
const uint32_t UBLOX_CFG_RATE_NAV_PRIO = UBX_CFG_U1 | 0x20210004; // Output rate of priority navigation mode messages

// These are the Bitfield layers definitions for the UBX-CFG-VALSET message (not to be confused with Bitfield deviceMask in UBX-CFG-CFG)
const uint8_t VAL_LAYER_DEFAULT = 0x7; // ONLY valid with getVal()
const uint8_t VAL_LAYER_RAM = (1 << 0);
const uint8_t VAL_LAYER_BBR = (1 << 1);
const uint8_t VAL_LAYER_FLASH = (1 << 2);
const uint8_t VAL_LAYER_RAM_BBR = VAL_LAYER_RAM | VAL_LAYER_BBR;               // Not valid with getVal()
const uint8_t VAL_LAYER_ALL = VAL_LAYER_RAM | VAL_LAYER_BBR | VAL_LAYER_FLASH; // Not valid with getVal()



/*
---------------------------------------- Class and ID --------------------------------------
*/

// The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
const uint8_t UBX_CLASS_CFG = 0x06;  // Configuration Input Messages: Configure the receiver.

// Class: CFG
// The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
const uint8_t UBX_CFG_VALSET = 0x8A;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.


/*
---------------------------------------- typedefs ----------------------------------------
*/


// A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
// If you know you are only going to be using I2C / Qwiic communication, you can
// safely reduce kUBLOXGNSSDefaultMaxWait to 250.
#ifndef kUBLOXGNSSDefaultMaxWait // Let's allow the user to define their own value if they want to
#define kUBLOXGNSSDefaultMaxWait 1100
#endif

// ubxPacket validity
typedef enum
{
  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
  SFE_UBLOX_PACKET_VALIDITY_VALID,
  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
  SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} sfe_ublox_packet_validity_e;

// Global Status Returns
typedef enum
{
  SFE_UBLOX_STATUS_SUCCESS,
  SFE_UBLOX_STATUS_FAIL,
  SFE_UBLOX_STATUS_CRC_FAIL,
  SFE_UBLOX_STATUS_TIMEOUT,
  SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
  SFE_UBLOX_STATUS_OUT_OF_RANGE,
  SFE_UBLOX_STATUS_INVALID_ARG,
  SFE_UBLOX_STATUS_INVALID_OPERATION,
  SFE_UBLOX_STATUS_MEM_ERR,
  SFE_UBLOX_STATUS_HW_ERR,
  SFE_UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
  SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
  SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
  SFE_UBLOX_STATUS_SPI_COMM_FAILURE,
  SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} sfe_ublox_status_e;


//-=-=-=-=- UBX binary specific variables
struct ubxPacket
{
  uint8_t cls;
  uint8_t id;
  uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
  uint16_t counter;      // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  uint16_t startingSpot; // The counter value needed to go past before we begin recording into payload array
  uint8_t *payload;      // We will allocate RAM for the payload if/when needed.
  uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
  uint8_t checksumB;
  sfe_ublox_packet_validity_e valid;           // Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};