#pragma once

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_PVT_data_t data;
  UBX_NAV_PVT_moduleQueried_t moduleQueried;
  void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *);
  UBX_NAV_PVT_data_t *callbackData;
} UBX_NAV_PVT_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t ecefX : 1;
      uint32_t ecefY : 1;
      uint32_t ecefZ : 1;
      uint32_t pAcc : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_POSECEF_moduleQueried_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t year : 1;
      uint32_t month : 1;
      uint32_t day : 1;
      uint32_t hour : 1;
      uint32_t min : 1;
      uint32_t sec : 1;

      uint32_t validDate : 1;
      uint32_t validTime : 1;
      uint32_t fullyResolved : 1;
      uint32_t validMag : 1;

      uint32_t tAcc : 1;
      uint32_t nano : 1;
      uint32_t fixType : 1;
      uint32_t gnssFixOK : 1;
      uint32_t diffSoln : 1;
      uint32_t psmState : 1;
      uint32_t headVehValid : 1;
      uint32_t carrSoln : 1;

      uint32_t confirmedAvai : 1;
      uint32_t confirmedDate : 1;
      uint32_t confirmedTime : 1;

      uint32_t numSV : 1;
      uint32_t lon : 1;
      uint32_t lat : 1;
      uint32_t height : 1;
      uint32_t hMSL : 1;
      uint32_t hAcc : 1;
      uint32_t vAcc : 1;
      uint32_t velN : 1;
      uint32_t velE : 1;
    } bits;
  } moduleQueried1;
  union
  {
    uint32_t all;
    struct
    {
      uint32_t velD : 1;
      uint32_t gSpeed : 1;
      uint32_t headMot : 1;
      uint32_t sAcc : 1;
      uint32_t headAcc : 1;
      uint32_t pDOP : 1;

      uint32_t invalidLlh : 1;

      uint32_t headVeh : 1;
      uint32_t magDec : 1;
      uint32_t magAcc : 1;
    } bits;
  } moduleQueried2;
} UBX_NAV_PVT_moduleQueried_t;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the HNR epoch: ms
  uint16_t year; // Year (UTC)
  uint8_t month; // Month, range 1..12 (UTC)
  uint8_t day;   // Day of month, range 1..31 (UTC)
  uint8_t hour;  // Hour of day, range 0..23 (UTC)
  uint8_t min;   // Minute of hour, range 0..59 (UTC)
  uint8_t sec;   // Seconds of minute, range 0..60 (UTC)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t validDate : 1;     // 1 = Valid UTC Date
      uint8_t validTime : 1;     // 1 = Valid UTC Time of Day
      uint8_t fullyResolved : 1; // 1 = UTC Time of Day has been fully resolved
    } bits;
  } valid;
  int32_t nano;   // Fraction of second (UTC): ns
  uint8_t gpsFix; // GPSfix Type, range 0..5
                  // 0x00 = No Fix
                  // 0x01 = Dead Reckoning only
                  // 0x02 = 2D-Fix
                  // 0x03 = 3D-Fix
                  // 0x04 = GPS + dead reckoning combined
                  // 0x05 = Time only fix
                  // 0x06..0xff: reserved
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gpsFixOK : 1;     // >1 = Fix within limits (e.g. DOP & accuracy)
      uint8_t diffSoln : 1;     // 1 = DGPS used
      uint8_t WKNSET : 1;       // 1 = Valid GPS week number
      uint8_t TOWSET : 1;       // 1 = Valid GPS time of week (iTOW & fTOW)
      uint8_t headVehValid : 1; // 1= Heading of vehicle is valid
    } bits;
  } flags;
  uint8_t reserved1[2];
  int32_t lon;      // Longitude: Degrees * 1e-7
  int32_t lat;      // Latitude: Degrees * 1e-7
  int32_t height;   // Height above ellipsoid: mm
  int32_t hMSL;     // Height above MSL: mm
  int32_t gSpeed;   // Ground Speed (2-D): mm/s
  int32_t speed;    // Speed (3-D): mm/s
  int32_t headMot;  // Heading of motion (2-D): Degrees * 1e-5
  int32_t headVeh;  // Heading of vehicle (2-D): Degrees * 1e-5
  uint32_t hAcc;    // Horizontal accuracy: mm
  uint32_t vAcc;    // Vertical accuracy: mm
  uint32_t sAcc;    // Speed accuracy: mm/s
  uint32_t headAcc; // Heading accuracy: Degrees * 1e-5
  uint8_t reserved2[4];
} UBX_HNR_PVT_data_t;
  // Additional flags and pointers that need to be stored with each message type
  
struct ubxAutomaticFlags
{
  union
  {
    uint8_t all;
    struct
    {
      uint8_t automatic : 1;         // Will this message be delivered and parsed "automatically" (without polling)
      uint8_t implicitUpdate : 1;    // Is the update triggered by accessing stale data (=true) or by a call to checkUblox (=false)
      uint8_t addToFileBuffer : 1;   // Should the raw UBX data be added to the file buffer?
      uint8_t callbackCopyValid : 1; // Is the copy of the data struct used by the callback valid/fresh?
    } bits;
  } flags;
};

// Additional flags and pointers that need to be stored with each message type