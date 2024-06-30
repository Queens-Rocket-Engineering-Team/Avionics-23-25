// CANPackets.h

/*
 * Header file containing standardized CANBUS packet ID details.
 * 
 * The four MSB of a packet ID corresponds to the origin device.
 * The remaining 7 LSB correspond to the packet type. In practice
 * the four MSB can be mostly ignored, and are there to prevent
 * collisions if two modules send the same packet type.
 *
 * MSB can be isolated using (ID & 0x780)
 * LSB can be isolated using (ID & 0x07F)
*/


// Origin Module IDs (4 MSB of a packet ID)
#define COMM_MOD_CANID 0x10
#define GPS_MOD_CANID 0x20
#define ALTIMETER_MOD_CANID 0x30
#define SENSOR_MOD_CANID 0x40


// Packet Type IDs (7 LSB of a packet ID)
#define STATUS_CANID 0x01           // Len=1, status of a module
#define BATT_A_VOLT_CANID 0x02      // Len=2, primary battery voltage in mV, (uint16_t)
#define BATT_B_VOLT_CANID 0x03      // Len=2, secondary battery voltage in mV, (uint16_t)

#define GPS_NUMSAT_CANID 0x04       // Len=1, number of GPS satellites locked
#define GPS_LAT_CANID 0x05          // Len=8
#define GPS_LON_CANID 0x06          // Len=8

#define FLIGHT_STAGE_CANID 0x07     // Len=1, current flight stage
#define ALTITUDE_CANID 0x08         // Len=4, current altitude, uint32_t (Pa)
#define TEMP_CANID 0x09             // Len=4, current temperature, uint32_t (millicelcius)
#define MATCH_STATUS_CANID 0x0A     // Len=1, e-match status (2x continuity + 2x fired state)
#define ACCELERATIOM_CANID 0x0B     // Len=6, 3x uint16_t
