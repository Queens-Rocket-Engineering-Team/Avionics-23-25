
#pragma once

// These are the Bitfield layers definitions for the UBX-CFG-VALSET message (not to be confused with Bitfield deviceMask in UBX-CFG-CFG)
const uint8_t VAL_LAYER_DEFAULT = 0x7; // ONLY valid with getVal()
const uint8_t VAL_LAYER_RAM = (1 << 0);
const uint8_t VAL_LAYER_BBR = (1 << 1);
const uint8_t VAL_LAYER_FLASH = (1 << 2);

// These are the Bitfield layers definitions for the UBX-CFG-VALSET message (not to be confused with Bitfield deviceMask in UBX-CFG-CFG)
const uint8_t VAL_LAYER_RAM_BBR = VAL_LAYER_RAM | VAL_LAYER_BBR;               // Not valid with getVal()

// The following enum allows automatic identification of the Configuration Item data type.
// These are OR'd into the reserved bits in each Config Key ID.
// Based on an idea by Michael Ammann. Thank you @mazgch
const uint32_t UBX_CFG_U2 = 0x01003000;        // uint16_t
const uint32_t UBX_CFG_SIZE_MASK = 0x0F00F000; // Bit mask

// CFG-RATE: Navigation and measurement rate configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_RATE_MEAS = UBX_CFG_U2 | 0x30210001;     // Nominal time between GNSS measurements
const uint32_t UBLOX_CFG_RATE_NAV = UBX_CFG_U2 | 0x30210002;      // Ratio of number of measurements to number of navigation solutions