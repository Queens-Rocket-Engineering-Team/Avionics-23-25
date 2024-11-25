/*
 * Pin definitions for the Kuhglocke ground station (Rev 1.0)
 */

#define GPS_TX_PIN 20
#define GPS_RX_PIN 19
#define GPS_RESET_PIN 8

#define MENU_BTNS_PIN 1
#define ARGB_DATA_PIN 46
#define DB_LED_PIN ARGB_DATA_PIN //same pin as RGB

#define CHRG_STAT_PIN 2
#define DISABLE_5V_PIN 3

#define RF_DIO2_PIN 4
#define RF_DIO0_PIN 5
#define RF_DIO1_PIN 6
#define RF_RESET_PIN 7
#define RF_MOSI_PIN 17
#define RF_MISO_PIN 18
#define RF_CS_PIN 15
#define RF_SCK_PIN 16

#define EINK_RESET_PIN 9
#define EINK_DC_PIN 10
#define EINK_CS_PIN 11
#define EINK_SCK_PIN 12
#define EINK_MOSI_PIN 13
#define EINK_MISO_PIN 42
#define EINK_BUSY_PIN 40

#define SPK_I2S_LRCLK_PIN 14
#define SPK_I2S_DIN_PIN 21
#define SPK_I2S_BCLK_PIN 38
#define SPK_ON_PIN 41

#define SDMMC_CMD_PIN 35
#define SDMMC_CLK_PIN 36
#define SDMMC_D0_PIN 37

#define I2C_SCL_PIN 47
#define I2C_SDA_PIN 48
#define NAU7802_ADDR 0x2A
#define P3T1755_ADDR 0x48
