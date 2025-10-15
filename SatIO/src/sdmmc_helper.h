/*
  SDCARD - Written By Benjamin Jack Cullen.
  Requirements:
    (1) Modified SD_MMC library to release LDO on end().
    (2) Optional: Modified ffconf library for exFat support (FAT only by default).
  
  Steps for intended use (looping on a task):
  (1) Firstly, in loop call sdcardBegin() to initialize, card detect, automatically mount/unmount.
  (2) Secondly, in loop call sdcardFlagHandler() to actually utilize the card.
  (3) Optionally (lastly) in loop call sdcardSleepMode0() to save power during intervals.

  - sdcardEnd() is handled internally when required in various scenarios.

*/

#ifndef SDMMC_HELPER_H
#define SDMMC_HELPER_H

#include <stdint.h>
#include "sd_protocol_types.h"
#include "driver/sdmmc_types.h"
#include "SD_MMC.h"
#include "driver/sdmmc_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "config.h"

#define MAX_CARD_TYPES 5
/**
 * Defaults are for ESP32-P4.
 */
#define SD_CLK 43
#define SD_CMD 44
#define SD_D0  39
#define SD_D1  40
#define SD_D2  42
#define SD_D3  43
#define SD_PWR 45
#define SD_TEST_RW false
#define SD_SET_PINS false
#define SD_1BITMODE false
#define SD_FORMAT_ON_FAIL false
#define SD_FREQ BOARD_MAX_SDMMC_FREQ
#define SD_MAX_OPEN_FILES 5

/**
 * @struct SDCardStruct
 */
struct SDCardStruct {
    bool sdcard_inserted;
    bool allow_mount;
    bool sdcard_mounted;
    char sdcard_mountpoint[56];
    uint8_t sdcard_type;
    char sdcard_type_names[MAX_CARD_TYPES][12];
    uint64_t sdcard_card_size;
    uint64_t sdcard_total_bytes;
    uint64_t sdcard_used_bytes;
    int sdcard_sector_size;
    sdmmc_card_t *card;
    sdmmc_host_t host;
    sdmmc_slot_config_t slot_config;
    sd_pwr_ctrl_ldo_config_t ldo_config;
};
extern struct SDCardStruct sdcardData;

/**
 * @struct sdmmcFlagStruct
 */
struct sdmmcFlagStruct {
  bool no_delay_flag;
  bool mount_sdcard_flag;
  bool unmount_sdcard_flag;
  bool list_dir_flag;
  bool save_mapping;
  bool load_mapping;
  bool delete_mapping;
  bool save_matrix;
  bool load_matrix;
  bool delete_matrix;
  bool save_system;
  bool load_system;
  bool delete_system;
};
extern struct sdmmcFlagStruct sdmmcFlagData;

/**
 * @struct sdmmcArgStruct
 */
struct sdmmcArgStruct {
  bool recursive;
  int maxlevels;
  char buffer[MAX_GLOBAL_SERIAL_BUFFER_SIZE];
};
extern struct sdmmcArgStruct sdmmcArgData;


/**
 * SDCard begin is a wrapper for SDMMC.begin().
 * This should always be the 1st function called.
 * 
 * @brief Designed to be called repeatedly to:
 *  (1) Check card insertion
 *  (2) Mount automatically
 *  (3) Unmount automatically
 *  (4) Populate/clear card information according to mounted/unmounted
 * 
 * @param test_rw Attempt write/read test file for debug purposes only
 * @param set_pins If true then custom pins can be set
 * @param bit1_mode If true then 1 bit mode will be enabled
 * @param format_on_fail Automatically attempt to format sdcard if initialization failed
 * @param freq Card reader frequency
 * @param clk Custom clock pin
 * @param cmd Custom command pin
 * @param d0 Custom data 0 pin
 * @param d1 Custom data 1 pin
 * @param d2 Custom data 2 pin
 * @param d3 Custom data 3 pin
 * @param pwr Custom power pin
 * @param max_open_files Maximum number of open files
 * @return Returns None
 */
void sdcardBegin(
  bool test_rw=SD_TEST_RW, bool set_pins=SD_SET_PINS, bool bit1_mode=SD_1BITMODE,
  bool format_on_fail=SD_FORMAT_ON_FAIL, long freq=SD_FREQ, signed int clk=SD_CLK,
  signed int cmd=SD_CMD, signed int d0=SD_D0, signed int d1=SD_D1, signed int d2=SD_D2,
  signed int d3=SD_D3, signed int pwr=SD_PWR, int max_open_files=SD_MAX_OPEN_FILES);

/**
 * SDCard Flag Handler.
 * This should always be the 2nd function called.
 * Call sdcardBegin() before calling this function.
 * Checks sdmmcFlagData() flags for instructions.
 * 
 * @brief Designed to be called repeatedly to check operation flags.
 * 
 * @param test_rw Attempt write/read test file for debug purposes only
 * @param set_pins If true then custom pins can be set
 * @param bit1_mode If true then 1 bit mode will be enabled
 * @param format_on_fail Automatically attempt to format sdcard if initialization failed
 * @param freq Card reader frequency
 * @param clk Custom clock pin
 * @param cmd Custom command pin
 * @param d0 Custom data 0 pin
 * @param d1 Custom data 1 pin
 * @param d2 Custom data 2 pin
 * @param d3 Custom data 3 pin
 * @param pwr Custom power pin
 * @param max_open_files Maximum number of open files
 * * @return Returns None
 */
void sdcardFlagHandler(
  bool test_rw=SD_TEST_RW, bool set_pins=SD_SET_PINS, bool bit1_mode=SD_1BITMODE,
  bool format_on_fail=SD_FORMAT_ON_FAIL, long freq=SD_FREQ, signed int clk=SD_CLK,
  signed int cmd=SD_CMD, signed int d0=SD_D0, signed int d1=SD_D1, signed int d2=SD_D2,
  signed int d3=SD_D3, signed int pwr=SD_PWR, int max_open_files=SD_MAX_OPEN_FILES);

/**
 * Sleep mode 0.
 * This should always be the last function called.
 * 
 * @brief Power down the card after card has been utilized.
 * 
 * @param clk Custom clock pin
 * @param cmd Custom command pin
 * @param d0 Custom data 0 pin
 * @param d1 Custom data 1 pin
 * @param d2 Custom data 2 pin
 * @param d3 Custom data 3 pin
 * @param pwr Custom power pin
 * @return Returns None
 */
void sdcardSleepMode0(signed int clk=SD_CLK, signed int cmd=SD_CMD, signed int d0=SD_D0,
  signed int d1=SD_D1, signed int d2=SD_D2, signed int d3=SD_D3, signed int pwr=SD_PWR);


/**
 * List Dir.
 * 
 * @brief Lists files and directories.
 * 
 * @param fs Filesysetm. Example: SD_MMC
 * @param dirname Specify directory to list
 * @param maxLevels Recursion depth
 * @param currentLevel Set automatically. Default 0.
 * @return Returns None
 */
void listDir(FS &fs, const char *dirname, signed int maxLevels=-1, int currentLevel=0);

/**
 * Print SDCard pin states.
 * 
 * @brief Used for debugging purposes only, such as testing CD pin.
 * @return Returns None
 */
void statSDCardPins();

#endif