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

// ----------------------------------------------------------------------------------------
// SDCard Structure.
// ----------------------------------------------------------------------------------------
#define MAX_CARD_TYPES 5
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
// -------------------------------------------------------------------------------------------
// SDMMC Flag Structure.
// -------------------------------------------------------------------------------------------
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
// -------------------------------------------------------------------------------------------
// SDMMC Arg Structure.
// -------------------------------------------------------------------------------------------
struct sdmmcArgStruct {
  bool recursive;
  int maxlevels;
  char buffer[MAX_GLOBAL_SERIAL_BUFFER_SIZE];
};
extern struct sdmmcArgStruct sdmmcArgData;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
// ----------------------------------
// Defaults are for ESP32-P4.
// ----------------------------------
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
// ----------------------------------
void sdcardBegin(
  bool test_rw=SD_TEST_RW, bool set_pins=SD_SET_PINS, bool bit1_mode=SD_1BITMODE,
  bool format_on_fail=SD_FORMAT_ON_FAIL, long freq=SD_FREQ, signed int clk=SD_CLK,
  signed int cmd=SD_CMD, signed int d0=SD_D0, signed int d1=SD_D1, signed int d2=SD_D2,
  signed int d3=SD_D3, signed int pwr=SD_PWR, int max_open_files=SD_MAX_OPEN_FILES);
void mountSD(
  bool tmp_mount=false, bool test_rw=SD_TEST_RW, bool set_pins=SD_SET_PINS,
  bool bit1_mode=SD_1BITMODE, bool format_on_fail=SD_FORMAT_ON_FAIL, long freq=SD_FREQ,
  signed int clk=SD_CLK, signed int cmd=SD_CMD, signed int d0=SD_D0, signed int d1=SD_D1,
  signed int d2=SD_D2, signed int d3=SD_D3, signed int pwr=SD_PWR,int max_open_files=SD_MAX_OPEN_FILES);
void sdcardEnd(signed int pwr=SD_PWR);
void sdcardSleepMode0(signed int clk=SD_CLK, signed int cmd=SD_CMD, signed int d0=SD_D0, signed int d1=SD_D1,
  signed int d2=SD_D2, signed int d3=SD_D3, signed int pwr=SD_PWR);
void cycleSDPower(signed int pwr=SD_PWR);
void sdPowerON(signed int pwr=SD_PWR);
void sdPowerOFF(signed int pwr=SD_PWR);
void sdcardFlagHandler(
  bool test_rw=SD_TEST_RW, bool set_pins=SD_SET_PINS, bool bit1_mode=SD_1BITMODE,
  bool format_on_fail=SD_FORMAT_ON_FAIL, long freq=SD_FREQ, signed int clk=SD_CLK,
  signed int cmd=SD_CMD, signed int d0=SD_D0, signed int d1=SD_D1, signed int d2=SD_D2,
  signed int d3=SD_D3, signed int pwr=SD_PWR, int max_open_files=SD_MAX_OPEN_FILES);
void clearSDCardStruct();
void getMountPoint();
void getSDCardType();
void getCardSize();
void getTotalBytes();
void getUsedBytes();
void getSectorSize();
void statSDCard();
void statSDCardPins();

#endif