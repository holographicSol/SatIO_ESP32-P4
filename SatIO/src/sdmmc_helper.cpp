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

#include "sdmmc_helper.h"
#include <Arduino.h>
#include <FS.h>
#include "SD_MMC.h"
#include "SPIFFS.h"
#include "satio_file.h"
#include "matrix.h"
#include "custommapping.h"

// -------------------------------------------------------------------------------------------
// SDCard Structure.
// -------------------------------------------------------------------------------------------
struct SDCardStruct sdcardData = {
    .sdcard_inserted=false,
    .allow_mount=true,
    .sdcard_mounted=false,
    .sdcard_mountpoint={0},
    .sdcard_type=CARD_NONE,
    .sdcard_type_names = {
        "None", "MMC", "SD", "SDHC/SDXC", "Unknown"
    },
    .sdcard_card_size=0,
    .sdcard_total_bytes=0,
    .sdcard_used_bytes=0,
    .sdcard_sector_size=0,
    .card=NULL,
    .host={},
    .slot_config={},
    .ldo_config={},
};
// -------------------------------------------------------------------------------------------
// SDMMC Flag Structure.
// -------------------------------------------------------------------------------------------
struct sdmmcFlagStruct sdmmcFlagData = {
  .no_delay_flag=false,
  .mount_sdcard_flag=false,
  .unmount_sdcard_flag=false,
  .list_dir_flag=false,
  .save_mapping=false,
  .load_mapping=false,
  .delete_mapping=false,
  .save_matrix=false,
  .load_matrix=false,
  .delete_matrix=false,
  .save_system=false,
  .load_system=false,
  .delete_system=false,
};
// -------------------------------------------------------------------------------------------
// SDMMC Arg Structure.
// -------------------------------------------------------------------------------------------
struct sdmmcArgStruct sdmmcArgData = {
  .recursive=false,
  .maxlevels=0,
  .buffer={0},
};
// -------------------------------------------------------------------------------------------
// Clear SDMMC Arg Structure.
// -------------------------------------------------------------------------------------------
void clearSDMMCArgStruct() {
  sdmmcArgData.recursive=false;
  sdmmcArgData.maxlevels=0;
  memset(sdmmcArgData.buffer, 0, sizeof(sdmmcArgData.buffer)); 
}
// -------------------------------------------------------------------------------------------
// Clear SDCard Structure.
// -------------------------------------------------------------------------------------------
void clearSDCardStruct() {
  // sdcardData.sdcard_inserted=false, // allow persitance
  // sdcardData.allow_mount=true, // allow persitance
  // sdcardData.unmount_sdcard_flag=false, // allow persitance
  // sdcardData.mount_sdcard_flag=false, // allow persitance
  // sdcardData.sdcard_mounted=false; // allow persitance
  memset(sdcardData.sdcard_mountpoint, 0, sizeof(sdcardData.sdcard_mountpoint));
  sdcardData.sdcard_type=CARD_NONE;
  sdcardData.sdcard_card_size=0;
  sdcardData.sdcard_total_bytes=0;
  sdcardData.sdcard_used_bytes=0;
  sdcardData.sdcard_sector_size=0;
  // sdcardData.card=NULL, // set according to sd_mmc
  // sdcardData.host={}, // set according to sd_mmc
  // sdcardData.slot_config={}, // set according to sd_mmc
  // sdcardData.ldo_config={}, // set according to sd_mmc
}
// -------------------------------------------------------------------------------------------
// Power Cycling.
// -------------------------------------------------------------------------------------------
void sdPowerOFF(signed int pwr) {
  pinMode(pwr, OUTPUT);
  digitalWrite(pwr, LOW);
  delay(10);
}
void sdPowerON(signed int pwr) {
  pinMode(pwr, OUTPUT);
  digitalWrite(pwr, HIGH);
  delay(10);
}
void cycleSDPower(signed int pwr) {
  sdPowerOFF(pwr);
  sdPowerON(pwr);
}
// -------------------------------------------------------------------------------------------
// Sleep Mode 0: Turns off everything.
// -------------------------------------------------------------------------------------------
void sdcardSleepMode0(signed int clk, signed int cmd, signed int d0, signed int d1,
    signed int d2, signed int d3, signed int pwr) {
  pinMode(pwr, OUTPUT);
  digitalWrite(pwr, LOW);
  pinMode(clk, OUTPUT);
  pinMode(cmd, OUTPUT);
  pinMode(d0, OUTPUT);
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(d3, OUTPUT); // chip detect
  digitalWrite(clk, LOW);
  digitalWrite(cmd, LOW);
  digitalWrite(d0, LOW);
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  digitalWrite(d3, LOW); // chip detect
  delay(10);
}
// -------------------------------------------------------------------------------------------
// Test R/W (Debug).
// -------------------------------------------------------------------------------------------
bool testWrite(FS &fs, const char* path) {
    File f = fs.open(path, "w", true); if (!f) return false; f.print("test");
    f.close(); return fs.exists(path);}
bool testRead(FS &fs, const char* path) {
    File f = fs.open(path, "r"); if (!f) return false; String s = f.readString();
    f.close(); fs.remove(path); return s == "test";}
void testRW() {
  if (testWrite(SD_MMC, "/EXTERNAL/test.txt") && testRead(SD_MMC, "/EXTERNAL/test.txt")) {
    Serial.println("[sdmmc] FS read/write OK");}
  else {Serial.println("[sdmmc] FS test failed");}}
// -------------------------------------------------------------------------------------------
// Card Check.
// -------------------------------------------------------------------------------------------
void checkCardPointer() {
  sdcardData.card=SD_MMC.returnCard(); // pull the card out of sd_mmc to see its populated.
  sdcardData.host=SD_MMC.returnHost();
  sdcardData.slot_config=SD_MMC.returnSlotConfig();
  sdcardData.ldo_config=SD_MMC.returnLDOConfig();
  if (sdcardData.card!=NULL) {sdcardData.sdcard_mounted=true;}
  else {sdcardData.sdcard_mounted=false;}
}
// -------------------------------------------------------------------------------------------
// Mount.
// -------------------------------------------------------------------------------------------
// Provides functionality for various mounting scenarios and is customizable.
// -------------------------------------------------------------------------------------------
void mountSD(bool tmp_mount, bool test_rw, bool set_pins, bool bit1_mode, bool format_on_fail,
    long freq, signed int clk, signed int cmd, signed int d0, signed int d1, signed int d2,
    signed int d3, signed int pwr, int max_open_files) {
  // -------------------------------------
  // Custum pin configuration.
  // -------------------------------------
  if (set_pins==true) {
    if(SD_MMC.setPins(clk, cmd, d0, d1, d2, d3)) {Serial.println("[sdmmc] pins set successfully.");}
    else {Serial.println("[sdmmc] error setting pins.");}
    delay(10);
  }
  // -------------------------------------
  // SD_MMC begin will set data pins high.
  // -------------------------------------
  if (SD_MMC.begin("/EXTERNAL", bit1_mode, format_on_fail, freq)) {
    if (tmp_mount==true) {} // do not override allow_mount on successfull mount.
    else {sdcardData.allow_mount=true;} // override allow_mount on successfull mount.
    if (test_rw==true) {testRW();} // verify read/write.  
  }
  delay(10);
}
// -------------------------------------------------------------------------------------------
// SDCard End (Unmount Macro).
// -------------------------------------------------------------------------------------------
void sdcardEnd(signed int pwr) {
  // --------------------------------------------------------------------------------------
  // SD_MMC end will not set sd data pins low, currently power is cycled to refresh states.
  // --------------------------------------------------------------------------------------
  SD_MMC.end(); // unmount card & release LDO via pwr_ctrl_handle so the card can be remounted later.
  // clearSDCardStruct();
  sdcardSleepMode0(pwr); // power down the slot.
}
// -------------------------------------------------------------------------------------------
// SDCard Begin (Mount Macro for various mounting scenarios & various card insertion scenarios).
// -------------------------------------------------------------------------------------------
// Mounts/Unmounts automatically when card inserted/removed and provides card inserted bool.
// -------------------------------------------------------------------------------------------
bool prev_sdcard_inserted=false;
void sdcardBegin(
    bool test_rw, bool set_pins, bool bit1_mode, bool format_on_fail, long freq, signed int clk,
    signed int cmd, signed int d0, signed int d1, signed int d2, signed int d3, signed int pwr,
    int max_open_files
  ) {
  // -------------------------------------------------------------------
  // Test card presence: power up the slot and see if a card is present.
  // -------------------------------------------------------------------
  cycleSDPower(pwr); // cycle power every run for card presence detection.
  sdcardEnd(pwr); // ensure dead before resurrecting.
  cycleSDPower(pwr); // cycle power after unmount.
  mountSD(true, test_rw, set_pins, bit1_mode, format_on_fail, freq, clk, cmd, d0, d1, d2, d3);
  if (digitalRead(SD_DATA3_GPIO_NUM)==true) {sdcardData.sdcard_inserted=true;} // chip detect high.
  else {sdcardData.sdcard_inserted=false;} // chip detect low.
  if (prev_sdcard_inserted!=sdcardData.sdcard_inserted) { // override allow_mount on change. 
  sdcardData.allow_mount=true; prev_sdcard_inserted=sdcardData.sdcard_inserted;}
  sdcardEnd(pwr); // ensure dead before continuing.
  // -----------------------------------------------------------------------------------
  // Mount: Attempt mount if card inserted providing unmount_sdcard_flag did not occurr.
  // -----------------------------------------------------------------------------------
  if (sdcardData.allow_mount==true && sdcardData.sdcard_inserted==true) {
    cycleSDPower(pwr); // cycle power every run for card presence detection.
    sdcardEnd(pwr); // ensure dead before resurrecting.
    cycleSDPower(pwr); // cycle power after unmount.
    mountSD(true, test_rw, set_pins, bit1_mode, format_on_fail, freq, clk, cmd, d0, d1, d2, d3);
    if (sdcardData.sdcard_mounted==true) {
      getMountPoint();
      getSDCardType();
      getCardSize();
      getTotalBytes();
      getUsedBytes();
      getSectorSize();
    }
  }
  else {clearSDCardStruct();} // todo: clear if not already cleared.
  // -----------------------------------------------------------------------------------
  /*
    Lastly, pull the card out of SD_MMC to see if it's populated (mount check).
    Checking pointer here will mask any intermediary mounts/unmounts above.
  */
  checkCardPointer();
}

// Recursive function to list directory contents (ls -R style with full paths)
// - fs: The filesystem instance (e.g., SD_MMC)
// - dirname: The directory path to list (e.g., "/EXTERNAL" or "/EXTERNAL/subdir")
// - maxLevels: Recursion depth limit (-1 for infinite/unlimited, 0 for no recursion, 1+ for limited levels)
// - currentLevel: Internal recursion depth (default 0, unused for output but for limit check)
void listDir(FS &fs, const char *dirname, int maxLevels=-1, int currentLevel=0) {
  // Early exit if maxLevels reached (unless -1)
  if (maxLevels != -1 && currentLevel >= maxLevels) {
    return;
  }
  
  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Error: Path is not a directory");
    root.close();
    return;
  }

  File file = root.openNextFile();
  while (file) {
    String fullpath = String(file.path());
    if (file.isDirectory()) {
      Serial.println(fullpath + "/");
      // Recurse if unlimited or within limit
      listDir(fs, fullpath.c_str(), maxLevels, currentLevel + 1);
    } else {
      Serial.println(fullpath);
    }
    file.close();  // Close each entry explicitly
    file = root.openNextFile();
  }
  root.close();
}

// -------------------------------------------------------------------------------------------
// Flag Handler.
// -------------------------------------------------------------------------------------------
// Uses Flags set from anywhere that includes sdmmc_helper.
// Uses buffer when required.
// -------------------------------------------------------------------------------------------
void sdcardFlagHandler(bool test_rw, bool set_pins, bool bit1_mode, bool format_on_fail,
    long freq, signed int clk, signed int cmd, signed int d0, signed int d1, signed int d2,
    signed int d3, signed int pwr, int max_open_files) {
  if (sdmmcFlagData.unmount_sdcard_flag==true) {
    Serial.println("[sdmmc] card unmount flag detected");
    sdcardEnd(pwr);
    sdcardData.allow_mount=false; // override allow_mount (prevent remounting on unmount flag).
    sdmmcFlagData.no_delay_flag=false;
    sdmmcFlagData.unmount_sdcard_flag=false; // reset flag
  }
  else if (sdmmcFlagData.mount_sdcard_flag==true) {
    Serial.println("[sdmmc] card mount flag detected");
    sdcardBegin(test_rw, set_pins, bit1_mode, format_on_fail, freq, clk, cmd, d0, d1, d2, d3, pwr);
    sdcardData.allow_mount=true; // override allow_mount on mount flag.
    sdmmcFlagData.no_delay_flag=false;
    sdmmcFlagData.mount_sdcard_flag=false; // reset flag
  }
  if (sdcardData.sdcard_mounted==true) {
    /*
    // test flag fs operation.
    */
    if (sdmmcFlagData.list_dir_flag==true) {
      Serial.println("path: " + String(sdmmcArgData.buffer));
      Serial.println("maxlevels: " + String(sdmmcArgData.maxlevels));
      Serial.println();
      listDir(SD_MMC, sdmmcArgData.buffer, sdmmcArgData.maxlevels);
      Serial.println();
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.list_dir_flag=false;
    }
    else if (sdmmcFlagData.save_mapping) {
      Serial.println("[sdmmc] saving mapping ...");
      if (saveMappingFile(SD_MMC, satioFileData.mapping_filepath)) {Serial.println("[sdmmc] saved mapping successfully.");}
      else {Serial.println("[sdmmc] save mapping failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.save_mapping=false;
    }
    else if (sdmmcFlagData.load_mapping) {
      zero_mapping(); // avoid mixing current values with loaded values
      delay(100);
      Serial.println("[sdmmc] loading mapping...");
      if (loadMappingFile(SD_MMC, satioFileData.mapping_filepath)) {Serial.println("[sdmmc] loaded mapping successfully.");}
      else {Serial.println("[sdmmc] load mapping failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.load_mapping=false;
    }
    else if (sdmmcFlagData.delete_mapping) {
      Serial.println("[sdmmc] deleting mapping...");
      if (deleteMappingFile(SD_MMC, satioFileData.mapping_filepath)) {Serial.println("[sdmmc] deleted mapping successfully.");}
      else {Serial.println("[sdmmc] delete mapping failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.delete_mapping=false;
    }

    else if (sdmmcFlagData.save_matrix) {
      Serial.println("[sdmmc] saving matrix ...");
      if (saveMatrixFile(SD_MMC, satioFileData.current_matrix_filepath)) {Serial.println("[sdmmc] saved matrix successfully.");}
      else {Serial.println("[sdmmc] save matrix failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.save_matrix=false;
    }
    else if (sdmmcFlagData.load_matrix) {
      zero_matrix(); // avoid mixing current values with loaded values
      delay(100);
      Serial.println("[sdmmc] loading matrix...");
      if (loadMatrixFile(SD_MMC, satioFileData.current_matrix_filepath)) {Serial.println("[sdmmc] loaded matrix successfully.");}
      else {Serial.println("[sdmmc] load matrix failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.load_matrix=false;
    }
    else if (sdmmcFlagData.delete_matrix) {
      Serial.println("[sdmmc] deleting matrix...");
      if (deleteMatrixFile(SD_MMC, satioFileData.current_matrix_filepath)) {Serial.println("[sdmmc] deleted matrix successfully.");}
      else {Serial.println("[sdmmc] delete matrix failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.delete_matrix=false;
    }

    else if (sdmmcFlagData.save_system) {
      Serial.println("[sdmmc] saving system...");
      if (saveSystemFile(SD_MMC, satioFileData.system_filepath)) {Serial.println("[sdmmc] saved system successfully.");}
      else {Serial.println("[sdmmc] save system failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.save_system=false;
    }
    else if (sdmmcFlagData.load_system) {
      Serial.println("[sdmmc] loading system...");
      if (loadSystemFile(SD_MMC, satioFileData.system_filepath)) {Serial.println("[sdmmc] loaded system successfully.");}
      else {Serial.println("[sdmmc] load system failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.load_system=false;
    }
    else if (sdmmcFlagData.delete_system) {
      Serial.println("[sdmmc] deleting system...");
      if (deleteSystemFile(SD_MMC, satioFileData.system_filepath)) {Serial.println("[sdmmc] deleted system successfully.");}
      else {Serial.println("[sdmmc] delete system failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.delete_system=false;
    }
  }
  clearSDMMCArgStruct();
}
// -------------------------------------------------------------------------------------------
// Stat Card.
// -------------------------------------------------------------------------------------------
void getMountPoint() {memset(sdcardData.sdcard_mountpoint,0 , sizeof(sdcardData.sdcard_mountpoint)); strcpy(sdcardData.sdcard_mountpoint, SD_MMC.mountpoint());}
void getSDCardType() {sdcardData.sdcard_type = SD_MMC.cardType();}
void getCardSize() {sdcardData.sdcard_card_size=SD_MMC.cardSize();}
void getTotalBytes() {sdcardData.sdcard_total_bytes=SD_MMC.totalBytes();}
void getUsedBytes() {sdcardData.sdcard_used_bytes=SD_MMC.usedBytes();}
void getSectorSize() {sdcardData.sdcard_sector_size=SD_MMC.sectorSize();}
// -------------------------------------------------------------------------------------------
// Print Stats.
// -------------------------------------------------------------------------------------------
void statSDCardPins() {
  Serial.println("SD_DATA0_GPIO_NUM : " + String(digitalRead(SD_DATA0_GPIO_NUM))); // 39
  Serial.println("SD_DATA1_GPIO_NUM : " + String(digitalRead(SD_DATA1_GPIO_NUM))); // 40
  Serial.println("SD_DATA2_GPIO_NUM : " + String(digitalRead(SD_DATA2_GPIO_NUM))); // 41
  Serial.println("SD_DATA3_GPIO_NUM : " + String(digitalRead(SD_DATA3_GPIO_NUM))); // 42
  Serial.println("SD_DATA4_GPIO_NUM : " + String(digitalRead(SD_DATA4_GPIO_NUM))); // 45
  Serial.println("SD_CLK_GPIO_NUM : " + String(digitalRead(SD_CLK_GPIO_NUM)));     // 43
  Serial.println("SD_CMD_GPIO_NUM : " + String(digitalRead(SD_CMD_GPIO_NUM)));     // 44
}
void statSDCard() {
  Serial.println("[sdmmc] sdcard inserted:    " + String(sdcardData.sdcard_inserted));
  Serial.println("[sdmmc] sdcard mounted:     " + String(sdcardData.sdcard_mounted));
  Serial.println("[sdmmc] sdcard mount point: " + String(sdcardData.sdcard_mountpoint));
  Serial.println("[sdmmc] sdcard type:        " +
    String(sdcardData.sdcard_type_names[sdcardData.sdcard_type]) +
    " (type: " + String(sdcardData.sdcard_type) + ").");
  Serial.println("[sdmmc] sdcard card size:   " + String(sdcardData.sdcard_card_size));
  Serial.println("[sdmmc] sdcard total bytes: " + String(sdcardData.sdcard_total_bytes));
  Serial.println("[sdmmc] sdcard used bytes:  " + String(sdcardData.sdcard_used_bytes));
  Serial.println("[sdmmc] sdcard sector size: " + String(sdcardData.sdcard_sector_size));
  // statSDCardPins();
}
