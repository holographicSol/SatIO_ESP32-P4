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

void getMountPoint();
void getSDCardType();
void getCardSize();
void getTotalBytes();
void getUsedBytes();
void getSectorSize();

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

struct sdmmcArgStruct sdmmcArgData = {
  .recursive=false,
  .maxlevels=0,
  .buffer={0},
};

void clearSDMMCArgStruct() {
  sdmmcArgData.recursive=false;
  sdmmcArgData.maxlevels=0;
  memset(sdmmcArgData.buffer, 0, sizeof(sdmmcArgData.buffer)); 
}

void clearSDCardStruct() {
  memset(sdcardData.sdcard_mountpoint, 0, sizeof(sdcardData.sdcard_mountpoint));
  sdcardData.sdcard_type=CARD_NONE;
  sdcardData.sdcard_card_size=0;
  sdcardData.sdcard_total_bytes=0;
  sdcardData.sdcard_used_bytes=0;
  sdcardData.sdcard_sector_size=0;
}

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

// -----------------------------------------------------------------------------------
// Debug R/W.
// -----------------------------------------------------------------------------------

bool testWrite(FS &fs, const char* path) {
    File f = fs.open(path, "w", true); if (!f) return false; f.print("test");
    f.close(); return fs.exists(path);}

bool testRead(FS &fs, const char* path) {
    File f = fs.open(path, "r"); if (!f) return false; String s = f.readString();
    f.close(); fs.remove(path); return s == "test";}

void testRW() {
  if (testWrite(SD_MMC, "/EXTERNAL/test.txt") && testRead(SD_MMC, "/EXTERNAL/test.txt")) {
    Serial.println("FS read/write OK");}
  else {Serial.println("FS test failed");}
}

void checkCardPointer() {
  sdcardData.card=SD_MMC.returnCard(); // pull the card out of sd_mmc to see its populated.
  sdcardData.host=SD_MMC.returnHost();
  sdcardData.slot_config=SD_MMC.returnSlotConfig();
  sdcardData.ldo_config=SD_MMC.returnLDOConfig();
  if (sdcardData.card!=NULL) {sdcardData.sdcard_mounted=true;}
  else {sdcardData.sdcard_mounted=false;}
}

void mountSD(bool tmp_mount, bool test_rw, bool set_pins, bool bit1_mode, bool format_on_fail,
    long freq, signed int clk, signed int cmd, signed int d0, signed int d1, signed int d2,
    signed int d3, signed int pwr, int max_open_files) {
  if (set_pins==true) {
    if(SD_MMC.setPins(clk, cmd, d0, d1, d2, d3)) {Serial.println("pins set successfully.");}
    else {Serial.println("error setting pins.");}
    delay(10);
  }
  if (SD_MMC.begin("/EXTERNAL", bit1_mode, format_on_fail, freq)) {
    if (tmp_mount==true) {} // do not override allow_mount on successfull mount.
    else {sdcardData.allow_mount=true;} // override allow_mount on successfull mount.
    if (test_rw==true) {testRW();} // verify read/write.  
  }
  delay(10);
}

void sdcardEnd(signed int pwr) {
  // unmount card & release LDO via pwr_ctrl_handle so the card can be remounted later.
  SD_MMC.end();
  sdcardSleepMode0(pwr); // power down the slot.
}

bool prev_sdcard_inserted=false;
void sdcardBegin(
    bool test_rw, bool set_pins, bool bit1_mode, bool format_on_fail, long freq, signed int clk,
    signed int cmd, signed int d0, signed int d1, signed int d2, signed int d3, signed int pwr,
    int max_open_files
  ) {
  // -----------------------------------------------------------------------------------
  // Test card presence: power up the slot and see if a card is present.
  // -----------------------------------------------------------------------------------
  cycleSDPower(pwr); // cycle power every run for card presence detection.
  sdcardEnd(pwr); // ensure dead before resurrecting.
  cycleSDPower(pwr); // cycle power after unmount.
  mountSD(true, test_rw, set_pins, bit1_mode, format_on_fail, freq, clk, cmd, d0, d1, d2, d3,
          pwr, max_open_files);
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
    mountSD(true, test_rw, set_pins, bit1_mode, format_on_fail, freq, clk, cmd, d0, d1, d2, d3,
            pwr, max_open_files);
    if (sdcardData.sdcard_mounted==true) {
      getMountPoint();
      getSDCardType();
      getCardSize();
      getTotalBytes();
      getUsedBytes();
      getSectorSize();
    }
  }
  else {clearSDCardStruct();} // todo: only clear if not already cleared.
  /*
    Lastly, pull the card out of SD_MMC to see if it's populated (mount check).
    Checking pointer here will mask any intermediary mounts/unmounts above.
  */
  checkCardPointer();
}

void listDir(FS &fs, const char *dirname, signed int maxLevels, int currentLevel) {
  if (maxLevels != -1 && currentLevel >= maxLevels) {return;}
  File root = fs.open(dirname);
  if (!root) {Serial.println("Failed to open directory"); {return;}}
  if (!root.isDirectory()) {Serial.println("Path is not a directory"); root.close(); {return;}}
  File file = root.openNextFile();
  while (file) {
    String fullpath = String(file.path());
    if (file.isDirectory()) {Serial.println(fullpath + "/");
      // Recurse if unlimited or within limit
      listDir(fs, fullpath.c_str(), maxLevels, currentLevel + 1);
    }
    else {Serial.println(fullpath);}
    file.close(); // Close each entry explicitly
    file = root.openNextFile();
  }
  root.close();
}

void sdcardFlagHandler(bool test_rw, bool set_pins, bool bit1_mode, bool format_on_fail,
    long freq, signed int clk, signed int cmd, signed int d0, signed int d1, signed int d2,
    signed int d3, signed int pwr, int max_open_files) {
  if (sdmmcFlagData.unmount_sdcard_flag==true) {
    Serial.println("card unmount flag detected");
    sdcardEnd(pwr);
    sdcardData.allow_mount=false; // override allow_mount (prevent remounting on unmount flag).
    sdmmcFlagData.no_delay_flag=false; // reset flag
    sdmmcFlagData.unmount_sdcard_flag=false; // reset flag
  }
  else if (sdmmcFlagData.mount_sdcard_flag==true) {
    Serial.println("card mount flag detected");
    sdcardBegin(test_rw, set_pins, bit1_mode, format_on_fail, freq, clk, cmd, d0, d1, d2, d3,
                pwr, max_open_files);
    sdcardData.allow_mount=true; // override allow_mount on mount flag.
    sdmmcFlagData.no_delay_flag=false;
    sdmmcFlagData.mount_sdcard_flag=false;
  }
  if (sdcardData.sdcard_mounted==true) {

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
      Serial.println("saving mapping ...");
      if (saveMappingFile(SD_MMC, satioFileData.mapping_filepath))
        {Serial.println("saved mapping successfully.");}
      else {Serial.println("save mapping failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.save_mapping=false;
    }
    else if (sdmmcFlagData.load_mapping) {
      Serial.println("loading mapping...");
      if (loadMappingFile(SD_MMC, satioFileData.mapping_filepath))
        {Serial.println("loaded mapping successfully.");}
      else {Serial.println("load mapping failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.load_mapping=false;
    }
    else if (sdmmcFlagData.delete_mapping) {
      Serial.println("deleting mapping...");
      if (deleteMappingFile(SD_MMC, satioFileData.mapping_filepath))
        {Serial.println("deleted mapping successfully.");}
      else {Serial.println("delete mapping failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.delete_mapping=false;
    }

    else if (sdmmcFlagData.save_matrix) {
      Serial.println("saving matrix ...");
      if (saveMatrixFile(SD_MMC, satioFileData.current_matrix_filepath))
        {Serial.println("saved matrix successfully.");}
      else {Serial.println("save matrix failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.save_matrix=false;
    }
    else if (sdmmcFlagData.load_matrix) {
      Serial.println("loading matrix...");
      if (loadMatrixFile(SD_MMC, satioFileData.current_matrix_filepath))
        {Serial.println("loaded matrix successfully.");}
      else {Serial.println("load matrix failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.load_matrix=false;
    }
    else if (sdmmcFlagData.delete_matrix) {
      Serial.println("deleting matrix...");
      if (deleteMatrixFile(SD_MMC, satioFileData.current_matrix_filepath))
        {Serial.println("deleted matrix successfully.");}
      else {Serial.println("delete matrix failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.delete_matrix=false;
    }

    else if (sdmmcFlagData.save_system) {
      Serial.println("saving system...");
      if (saveSystemFile(SD_MMC, satioFileData.system_filepath))
        {Serial.println("saved system successfully.");}
      else {Serial.println("save system failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.save_system=false;
    }
    else if (sdmmcFlagData.load_system) {
      Serial.println("loading system...");
      if (loadSystemFile(SD_MMC, satioFileData.system_filepath))
        {Serial.println("loaded system successfully.");}
      else {Serial.println("load system failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.load_system=false;
    }
    else if (sdmmcFlagData.delete_system) {
      Serial.println("deleting system...");
      if (deleteSystemFile(SD_MMC, satioFileData.system_filepath))
        {Serial.println("deleted system successfully.");}
      else {Serial.println("delete system failed.");}
      sdmmcFlagData.no_delay_flag=false;
      sdmmcFlagData.delete_system=false;
    }
  }
  clearSDMMCArgStruct();
}

void getMountPoint() {memset(sdcardData.sdcard_mountpoint,
                             0,
                             sizeof(sdcardData.sdcard_mountpoint));
                      strcpy(sdcardData.sdcard_mountpoint,
                             SD_MMC.mountpoint());}
void getSDCardType() {sdcardData.sdcard_type = SD_MMC.cardType();}
void getCardSize() {sdcardData.sdcard_card_size=SD_MMC.cardSize();}
void getTotalBytes() {sdcardData.sdcard_total_bytes=SD_MMC.totalBytes();}
void getUsedBytes() {sdcardData.sdcard_used_bytes=SD_MMC.usedBytes();}
void getSectorSize() {sdcardData.sdcard_sector_size=SD_MMC.sectorSize();}

void statSDCardPins() {
  Serial.println("SD_DATA0_GPIO_NUM : " + String(digitalRead(SD_DATA0_GPIO_NUM))); // 39
  Serial.println("SD_DATA1_GPIO_NUM : " + String(digitalRead(SD_DATA1_GPIO_NUM))); // 40
  Serial.println("SD_DATA2_GPIO_NUM : " + String(digitalRead(SD_DATA2_GPIO_NUM))); // 41
  Serial.println("SD_DATA3_GPIO_NUM : " + String(digitalRead(SD_DATA3_GPIO_NUM))); // 42
  Serial.println("SD_DATA4_GPIO_NUM : " + String(digitalRead(SD_DATA4_GPIO_NUM))); // 45
  Serial.println("SD_CLK_GPIO_NUM : " + String(digitalRead(SD_CLK_GPIO_NUM)));     // 43
  Serial.println("SD_CMD_GPIO_NUM : " + String(digitalRead(SD_CMD_GPIO_NUM)));     // 44
}
