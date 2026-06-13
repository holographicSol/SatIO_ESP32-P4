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

// #include "./sdcard_helper.h"
// #include <Arduino.h>
// #include <FS.h>
// #include "SD_MMC.h"
// #include "SPIFFS.h"
// #include "satio_file.h"
// #include "matrix.h"
// #include "custommapping.h"
// #include "ff.h"
// #include <SdFat.h>
#include "bsp/esp32_p4_wifi6_touch_lcd_xc.h"
#include <limits.h>
#include "esp_log.h"
#include "lvgl.h"
// #include "./strval.h"
#include <stdlib.h>       // malloc/free
#include "ff.h"           // FatFs core
#include "diskio.h"       // Disk I/O
#include "diskio_impl.h"  // ESP32 disk impl
#include "esp_vfs_fat.h"  // VFS integration
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include <stdio.h>
#include <dirent.h>
#include "bsp/esp-bsp.h"

#include <stdio.h>
#include <limits.h>
#include <string.h>
// #include <iostream>
#include <assert.h>
#include <float.h>
#include <math.h>
#include <Arduino.h>
// #include <Wire.h>

#include "./config.h"
#include "./REG.h"
#include "./strval.h"
#include "./meteors.h"
#include "./wtgps300p.h"
#include "./wt901.h"
#include "./multiplexers.h"
#include "./sidereal_helper.h"
#include "./hextodig.h"
#include "./ins.h"
#include "./satio.h"
#include "./custommapping.h"
#include "./matrix.h"
#include "./serial_infocmd.h"
#include "./system_data.h"
#include "./sdcard_helper.h"
#include "./task_handler.h"
// #include "./multi_display_controller.h"
#include "./i2c_helper.h"
#include "./wit_c_sdk.h"

#include <sys/time.h>
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "esp_pm.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/uart.h"

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
    // .sdcard_mountpoint={0},
    // .sdcard_type=CARD_NONE,
    // .sdcard_type_names = {
    //     "None", "MMC", "SD", "SDHC/SDXC", "Unknown"
    // },
    // .sdcard_card_size=0,
    // .sdcard_total_bytes=0,
    // .sdcard_used_bytes=0,
    // .sdcard_sector_size=0,
    // .card=NULL,
    // .host={},
    // .slot_config={},
    // .ldo_config={},
};

struct sdcardFlagStruct sdcardFlagData = {

  .no_delay_flag=false,

  .success_flag=0,
  .rw_failed=NULL,
  .rw_running=NULL,

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

  .write_log=false,
};

/** ----------------------------------------------------------------------------------------
 * @brief Get the size of a file in bytes.
 * 
 * @param filename Path to the file on the SD card.
 * @return Size of the file in bytes, or 0 if the file cannot be accessed.
 */
uint32_t get_file_size(const char* filename) {
    struct stat st;
    if(stat(filename, &st) == 0) {
        printf("File '%s' size: %lu bytes\n", filename, (uint32_t)st.st_size);
        return (uint32_t)st.st_size;
    }
    
    printf("get_file_size: Cannot stat '%s'\n", filename);
    return 0;
}

/** ----------------------------------------------------------------------------------------
 * @brief Open a file.
 * 
 * @param drv Pointer to the LVGL file system driver (not used in this implementation).
 * @param path Path to the file to open.
 * @param mode File open mode (read, write, etc.).
 * @return Pointer to the opened file object, or NULL on failure.
 */
void * sd_file_open(lv_fs_drv_t * drv, const char * path, lv_fs_mode_t mode)
{
    printf("LVGL_FS: Got path '%s'\n", path);

    char full_path[256];
    snprintf(full_path, sizeof(full_path), "/sdcard/%s", path);
    
    printf("LVGL_FS: Full path '%s'\n", full_path);
    
    FILE * f = fopen(full_path, "r");
    if (!f) {
        printf("LVGL_FS: FAILED '%s'\n", full_path);
    } else {
        printf("LVGL_FS: SUCCESS '%s' -> %p\n", full_path, f);
    }
    
    return (void *)f;
}

/** ----------------------------------------------------------------------------------------
 * @brief Close the file.
 * 
 * @param drv Pointer to the LVGL file system driver (not used in this implementation).
 * @param file_p Pointer to the file object (cast to FILE*).
 * @return LV_FS_RES_OK.
 * 
 */
lv_fs_res_t sd_file_close(lv_fs_drv_t * drv, void * file_p)
{
    FILE * f = (FILE *)file_p;
    fclose(f);
    printf("LVGL_FS: Closed\n");
    return LV_FS_RES_OK;
}

/** ----------------------------------------------------------------------------------------
 * @brief Read from file into buffer.
 * 
 * Reads up to btr bytes from the file into the provided buffer and sets br to the number of
 * bytes actually read.
 * 
 * @param drv Pointer to the LVGL file system driver (not used in this implementation).
 * @param file_p Pointer to the file object (cast to FILE*).
 * @param buf Pointer to the buffer where the read data should be stored.
 * @param btr Number of bytes to read from the file.
 * @param br Pointer to a variable where the actual number of bytes read should be stored.
 * @return LV_FS_RES_OK if the read was successful (bytes read > 0 or end of file reached),
 *         otherwise LV_FS_RES_UNKNOWN.
 */
lv_fs_res_t sd_file_read(lv_fs_drv_t * drv, void * file_p, void * buf, uint32_t btr, uint32_t * br)
{
    FILE * f = (FILE *)file_p;
    *br = fread(buf, 1, btr, f);
    // printf("LVGL_FS: Read %ld/%lu\n", *br, btr);
    return (*br > 0 || feof(f)) ? LV_FS_RES_OK : LV_FS_RES_UNKNOWN;
}

/** ----------------------------------------------------------------------------------------
 * @brief Seek to a specific position in the file.
*/
lv_fs_res_t sd_file_seek(lv_fs_drv_t * drv, void * file_p, uint32_t pos, lv_fs_whence_t whence)
{
    FILE * f = (FILE *)file_p;
    // printf("LVGL_FS Seeking file: %p\n", file_p);
    int stdio_whence = whence == LV_FS_SEEK_CUR ? SEEK_CUR : 
                       whence == LV_FS_SEEK_END ? SEEK_END : SEEK_SET;
    int res = fseek(f, pos, stdio_whence);
    // printf("LVGL_FS: Seek %lu -> %d\n", pos, res);
    return res == 0 ? LV_FS_RES_OK : LV_FS_RES_UNKNOWN;
}

/** ----------------------------------------------------------------------------------------
 * @brief Get current file position in bytes.
 */
lv_fs_res_t sd_file_tell(lv_fs_drv_t * drv, void * file_p, uint32_t * pos_p)
{
    FILE * f = (FILE *)file_p;
    *pos_p = ftell(f);
    return LV_FS_RES_OK;
}

/** ----------------------------------------------------------------------------------------
 * @brief Open a directory.
 * 
 * @param drv Pointer to the LVGL file system driver (not used in this implementation).
 * @param path Path to the directory to open.
 * @return Pointer to the opened directory object, or NULL on failure.
 */
void * sd_dir_open(lv_fs_drv_t * drv, const char * path)
{
    printf("LVGL_FS: Got dir path '%s'\n", path);

    char full_path[256];
    snprintf(full_path, sizeof(full_path), "/sdcard/%s", path);
    
    printf("LVGL_FS: Full dir path '%s'\n", full_path);
    
    DIR * d = opendir(full_path);
    if (!d) {
        printf("LVGL_FS: DIR FAILED '%s'\n", full_path);
    } else {
        printf("LVGL_FS: DIR SUCCESS '%s' -> %p\n", full_path, d);
    }
    
    return (void *)d;
}

/** ----------------------------------------------------------------------------------------
 * @brief Read the next directory entry.
 */
lv_fs_res_t sd_dir_read(lv_fs_drv_t * drv, void * dir_p, char * fn, uint32_t btr)
{
    DIR * d = (DIR *)dir_p;
    struct dirent * entry;
    
    while ((entry = readdir(d)) != NULL) {
        // Skip . and .. entries
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        
        // btr is max buffer size - use min(entry name len, btr)
        size_t fn_len = strlen(entry->d_name) + 1;
        size_t copy_len = (fn_len < btr) ? fn_len : btr;
        strncpy(fn, entry->d_name, copy_len - 1);
        fn[copy_len - 1] = '\0';
        
        printf("LVGL_FS: Found '%s'\n", fn);
        return LV_FS_RES_OK;
    }
    
    printf("LVGL_FS: End of directory\n");
    *fn = '\0';  // Clear buffer at EOF
    return LV_FS_RES_OK;
}

/** ----------------------------------------------------------------------------------------
 * @brief Close the directory.
 * 
 * @param drv Pointer to the LVGL file system driver (not used in this implementation).
 * @param dir_p Pointer to the directory object (cast to DIR*).
 * @return LV_FS_RES_OK.
 */
lv_fs_res_t sd_dir_close(lv_fs_drv_t * drv, void * dir_p)
{
    DIR * d = (DIR *)dir_p;
    closedir(d);
    printf("LVGL_FS: Directory closed\n");
    return LV_FS_RES_OK;
}

/** ----------------------------------------------------------------------------------------
 * @brief Load file from SD card into PSRAM and return pointer.
 * 
 * @param filename Path to the file on the SD card.
 * @param f_size Size of the file in bytes.
 * @return Pointer to the file data in PSRAM, or NULL on failure.
 * 
 */
uint8_t * load_file_bytes_to_psram(const char * filename, uint32_t f_size) {

    // Allocate PSRAM
    uint8_t * psram_pointer = (uint8_t *)heap_caps_malloc(f_size, MALLOC_CAP_SPIRAM);
    if(!psram_pointer) {printf("ERROR: PSRAM allocation failed\n"); return NULL;}

    printf("PSRAM allocated at %p\n", psram_pointer);

    // Read bytes into PSRAM
    FILE * f = fopen(filename, "rb");
    if (f) {
        size_t bytes_read = fread(psram_pointer, 1, f_size, f);
        fclose(f);
        printf("Read %zu/%lu bytes from SD\n", bytes_read, f_size);
    }
    else {
        printf("ERROR: Cannot open %s\n", filename);
        heap_caps_free(psram_pointer);
        psram_pointer = NULL;
    }

    return psram_pointer;
}

/** ----------------------------------------------------------------------------------------
 * @brief Mount the SD card using the Board Support Package (BSP) function.
 * 
 */
bool mount_sd(void) {
    esp_err_t err = bsp_sdcard_mount();
    if (err != ESP_OK) {
        printf("Failed to mount SD card, error: %s\n", esp_err_to_name(err));
    }
    if (err == ESP_OK) {
        printf("SD card mounted successfully\n");
        return true;
    }
    return false;
}

/** ----------------------------------------------------------------------------------------
 * @brief Unmount the SD card using the Board Support Package (BSP) function.
 * 
 */
bool unmount_sd(void) {
    esp_err_t err = bsp_sdcard_unmount();
    if (err != ESP_OK) {
        printf("Failed to unmount SD card, error: %s\n", esp_err_to_name(err));
    }
    if (err == ESP_OK) {
        printf("SD card unmounted successfully\n");
        return true;
    }
    return false;
}

#include <sys/stat.h>

/** ----------------------------------------------------------------------------------------
 * @brief List directory.
 * 
 * @param dirname Path to the directory on the SD card to list.
 * 
 */
void list_directory(const char * dirname) {
    DIR *dir = opendir(dirname);
    if (!dir) {
        printf("Failed to open %s\n", dirname);
        return;
    }
    
    printf("%s contents:\n", dirname);
    printf("%-20s %12s\n", "Name", "Size (bytes)");
    printf("----------------------------------------\n");
    
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        char fullpath[512];
        snprintf(fullpath, sizeof(fullpath), "%s/%s", dirname, entry->d_name);
        
        struct stat st;
        if (stat(fullpath, &st) == 0) {
            printf("%-20s %12llu\n", entry->d_name, (unsigned long long)st.st_size);
        } else {
            printf("%-20s %12s\n", entry->d_name, "?");
        }
    }
    closedir(dir);
}

uint64_t getFreeBytes() {
  FATFS *fs;
  DWORD fre_clust;
  FRESULT res = f_getfree("0:", &fre_clust, &fs);  // or "/sdcard:" if using that mount point
  if (res != FR_OK) {
    Serial.printf("f_getfree failed: %d\n", res);
    return 0;
  }
  uint64_t cluster_size   = (uint64_t)fs->csize * 512ULL;  // sector size usually 512
  return fre_clust * cluster_size;
}

bool isAvailableBytes(uint64_t num_bytes) {
  if (num_bytes < getFreeBytes()) {return true;}
  return false;
}

/** ----------------------------------------------------------------------------------------
 * @brief Check if SD card root directory is ready.
 * 
 * @return true if the SD card root directory can be opened, false otherwise.
 * 
 */
bool is_fs_ready(const char * dirname) {
    DIR *dir = opendir(dirname);
    if (dir) {
        closedir(dir);
        // printf("Accessed directory successfully: %s\n", dirname);
        return true;
    }
    // printf("Could not access directory: %s\n", dirname);
    return false;
}

/** ----------------------------------------------------------------------------------------
 * @brief Register LVGL callbacks for SD card file system access.
 * 
 */
void sd_lvgl_register(void)
{
    lv_fs_drv_t fs_drv;
    lv_fs_drv_init(&fs_drv);
    fs_drv.letter = 'S';
    fs_drv.open_cb = sd_file_open;
    fs_drv.close_cb = sd_file_close;
    fs_drv.read_cb = sd_file_read;
    fs_drv.seek_cb = sd_file_seek;
    fs_drv.tell_cb = sd_file_tell;

    fs_drv.dir_open_cb = sd_dir_open;      // Opens directory
    fs_drv.dir_read_cb = sd_dir_read;      // Reads directory entries
    fs_drv.dir_close_cb = sd_dir_close;    // Closes directory

    lv_fs_drv_register(&fs_drv);
    // printf("LVGL S: drive registered\n");
}

int last_cd_state = -1;           // unknown at start
bool sd_is_mounted = false;       // track your own mount status

/** ----------------------------------------------------------------------------------------
 * @brief Unmount
 * 
 */
void sdcard_unmount() {
    // unmount
    // printf("Attempting SD card unmount...\n");
    if (sdcardData.sdcard_mounted) {
        esp_err_t err = bsp_sdcard_unmount();
        if (err != ESP_OK) {
            // printf("Failed to unmount SD card, error: %s\n", esp_err_to_name(err));
        }
        if (err == ESP_OK) {
            // printf("SD card unmounted successfully\n");
        }
    }
}

/** ----------------------------------------------------------------------------------------
 * @brief Mount
 * 
 * @note Mount is expected to fail if card is not present. Should be caled periodically to
 * handle hotplug events.
 * 
 */
void sdcard_mount() {
    // 
    if (sdcardData.allow_mount == false) {return;}

    // mount
    // printf("Attempting SD card mount...\n");
    esp_err_t err = bsp_sdcard_mount();
    if (err != ESP_OK) {
        // printf("Failed to mount SD card, error: %s\n", esp_err_to_name(err));
        sdcardData.sdcard_mounted = false;

        // (todo) clear any pending operations
        // sdcardFlagData.save_mapping = false;
        // sdcardFlagData.load_mapping = false;
        // sdcardFlagData.delete_mapping = false;

        // sdcardFlagData.save_matrix = false;
        // sdcardFlagData.load_matrix = false;
        // sdcardFlagData.delete_matrix = false;

        // sdcardFlagData.save_system = false;
        // sdcardFlagData.load_system = false;
        // sdcardFlagData.delete_system = false;
    }
    else {
        // printf("SD card mounted successfully\n");
        // sd_lvgl_register(); // register file related callback functions with LVGL
        sdcardData.sdcard_mounted = true;
    }
    // printf("SDCLK: %d\n", gpio_get_level((gpio_num_t)SD_CLK));
    // printf("SDCMD: %d\n", gpio_get_level((gpio_num_t)SD_CMD));
    // printf("SDD0: %d\n", gpio_get_level((gpio_num_t)SD_D0));
    // printf("SDD1: %d\n", gpio_get_level((gpio_num_t)SD_D1));
    // printf("SDD2: %d\n", gpio_get_level((gpio_num_t)SD_D2));
    // printf("SDD3: %d\n", gpio_get_level((gpio_num_t)SD_D3)); // card detect
    // printf("SD_PWR: %d\n", gpio_get_level((gpio_num_t)SD_PWR));
}
