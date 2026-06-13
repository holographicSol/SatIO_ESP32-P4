/*
  SatioFile - Written By Benjamin Jack Cullen.
*/

#include "satio_file.h"
#include <Arduino.h>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "matrix.h"
#include "strval.h"
#include "satio.h"
#include "system_data.h"
#include "matrix.h"
#include "custommapping.h"
#include "ins.h"
#include "config.h"
#include "multiplexers.h"
#include "wtgps300p.h"
#include "wt901.h"
#include "sdcard_helper.h"

struct satioFileStruct satioFileData = {
    .i_token=0,
    .token={},
    .tmp_chars="",

    .matix_filepaths=
    {
        "/MATRIX/MATRIX_0.csv",
        "/MATRIX/MATRIX_1.csv",
        "/MATRIX/MATRIX_2.csv",
        "/MATRIX/MATRIX_3.csv",
        "/MATRIX/MATRIX_4.csv",
        "/MATRIX/MATRIX_5.csv",
        "/MATRIX/MATRIX_6.csv",
        "/MATRIX/MATRIX_7.csv",
        "/MATRIX/MATRIX_8.csv",
        "/MATRIX/MATRIX_9.csv",
    },
    .current_matrix_filepath="/MATRIX/MATRIX_0.csv", // default
    .i_current_matrix_file_path=0,
    .mapping_filepath="/MAPPING/mapping_conf.csv",
    .system_filepath="/SYSTEM/system_conf.csv",
    .log_dir="/LOG/",
    .log_files = {},
    .log_filepath = "",
    .unixtimestamp=0,
    .tmp_unixtimestamp=0,
    .number_of_log_files=0,
};

/**
 * @brief Yield for other tasks.
 * @note Current setup yields on every N lines loaded/saved, with no special cases.
 * @warning This increases loading/saving time.
 * 
 * yield_every_n_lines   0: very slow read/write (prioritize other tasks).
 * yield_every_n_lines > 0: faster read/write (more time blocking other tasks).
 */
int yield_every_n_lines=8;
int yield_counter=0;

void yieldForTasks() {
    if (yield_every_n_lines <= 0) { 
        // Always yield if yield_every_n_lines set to 0 or negative
        vTaskDelay(1);
        esp_task_wdt_reset();
        return;
    }
    yield_counter++;
    if (yield_counter >= yield_every_n_lines) {
        // Perform the actual yield
        vTaskDelay(1);
        esp_task_wdt_reset();
        yield_counter = 0; // Reset counter after yielding
    }
    // Otherwise, just pass through
}

void set_rw_running_true() {sdcardFlagData.rw_running=true;}
void set_rw_running_false() {sdcardFlagData.rw_running=false;}

/**
 * @brief Set Read/Write Success Flag.
 * 
 * Sustains flag for a period of time while also non-blocking before setting flag back to NULL.
 * 
 * @param flag Specify result of a read/write operation.
 */
void set_storage_success_flag(bool flag) {
    if (flag==true) {sdcardFlagData.success_flag=2; vTaskDelay(1000 / portTICK_PERIOD_MS);}
    else            {sdcardFlagData.success_flag=1; vTaskDelay(1000 / portTICK_PERIOD_MS);}
                     sdcardFlagData.success_flag=0;
}

// ----------------------------------------------------------------------------------------
// SD Card File Helper Functions (using standard C I/O with /sdcard prefix)
// ----------------------------------------------------------------------------------------

/**
 * @brief Create full path with /sdcard prefix
 */
void sd_fullpath(const char* path, char* fullpath, size_t size) {
    snprintf(fullpath, size, "/sdcard%s", path);
}

/**
 * @brief Check if file exists
 */
bool sd_exists(const char* path) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    struct stat st;
    return (stat(fullpath, &st) == 0);
}

/**
 * @brief Remove file
 */
bool sd_remove(const char* path) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    return (remove(fullpath) == 0);
}

/**
 * @brief Get file size
 */
uint64_t sd_file_size(const char* path) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    struct stat st;
    if (stat(fullpath, &st) == 0) {
        return (uint64_t)st.st_size;
    }
    return 0;
}

/**
 * @brief Create directory recursively
 */
bool sd_mkdir(const char* path) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    
    char tmp[256];
    strncpy(tmp, fullpath, sizeof(tmp));
    size_t len = strlen(tmp);
    
    // Remove trailing slash
    if (tmp[len - 1] == '/') {
        tmp[len - 1] = '\0';
    }
    
    // Create directories recursively
    for (char* p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    return (mkdir(tmp, 0755) == 0 || errno == EEXIST);
}

/**
 * @brief Open file with /sdcard prefix
 */
FILE* sd_fopen(const char* path, const char* mode) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    printf("[sd_fopen] fullpath: %s, mode: %s\n", fullpath, mode);
    fflush(stdout);
    
    // Create parent directory if writing
    if (strchr(mode, 'w') || strchr(mode, 'a')) {
        char dirpath[256];
        strncpy(dirpath, fullpath, sizeof(dirpath) - 1);
        dirpath[sizeof(dirpath) - 1] = '\0';
        char* lastSlash = strrchr(dirpath, '/');
        if (lastSlash && lastSlash != dirpath) {
            *lastSlash = '\0';
            printf("[sd_fopen] creating dir: %s\n", dirpath);
            fflush(stdout);
            // Create directory directly with full path (already has /sdcard)
            char tmp[256];
            strncpy(tmp, dirpath, sizeof(tmp) - 1);
            tmp[sizeof(tmp) - 1] = '\0';
            for (char* p = tmp + 1; *p; p++) {
                if (*p == '/') {
                    *p = '\0';
                    int ret = mkdir(tmp, 0755);
                    if (ret != 0 && errno != EEXIST) {
                        printf("[sd_fopen] mkdir(%s) failed: %s\n", tmp, strerror(errno));
                    }
                    *p = '/';
                }
            }
            int ret = mkdir(tmp, 0755);
            if (ret != 0 && errno != EEXIST) {
                printf("[sd_fopen] final mkdir(%s) failed: %s\n", tmp, strerror(errno));
            }
            fflush(stdout);
        }
    }
    
    // Create file
    FILE* f = fopen(fullpath, mode);
    if (!f) {
        printf("[sd_fopen] fopen(%s) failed: errno=%d (%s)\n", fullpath, errno, strerror(errno));
        fflush(stdout);
    }
    return f;
}

// /** 
//  * @brief Write string to file with newline
//  */
// bool sd_fprintln(FILE* f, const char* str) {
//     if (!f) return false;
//     fputs(str, f);
//     fputc('\n', f);
//     return true;
// }

// bool isOpen(File root) {
//  if (!root) {
//     Serial.println("[getLogFiles] Failed to open directory");
//     return false;
//   }
//   if (!root.isDirectory()) {
//     Serial.println("[getLogFiles] Not a directory");
//     root.close();
//     return false;
//   }
//   return true;
// }

/**
 * @brief Get a log filename.
 * @param mode Specify mode: 0=oldest 1=latest.
 */
bool getLogFile(int mode) {
  // printf("[getLogFiles] Listing directory: %s\n", satioFileData.log_dir);

  // Open dir (and create dir if not exist)
  char fullpath[256];
  sd_fullpath(satioFileData.log_dir, fullpath, sizeof(fullpath));
  sd_mkdir(satioFileData.log_dir); // ensure directory exists
  
  DIR* root = opendir(fullpath);
  if (!root) {return false;}

  // Set target
  if (mode==0) {satioFileData.unixtimestamp=INT64_MAX;} // for finding oldest
  else {satioFileData.unixtimestamp=0;} // for finding latest
  
  // Iterate through files in directory
  satioFileData.number_of_log_files=0;
  struct dirent* entry;
  while ((entry = readdir(root)) != NULL) {
    // Skip . and .. and directories
    if (entry->d_name[0] == '.') continue;
    if (entry->d_type == DT_DIR) continue;
    
    // printf("  FILE : %s\n", entry->d_name);

    // Create a copy of filename
    memset(satioFileData.tmp_chars, 0, sizeof(satioFileData.tmp_chars));
    strncpy(satioFileData.tmp_chars, entry->d_name, sizeof(satioFileData.tmp_chars));

    // Tokenize
    int i_san=0;
    satioFileData.tmp_unixtimestamp=0;
    satioFileData.i_token = 0;
    satioFileData.token = strtok(satioFileData.tmp_chars, ".");
    while (satioFileData.token != NULL) {
      switch (satioFileData.i_token) {
          case 0: if (satioFileData.token != NULL && str_is_int64(satioFileData.token)) {
              i_san++; char *endptr; satioFileData.tmp_unixtimestamp = strtoll(satioFileData.token, &endptr, 10);} break;
          case 1: if (strcmp(satioFileData.token, "csv")==0) {i_san++;} break;
      }
      satioFileData.token = strtok(NULL, ".");
      satioFileData.i_token++;
    }
    // Update unixtimestamp
    if (i_san==2) {
      satioFileData.number_of_log_files++;
      if      (mode==0 && satioFileData.tmp_unixtimestamp<satioFileData.unixtimestamp) {satioFileData.unixtimestamp=satioFileData.tmp_unixtimestamp;}
      else if (mode==1 && satioFileData.tmp_unixtimestamp>satioFileData.unixtimestamp) {satioFileData.unixtimestamp=satioFileData.tmp_unixtimestamp;}
    }
  }
  closedir(root);
  // printf("[getLogFiles] number_of_log_files: %llu\n", satioFileData.number_of_log_files);
  if (satioFileData.unixtimestamp==0 || satioFileData.unixtimestamp==INT64_MIN) {return false;} 

  // Store filename of interest
  memset(satioFileData.log_filepath, 0, sizeof(satioFileData.log_filepath));
  int written = snprintf(satioFileData.log_filepath, sizeof(satioFileData.log_filepath), "%s%lld.csv", satioFileData.log_dir, (long long)satioFileData.unixtimestamp);
  if (written < 0 || (size_t)written >= sizeof(satioFileData.log_filepath)) {
    printf("[getLogFiles] log_filepath truncated\n");
  }
  // printf("[getLogFiles] current log file: %s\n", satioFileData.log_filepath);
  return true;
}

void createNewLogFilename() {
  memset(satioFileData.log_filepath, 0, sizeof(satioFileData.log_filepath));
  // Use local_unixtime_uS for filename
  int written = snprintf(satioFileData.log_filepath, sizeof(satioFileData.log_filepath), "%s%lld.csv", satioFileData.log_dir, (long long)satioData.local_unixtime_uS);
  if (written < 0 || (size_t)written >= sizeof(satioFileData.log_filepath)) {
    printf("[createNewLogFilename] log_filepath truncated\n");
  }
  // printf("[createNewLogFilename] current log file: %s\n", satioFileData.log_filepath);
}

void deleteOldestLogFile() {
  if (getLogFile(0)==true && satioFileData.number_of_log_files>MAX_LOG_FILES) {
    // printf("[deleteOldestLogFile] attempting to delete log file: %s\n", satioFileData.log_filepath);
    sd_remove(satioFileData.log_filepath);
    if (!sd_exists(satioFileData.log_filepath)) {
        printf("[deleteOldestLogFile] log file deleted successfully.\n");
    }
    else {printf("[deleteOldestLogFile] failed to delete log file.\n");}
  }
}

void printLogLine(const char* line_str) {
    // Critical! This may take a moment, yield for other tasks!
    yieldForTasks();

    // printf("[printLogLine]\n");
    size_t line_len = strlen(line_str);

    // Check disk space
    if (!isAvailableBytes(line_len + 1)) {printf("No more diskspace available!\n"); return;}

    // Select log filename
    deleteOldestLogFile();
    if (getLogFile(1)==false) {createNewLogFilename();}
    
    // Ensure file exists (create if needed)
    FILE* f = sd_fopen(satioFileData.log_filepath, "a");
    if (f) fclose(f);

    // Check file size
    uint64_t file_size = sd_file_size(satioFileData.log_filepath);
    // printf("[printLogLine] line_size=%zu, file_size=%llu\n", line_len, file_size);
    if (file_size + line_len + 1 > MAX_LOG_FILE_SIZE) {
      printf("[printLogLine] creating new log file..\n");
      deleteOldestLogFile();
      createNewLogFilename();
    }

    // Write to log file
    // printf("[printLogLine] writing to log file: %s\n", satioFileData.log_filepath);
    f = sd_fopen(satioFileData.log_filepath, "a");
    if (f) {
      fputs(line_str, f);
      fputc('\n', f);
      fclose(f);
    }
    else {printf("[printLogLine] file does not exist: %s\n", satioFileData.log_filepath);}
}

bool writeLog() {
    // printf("--------------------------------------\n");
    printf("[writeLog] writing log\n");

    // --------------------------------
    // Log Line: Timestamp & Basic Stat
    // --------------------------------
    String line="";
    // --------------------------------
    // Log Line: Satio
    // --------------------------------
    line = "$SATIO,";
    line=line+ String(satioData.padded_rtc_time_HHMMSS) + ",";
    line=line+ String(satioData.padded_rtc_date_DDMMYYYY) + ",";
    line=line+ String(satioData.padded_rtc_sync_time_HHMMSS) + ",";
    line=line+ String(satioData.padded_rtc_sync_date_DDMMYYYY) + ",";
    line=line+ String(satioData.padded_local_time_HHMMSS) + ",";
    line=line+ String(satioData.padded_local_date_DDMMYYYY) + ",";
    line=line+ String(systemData.uptime_seconds) + ",";
    line=line+ String(satioData.degrees_latitude) + ",";
    line=line+ String(satioData.degrees_longitude) + ",";
    line=line+ String(satioData.user_degrees_latitude) + ",";
    line=line+ String(satioData.user_degrees_longitude) + ",";
    line=line+ String(satioData.system_degrees_latitude) + ",";
    line=line+ String(satioData.system_degrees_longitude) + ",";
    line=line+ String(satioData.altitude) + ",";
    line=line+ String(satioData.speed, 7) + ",";
    line=line+ String(satioData.ground_heading, 7) + ",";
    line=line+ String(insData.ins_latitude, 7) + ",";
    line=line+ String(insData.ins_longitude, 7) + ",";
    line=line+ String(insData.ins_altitude) + ",";
    line=line+ String(insData.ins_heading) + ",";
    line=line+ String(insData.INS_INITIALIZATION_FLAG) + ",";
    line=line+ String(insData.INS_MODE) + ",";
    line=line+ String(insData.INS_FORCED_ON_FLAG) + ",";
    line=line+ String(insData.INS_REQ_GPS_PRECISION) + ",";
    line=line+ String(insData.INS_REQ_HEADING_RANGE_DIFF) + ",";
    line=line+ String(insData.INS_REQ_MIN_SPEED) + ",";
    line=line+ String(insData.INS_USE_GYRO_HEADING) + ",";
    line=line+ String(insData.INS_ENABLED) + ",";
    printLogLine(line.c_str());

    // --------------------------------
    // Log Line: Analog/Digital
    // --------------------------------
    line="$MPLEX0,";
    for (int i=0; i<MAX_AD_MUX_CHANNELS; i++) {line=line+String(ad_mux_0.data[i])+",";}
    printLogLine(line.c_str());
    // --------------------------------
    // Log Line: Port Controller Input
    // --------------------------------
    line="$PCINPT,";
    for (int i=0; i<MAX_MATRIX_SWITCHES; i++) {line=line+String(matrixData.input_value[0][i])+",";}
    printLogLine(line.c_str());
    // --------------------------------
    // Log Line: Gyro0
    // --------------------------------
    line="$GYRO0,";
    line=line+ String(gyroData.gyro_0_acc_x) + ",";
    line=line+ String(gyroData.gyro_0_acc_y) + ",";
    line=line+ String(gyroData.gyro_0_acc_z) + ",";
    line=line+ String(gyroData.gyro_0_ang_x) + ",";
    line=line+ String(gyroData.gyro_0_ang_y) + ",";
    line=line+ String(gyroData.gyro_0_ang_z) + ",";
    line=line+ String(gyroData.gyro_0_gyr_x) + ",";
    line=line+ String(gyroData.gyro_0_gyr_y) + ",";
    line=line+ String(gyroData.gyro_0_gyr_z) + ",";
    line=line+ String(gyroData.gyro_0_mag_x) + ",";
    line=line+ String(gyroData.gyro_0_mag_y) + ",";
    line=line+ String(gyroData.gyro_0_mag_z) + ",";
    printLogLine(line.c_str());

    return true;
}

void printLine(FILE* f, const char* line) {
    if (!f) return;
    // if (!isAvailableBytes(strlen(line) + 1)) {printf("No more diskspace available!\n"); return;}
    fputs(line, f);
    fputc('\n', f);
    // Critical! This may take a moment, yield for other tasks!
    yieldForTasks();
}

char *endptr;

// ---------------------------------------------------------------------------------------------------------------------------
// MAPPING
// ---------------------------------------------------------------------------------------------------------------------------

/**
 * Enum and get tag are for order agnostic tag+data reads/writes for ease of maintenance/ammendments.
 */

typedef enum {
    MAP_MODE,
    FUNCTION_N,
    MAP_CONFIG_1,
    MAP_CONFIG_2,
    MAP_CONFIG_3,
    MAP_CONFIG_4,
    MAP_CONFIG_5,
} mapping_tag_t;

char * getMapTag(int t) {
    switch (t) {
        case MAP_MODE:     return "MAP_MODE";
        case FUNCTION_N:   return "FUNCTION_N";
        case MAP_CONFIG_1: return "MAP_CONFIG_1";
        case MAP_CONFIG_2: return "MAP_CONFIG_2";
        case MAP_CONFIG_3: return "MAP_CONFIG_3";
        case MAP_CONFIG_4: return "MAP_CONFIG_4";
        case MAP_CONFIG_5: return "MAP_CONFIG_5";
        default:           return "?";
    }
}
bool saveMappingFile(const char *filepath) {
    printf("[saveMappingFile] Attempting to save mapping file...\n");

    FILE* f = sd_fopen(filepath, "w");
    if (!f) { printf("[saveMappingFile] Failed to open mapping file.\n"); return false; }
    
    char lineBuf[256];
    for (int i_tag=0; i_tag<MAX_MAPPING_TAGS; i_tag++) {

        for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {
            if      (i_tag==MAP_MODE)     { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d",  getMapTag(MAP_MODE), i_map, (int)mappingData.map_mode[0][i_map]); printLine(f, lineBuf);}
            else if (i_tag==FUNCTION_N)   { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", getMapTag(FUNCTION_N), i_map, (long)mappingData.mapping_config[0][i_map][INDEX_MAP_C0]); printLine(f, lineBuf);}
            else if (i_tag==MAP_CONFIG_1) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", getMapTag(MAP_CONFIG_1), i_map, (long)mappingData.mapping_config[0][i_map][INDEX_MAP_C1]); printLine(f, lineBuf);}
            else if (i_tag==MAP_CONFIG_2) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", getMapTag(MAP_CONFIG_2), i_map, (long)mappingData.mapping_config[0][i_map][INDEX_MAP_C2]); printLine(f, lineBuf);}
            else if (i_tag==MAP_CONFIG_3) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", getMapTag(MAP_CONFIG_3), i_map, (long)mappingData.mapping_config[0][i_map][INDEX_MAP_C3]); printLine(f, lineBuf);}
            else if (i_tag==MAP_CONFIG_4) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", getMapTag(MAP_CONFIG_4), i_map, (long)mappingData.mapping_config[0][i_map][INDEX_MAP_C4]); printLine(f, lineBuf);}
            else if (i_tag==MAP_CONFIG_5) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", getMapTag(MAP_CONFIG_5), i_map, (long)mappingData.mapping_config[0][i_map][INDEX_MAP_C5]); printLine(f, lineBuf);}
        }
    }
    
    fclose(f);
    printf("[saveMappingFile] done.\n");
    return true;
}

bool loadMappingFile(const char *filepath) {
    printf("[loadMappingFile] Attempting to load mapping file...\n");
    
    if (!sd_exists(filepath)) {printf("[loadMappingFile] Could not find mapping file.\n");}
    
    FILE* f = sd_fopen(filepath, "r");
    if (!f) { printf("[loadMappingFile] Failed to open mapping file.\n"); return false; }
    
    override_all_computer_assists();
    set_all_mapping_default(); // avoid mixing current values with loaded values

    char lineBuffer[1024];
    int currentTag = 0;
    while (fgets(lineBuffer, sizeof(lineBuffer), f) != NULL) {
        yieldForTasks();

        size_t len = strlen(lineBuffer);
        while (len > 0 && (lineBuffer[len-1] == '\n' || lineBuffer[len-1] == '\r')) { lineBuffer[--len] = '\0'; }
        if (len == 0) continue;
        // printf("Processing Tag Token Number: %d (data: %s)\n", currentTag, lineBuffer); // uncomment to debug
        
        char *token = strtok(lineBuffer, ",");
        int tokenCount = 0;
        signed int tag_index=-1;
        for (int i_find_tag=0; i_find_tag<MAX_MAPPING_TAGS; i_find_tag++) {if (strcmp(getMapTag(i_find_tag), token)==0) {tag_index=i_find_tag; break;}}
        if (tag_index==-1) {printf("Unrecognized tag found in mapping file: %s\n", token); continue;}

        String data_0; String data_1; String data_2;
        token = strtok(NULL, ","); // remove tag

        while (token != NULL) {if (tokenCount==0) {data_0=token;} else if (tokenCount==1) {data_1=token;} else if (tokenCount==2) {data_2=token;} token = strtok(NULL, ","); tokenCount++;}
        if      (tag_index==MAP_MODE)     {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {mappingData.map_mode[0][atoi(data_0.c_str())]=atoi(data_1.c_str());}}
        else if (tag_index==FUNCTION_N)   {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][INDEX_MAP_C0]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==MAP_CONFIG_1) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][INDEX_MAP_C1]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==MAP_CONFIG_2) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][INDEX_MAP_C2]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==MAP_CONFIG_3) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][INDEX_MAP_C3]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==MAP_CONFIG_4) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][INDEX_MAP_C4]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==MAP_CONFIG_5) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][INDEX_MAP_C5]=strtol(data_1.c_str(), &endptr, 10);}}
        currentTag++;
    }
    fclose(f);
    if (currentTag == 0) {printf("$MAPPINGLOADFAILED\n"); return false;}
    printf("$MAPPINGLOADED\n");
    return true;
}

bool deleteMappingFile(const char *filepath) {
    if (sd_exists(filepath)) {if (sd_remove(filepath)) {printf("$MAPPINGDELETED\n"); return true;}}
    printf("$MAPPINGDELETEFAILED\n");
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------------
// MATRIX
// ---------------------------------------------------------------------------------------------------------------------------

/**
 * Enum and get tag are for order agnostic tag+data reads/writes for ease of maintenance/ammendments.
 */

typedef enum {
    SWITCH_PORT,
    SWITCH_FUNCTION,
    FUNCTION_X,
    FUNCTION_Y,
    FUNCTION_Z,
    FUNCTION_OPERATOR,
    FUNCTION_INVERT,
    SWITCH_OUTPUT_MODE,
    SWITCH_PWM_VALUE_0,
    SWITCH_PWM_VALUE_1,
    SWITCH_FLUX,
    COMPUTER_ASSIST,
    MAP_SLOT,
    XYZ_MODE_X,
    XYZ_MODE_Y,
    XYZ_MODE_Z
} matrix_tag_t;

char * getMatrixTag(int t) {
    switch (t) {
        case SWITCH_PORT:        return "SWITCH_PORT";
        case SWITCH_FUNCTION:    return "SWITCH_FUNCTION";
        case FUNCTION_X:         return "FUNCTION_X";
        case FUNCTION_Y:         return "FUNCTION_Y";
        case FUNCTION_Z:         return "FUNCTION_Z";
        case FUNCTION_OPERATOR:  return "FUNCTION_OPERATOR";
        case FUNCTION_INVERT:    return "FUNCTION_INVERT";
        case SWITCH_OUTPUT_MODE: return "SWITCH_OUTPUT_MODE";
        case SWITCH_PWM_VALUE_0: return "SWITCH_PWM_VALUE_0";
        case SWITCH_PWM_VALUE_1: return "SWITCH_PWM_VALUE_1";
        case SWITCH_FLUX:        return "SWITCH_FLUX";
        case COMPUTER_ASSIST:    return "COMPUTER_ASSIST";
        case MAP_SLOT:           return "MAP_SLOT";
        case XYZ_MODE_X:         return "XYZ_MODE_X";
        case XYZ_MODE_Y:         return "XYZ_MODE_Y";
        case XYZ_MODE_Z:         return "XYZ_MODE_Z";
        default:                 return "?";
    }
}

bool saveMatrixFile() {    
    printf("[saveMatrixFile] Attempting to save matrix file.\n");

    FILE* f = sd_fopen(satioFileData.matix_filepaths[satioFileData.i_current_matrix_file_path], "w");
    if (!f) {printf("[saveMatrixFile] Failed to open matrix file.\n"); return false;}
    
    char lineBuf[256];

    // SWITCH_PORT
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", getMatrixTag(SWITCH_PORT), i_switch, (int)matrixData.matrix_port_map[0][i_switch]);
        printLine(f, lineBuf);
    }

    // SWITCH_FUNCTION
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", getMatrixTag(SWITCH_FUNCTION), i_switch, i_func, (int)matrixData.matrix_function[0][i_switch][i_func]);
            printLine(f, lineBuf);
        }
    }

    // FUNCTION_X
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%.10f", getMatrixTag(FUNCTION_X), i_switch, i_func, matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_X]);
            printLine(f, lineBuf);
        }
    }

    // FUNCTION_Y
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%.10f", getMatrixTag(FUNCTION_Y), i_switch, i_func, matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_Y]);
            printLine(f, lineBuf);
        }
    }

    // FUNCTION_Z
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%.10f", getMatrixTag(FUNCTION_Z), i_switch, i_func, matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_Z]);
            printLine(f, lineBuf);
        }
    }

    // FUNCTION_OPERATOR
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", getMatrixTag(FUNCTION_OPERATOR), i_switch, i_func, (int)matrixData.matrix_switch_operator_index[0][i_switch][i_func]);
            printLine(f, lineBuf);
        }
    }

    // FUNCTION_INVERT
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", getMatrixTag(FUNCTION_INVERT), i_switch, i_func, (int)matrixData.matrix_switch_inverted_logic[0][i_switch][i_func]);
            printLine(f, lineBuf);
        }
    }

    // SWITCH_OUTPUT_MODE
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", getMatrixTag(SWITCH_OUTPUT_MODE), i_switch, (int)matrixData.output_mode[0][i_switch]);
        printLine(f, lineBuf);
    }

    // SWITCH_PWM_VALUE_0
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%lu", getMatrixTag(SWITCH_PWM_VALUE_0), i_switch, (unsigned long)matrixData.output_pwm[0][i_switch][0]);
        printLine(f, lineBuf);
    }

    // SWITCH_PWM_VALUE_1
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%lu", getMatrixTag(SWITCH_PWM_VALUE_1), i_switch, (unsigned long)matrixData.output_pwm[0][i_switch][1]);
        printLine(f, lineBuf);
    }

    // SWITCH_FLUX
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", getMatrixTag(SWITCH_FLUX), i_switch, (long)matrixData.flux_value[0][i_switch]);
        printLine(f, lineBuf);
    }

    // COMPUTER_ASSIST
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", getMatrixTag(COMPUTER_ASSIST), i_switch, (int)matrixData.computer_assist[0][i_switch]);
        printLine(f, lineBuf);
    }

    // MAP_SLOT
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", getMatrixTag(MAP_SLOT), i_switch, (int)matrixData.index_mapped_value[0][i_switch]);
        printLine(f, lineBuf);
    }

    // XYZ_MODE_X
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", getMatrixTag(XYZ_MODE_X), i_switch, i_func, (int)matrixData.matrix_function_mode_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_X]);
            printLine(f, lineBuf);
        }
    }

    // XYZ_MODE_Y
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", getMatrixTag(XYZ_MODE_Y), i_switch, i_func, (int)matrixData.matrix_function_mode_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_Y]);
            printLine(f, lineBuf);
        }
    }

    // XYZ_MODE_Z
    for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
        for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
            snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", getMatrixTag(XYZ_MODE_Z), i_switch, i_func, (int)matrixData.matrix_function_mode_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_Z]);
            printLine(f, lineBuf);
        }
    }
    
    fclose(f);
    printf("[saveMatrixFile] done.\n");
    return true;
}

bool loadMatrixFile() {
    printf("[loadMatrixFile] Attempting to load matrix file...\n");

    if (!sd_exists(satioFileData.matix_filepaths[satioFileData.i_current_matrix_file_path])) { printf("[loadMatrixFile] Could not find matrix file.\n"); return false; }
    
    FILE* f = sd_fopen(satioFileData.matix_filepaths[satioFileData.i_current_matrix_file_path], "r");
    if (!f) { printf("[loadMatrixFile] Failed to open matrix file.\n"); return false; }
    
    printf("[loadMatrixFile] attempting to clear matrix...");
    override_all_computer_assists();
    set_all_matrix_default(); // avoid mixing current values with loaded values

    char lineBuffer[1024];
    int currentTag = 0;
    while (fgets(lineBuffer, sizeof(lineBuffer), f) != NULL) {
        yieldForTasks();

        size_t len = strlen(lineBuffer);
        while (len > 0 && (lineBuffer[len-1] == '\n' || lineBuffer[len-1] == '\r')) { lineBuffer[--len] = '\0'; }
        if (len == 0) continue;
        // printf("Processing Tag Token Number: %d (data: %s)\n", currentTag, lineBuffer); // uncomment to debug
        
        char *token = strtok(lineBuffer, ",");
        int tokenCount = 0;
        signed int tag_index = -1;
        for (int i_find_tag=0; i_find_tag<MAX_MATRIX_TAGS; i_find_tag++) {if (strcmp(getMatrixTag(i_find_tag), token)==0) {tag_index=i_find_tag; break;}}
        if (tag_index==-1) {printf("Unrecognized tag found in matrix file: %s\n", token); continue;}

        String data_0; String data_1; String data_2;
        token = strtok(NULL, ","); // remove tag
        while (token != NULL) {if (tokenCount==0) {data_0=token;} else if (tokenCount==1) {data_1=token;} else if (tokenCount==2) {data_2=token;} token = strtok(NULL, ","); tokenCount++;}

        if      (tag_index==SWITCH_PORT) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {matrixData.matrix_port_map[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==SWITCH_FUNCTION) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_function[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==FUNCTION_X) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_X]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==FUNCTION_Y) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_Y]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==FUNCTION_Z) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_Z]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==FUNCTION_OPERATOR) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_switch_operator_index[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==FUNCTION_INVERT) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_switch_inverted_logic[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==SWITCH_OUTPUT_MODE) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {matrixData.output_mode[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==SWITCH_PWM_VALUE_0) {if (str_is_int8(data_0.c_str()) && str_is_uint32(data_1.c_str())) {matrixData.output_pwm[0][atoi(data_0.c_str())][INDEX_MATRIX_SWITCH_PWM_OFF]=strtoul(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==SWITCH_PWM_VALUE_1) {if (str_is_int8(data_0.c_str()) && str_is_uint32(data_1.c_str())) {matrixData.output_pwm[0][atoi(data_0.c_str())][INDEX_MATRIX_SWITCH_PWM_ON]=strtoul(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==SWITCH_FLUX) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {matrixData.flux_value[0][atoi(data_0.c_str())]=strtol(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==COMPUTER_ASSIST) {if (str_is_int8(data_0.c_str()) && str_is_bool(data_1.c_str())) {matrixData.computer_assist[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==MAP_SLOT) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {matrixData.index_mapped_value[0][atoi(data_0.c_str())]=atoi(data_1.c_str());}}
        else if (tag_index==XYZ_MODE_X) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_function_mode_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_X]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==XYZ_MODE_Y) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_function_mode_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_Y]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==XYZ_MODE_Z) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_function_mode_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_Z]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        currentTag++;
    }
    fclose(f);
    if (currentTag == 0) {printf("[loadMatrixFile] No tags found in matrix file.\n"); return false;}
    printf("[loadMatrixFile] done.\n");
    return true;
}

bool deleteMatrixFile() {
    if (sd_exists(satioFileData.matix_filepaths[satioFileData.i_current_matrix_file_path])) {if (sd_remove(satioFileData.matix_filepaths[satioFileData.i_current_matrix_file_path])) {printf("[deleteMatrixFile] done.\n"); return true;}}
    printf("[deleteMatrixFile] Failed.\n");
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------------
// SYSTEM
// ---------------------------------------------------------------------------------------------------------------------------

/**
 * Enum and get tag are for order agnostic tag+data reads/writes for ease of maintenance/ammendments.
 * Symmetry is only required between the enum typedef and getTag() function.
 */
typedef enum {
    SYSTEM_FILE_MATRIX_FILE = 0,
    SYSTEM_FILE_LOAD_MATRIX_ON_STARTUP,
    SYSTEM_FILE_LOGGING,

    SYSTEM_FILE_SERIAL_COMMAND,
    SYSTEM_FILE_OUTPUT_ALL,
    SYSTEM_FILE_OUTPUT_SATIO,
    SYSTEM_FILE_OUTPUT_INS,
    SYSTEM_FILE_OUTPUT_GNGGA,
    SYSTEM_FILE_OUTPUT_GNRMC,
    SYSTEM_FILE_OUTPUT_GPATT,
    SYSTEM_FILE_OUTPUT_MATRIX,
    SYSTEM_FILE_OUTPUT_ADMPLEX0,
    SYSTEM_FILE_OUTPUT_GYRO0,

    SYSTEM_FILE_OUTPUT_SUN,
    SYSTEM_FILE_OUTPUT_MERCURY,
    SYSTEM_FILE_OUTPUT_VENUS,
    SYSTEM_FILE_OUTPUT_EARTH,
    SYSTEM_FILE_OUTPUT_LUNA,
    SYSTEM_FILE_OUTPUT_MARS,
    SYSTEM_FILE_OUTPUT_JUPITER,
    SYSTEM_FILE_OUTPUT_SATURN,
    SYSTEM_FILE_OUTPUT_URANUS,
    SYSTEM_FILE_OUTPUT_NEPTUNE,
    SYSTEM_FILE_OUTPUT_METEORS,

    SYSTEM_FILE_UTC_SECOND_OFFSET,
    SYSTEM_FILE_UTC_AUTO_OFFSET_FLAG,
    SYSTEM_FILE_SET_DATETIME_AUTOMATICALLY,

    SYSTEM_FILE_INS_REQ_GPS_PRECISION,
    SYSTEM_FILE_INS_REQ_MIN_SPEED,
    SYSTEM_FILE_INS_REQ_HEADING_RANGE_DIFF,
    SYSTEM_FILE_INS_MODE,
    SYSTEM_FILE_INS_USE_GYRO_HEADING,

    SYSTEM_FILE_USER_LATITUDE,
    SYSTEM_FILE_USER_LONGITUDE,
    SYSTEM_FILE_USER_SPEED,
    SYSTEM_FILE_USER_GROUND_HEADING,
    SYSTEM_FILE_USER_ALTITUDE,

    SYSTEM_FILE_SATIO_LOCATION_VALUE_MODE,
    SYSTEM_FILE_SATIO_ALTITUDE_VALUE_MODE,
    SYSTEM_FILE_SATIO_SPEED_VALUE_MODE,
    SYSTEM_FILE_SATIO_GROUND_HEADING_VALUE_MODE, 
} system_tag_t;

char * getSystemTag(int t) {
    switch (t) {
        case SYSTEM_FILE_MATRIX_FILE:                    return "MATRIX_FILE";
        case SYSTEM_FILE_LOAD_MATRIX_ON_STARTUP:         return "LOAD_MATRIX_ON_STARTUP";
        case SYSTEM_FILE_LOGGING:                        return "LOGGING";

        case SYSTEM_FILE_SERIAL_COMMAND:                 return "SERIAL_COMMAND";
        case SYSTEM_FILE_OUTPUT_ALL:                     return "OUTPUT_ALL";
        case SYSTEM_FILE_OUTPUT_SATIO:                   return "OUTPUT_SATIO";
        case SYSTEM_FILE_OUTPUT_INS:                     return "OUTPUT_INS";
        case SYSTEM_FILE_OUTPUT_GNGGA:                   return "OUTPUT_GNGGA";
        case SYSTEM_FILE_OUTPUT_GNRMC:                   return "OUTPUT_GNRMC";
        case SYSTEM_FILE_OUTPUT_GPATT:                   return "OUTPUT_GPATT";
        case SYSTEM_FILE_OUTPUT_MATRIX:                  return "OUTPUT_MATRIX";
        case SYSTEM_FILE_OUTPUT_ADMPLEX0:                return "OUTPUT_ADMPLEX0";
        case SYSTEM_FILE_OUTPUT_GYRO0:                   return "OUTPUT_GYRO0";

        case SYSTEM_FILE_OUTPUT_SUN:                     return "OUTPUT_SUN";
        case SYSTEM_FILE_OUTPUT_MERCURY:                 return "OUTPUT_MERCURY";
        case SYSTEM_FILE_OUTPUT_VENUS:                   return "OUTPUT_VENUS";
        case SYSTEM_FILE_OUTPUT_EARTH:                   return "OUTPUT_EARTH";
        case SYSTEM_FILE_OUTPUT_LUNA:                    return "OUTPUT_LUNA";
        case SYSTEM_FILE_OUTPUT_MARS:                    return "OUTPUT_MARS";
        case SYSTEM_FILE_OUTPUT_JUPITER:                 return "OUTPUT_JUPITER";
        case SYSTEM_FILE_OUTPUT_SATURN:                  return "OUTPUT_SATURN";
        case SYSTEM_FILE_OUTPUT_URANUS:                  return "OUTPUT_URANUS";
        case SYSTEM_FILE_OUTPUT_NEPTUNE:                 return "OUTPUT_NEPTUNE";
        case SYSTEM_FILE_OUTPUT_METEORS:                 return "OUTPUT_METEORS";

        case SYSTEM_FILE_UTC_SECOND_OFFSET:              return "UTC_SECOND_OFFSET";
        case SYSTEM_FILE_UTC_AUTO_OFFSET_FLAG:           return "UTC_AUTO_OFFSET_FLAG";
        case SYSTEM_FILE_SET_DATETIME_AUTOMATICALLY:     return "SET_DATETIME_AUTOMATICALLY";

        case SYSTEM_FILE_INS_REQ_GPS_PRECISION:          return "INS_REQ_GPS_PRECISION";
        case SYSTEM_FILE_INS_REQ_MIN_SPEED:              return "INS_REQ_MIN_SPEED";
        case SYSTEM_FILE_INS_REQ_HEADING_RANGE_DIFF:     return "INS_REQ_HEADING_RANGE_DIFF";
        case SYSTEM_FILE_INS_MODE:                       return "INS_MODE";
        case SYSTEM_FILE_INS_USE_GYRO_HEADING:           return "INS_USE_GYRO_HEADING";

        case SYSTEM_FILE_USER_LATITUDE:                  return "USER_LATITUDE";
        case SYSTEM_FILE_USER_LONGITUDE:                 return "USER_LONGITUDE";
        case SYSTEM_FILE_USER_SPEED:                     return "USER_SPEED";
        case SYSTEM_FILE_USER_GROUND_HEADING:                   return "USER_HEADING";
        case SYSTEM_FILE_USER_ALTITUDE:                  return "USER_ALTITUDE";

        case SYSTEM_FILE_SATIO_LOCATION_VALUE_MODE:      return "SATIO_LOCATION_VALUE_MODE";
        case SYSTEM_FILE_SATIO_ALTITUDE_VALUE_MODE:      return "SATIO_ALTITUDE_VALUE_MODE";
        case SYSTEM_FILE_SATIO_SPEED_VALUE_MODE:         return "SATIO_SPEED_VALUE_MODE";
        case SYSTEM_FILE_SATIO_GROUND_HEADING_VALUE_MODE:return "SATIO_GROUND_HEADING_VALUE_MODE";

        default:                                         return "?";
    }
}

bool saveSystemFile(const char *filepath) {
    printf("[saveSystemFile] Attempting to save system file...\n");

    FILE* f = sd_fopen(filepath, "w");
    if (!f) { printf("[saveSystemFile] fopen failed, errno: %d (%s)\n", errno, strerror(errno)); return false; }
    
    // Use heap-allocated buffer to avoid stack overflow
    char* lineBuf = (char*)malloc(256);
    if (!lineBuf) {fclose(f); printf("[saveSystemFile] Failed to allocate memory.\n"); return false;}
    
    #define WRITE_INT_TAG(idx, val)  snprintf(lineBuf, 256, "%s,%d",   getSystemTag(idx), (int)(val));    printLine(f, lineBuf)
    #define WRITE_LONG_TAG(idx, val) snprintf(lineBuf, 256, "%s,%ld",  getSystemTag(idx), (long)(val));   printLine(f, lineBuf)
    #define WRITE_DBL_TAG(idx, val)  snprintf(lineBuf, 256, "%s,%.6f", getSystemTag(idx), (double)(val)); printLine(f, lineBuf)
    #define WRITE_STR_TAG(idx, val)  snprintf(lineBuf, 256, "%s,%s",   getSystemTag(idx), (val));         printLine(f, lineBuf)
    
    WRITE_INT_TAG(SYSTEM_FILE_MATRIX_FILE, satioFileData.i_current_matrix_file_path);
    WRITE_INT_TAG(SYSTEM_FILE_LOAD_MATRIX_ON_STARTUP, matrixData.load_matrix_on_startup);
    WRITE_INT_TAG(SYSTEM_FILE_LOGGING, systemData.logging_enabled);
 
    WRITE_INT_TAG(SYSTEM_FILE_SERIAL_COMMAND, systemData.serial_command);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_ALL, systemData.output_satio_all);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_SATIO, systemData.output_satio_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_GNGGA, systemData.output_ins_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_GNRMC, systemData.output_gngga_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_GPATT, systemData.output_gnrmc_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_INS, systemData.output_gpatt_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_MATRIX, systemData.output_matrix_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_ADMPLEX0, systemData.output_admplex0_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_GYRO0, systemData.output_gyro_0_enabled);

    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_SUN, systemData.output_sun_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_MERCURY, systemData.output_mercury_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_VENUS, systemData.output_venus_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_EARTH, systemData.output_earth_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_LUNA, systemData.output_luna_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_MARS, systemData.output_mars_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_JUPITER, systemData.output_jupiter_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_SATURN, systemData.output_saturn_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_URANUS, systemData.output_uranus_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_NEPTUNE, systemData.output_neptune_enabled);
    WRITE_INT_TAG(SYSTEM_FILE_OUTPUT_METEORS, systemData.output_meteors_enabled);

    WRITE_LONG_TAG(SYSTEM_FILE_UTC_SECOND_OFFSET, satioData.utc_second_offset);
    WRITE_INT_TAG(SYSTEM_FILE_UTC_AUTO_OFFSET_FLAG, satioData.utc_auto_offset_flag);
    WRITE_INT_TAG(SYSTEM_FILE_SET_DATETIME_AUTOMATICALLY, satioData.set_time_automatically);

    WRITE_DBL_TAG(SYSTEM_FILE_INS_REQ_GPS_PRECISION, insData.INS_REQ_GPS_PRECISION);
    WRITE_DBL_TAG(SYSTEM_FILE_INS_REQ_MIN_SPEED, insData.INS_REQ_MIN_SPEED);
    WRITE_DBL_TAG(SYSTEM_FILE_INS_REQ_HEADING_RANGE_DIFF, insData.INS_REQ_HEADING_RANGE_DIFF);
    WRITE_DBL_TAG(SYSTEM_FILE_INS_MODE, insData.INS_MODE);
    WRITE_INT_TAG(SYSTEM_FILE_INS_USE_GYRO_HEADING, insData.INS_USE_GYRO_HEADING);

    WRITE_DBL_TAG(SYSTEM_FILE_USER_LATITUDE, satioData.user_degrees_latitude);
    WRITE_DBL_TAG(SYSTEM_FILE_USER_LONGITUDE, satioData.user_degrees_longitude);
    WRITE_DBL_TAG(SYSTEM_FILE_USER_SPEED, satioData.user_speed);
    WRITE_DBL_TAG(SYSTEM_FILE_USER_GROUND_HEADING, satioData.user_ground_heading);
    WRITE_DBL_TAG(SYSTEM_FILE_USER_ALTITUDE, satioData.user_altitude);

    WRITE_INT_TAG(SYSTEM_FILE_SATIO_LOCATION_VALUE_MODE, satioData.location_value_mode);
    WRITE_INT_TAG(SYSTEM_FILE_SATIO_ALTITUDE_VALUE_MODE, satioData.altitude_value_mode);
    WRITE_INT_TAG(SYSTEM_FILE_SATIO_SPEED_VALUE_MODE, satioData.speed_value_mode);
    WRITE_INT_TAG(SYSTEM_FILE_SATIO_GROUND_HEADING_VALUE_MODE, satioData.ground_heading_value_mode);
    
    #undef WRITE_INT_TAG
    #undef WRITE_LONG_TAG
    #undef WRITE_DBL_TAG
    #undef WRITE_STR_TAG
    
    free(lineBuf);
    fclose(f);
    printf("[saveSystemFile] done.\n");
    return true;
}

bool loadSystemFile(const char *filepath) {
    printf("[loadSystemFile] Attempting to load system file..\n");

    if (!sd_exists(filepath)) {printf("[loadSystemFile] Could not find system file.\n"); return false;}

    FILE* f = sd_fopen(filepath, "r");
    if (!f) {printf("[loadSystemFile] Could not open system file.\n"); return false;}

    char lineBuffer[256];

    #define READ_BOOL_TAG(idx, var) if (tag_index == idx) { if (str_is_bool(val)) { var = atoi(val); } }
    #define READ_INT8_TAG(idx, var) if (tag_index == idx) { if (str_is_int8(val)) { var = atoi(val); } }
    #define READ_LONG_TAG(idx, var) if (tag_index == idx) { if (str_is_long(val)) { var = strtol(val, &endptr, 10); } }
    #define READ_DBL_TAG(idx, var)  if (tag_index == idx) { if (str_is_double(val)) { var = strtod(val, &endptr); } }

    while (fgets(lineBuffer, sizeof(lineBuffer), f) != NULL) {

        yieldForTasks();

        size_t len = strlen(lineBuffer);
        while (len > 0 && (lineBuffer[len-1] == '\n' || lineBuffer[len-1] == '\r')) lineBuffer[--len] = '\0';
        if (len == 0) continue;

        char *token = strtok(lineBuffer, ",");
        if (!token) continue;

        int tag_index = -1;
        for (int i = 0; i < MAX_SYSTEM_TAGS; i++) {if (strcmp(getSystemTag(i), token) == 0) {tag_index = i; break;}}

        if (tag_index == -1) {printf("Unrecognized tag: %s\n", token); continue;}

        char *val = strtok(NULL, ",");
        if (!val) continue;

        char *endptr;

        READ_INT8_TAG(SYSTEM_FILE_MATRIX_FILE, satioFileData.i_current_matrix_file_path);
        READ_BOOL_TAG(SYSTEM_FILE_LOAD_MATRIX_ON_STARTUP, matrixData.load_matrix_on_startup);
        READ_BOOL_TAG(SYSTEM_FILE_LOGGING, systemData.logging_enabled);

        READ_BOOL_TAG(SYSTEM_FILE_SERIAL_COMMAND, systemData.serial_command);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_ALL, systemData.output_satio_all);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_SATIO, systemData.output_satio_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_INS, systemData.output_ins_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_GNGGA, systemData.output_gngga_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_GNRMC, systemData.output_gnrmc_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_GPATT, systemData.output_gpatt_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_MATRIX, systemData.output_matrix_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_ADMPLEX0, systemData.output_admplex0_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_GYRO0, systemData.output_gyro_0_enabled);

        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_SUN, systemData.output_sun_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_MERCURY, systemData.output_mercury_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_VENUS, systemData.output_venus_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_EARTH, systemData.output_earth_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_LUNA, systemData.output_luna_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_MARS, systemData.output_mars_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_JUPITER, systemData.output_jupiter_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_SATURN, systemData.output_saturn_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_URANUS, systemData.output_uranus_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_NEPTUNE, systemData.output_neptune_enabled);
        READ_BOOL_TAG(SYSTEM_FILE_OUTPUT_METEORS, systemData.output_meteors_enabled);

        READ_LONG_TAG(SYSTEM_FILE_UTC_SECOND_OFFSET, satioData.utc_second_offset);
        READ_BOOL_TAG(SYSTEM_FILE_UTC_AUTO_OFFSET_FLAG, satioData.utc_auto_offset_flag);
        READ_BOOL_TAG(SYSTEM_FILE_SET_DATETIME_AUTOMATICALLY, satioData.set_time_automatically);

        READ_DBL_TAG(SYSTEM_FILE_INS_REQ_GPS_PRECISION, insData.INS_REQ_GPS_PRECISION);
        READ_DBL_TAG(SYSTEM_FILE_INS_REQ_MIN_SPEED, insData.INS_REQ_MIN_SPEED);
        READ_DBL_TAG(SYSTEM_FILE_INS_REQ_HEADING_RANGE_DIFF, insData.INS_REQ_HEADING_RANGE_DIFF);
        READ_DBL_TAG(SYSTEM_FILE_INS_MODE, insData.INS_MODE);
        READ_BOOL_TAG(SYSTEM_FILE_INS_USE_GYRO_HEADING, insData.INS_USE_GYRO_HEADING);

        READ_DBL_TAG(SYSTEM_FILE_USER_LATITUDE, satioData.user_degrees_latitude);
        READ_DBL_TAG(SYSTEM_FILE_USER_LONGITUDE, satioData.user_degrees_longitude);
        READ_DBL_TAG(SYSTEM_FILE_USER_SPEED, satioData.user_speed);
        READ_DBL_TAG(SYSTEM_FILE_USER_GROUND_HEADING, satioData.user_ground_heading);
        READ_DBL_TAG(SYSTEM_FILE_USER_ALTITUDE, satioData.user_altitude);

        READ_INT8_TAG(SYSTEM_FILE_SATIO_LOCATION_VALUE_MODE, satioData.location_value_mode);
        READ_INT8_TAG(SYSTEM_FILE_SATIO_ALTITUDE_VALUE_MODE, satioData.altitude_value_mode);
        READ_INT8_TAG(SYSTEM_FILE_SATIO_SPEED_VALUE_MODE, satioData.speed_value_mode);
        READ_INT8_TAG(SYSTEM_FILE_SATIO_GROUND_HEADING_VALUE_MODE, satioData.ground_heading_value_mode);
    }

    #undef READ_INT_TAG
    #undef READ_INT8_TAG
    #undef READ_LONG_TAG
    #undef READ_DBL_TAG

    fclose(f);

    printf("[loadSystemFile] done.\n");
    return true;
}

bool deleteSystemFile(const char *filepath) {
    if (sd_exists(filepath)) {if (sd_remove(filepath)) {printf("[deleteSystemFile] done.\n"); return true;}}
    printf("[deleteSystemFile] Failed.\n");
    return false;
}

bool ran_startup_file_operations = false;

void sdcardFlagHandler() {

  // ---- MOUNT ----

//   if (sdcardFlagData.unmount_sdcard_flag==true) {
//     printf("card unmount flag detected\n");
//     sdcardData.allow_mount=false; // override allow_mount (prevent remounting on unmount flag).
//     sdcardFlagData.no_delay_flag=false; // reset flag
//     sdcardFlagData.unmount_sdcard_flag=false; // reset flag
//     sdcard_unmount();
//   }
//   else if (sdcardFlagData.mount_sdcard_flag==true) {
//     printf("card mount flag detected\n");
//     sdcardData.allow_mount=true; // override allow_mount on mount flag.
//     sdcardFlagData.no_delay_flag=false;
//     sdcardFlagData.mount_sdcard_flag=false;
//     sdcard_mount();
//   }

  if (sdcardData.sdcard_mounted==false) {
  }

  else if (sdcardData.sdcard_mounted==true) {

    // ---- MAPPING ----

    if (sdcardFlagData.save_mapping) {
      printf("saving mapping ...\n");
      if (saveMappingFile(satioFileData.mapping_filepath))
        {printf("saved mapping successfully.\n"); set_storage_success_flag(true);}
      else {printf("save mapping failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.save_mapping=false;
    }
    else if (sdcardFlagData.load_mapping) {
      printf("loading mapping...\n");
      if (loadMappingFile(satioFileData.mapping_filepath))
        {printf("loaded mapping successfully.\n"); set_storage_success_flag(true);}
      else {printf("load mapping failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.load_mapping=false;
    }
    else if (sdcardFlagData.delete_mapping) {
      printf("deleting mapping...\n");
      if (deleteMappingFile(satioFileData.mapping_filepath))
        {printf("deleted mapping successfully.\n"); set_storage_success_flag(true);}
      else {printf("delete mapping failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.delete_mapping=false;
    }

    // ---- MATRIX ----

    else if (sdcardFlagData.save_matrix) {
      printf("saving matrix ...\n");
      if (saveMatrixFile())
        {printf("saved matrix successfully.\n"); set_storage_success_flag(true);}
      else {printf("save matrix failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.save_matrix=false;
    }
    else if (sdcardFlagData.load_matrix) {
      printf("loading matrix...\n");
      if (loadMatrixFile())
        {printf("loaded matrix successfully.\n"); set_storage_success_flag(true);}
      else {printf("load matrix failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.load_matrix=false;
    }
    else if (sdcardFlagData.delete_matrix) {
      printf("deleting matrix...\n");
      if (deleteMatrixFile())
        {printf("deleted matrix successfully.\n"); set_storage_success_flag(true);}
      else {printf("delete matrix failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.delete_matrix=false;
    }

    // ---- SYSTEM ----

    else if (sdcardFlagData.save_system) {
      printf("saving system...\n");
      if (saveSystemFile(satioFileData.system_filepath))
        {printf("saved system successfully.\n"); set_storage_success_flag(true);}
      else {printf("save system failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.save_system=false;
    }
    else if (sdcardFlagData.delete_system) {
      printf("deleting system...\n");
      if (deleteSystemFile(satioFileData.system_filepath))
        {printf("deleted system successfully.\n"); set_storage_success_flag(true);}
      else {printf("delete system failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.delete_system=false;
    }

    else if (sdcardFlagData.load_system) {
      // load system
      printf("loading system...\n");
      if (loadSystemFile(satioFileData.system_filepath))
        {printf("loaded system successfully.\n"); set_storage_success_flag(true);}
      else {printf("load system failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.load_system=false;

      if (ran_startup_file_operations==false && matrixData.load_matrix_on_startup==true) {
        // load mapping
        printf("loading mapping...\n");
        if (loadMappingFile(satioFileData.mapping_filepath))
            {printf("loaded mapping successfully.\n"); set_storage_success_flag(true);}
        else {printf("load mapping failed.\n"); set_storage_success_flag(false);}
        sdcardFlagData.load_mapping=false;

        // load matrix
        printf("loading matrix on startup...\n");
        if (loadMatrixFile())
          {printf("loaded matrix successfully.\n"); set_storage_success_flag(true);}
        else {printf("load matrix failed.\n"); set_storage_success_flag(false);}
        sdcardFlagData.load_matrix=false;

        // end
        ran_startup_file_operations = true;
      }
      sdcardFlagData.no_delay_flag=false;
    }

    // ---- LOG ----

    else if (sdcardFlagData.write_log) {
      if (writeLog())
        {printf("[log] successfull.\n"); set_storage_success_flag(true);}
      else {printf("[log] failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.write_log=false;
    }
  }
//   clearSDMMCArgStruct();
}