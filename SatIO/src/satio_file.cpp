/*
  SatioFile - Written By Benjamin Jack Cullen.
*/

#include "satio_file.h"
#include <Arduino.h>
#include <FS.h>
#include "SD_MMC.h"
#include "SPIFFS.h"
#include "matrix.h"
#include <esp_task_wdt.h>
#include "strval.h"
#include "satio.h"
#include "system_data.h"
#include "matrix.h"
#include "custommapping.h"
#include "ins.h"

struct satioFileStruct satioFileData = {
    .matrix_tags=
    {
        "SWITCH_PORT",        // 0
        "SWITCH_FUNCTION",    // 1
        "FUNCTION_X",         // 2
        "FUNCTION_Y",         // 3
        "FUNCTION_Z",         // 4
        "FUNCTION_OPERATOR",  // 5
        "FUNCTION_INVERT",    // 6
        "SWITCH_OUTPUT_MODE", // 7
        "SWITCH_PWM_VALUE_0", // 8
        "SWITCH_PWM_VALUE_1", // 9
        "SWITCH_FLUX",        // 10
        "COMPUTER_ASSIST",    // 11
        
    },
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
    .matrix_file_slots={0},
    .current_matrix_filepath="/MATRIX/MATRIX_0.csv", // default
    .mapping_tags=
    {
        "MAP_MODE",     // 0
        "INDEX_MAPPED", // 1
        "FUNCTION_N",   // 2
        "MAP_CONFIG_1", // 3
        "MAP_CONFIG_2", // 4
        "MAP_CONFIG_3", // 5
        "MAP_CONFIG_4", // 6
        "MAP_CONFIG_5", // 7   
    },
    .mapping_filepath="/MAPPING/mapping_conf.csv",
    .system_tags=
    {
        "SERIAL_COMMAND",             // 0
        "OUTPUT_ALL",                 // 1
        "OUTPUT_SATIO",               // 2
        "OUTPUT_INS",                 // 3
        "OUTPUT_GNGGA",               // 4
        "OUTPUT_GNRMC",               // 5
        "OUTPUT_GPATT",               // 6
        "OUTPUT_MATRIX",              // 7
        "OUTPUT_ADMPLEX0",            // 8
        "OUTPUT_GYRO0",               // 9
        "OUTPUT_SUN",                 // 10
        "OUTPUT_MOON",                // 11
        "OUTPUT_MERCURY",             // 12
        "OUTPUT_VENUS",               // 13
        "OUTPUT_MARS",                // 14
        "OUTPUT_JUPITER",             // 15
        "OUTPUT_SATURN",              // 16
        "OUTPUT_URANUS",              // 17
        "OUTPUT_NEPTUNE",             // 18
        "OUTPUT_METEORS",             // 19
        "COORDINATE_CONVERSION_MODE", // 20
        "SPEED_CONVERSION_MODE",      // 21
        "ALTITUDE_CONVERSION_MODE",   // 22
        "UTC_SECOND_OFFSET",          // 23
        "UTC_AUTO_OFFSET_FLAG",       // 24
        "SET_DATETIME_AUTOMATICALLY", // 25
        "INS_REQ_GPS_PRECISION",      // 26
        "INS_REQ_MIN_SPEED",          // 27
        "INS_REQ_HEADING_RANGE_DIFF", // 28
        "INS_MODE",                   // 29
        "INS_USE_GYRO_HEADING",       // 30
        "MATRIX_FILE",                // 31
        "LOAD_MATRIX_ON_STARTUP",     // 32
    },
    .system_filepath="/SYSTEM/system_conf.csv",

};

void printLine(File f, String line) {
    line = line+"\n";
    Serial.print(line);
    f.print(line);
}

char *endptr;

bool saveMappingFile(FS &fs, const char *filepath) {
    Serial.println("saveMappingFile");
    File f = fs.open(filepath, "w", true);
    if (!f) return false;
    if (fs.exists(filepath)) {
        for (int i_tag=0; i_tag<MAX_MAPPING_TAGS; i_tag++) {
            String line="";
            if      (i_tag==0) {for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {line = String(satioFileData.mapping_tags[i_tag]) + String("," + String(i_map) + "," + String(mappingData.map_mode[0][i_map])).c_str(); printLine(f, line);}}
            else if (i_tag==1) {for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {line = String(satioFileData.mapping_tags[i_tag]) + String("," + String(i_map) + "," + String(mappingData.index_mapped_value[0][i_map])).c_str(); printLine(f, line);}}
            else if (i_tag==2) {for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {line = String(satioFileData.mapping_tags[i_tag]) + String("," + String(i_map) + "," + String(mappingData.mapping_config[0][i_map][0])).c_str(); printLine(f, line);}}
            else if (i_tag==3) {for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {line = String(satioFileData.mapping_tags[i_tag]) + String("," + String(i_map) + "," + String(mappingData.mapping_config[0][i_map][1])).c_str(); printLine(f, line);}}
            else if (i_tag==4) {for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {line = String(satioFileData.mapping_tags[i_tag]) + String("," + String(i_map) + "," + String(mappingData.mapping_config[0][i_map][2])).c_str(); printLine(f, line);}}
            else if (i_tag==5) {for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {line = String(satioFileData.mapping_tags[i_tag]) + String("," + String(i_map) + "," + String(mappingData.mapping_config[0][i_map][3])).c_str(); printLine(f, line);}}
            else if (i_tag==6) {for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {line = String(satioFileData.mapping_tags[i_tag]) + String("," + String(i_map) + "," + String(mappingData.mapping_config[0][i_map][4])).c_str(); printLine(f, line);}}
            else if (i_tag==7) {for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {line = String(satioFileData.mapping_tags[i_tag]) + String("," + String(i_map) + "," + String(mappingData.mapping_config[0][i_map][5])).c_str(); printLine(f, line);}}
        }
    }
    else {return false;}
    f.close();
    Serial.println();
    return true;
}

bool loadMappingFile(FS &fs, const char *filepath) {
    if (fs.exists(filepath)) {
        File f = fs.open(filepath, "r", false);
        if (!f) return false;
        override_all_computer_assists();
        set_all_mapping_default(); // avoid mixing current values with loaded values
        char lineBuffer[1024];
        int currentTag = 0;
        while (f.available()) {
            int len = f.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer) - 1);
            if (len <= 0) break;
            lineBuffer[len] = '\0'; // null-terminate
            if (strlen(lineBuffer) == 0) continue;
            Serial.println("Processing Tag Token Number: " + String(currentTag) + " (data: " + String(lineBuffer) + ")"); // uncomment to debug
            char *commaToken = strtok(lineBuffer, ",");
            int tokenCount = 0;
            int tag_index;
            for (int i_find_tag=0; i_find_tag<MAX_MAPPING_TAGS; i_find_tag++) {if (strcmp(satioFileData.mapping_tags[i_find_tag], commaToken)==0) {tag_index=i_find_tag; break;}}
            // Serial.println("Tag Found: " + String(satioFileData.mapping_tags[tag_index]));
            String data_0; String data_1; String data_2;
            commaToken = strtok(NULL, ","); // remove tag
            while (commaToken != NULL) {if (tokenCount==0) {data_0=commaToken;} else if (tokenCount==1) {data_1=commaToken;} else if (tokenCount==2) {data_2=commaToken;} commaToken = strtok(NULL, ","); tokenCount++;}
            if      (tag_index==0) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {mappingData.map_mode[0][atoi(data_0.c_str())]=atoi(data_1.c_str());}}
            else if (tag_index==1) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {mappingData.index_mapped_value[0][atoi(data_0.c_str())]=atoi(data_1.c_str());}}
            else if (tag_index==2) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][0]=strtol(data_1.c_str(), &endptr, 10);}}
            else if (tag_index==3) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][1]=strtol(data_1.c_str(), &endptr, 10);}}
            else if (tag_index==4) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][2]=strtol(data_1.c_str(), &endptr, 10);}}
            else if (tag_index==5) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][3]=strtol(data_1.c_str(), &endptr, 10);}}
            else if (tag_index==6) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][4]=strtol(data_1.c_str(), &endptr, 10);}}
            else if (tag_index==7) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][5]=strtol(data_1.c_str(), &endptr, 10);}}
            currentTag++;
        }
        f.close();
        if (currentTag == 0) {Serial.println("No valid lines found"); return false;}
        Serial.println();
        return true;
    }
    else {return false;}
}

bool deleteMappingFile(FS &fs, const char *filepath) {
    if (fs.exists(filepath)) {if (fs.remove(filepath)) {return true;}}
    return false;
}

// todo: mapping data will saved with matrix data
bool saveMatrixFile(FS &fs, const char *filepath) {    
    Serial.println("saveMatrixFile");
    File f = fs.open(filepath, "w", true);
    if (!f) return false;
    if (fs.exists(filepath)) {
        for (int i_tag=0; i_tag<MAX_MATRIX_TAGS; i_tag++) {
            String line="";
            if  (i_tag==0) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(matrixData.matrix_port_map[0][i_switch])).c_str();
                printLine(f, line);}}
            else if  (i_tag==1) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(i_func) + "," + String(matrixData.matrix_function[0][i_switch][i_func])).c_str();
                printLine(f, line);}}}
            else if  (i_tag==2) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(i_func) + "," + String(matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_X], 10)).c_str();
                printLine(f, line);}}}
            else if  (i_tag==3) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(i_func) + "," + String(matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_Y], 10)).c_str();
                printLine(f, line);}}}
            else if  (i_tag==4) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(i_func) + "," + String(matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_Z], 10)).c_str();
                printLine(f, line);}}}
            else if  (i_tag==5) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(i_func) + "," + String(matrixData.matrix_switch_operator_index[0][i_switch][i_func])).c_str();
                printLine(f, line);}}}
            else if  (i_tag==6) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(i_func) + "," + String(matrixData.matrix_switch_inverted_logic[0][i_switch][i_func])).c_str();
                printLine(f, line);}}}
            else if  (i_tag==7) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(matrixData.output_mode[0][i_switch])).c_str();
                printLine(f, line);}}
            else if  (i_tag==8) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(matrixData.output_pwm[0][i_switch][0])).c_str();
                printLine(f, line);}}
            else if  (i_tag==9) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(matrixData.output_pwm[0][i_switch][1])).c_str();
                printLine(f, line);}}
            else if  (i_tag==10) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(matrixData.flux_value[0][i_switch])).c_str();
                printLine(f, line);}}
            else if  (i_tag==11) {for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {line = String(satioFileData.matrix_tags[i_tag]) + String("," + String(i_switch) + "," + String(matrixData.computer_assist[0][i_switch])).c_str();
                printLine(f, line);}}
        }
    }
    else {return false;}
    f.close();
    Serial.println();
    return true;
}

bool loadMatrixFile(FS &fs, const char *filepath) {
    if (fs.exists(filepath)) {
        File f = fs.open(filepath, "r", false);
        if (!f) return false;
        override_all_computer_assists();
        set_all_matrix_default(); // avoid mixing current values with loaded values
        char lineBuffer[1024];
        int currentTag = 0;
        while (f.available()) {
            int len = f.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer) - 1);
            if (len <= 0) break;
            lineBuffer[len] = '\0'; // null-terminate
            if (strlen(lineBuffer) == 0) continue;
            Serial.println("Processing Tag Token Number: " + String(currentTag) + " (data: " + String(lineBuffer) + ")"); // uncomment to debug
            char *commaToken = strtok(lineBuffer, ",");
            int tokenCount = 0;
            int tag_index;
            for (int i_find_tag=0; i_find_tag<MAX_MATRIX_TAGS; i_find_tag++) {if (strcmp(satioFileData.matrix_tags[i_find_tag], commaToken)==0) {tag_index=i_find_tag; break;}}
            // Serial.println("Tag Found: " + String(satioFileData.matrix_tags[tag_index]));
            String data_0; String data_1; String data_2;
            commaToken = strtok(NULL, ","); // remove tag
            while (commaToken != NULL) {if (tokenCount==0) {data_0=commaToken;} else if (tokenCount==1) {data_1=commaToken;} else if (tokenCount==2) {data_2=commaToken;} commaToken = strtok(NULL, ","); tokenCount++;}
            if      (tag_index==0) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {matrixData.matrix_port_map[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==1) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_function[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==2) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_X]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==3) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_Y]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==4) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_Z]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==5) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_switch_operator_index[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==6) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_switch_inverted_logic[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==7) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {matrixData.output_mode[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==8) {if (str_is_int8(data_0.c_str()) && str_is_uint32(data_1.c_str())) {matrixData.output_pwm[0][atoi(data_0.c_str())][0]=strtoul(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==9) {if (str_is_int8(data_0.c_str()) && str_is_uint32(data_1.c_str())) {matrixData.output_pwm[0][atoi(data_0.c_str())][1]=strtoul(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==10) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {matrixData.flux_value[0][atoi(data_0.c_str())]=strtol(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            else if (tag_index==11) {if (str_is_int8(data_0.c_str()) && str_is_bool(data_1.c_str())) {matrixData.computer_assist[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
            currentTag++;
        }
        f.close();
        if (currentTag == 0) {Serial.println("No valid lines found"); return false;}
        Serial.println();
        return true;
    }
    else {return false;}
}

bool deleteMatrixFile(FS &fs, const char *filepath) {
    if (fs.exists(filepath)) {if (fs.remove(filepath)) {return true;}}
    return false;
}

bool saveSystemFile(FS &fs, const char *filepath) {
    Serial.println("saveSystemFile");
    File f = fs.open(filepath, "w", true);
    if (!f) return false;
    if (fs.exists(filepath)) {
        for (int i_tag=0; i_tag<MAX_SYSTEM_TAGS; i_tag++) {
            String line="";
            if       (i_tag==0) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.serial_command)); printLine(f, line);}
            else if  (i_tag==1) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_satio_all)); printLine(f, line);}
            else if  (i_tag==2) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_satio_enabled)); printLine(f, line);}
            else if  (i_tag==3) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_ins_enabled)); printLine(f, line);}
            else if  (i_tag==4) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_gngga_enabled)); printLine(f, line);}
            else if  (i_tag==5) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_gnrmc_enabled)); printLine(f, line);}
            else if  (i_tag==6) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_gpatt_enabled)); printLine(f, line);}
            else if  (i_tag==7) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_matrix_enabled)); printLine(f, line);}
            else if  (i_tag==8) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_admplex0_enabled)); printLine(f, line);}
            else if  (i_tag==9) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_gyro_0_enabled)); printLine(f, line);}
            else if  (i_tag==10) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_sun_enabled)); printLine(f, line);}
            else if  (i_tag==11) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_moon_enabled)); printLine(f, line);}
            else if  (i_tag==12) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_mercury_enabled)); printLine(f, line);}
            else if  (i_tag==13) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_venus_enabled)); printLine(f, line);}
            else if  (i_tag==14) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_mars_enabled)); printLine(f, line);}
            else if  (i_tag==15) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_jupiter_enabled)); printLine(f, line);}
            else if  (i_tag==16) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_saturn_enabled)); printLine(f, line);}
            else if  (i_tag==17) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_uranus_enabled)); printLine(f, line);}
            else if  (i_tag==18) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_neptune_enabled)); printLine(f, line);}
            else if  (i_tag==19) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(systemData.output_meteors_enabled)); printLine(f, line);}
            else if  (i_tag==20) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(satioData.coordinate_conversion_mode)); printLine(f, line);}
            else if  (i_tag==21) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(satioData.speed_conversion_mode)); printLine(f, line);}
            else if  (i_tag==22) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(satioData.altitude_conversion_mode)); printLine(f, line);}
            else if  (i_tag==23) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(satioData.utc_second_offset)); printLine(f, line);}
            else if  (i_tag==24) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(satioData.utc_auto_offset_flag)); printLine(f, line);}
            else if  (i_tag==25) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(satioData.set_time_automatically)); printLine(f, line);}
            else if  (i_tag==26) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(insData.INS_REQ_GPS_PRECISION)); printLine(f, line);}
            else if  (i_tag==27) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(insData.INS_REQ_MIN_SPEED)); printLine(f, line);}
            else if  (i_tag==28) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(insData.INS_REQ_HEADING_RANGE_DIFF)); printLine(f, line);}
            else if  (i_tag==29) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(insData.INS_MODE)); printLine(f, line);}
            else if  (i_tag==30) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(insData.INS_USE_GYRO_HEADING)); printLine(f, line);}
            else if  (i_tag==31) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(satioFileData.current_matrix_filepath)); printLine(f, line);}
            else if  (i_tag==32) {line = String(satioFileData.system_tags[i_tag]) + String("," + String(matrixData.load_matrix_on_startup)); printLine(f, line);}
        }
    }
    else {return false;}
    f.close();
    Serial.println();
    return true;
    return false;
}

bool loadSystemFile(FS &fs, const char *filepath) {
    Serial.println("loadSystemFile");
    if (fs.exists(filepath)) {
        File f = fs.open(filepath, "r", false);
        if (!f) return false;
        char lineBuffer[1024];
        int currentTag = 0;
        while (f.available()) {
            int len = f.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer) - 1);
            if (len <= 0) break;
            lineBuffer[len] = '\0'; // null-terminate
            if (strlen(lineBuffer) == 0) continue;
            Serial.println("Processing Tag Token Number: " + String(currentTag) + " (data: " + String(lineBuffer) + ")"); // uncomment to debug
            char *commaToken = strtok(lineBuffer, ",");
            int tokenCount = 0;
            int tag_index;
            for (int i_find_tag=0; i_find_tag<MAX_SYSTEM_TAGS; i_find_tag++) {if (strcmp(satioFileData.system_tags[i_find_tag], commaToken)==0) {tag_index=i_find_tag; break;}}
            // Serial.println("Tag Found: " + String(satioFileData.system_tags[tag_index]));
            String data_0; String data_1; String data_2;
            commaToken = strtok(NULL, ","); // remove tag
            while (commaToken != NULL) {if (tokenCount==0) {data_0=commaToken;} else if (tokenCount==1) {data_1=commaToken;} else if (tokenCount==2) {data_2=commaToken;} commaToken = strtok(NULL, ","); tokenCount++;}
            if      (tag_index==0) {if (str_is_bool(data_0.c_str())) {systemData.serial_command=atoi(data_0.c_str());}}
            else if (tag_index==1) {if (str_is_bool(data_0.c_str())) {systemData.output_satio_all=atoi(data_0.c_str());}}
            else if (tag_index==2) {if (str_is_bool(data_0.c_str())) {systemData.output_satio_enabled=atoi(data_0.c_str());}}
            else if (tag_index==3) {if (str_is_bool(data_0.c_str())) {systemData.output_ins_enabled=atoi(data_0.c_str());}}
            else if (tag_index==4) {if (str_is_bool(data_0.c_str())) {systemData.output_gngga_enabled=atoi(data_0.c_str());}}
            else if (tag_index==5) {if (str_is_bool(data_0.c_str())) {systemData.output_gnrmc_enabled=atoi(data_0.c_str());}}
            else if (tag_index==6) {if (str_is_bool(data_0.c_str())) {systemData.output_gpatt_enabled=atoi(data_0.c_str());}}
            else if (tag_index==7) {if (str_is_bool(data_0.c_str())) {systemData.output_matrix_enabled=atoi(data_0.c_str());}}
            else if (tag_index==8) {if (str_is_bool(data_0.c_str())) {systemData.output_admplex0_enabled=atoi(data_0.c_str());}}
            else if (tag_index==9) {if (str_is_bool(data_0.c_str())) {systemData.output_gyro_0_enabled=atoi(data_0.c_str());}}
            else if (tag_index==10) {if (str_is_bool(data_0.c_str())) {systemData.output_sun_enabled=atoi(data_0.c_str());}}
            else if (tag_index==11) {if (str_is_bool(data_0.c_str())) {systemData.output_moon_enabled=atoi(data_0.c_str());}}
            else if (tag_index==12) {if (str_is_bool(data_0.c_str())) {systemData.output_mercury_enabled=atoi(data_0.c_str());}}
            else if (tag_index==13) {if (str_is_bool(data_0.c_str())) {systemData.output_venus_enabled=atoi(data_0.c_str());}}
            else if (tag_index==14) {if (str_is_bool(data_0.c_str())) {systemData.output_mars_enabled=atoi(data_0.c_str());}}
            else if (tag_index==15) {if (str_is_bool(data_0.c_str())) {systemData.output_jupiter_enabled=atoi(data_0.c_str());}}
            else if (tag_index==16) {if (str_is_bool(data_0.c_str())) {systemData.output_saturn_enabled=atoi(data_0.c_str());}}
            else if (tag_index==17) {if (str_is_bool(data_0.c_str())) {systemData.output_uranus_enabled=atoi(data_0.c_str());}}
            else if (tag_index==18) {if (str_is_bool(data_0.c_str())) {systemData.output_neptune_enabled=atoi(data_0.c_str());}}
            else if (tag_index==19) {if (str_is_bool(data_0.c_str())) {systemData.output_meteors_enabled=atoi(data_0.c_str());}}
            else if (tag_index==20) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_COORDINATE_CONVERSION_CONVERSION_MODES) {satioData.coordinate_conversion_mode=atoi(data_0.c_str());}}}
            else if (tag_index==21) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_SPEED_CONVERSIO_MODES) {satioData.speed_conversion_mode=atoi(data_0.c_str());}}}
            else if (tag_index==22) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_ALTITUDE_CONVERSION_MODES) {satioData.altitude_conversion_mode=atoi(data_0.c_str());}}}
            else if (tag_index==23) {if (str_is_long(data_0.c_str())) {satioData.utc_second_offset=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==24) {if (str_is_bool(data_0.c_str())) {satioData.utc_auto_offset_flag=atoi(data_0.c_str());}}
            else if (tag_index==25) {if (str_is_bool(data_0.c_str())) {satioData.set_time_automatically=atoi(data_0.c_str());}}
            else if (tag_index==26) {if (str_is_double(data_0.c_str())) {insData.INS_REQ_GPS_PRECISION=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==27) {if (str_is_double(data_0.c_str())) {insData.INS_REQ_MIN_SPEED=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==28) {if (str_is_double(data_0.c_str())) {insData.INS_REQ_HEADING_RANGE_DIFF=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==29) {if (str_is_double(data_0.c_str())) {insData.INS_MODE=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==30) {if (str_is_double(data_0.c_str())) {insData.INS_USE_GYRO_HEADING=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==31) {memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath)); strcpy(satioFileData.current_matrix_filepath, data_0.c_str());}
            else if (tag_index==32) {if (str_is_bool(data_0.c_str())) {matrixData.load_matrix_on_startup=atoi(data_0.c_str());}}
            currentTag++;
        }
        f.close();
        if (currentTag == 0) {Serial.println("No valid lines found"); return false;}
        Serial.println();
        return true;
    }
    else {return false;}
}

bool deleteSystemFile(FS &fs, const char *filepath) {
    if (fs.exists(filepath)) {if (fs.remove(filepath)) {return true;}}
    return false;
}