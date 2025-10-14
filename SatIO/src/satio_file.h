/*
  SatioFile - Written By Benjamin Jack Cullen.

*/

#ifndef SATIO_FILE_HELPER_H
#define SATIO_FILE_HELPER_H

#include <stdint.h>
#include <FS.h>
#include "SD_MMC.h"
#include "SPIFFS.h"
#include "config.h"

#define MAX_MATRIX_TAGS  12
#define MAX_MAPPING_TAGS 8
#define MAX_SYSTEM_TAGS  32
#define MAX_MATRIX_SLOTS 10

struct satioFileStruct {
    /*
    ----------------------------------------------------------------------------------------
    Matrix.
    ----------------------------------------------------------------------------------------

    (0) SWITCH_PORT                signed int matrix_port_map[MAX_MATRIX_SWITCHES]
    
    (1) SWITCH_FUNCTION            int matrix_function[MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS]
    
    (2) FUNCTION_X                 double matrix_function_xyz[MAX_MATRIX_SWITCHES][ 0 ]
    
    (3) FUNCTION_Y                 double matrix_function_xyz[MAX_MATRIX_SWITCHES][ 1 ]
    
    (4) FUNCTION_Z                 double matrix_function_xyz[MAX_MATRIX_SWITCHES][ 2 ]
    
    (5) FUNCTION_OPERATOR          int matrix_switch_operator_index[MAX_MATRIX_SWITCHES]
    
    (6) FUNCTION_INVERT            bool matrix_switch_inverted_logic[MAX_MATRIX_SWITCHES]
    
    (7) SWITCH_OUTPUT_MODE         int key_output_value[MAX_MATRIX_SWITCHES]
    
    (8) SWITCH_PWM_VALUE_0         uint32_t matrix_modulation_time[MAX_MATRIX_SWITCHES][ 0 ]
    
    (9) SWITCH_PWM_VALUE_1         uint32_t matrix_modulation_time[MAX_MATRIX_SWITCHES][ 1 ]
    
    (10) VALUE_FLUX                signed long threshold_output_value[MAX_MATRIX_SWITCHES]

    (11) COMPUTER_ASSIST           bool computer_assist[MAX_MATRIX_SWITCHES]
    */
    char matrix_tags[MAX_MATRIX_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    /*
        Predefined matrix filepaths that if exist can populate a matrix file slot.
    */
    char matix_filepaths[MAX_MATRIX_SLOTS][MAX_GLOBAL_ELEMENT_SIZE];
    /*
        A store of existing matrix files as index numbers pointing to matrix_filepaths.
    */
    int matrix_file_slots[MAX_MATRIX_SLOTS];
    /*
        Currently opened matrix file if any.
    */
    char current_matrix_filepath[MAX_GLOBAL_ELEMENT_SIZE];
    /*
    ----------------------------------------------------------------------------------------
    Mapping.
    ----------------------------------------------------------------------------------------

    (0) MAP_MODE               signed long map_mode[MAX_MAP_SLOTS]

    (1) INDEX_MAPPED           int index_mapped_value[MAX_MAP_SLOTS]

    (2) MAP_CONFIG_0           int mapping_config[MAX_MAP_SLOTS][ 0 ]

    (3) MAP_CONFIG_1           int mapping_config[MAX_MAP_SLOTS][ 1 ]

    (4) MAP_CONFIG_2           int mapping_config[MAX_MAP_SLOTS][ 2 ]

    (5) MAP_CONFIG_3           int mapping_config[MAX_MAP_SLOTS][ 3 ]

    (6) MAP_CONFIG_4           int mapping_config[MAX_MAP_SLOTS][ 4 ]

    (7) MAP_CONFIG_5           int mapping_config[MAX_MAP_SLOTS][ 5 ]
    */
    char mapping_tags[MAX_MAPPING_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    char mapping_filepath[MAX_GLOBAL_ELEMENT_SIZE];
    /*
    ----------------------------------------------------------------------------------------
    System: File system.csv.
    ----------------------------------------------------------------------------------------

    systemData;

    (0)  SERIAL_COMMAND         unsigned serial_command

    (1)  OUTPUT_ALL             unsigned output_satio_all

    (2)  OUTPUT_SATIO           unsigned output_satio_enabled

    (3)  OUTPUT_INS             unsigned output_ins_enabled

    (4)  OUTPUT_GNGGA           unsigned output_gngga_enabled

    (5)  OUTPUT_GNRMC           unsigned output_gnrmc_enabled

    (6)  OUTPUT_GPATT           unsigned output_gpatt_enabled

    (7)  OUTPUT_MATRIX          unsigned output_matrix_enabled

    (8)  OUTPUT_ADMPLEX0        unsigned output_admplex0_enabled

    (9) OUTPUT_GYRO0           unsigned output_gyro_0_enabled

    (10) OUTPUT_SUN             unsigned output_sun_enabled

    (11) OUTPUT_MOON            unsigned output_moon_enabled

    (12) OUTPUT_MERCURY         unsigned output_mercury_enabled

    (13) OUTPUT_VENUS           unsigned output_venus_enabled

    (14) OUTPUT_MARS            unsigned output_mars_enabled

    (15) OUTPUT_JUPITER         unsigned output_jupiter_enabled

    (16) OUTPUT_SATURN          unsigned output_saturn_enabled

    (17) OUTPUT_URANUS          unsigned output_uranus_enabled

    (18) OUTPUT_NEPTUNE         unsigned output_neptune_enabled

    (19) OUTPUT_METEORS         unsigned output_meteors_enabled

    satioData;

    (20) COORDINATE_CONVERSION_MODE        int coordinate_conversion_mode

    (21) SPEED_CONVERSION_MODE             int speed_conversion_mode

    (22) UTC_SECOND_OFFSET                 long int utc_second_offset

    (23) UTC_AUTO_OFFSET_FLAG              bool utc_auto_offset_flag

    (24) SET_TIME_AUTOMATICALLY            bool set_time_automatically

    ins;

    (25) INS_REQ_GPS_PRECISION             double INS_REQ_GPS_PRECISION

    (26) INS_REQ_MIN_SPEED                 double INS_REQ_MIN_SPEED

    (27) INS_REQ_HEADING_RANGE_DIFF        double INS_REQ_HEADING_RANGE_DIFF

    (28) INS_MODE                          int INS_MODE

    (29) INS_USE_GYRO_HEADING              bool INS_USE_GYRO_HEADING

    matrix;

    (30) MATRIX_FILE                  matrix file slot N

    (31) LOAD_MATRIX_ON_STARTUP       run matrix on startup
    */
    char system_tags[MAX_SYSTEM_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    char system_filepath[MAX_GLOBAL_ELEMENT_SIZE];
};
extern struct satioFileStruct satioFileData;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
bool saveMappingFile(FS &fs, const char *filepath);

bool loadMappingFile(FS &fs, const char *filepath);

bool deleteMappingFile(FS &fs, const char *filepath);


bool saveMatrixFile(FS &fs, const char *filepath);

bool loadMatrixFile(FS &fs, const char *filepath);

bool deleteMatrixFile(FS &fs, const char *filepath);


bool saveSystemFile(FS &fs, const char *filepath);

bool loadSystemFile(FS &fs, const char *filepath);

bool deleteSystemFile(FS &fs, const char *filepath);

#endif