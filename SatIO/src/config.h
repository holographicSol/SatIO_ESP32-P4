/*
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>   // <<< REQUIRED FIRST
#include <Wire.h>   // REQUIRED for TwoWire type
#include "config.h"

extern long  DELAY_TASK_DISPLAY;
extern bool  TICK_DELAY_TASK_DISPLAY;

extern long  DELAY_TASK_SERIAL_INFOCMD;
extern bool  TICK_DELAY_TASK_SERIAL_INFOCMD;

extern long  DELAY_TASK_MULTIPLEXERS;
extern bool  TICK_DELAY_TASK_MULTIPLEXERS;

extern long  DELAY_TASK_GYRO0;
extern bool  TICK_DELAY_TASK_GYRO0;

extern long  DELAY_TASK_UNIVERSE;
extern bool  TICK_DELAY_TASK_UNIVERSE;

extern long  DELAY_TASK_GPS;
extern bool  TICK_DELAY_TASK_GPS;

extern long  DELAY_TASK_SWITCHES;
extern bool  TICK_DELAY_TASK_SWITCHES;

extern long  DELAY_TASK_PORTCONTROLLER_INPUT;
extern bool  TICK_DELAY_TASK_PORTCONTROLLER_INPUT;

extern long  DELAY_TASK_STORAGE;
extern bool  TICK_DELAY_TASK_STORAGE;

extern long  DELAY_TASK_LOGGING;
extern bool  TICK_DELAY_TASK_LOGGING;

extern bool global_task_sync;
extern long system_sync_retry_max;

extern bool ISR_Bool_MultiDisplayController_0;

#define EARTH_MEAN_RADIUS 6371000.0 // Mean Earth radius (meters)

// ----------------------------------------------------------------------------------------
// Power & POWER_CONFIG_ULTIMATE_PERFORMANCE.
// ----------------------------------------------------------------------------------------
/*
    Tasks using TICK_DELAY_xxx false will use millisecond timing. 
    Tasks DELAY_TASK_xxx can be milliseconds or ticks according to TICK_DELAY_xxx.
*/
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Power & POWER_CONFIG_ULTIMATE_PERFORMANCE – mutable at runtime, no multiple-definition errors
// ----------------------------------------------------------------------------------------


#define POWER_CONFIG_1_SECOND_DELAY_TASK_GPS                         1000
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_GPS                    false

#define POWER_CONFIG_1_SECOND_DELAY_TASK_SERIAL_INFOCMD              1
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_SERIAL_INFOCMD         true

#define POWER_CONFIG_1_SECOND_DELAY_TASK_MULTIPLEXERS                1000
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_MULTIPLEXERS           false

#define POWER_CONFIG_1_SECOND_DELAY_TASK_GYRO                        1000
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_GYRO                   false

#define POWER_CONFIG_1_SECOND_DELAY_TASK_UNIVERSE                    1000
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_UNIVERSE               false

#define POWER_CONFIG_1_SECOND_DELAY_TASK_SWITCHES                    1000
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_SWITCHES               false

#define POWER_CONFIG_1_SECOND_DELAY_TASK_PORTCONTROLLER_INPUT        1000
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_PORTCONTROLLER_INPUT   false

#define POWER_CONFIG_1_SECOND_DELAY_TASK_STORAGE                     500
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_STORAGE                false

#define POWER_CONFIG_1_SECOND_DELAY_TASK_LOGGING                     1000
#define POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_LOGGING                false

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_GPS                         1
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_GPS                    true

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_SERIAL_INFOCMD              1
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_SERIAL_INFOCMD         false

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_MULTIPLEXERS                1
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_MULTIPLEXERS           false

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_GYRO                        1
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_GYRO                   false

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_UNIVERSE                    1000
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_UNIVERSE               false

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_SWITCHES                    1
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_SWITCHES               true

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_PORTCONTROLLER_INPUT        1000
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_PORTCONTROLLER_INPUT   false

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_STORAGE                     500
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_STORAGE                false

#define POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_LOGGING                     1000
#define POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_LOGGING                false

// ----------------------------------------------------------------------------------------
// Buffers.
// ----------------------------------------------------------------------------------------
#define MAX_GLOBAL_SERIAL_BUFFER_SIZE 512
#define MAX_GLOBAL_ELEMENT_SIZE 56
#define MAX_INPUT_PORTCONTROLLER_RESPONSE_BYTES 32
// ----------------------------------------------------------------------------------------
// FS.
// ----------------------------------------------------------------------------------------
#define FORMAT_SPIFFS_IF_FAILED false
// ----------------------------------------------------------------------------------------
// Gyro.
// ----------------------------------------------------------------------------------------
#define GYRO_0_ACC_UPDATE   0x01
#define GYRO_0_UPDATE		0x02
#define GYRO_0_ANGLE_UPDATE	0x04
#define GYRO_0_MAG_UPDATE	0x08
#define GYRO_0_READ_UPDATE  0x80
// ----------------------------------------------------------------------------------------
// GPS.
// ----------------------------------------------------------------------------------------
#define MAX_GNGGA_ELEMENTS 16
#define MAX_GNRMC_ELEMENTS 14
#define MAX_GPATT_ELEMENTS 41
// ----------------------------------------------------------------------------------------
// Checksum.
// ----------------------------------------------------------------------------------------
#define MAX_CHECKSUM_SIZE 10
#define MAX_CHEKSUM_SUM_SIZE 4
// ----------------------------------------------------------------------------------------
// Mapping.
// ----------------------------------------------------------------------------------------
#define MAX_MAPPABLE_VALUES 29   // Maximum number of mappable values
#define MAX_MAPPING_PARAMETERS 6 // Number of parameters per mapping slot
#define MAX_MAP_SLOTS 45        // Maximum number of map slots.
#define MAX_MAP_MODES 3
#define INDEX_MAPPABLE_VALUES_DIGITAL      0
#define INDEX_MAPPABLE_VALUES_GYRO0ACCX    1
#define INDEX_MAPPABLE_VALUES_GYRO0ACCY    2
#define INDEX_MAPPABLE_VALUES_GYRO0ACCZ    3
#define INDEX_MAPPABLE_VALUES_GYRO0ANGX    4
#define INDEX_MAPPABLE_VALUES_GYRO0ANGY    5
#define INDEX_MAPPABLE_VALUES_GYRO0ANGZ    6
#define INDEX_MAPPABLE_VALUES_GYRO0MAGX    7
#define INDEX_MAPPABLE_VALUES_GYRO0MAGY    8
#define INDEX_MAPPABLE_VALUES_GYRO0MAGZ    9
#define INDEX_MAPPABLE_VALUES_GYRO0GYROX   10
#define INDEX_MAPPABLE_VALUES_GYRO0GYROY   11
#define INDEX_MAPPABLE_VALUES_GYRO0GYROZ   12
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_0   13
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_1   14
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_2   15
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_3   16
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_4   17
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_5   18
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_6   19
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_7   20
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_8   21
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_9   22
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_10  23
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_11  24
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_12  25
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_13  26
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_14  27
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_15  28
// map config
#define INDEX_MAP_C0 0 // value to map
#define INDEX_MAP_C1 1 // config 0
#define INDEX_MAP_C2 2 // config 2
#define INDEX_MAP_C3 3 // config 3
#define INDEX_MAP_C4 4 // config 4
#define INDEX_MAP_C5 5 // config 5
// standard map
#define INDEX_MAP_VALUE 0
#define INDEX_MAP_EMIN  1 // expected min
#define INDEX_MAP_EMAX  2 // expected max
#define INDEX_MAP_OMIN  3 // output min
#define INDEX_MAP_OMAX  4 // output max
// center map
#define INDEX_CMAP_VALUE     0
#define INDEX_CMAP_CENTER    1 // approximate center value.
#define INDEX_CMAP_NEG_RANGE 2 // 0 to approximate center value (approximate center value).
#define INDEX_CMAP_POS_RANGE 3 // ADC max - neg range.
#define INDEX_CMAP_OMAX      4 // maximum resulting value.
#define INDEX_CMAP_DEADZONE  5 // expected flutuation at center.
// map mode
#define MAP_MIN_TO_MAX 0 // normal map
#define MAP_CENTER_X0  1 // center map and select axis 0 (a side of center)
#define MAP_CENTER_X1  2 // center map and select axis 1 (opposite side of center)
#define MAP_MODE_MIN_TO_MAX 0
#define MAP_MODE_CENTER_MAP_AXIS_0 1
#define MAP_MODE_CENTER_MAP_AXIS_1 2

// ----------------------------------------------------------------------------------------
// INS.
// ----------------------------------------------------------------------------------------
#define INS_INITIALIZATION_FLAG_0 0
#define INS_INITIALIZATION_FLAG_1 1
#define INS_INITIALIZATION_FLAG_2 2
#define INS_INITIALIZATION_FLAG_3 3
#define INS_INITIALIZATION_FLAG_4 4
#define MAX_INS_INITIALIZATION_FLAG 4
#define INS_MODE_OFF 0
#define INS_MODE_DYNAMIC 1
#define INS_MODE_HOLD_THE_LINE 2
#define MAX_INS_MODE 3
// ----------------------------------------------------------------------------------------
// Matrix.
// ----------------------------------------------------------------------------------------
#define MAX_MATRIX_OUTPUT_MODES 2
#define MAX_MATRIX_OVERRIDE_TIME 1000000
#define MAX_MATRIX_SWITCHES 45         // logical max is current subjective max<=sytem memory capacity (actual max is subjective max<=sytem memory capacity and or limited by portcontroller max I/O range if using port controller for output)
#define MAX_MATRIX_SWITCH_FUNCTIONS 10 // logical max is current subjective max<=sytem memory capacity (actual max is subjective max<=sytem memory capacity and or limited by portcontroller max I/O range if using port controller for output)
#define MAX_MATRIX_FUNCTION_NAMES 113 // should match length matrixData.matrix_function_names[]
#define MAX_MATRIX_OPERATORS 5
#define MAX_MATRIX_FUNCTION_XYZ_MODES 2
#define MAX_MATRIX_FUNCTION_INVERTED_LOGIC_MODES 2
#define INDEX_MATRIX_FUNTION_X 0
#define INDEX_MATRIX_FUNTION_Y 1
#define INDEX_MATRIX_FUNTION_Z 2
#define INDEX_MATRIX_SWITCH_OPERATOR_NONE 0
#define INDEX_MATRIX_SWITCH_OPERATOR_EQUAL 1
#define INDEX_MATRIX_SWITCH_OPERATOR_OVER 2
#define INDEX_MATRIX_SWITCH_OPERATOR_UNDER 3
#define INDEX_MATRIX_SWITCH_OPERATOR_RANGE 4
#define INDEX_MATRIX_SWITCH_PWM_OFF 0
#define INDEX_MATRIX_SWITCH_PWM_ON 1
#define INDEX_MATRIX_FUNCTION_XYZ_MODE_USER 0
#define INDEX_MATRIX_FUNCTION_XYZ_MODE_SYSTEM 1

#define INDEX_MATRIX_SWITCH_FUNCTION_NONE 0
#define INDEX_MATRIX_SWITCH_FUNCTION_ON 1
#define INDEX_MATRIX_SWITCH_FUNCTION_SWITCH_LINK 2
#define INDEX_MATRIX_SWITCH_FUNCTION_TIME_HHMMSS 3
#define INDEX_MATRIX_SWITCH_FUNCTION_WEEK_DAY 4
#define INDEX_MATRIX_SWITCH_FUNCTION_MONTH_DAY 5
#define INDEX_MATRIX_SWITCH_FUNCTION_MONTH 6
#define INDEX_MATRIX_SWITCH_FUNCTION_YEAR 7
#define INDEX_MATRIX_SWITCH_FUNCTION_SATIO_DEG_LAT 8
#define INDEX_MATRIX_SWITCH_FUNCTION_SATIO_DEG_LON 9
#define INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_LAT 10
#define INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_LON 11
#define INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_HEADING 12
#define INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_ALT 13
#define INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_STATUS 14
#define INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_SAT_COUNT 15
#define INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_PRESCION 16
#define INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_ALTITUDE 17
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_GROUND_SPEED 18
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEADING 19
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_LINE 20
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_STATIC 21
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_RUN_STATE 22
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_INS 23
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_MILEAGE 24
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_GST 25
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_YAW 26
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_ROLL 27
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_PITCH 28
#define INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_VALID_CS 29
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_VALID_CS 30
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_VALID_CS 31
#define INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_BAD_CD 32
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_BAD_CD 33
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATT_BAD_CD 34
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_POS_STAT_A 35
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_POS_STAT_V 36
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_A 37
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_D 38
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_E 39
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_N 40
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_NORTH 41
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_SOUTH 42
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_EAST 43
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_WEST 44
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_X 45
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_Y 46
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_Z 47
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_X 48
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_Y 49
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_Z 50
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_X 51
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_Y 52
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_Z 53
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_X 54
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_Y 55
#define INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_Z 56
#define INDEX_MATRIX_SWITCH_FUNCTION_METEOR 57
#define INDEX_MATRIX_SWITCH_FUNCTION_SUN_AZIMUTH 58
#define INDEX_MATRIX_SWITCH_FUNCTION_SUN_ALTITUDE 59
#define INDEX_MATRIX_SWITCH_FUNCTION_SUN_HELIO_ECL_LAT 60
#define INDEX_MATRIX_SWITCH_FUNCTION_SUN_HELIO_ECL_LON 61
#define INDEX_MATRIX_SWITCH_FUNCTION_LUNA_AZIMUTH 62
#define INDEX_MATRIX_SWITCH_FUNCTION_LUNA_ALTITUDE 63
#define INDEX_MATRIX_SWITCH_FUNCTION_LUNA_PHASE 64
#define INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_AZIMUTH 65
#define INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ALTITUDE 66
#define INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_H_ECLIPTIC_LAT 67
#define INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_H_ECLIPTIC_LON 68
#define INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ECLIPTIC_LAT 69
#define INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ECLIPTIC_LON 70
#define INDEX_MATRIX_SWITCH_FUNCTION_VENUS_AZIMUTH 71
#define INDEX_MATRIX_SWITCH_FUNCTION_VENUS_ALTITUDE 72
#define INDEX_MATRIX_SWITCH_FUNCTION_VENUS_H_ECLIPTIC_LAT 73
#define INDEX_MATRIX_SWITCH_FUNCTION_VENUS_H_ECLIPTIC_LON 74
#define INDEX_MATRIX_SWITCH_FUNCTION_VENUS_ECLIPTIC_LAT 75
#define INDEX_MATRIX_SWITCH_FUNCTION_VENUS_ECLIPTIC_LON 76
#define INDEX_MATRIX_SWITCH_FUNCTION_EARTH_ECLIPTIC_LON 77
#define INDEX_MATRIX_SWITCH_FUNCTION_MARS_AZIMUTH 78
#define INDEX_MATRIX_SWITCH_FUNCTION_MARS_ALTITUDE 79
#define INDEX_MATRIX_SWITCH_FUNCTION_MARS_H_ECLIPTIC_LAT 80
#define INDEX_MATRIX_SWITCH_FUNCTION_MARS_H_ECLIPTIC_LON 81
#define INDEX_MATRIX_SWITCH_FUNCTION_MARS_ECLIPTIC_LAT 82
#define INDEX_MATRIX_SWITCH_FUNCTION_MARS_ECLIPTIC_LON 83
#define INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_AZIMUTH 84
#define INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ALTITUDE 85
#define INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_H_ECLIPTIC_LAT 86
#define INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_H_ECLIPTIC_LON 87
#define INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ECLIPTIC_LAT 88
#define INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ECLIPTIC_LON 89
#define INDEX_MATRIX_SWITCH_FUNCTION_SATURN_AZIMUTH 90
#define INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ALTITUDE 91
#define INDEX_MATRIX_SWITCH_FUNCTION_SATURN_H_ECLIPTIC_LAT 92
#define INDEX_MATRIX_SWITCH_FUNCTION_SATURN_H_ECLIPTIC_LON 93
#define INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ECLIPTIC_LAT 94
#define INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ECLIPTIC_LON 95
#define INDEX_MATRIX_SWITCH_FUNCTION_URANUS_AZIMUTH 96
#define INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ALTITUDE 97
#define INDEX_MATRIX_SWITCH_FUNCTION_URANUS_H_ECLIPTIC_LAT 98
#define INDEX_MATRIX_SWITCH_FUNCTION_URANUS_H_ECLIPTIC_LON 99
#define INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ECLIPTIC_LAT 100
#define INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ECLIPTIC_LON 101
#define INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_AZIMUTH 102
#define INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ALTITUDE 103
#define INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_H_ECLIPTIC_LAT 104
#define INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_H_ECLIPTIC_LON 105
#define INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ECLIPTIC_LAT 106
#define INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ECLIPTIC_LON 107
#define INDEX_MATRIX_SWITCH_FUNCTION_AD_MULTIPLEXER_0 108
#define INDEX_MATRIX_SWITCH_FUNCTION_MAP_SLOT 109
#define INDEX_MATRIX_SWITCH_FUNCTION_SD_CARD_INSERTED 110
#define INDEX_MATRIX_SWITCH_FUNCTION_SD_CARD_MOUNTED 111
#define INDEX_MATRIX_SWITCH_FUNCTION_PORT_CON_0 112

#define INDEX_MATRIX_MOD_0 0
#define INDEX_MATRIX_MOD_1 1
// ----------------------------------------------------------------------------------------
// TCA9548A I2C multiplexer address.
// ----------------------------------------------------------------------------------------
#define TCA9548AADDR_0 0x70
#define MAX_IIC_MUX_CHANNELS 8
// ----------------------------------------------------------------------------------------
// CH74HC4067 analog/digital multiplexer pins (Mux 0).
// ----------------------------------------------------------------------------------------
#define ADMPLEX_0_S0 23
#define ADMPLEX_0_S1 22
#define ADMPLEX_0_S2 21
#define ADMPLEX_0_S3 20
#define ADMPLEX_0_SIG 53
#define MAX_AD_MUX 1               // Maximum number of analog/digital multiplexers (CH74HC4067).
#define MAX_AD_MUX_CONTROL_PINS 4  // Number of control pins for CH74HC4067 (S0-S3).
#define MAX_AD_MUX_CHANNELS 16     // Number of channels for CH74HC4067.
#define INDEX_ADMPLEX_0_CH_0 0
#define INDEX_ADMPLEX_0_CH_1 1
#define INDEX_ADMPLEX_0_CH_2 2
#define INDEX_ADMPLEX_0_CH_3 3
#define INDEX_ADMPLEX_0_CH_4 4
#define INDEX_ADMPLEX_0_CH_5 5
#define INDEX_ADMPLEX_0_CH_6 6
#define INDEX_ADMPLEX_0_CH_7 7
#define INDEX_ADMPLEX_0_CH_8 8
#define INDEX_ADMPLEX_0_CH_9 9
#define INDEX_ADMPLEX_0_CH_10 10
#define INDEX_ADMPLEX_0_CH_11 11
#define INDEX_ADMPLEX_0_CH_12 12
#define INDEX_ADMPLEX_0_CH_13 13
#define INDEX_ADMPLEX_0_CH_14 14
#define INDEX_ADMPLEX_0_CH_15 15
// ----------------------------------------------------------------------------------------
// Volcanos.
// ----------------------------------------------------------------------------------------
#define MAX_VOLCANOS 435
#define MAX_LOCATIONS 57
#define MAX_TYPES 19
// ----------------------------------------------------------------------------------------
// Meteors.
// ----------------------------------------------------------------------------------------
#define MAX_METEOR_SHOWERS                8
#define MAX_METEOR_RESULT_ELEMENTS        2
#define INDEX_METEOR_RESULT_DATETIME      0
#define INDEX_METEOR_RESULT_PEAK_DATETIME 1
// datetime
#define MAX_METEOR_SHOWER_DATETIME          2
#define MAX_METEOR_SHOWER_DATETIME_ELEMENTS 2
#define INDEDX_METEOR_DATETIME_START        0
#define INDEDX_METEOR_DATETIME_END          1
#define INDEDX_METEOR_DATETIME_MONTH_START  0
#define INDEDX_METEOR_DATETIME_DAY_START    1
#define INDEDX_METEOR_DATETIME_MONTH_END    0
#define INDEDX_METEOR_DATETIME_DAY_END      1
// peak datetime
#define MAX_METEOR_SHOWER_PEAK_DATETIME               2
#define MAX_METEOR_SHOWER_PEAK_DATETIME_ELEMENTS      3
#define INDEDX_METEOR_PEAK_DATETIME_START             0
#define INDEDX_METEOR_PEAK_DATETIME_END               1
#define INDEDX_METEOR_PEAK_DATETIME_MONTH_0_START     0
#define INDEDX_METEOR_PEAK_DATETIME_MONTH_0_DAY_START 1
#define INDEDX_METEOR_PEAK_DATETIME_MONTH_0_DAY_END   2
#define INDEDX_METEOR_PEAK_DATETIME_MONTH_1_END       0
#define INDEDX_METEOR_PEAK_DATETIME_MONTH_1_DAY_START 1
#define INDEDX_METEOR_PEAK_DATETIME_MONTH_1_DAY_END   2
// ----------------------------------------------------------------------------------------
// SatIO.
// ----------------------------------------------------------------------------------------
#define MAX_DAYS_OF_THE_WEEK 7
#define MAX_MONTHS_OF_THE_YEAR 12

#define METERS_TO_MILES_RATIO      0.000621371
#define METERS_TO_KILOMETERS_RATIO 0.001

#define METERS_TO_MPH_RATIO 2.23694
#define METERS_TO_KPH_RATIO 3.6
#define METERS_TO_KTS_RATIO 1.94384

#define KNOTS_TO_METERS_PER_SECOND 0.514444
#define KNOTS_TO_MPH 1.15078
#define KNOTS_TO_KPH 1.852

#define LAST_EPOCH 1900

#define SATIO_MODE_GPS  0
#define SATIO_MODE_USER 1

// ----------------------------------------------------------------------------------------
// SIDEREAL HELPER.
// ----------------------------------------------------------------------------------------
#define INDEX_SIDEREAL_STAR_TABLE 0          
#define INDEX_SIDEREAL_NGC_TABLE 1           // New General Catalogue
#define INDEX_SIDEREAL_IC_TABLE 2            // The Index Catalogue of Nebulae and Clusters of Stars (IC)
#define INDEX_SIDEREAL_MESSIER_TABLE 3       
#define INDEX_SIDEREAL_CALDWELL_TABLE 4      
#define INDEX_SIDEREAL_HERSHEL400_TABLE 5    
#define INDEX_SIDEREAL_OTHER_OBJECTS_TABLE 6

#endif // CONFIG_H