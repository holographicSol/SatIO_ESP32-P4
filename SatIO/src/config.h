/*
*/

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


// ----------------------------------------------------------------------------------------
// Globals.
// ----------------------------------------------------------------------------------------
#define EARTH_MEAN_RADIUS 6371000.0 // Mean Earth radius (meters)
#define MAX_GLOBAL_SERIAL_BUFFER_SIZE 512
#define MAX_GLOBAL_ELEMENT_SIZE 56
#define FORMAT_SPIFFS_IF_FAILED false
// ----------------------------------------------------------------------------------------
// IIC.
// ----------------------------------------------------------------------------------------
#define MAX_I2C_MUX 1              // Maximum number of I2C multiplexers (TCA9548A).
#define MAX_IIC_BUFFER_SIZE 32
#define I2C_ADDR_CONTROL_PAD 8
#define I2C_ADDR_PORTCONTROLLER_0 9
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
#define MAX_MAPPABLE_VALUES 32   // Maximum number of mappable values
#define MAX_MAPPING_PARAMETERS 6 // Number of parameters per mapping slot
#define MAX_MAP_SLOTS 100        // Maximum number of map slots.
#define MAX_MAP_MODES 3
#define INDEX_MAPPABLE_VALUES_DIGITAL      0
#define INDEX_MAPPABLE_VALUES_YAWGPATT     1
#define INDEX_MAPPABLE_VALUES_ROLLGPATT    2
#define INDEX_MAPPABLE_VALUES_PITCHGPATT   3
#define INDEX_MAPPABLE_VALUES_GYRO0ACCX    4
#define INDEX_MAPPABLE_VALUES_GYRO0ACCY    5
#define INDEX_MAPPABLE_VALUES_GYRO0ACCZ    6
#define INDEX_MAPPABLE_VALUES_GYRO0ANGX    7
#define INDEX_MAPPABLE_VALUES_GYRO0ANGY    8
#define INDEX_MAPPABLE_VALUES_GYRO0ANGZ    9
#define INDEX_MAPPABLE_VALUES_GYRO0MAGX    10
#define INDEX_MAPPABLE_VALUES_GYRO0MAGY    11
#define INDEX_MAPPABLE_VALUES_GYRO0MAGZ    12
#define INDEX_MAPPABLE_VALUES_GYRO0GYROX   13
#define INDEX_MAPPABLE_VALUES_GYRO0GYROY   14
#define INDEX_MAPPABLE_VALUES_GYRO0GYROZ   15
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_0   16
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_1   17
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_2   18
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_3   19
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_4   20
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_5   21
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_6   22
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_7   23
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_8   24
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_9   25
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_10  26
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_11  27
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_12  28
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_13  29
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_14  30
#define INDEX_MAPPABLE_VALUES_ADMPLEX0_15  31
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
#define MAX_MATRIX_OVERRIDE_TIME 1000000
#define MAX_MATRIX_SWITCHES 70         // logical max is current subjective max<=sytem memory capacity (actual max is subjective max<=sytem memory capacity and or limited by portcontroller max I/O range if using port controller for output)
#define MAX_MATRIX_SWITCH_FUNCTIONS 10 // logical max is current subjective max<=sytem memory capacity (actual max is subjective max<=sytem memory capacity and or limited by portcontroller max I/O range if using port controller for output)
#define MAX_MATRIX_FUNCTION_NAMES 92 // should match length matrixData.matrix_function_names[]
#define MAX_MATRIX_OPERATORS 5
#define INDEX_MATRIX_FUNTION_X 0
#define INDEX_MATRIX_FUNTION_Y 1
#define INDEX_MATRIX_FUNTION_Z 2
#define INDEX_MATRIX_SWITCH_OPERATOR_NONE 0
#define INDEX_MATRIX_SWITCH_OPERATOR_EQUAL 1
#define INDEX_MATRIX_SWITCH_OPERATOR_OVER 2
#define INDEX_MATRIX_SWITCH_OPERATOR_UNDER 3
#define INDEX_MATRIX_SWITCH_OPERATOR_RANGE 4
#define INDEX_MATRIX_SWITCH_FUNCTION_NONE 0
#define INDEX_MATRIX_SWITCH_FUNCTION_ON 1
#define INDEX_MATRIX_SWITCH_FUNCTION_SWITCHLINK 2
#define INDEX_MATRIX_SWITCH_FUNCTION_LOCALTIME 3
#define INDEX_MATRIX_SWITCH_FUNCTION_WEEKDAY 4
#define INDEX_MATRIX_SWITCH_FUNCTION_DATEDAYX 5
#define INDEX_MATRIX_SWITCH_FUNCTION_DATEMONTHX 6
#define INDEX_MATRIX_SWITCH_FUNCTION_DATEYEARX 7
#define INDEX_MATRIX_SWITCH_FUNCTION_DEGLAT 8
#define INDEX_MATRIX_SWITCH_FUNCTION_DEGLON 9
#define INDEX_MATRIX_SWITCH_FUNCTION_INSLAT 11
#define INDEX_MATRIX_SWITCH_FUNCTION_INSLON 12
#define INDEX_MATRIX_SWITCH_FUNCTION_INSHEADING 14
#define INDEX_MATRIX_SWITCH_FUNCTION_INSSPEED 15
#define INDEX_MATRIX_SWITCH_FUNCTION_INSALTITUDE 16
#define INDEX_MATRIX_SWITCH_FUNCTION_POSSTATUSGNGGA 18
#define INDEX_MATRIX_SWITCH_FUNCTION_SATCOUNT 19
#define INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNGGANORTH 20
#define INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNGGASOUTH 21
#define INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNGGAEAST 22
#define INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNGGAWEST 23
#define INDEX_MATRIX_SWITCH_FUNCTION_GPSPRECISION 24
#define INDEX_MATRIX_SWITCH_FUNCTION_ALTGNGGA 25
#define INDEX_MATRIX_SWITCH_FUNCTION_POSSTATUSGNRMCA 27
#define INDEX_MATRIX_SWITCH_FUNCTION_POSSTATUSGNRMCV 28
#define INDEX_MATRIX_SWITCH_FUNCTION_MODEGNRMCA 29
#define INDEX_MATRIX_SWITCH_FUNCTION_MODEGNRMCD 30
#define INDEX_MATRIX_SWITCH_FUNCTION_MODEGNRMCE 31
#define INDEX_MATRIX_SWITCH_FUNCTION_MODEGNRMCN 32
#define INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNRMCNORTH 33
#define INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNRMCSOUTH 34
#define INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNRMCEAST 35
#define INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNRMCWEST 36
#define INDEX_MATRIX_SWITCH_FUNCTION_GSPEEDGNRMC 37
#define INDEX_MATRIX_SWITCH_FUNCTION_HEADINGGNRMC 38
#define INDEX_MATRIX_SWITCH_FUNCTION_LFLAGGPATT 40
#define INDEX_MATRIX_SWITCH_FUNCTION_SFLAGGPATT 41
#define INDEX_MATRIX_SWITCH_FUNCTION_RSFLAGGPATT 42
#define INDEX_MATRIX_SWITCH_FUNCTION_INSGPATT 43
#define INDEX_MATRIX_SWITCH_FUNCTION_SPEEDNUMGPATT 44
#define INDEX_MATRIX_SWITCH_FUNCTION_MILEAGEGPATT 45
#define INDEX_MATRIX_SWITCH_FUNCTION_GSTDATAGPATT 46
#define INDEX_MATRIX_SWITCH_FUNCTION_YAWGPATT 47
#define INDEX_MATRIX_SWITCH_FUNCTION_ROLLGPATT 48
#define INDEX_MATRIX_SWITCH_FUNCTION_PITCHGPATT 49
#define INDEX_MATRIX_SWITCH_FUNCTION_GNGGAVALIDCS 50
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMCVALIDCS 51
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATTVALIDCS 52
#define INDEX_MATRIX_SWITCH_FUNCTION_GNGGAVALIDCD 53
#define INDEX_MATRIX_SWITCH_FUNCTION_GNRMCVALIDCD 54
#define INDEX_MATRIX_SWITCH_FUNCTION_GPATTVALIDCD 55
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ACCX 56
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ACCY 57
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ACCZ 58
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ANGX 59
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ANGY 60
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ANGZ 61
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0MAGX 62
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0MAGY 63
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0MAGZ 64
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0GYROX 65
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0GYROY 66
#define INDEX_MATRIX_SWITCH_FUNCTION_GYRO0GYROZ 67
#define INDEX_MATRIX_SWITCH_FUNCTION_METEORS 68
#define INDEX_MATRIX_SWITCH_FUNCTION_SUNAZ 69
#define INDEX_MATRIX_SWITCH_FUNCTION_SUNALT 70
#define INDEX_MATRIX_SWITCH_FUNCTION_MOONAZ 71
#define INDEX_MATRIX_SWITCH_FUNCTION_MOONALT 72
#define INDEX_MATRIX_SWITCH_FUNCTION_MOONPHASE 73
#define INDEX_MATRIX_SWITCH_FUNCTION_MERCURYAZ 74
#define INDEX_MATRIX_SWITCH_FUNCTION_MERCURYALT 75
#define INDEX_MATRIX_SWITCH_FUNCTION_VENUSAZ 76
#define INDEX_MATRIX_SWITCH_FUNCTION_VENUSALT 77
#define INDEX_MATRIX_SWITCH_FUNCTION_MARSAZ 78
#define INDEX_MATRIX_SWITCH_FUNCTION_MARSALT 79
#define INDEX_MATRIX_SWITCH_FUNCTION_JUPITERAZ 80
#define INDEX_MATRIX_SWITCH_FUNCTION_JUPITERALT 81
#define INDEX_MATRIX_SWITCH_FUNCTION_SATURNAZ 82
#define INDEX_MATRIX_SWITCH_FUNCTION_SATURNALT 83
#define INDEX_MATRIX_SWITCH_FUNCTION_URANUSAZ 84
#define INDEX_MATRIX_SWITCH_FUNCTION_URANUSALT 85
#define INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNEAZ 86
#define INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNEALT 87
#define INDEX_MATRIX_SWITCH_FUNCTION_ADMPLEX0 88
#define INDEX_MATRIX_SWITCH_FUNCTION_MAPPEDVALUE 89
#define INDEX_MATRIX_SWITCH_FUNCTION_SDCARD_INSERTED 90
#define INDEX_MATRIX_SWITCH_FUNCTION_SDCARD_MOUNTED 91
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

#define MAX_COORDINATE_CONVERSION_CONVERSION_MODES 2
#define COORDINATE_CONVERSION_MODE_STATIC          0
#define COORDINATE_CONVERSION_MODE_GPS             1


#define MAX_ALTITUDE_UNIT_MODES       3
#define ALTITUDE_UNIT_MODE_METERS     0
#define ALTITUDE_UNIT_MODE_MILES      1
#define ALTITUDE_UNIT_MODE_KILOMETERS 2

#define MAX_ALTITUDE_CONVERSION_MODES   2
#define ALTITUDE_CONVERSION_MODE_STATIC 0
#define ALTITUDE_CONVERSION_MODE_GPS    1


#define MAX_SPEED_UNIT_MODES            4 // Number of speed conversion modes
#define SPEED_UNIT_MODE_METERS_A_SECOND 0
#define SPEED_UNIT_MODE_MPH             1
#define SPEED_UNIT_MODE_KPH             2
#define SPEED_UNIT_MODE_KTS             3

#define MAX_SPEED_CONVERSIO_MODES    2
#define SPEED_CONVERSION_MODE_STATIC 0
#define SPEED_CONVERSION_MODE_GPS    1

#define METERS_TO_MILES_RATIO      0.000621371
#define METERS_TO_KILOMETERS_RATIO 0.001

#define METERS_TO_MPH_RATIO 2.23694
#define METERS_TO_KPH_RATIO 3.6
#define METERS_TO_KTS_RATIO 1.94384
#define LAST_EPOCH 1900
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

#ifdef __cplusplus
}
#endif

#endif
