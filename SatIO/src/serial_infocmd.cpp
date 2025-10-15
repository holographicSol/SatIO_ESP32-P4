/*
  Serial Information Command - Written By Benjamin Jack Cullen
*/

#include "serial_infocmd.h"
#include <Arduino.h>
#include "matrix.h"
#include "wtgps300p.h"
#include "wt901.h"
#include "multiplexers.h"
#include "custommapping.h"
#include "sidereal_helper.h"
#include "satio.h"
#include "ins.h"
#include "meteors.h"
#include "hextodig.h"
#include "esp32_helper.h"
#include "sdmmc_helper.h"
#include "arg_parser.h"
#include "system_data.h"
#include <FS.h>
#include "SD_MMC.h"
#include "SPIFFS.h"
#include <stdlib.h>
#include "satio_file.h"
#include <limits.h>
#include <float.h>
#include "matrix.h"

ArgParser parser;
PlainArgParser plainparser;

struct Serial0Struct serial0Data = {
  .nbytes=0, // number of bytes read by serial.
  .iter_token=0, // count token iterations.
  .BUFFER={}, // serial buffer.
  .token=0, // token pointer.
  .checksum=0,
  .checksum_of_buffer=0,
  .checksum_in_buffer=0,
  .gotSum=0,
  .i_XOR=0,
  .XOR=0,
  .c_XOR=0,
};

int getCheckSumSerial0(char * string) {
  for (serial0Data.XOR=0, serial0Data.i_XOR=0; serial0Data.i_XOR < strlen(string); serial0Data.i_XOR++) {
    serial0Data.c_XOR=(unsigned char)string[serial0Data.i_XOR];
    if (serial0Data.c_XOR=='*') break;
    if (serial0Data.c_XOR != '$') serial0Data.XOR ^= serial0Data.c_XOR;
  }
  return serial0Data.XOR;
}

bool validateChecksumSerial0(char * buffer) {
  memset(serial0Data.gotSum, 0, sizeof(serial0Data.gotSum));
  serial0Data.gotSum[0]=buffer[strlen(buffer) - 3];
  serial0Data.gotSum[1]=buffer[strlen(buffer) - 2];
  serial0Data.checksum_of_buffer= getCheckSumSerial0(buffer);
  serial0Data.checksum_in_buffer=h2d2(serial0Data.gotSum[0], serial0Data.gotSum[1]);
  if (serial0Data.checksum_in_buffer==serial0Data.checksum_of_buffer) {return true;}
  return false;
}

void createChecksumSerial0(char * buffer) {
  serial0Data.checksum_of_buffer=getCheckSumSerial0(buffer);
  sprintf(serial0Data.checksum,"%X",serial0Data.checksum_of_buffer);
}

bool val_global_element_size(const char * data) {
  if (sizeof(data)>=MAX_GLOBAL_ELEMENT_SIZE) {return false;}
  return true;
}

bool val_switch_index(const char * data) {
  if (str_is_int8(data)) {if (atol(data)<MAX_MATRIX_SWITCHES) {return true;}}
  return false;
}

bool val_function_index(const char * data) {
  if (str_is_int8(data)) {if (atol(data)<MAX_MATRIX_SWITCH_FUNCTIONS) {return true;}}
  return false;
}

bool val_speed_units(const char * data) {
  if (str_is_int8(data)) {if (atol(data)<MAX_SPEED_CONVERSION_MODES) {return true;}}
  return false;
}

bool val_function_name_index(const char * data) {
  if (val_global_element_size(data)==false) {return false;}
  if (str_is_long(data)!=true) {return false;}
  if (atol(data) < MAX_MATRIX_FUNCTION_NAMES) {return true;}
  return false;
}

bool val_function_xyz(const char * data) {
  if (str_is_double(data)) {return true;}
  return false;
}

bool val_function_operator(const char * data) {
  if (str_is_int8(data)) {return true;}
  return false;
}

bool val_switch_port(const char * data) {
  if (str_is_int8(data)) {return true;}
  return false;
}

bool val_mappable_value_index(const char * data) {
  Serial.println(data);
  if (str_is_int8(data)!=true) {return false;}
  if (atoi(data)<MAX_MAPPABLE_VALUES) {return true;} 
  return false;
}

bool val_ins_mode(const char * data) {
  if (str_is_int8(data)) {if (atol(data)<MAX_INS_MODE) {return true;}}
  return false;
}

bool val_ins_gps_precision(const char * data) {
  if (str_is_float(data)) {return true;}
  return false;
}

bool val_ins_minimum_speed(const char * data) {
  if (str_is_float(data)) {return true;}
  return false;
}

bool val_ins_heading_range_diff(const char * data) {
  if (str_is_float(data)) {return true;}
  return false;
}

static void PrintHelp(void) {
  Serial.println(
  R"(
  [ System ]

      system --save
      system --load
      system --restore-defaults

  [ Matrix ]

      matrix --new                Clears matrix in memory.
      matrix --save n             Specify file slot.
      matrix --load n             Specify file slot.
      matrix --delete n           Specify file slot.
      matrix --startup-enable
      matrix --startup-disable
      matrix -s n                 Specify switch index n.
      matrix -f n                 Specify function index n.
      matrix -p n                 Set port for switch -s.

      matrix -fn n                Set function -f for switch -s. See available matrix functions.
                                  [0] None
                                  [1] On
                                  [2] SwitchLink
                                  [3] LocalTime
                                  [4] Weekday
                                  [5] DateDayX
                                  [6] DateMonthX
                                  [7] DateYearX
                                  [8] DegLat
                                  [9] DegLon
                                  [10] DegLatLon
                                  [11] INSLat
                                  [12] INSLon
                                  [13] INSLatLon
                                  [14] INSHeading
                                  [15] INSSpeed
                                  [16] INSAltitude
                                  [17] UTCTimeGNGGA
                                  [18] PosStatusGNGGA
                                  [19] SatCount
                                  [20] HemiGNGGANorth
                                  [21] HemiGNGGASouth
                                  [22] HemiGNGGAEast
                                  [23] HemiGNGGAWest
                                  [24] GPSPrecision
                                  [25] AltGNGGA
                                  [26] UTCTimeGNRMC
                                  [27] PosStatusGNRMCA
                                  [28] PosStatusGNRMCV
                                  [29] ModeGNRMCA
                                  [30] ModeGNRMCD
                                  [31] ModeGNRMCE
                                  [32] ModeGNRMCN
                                  [33] HemiGNRMCNorth
                                  [34] HemiGNRMCSouth
                                  [35] HemiGNRMCEast
                                  [36] HemiGNRMCWest
                                  [37] GSpeedGNRMC
                                  [38] HeadingGNRMC
                                  [39] UTCDateGNRMC
                                  [40] LFlagGPATT
                                  [41] SFlagGPATT
                                  [42] RSFlagGPATT
                                  [43] INSGPATT
                                  [44] SpeedNumGPATT
                                  [45] MileageGPATT
                                  [46] GSTDataGPATT
                                  [47] YawGPATT
                                  [48] RollGPATT
                                  [49] PitchGPATT
                                  [50] GNGGAValidCS
                                  [51] GNRMCValidCS
                                  [52] GPATTValidCS
                                  [53] GNGGAValidCD
                                  [54] GNRMCValidCD
                                  [55] GPATTValidCD
                                  [56] Gyro0AccX
                                  [57] Gyro0AccY
                                  [58] Gyro0AccZ
                                  [59] Gyro0AngX
                                  [60] Gyro0AngY
                                  [61] Gyro0AngZ
                                  [62] Gyro0MagX
                                  [63] Gyro0MagY
                                  [64] Gyro0MagZ
                                  [65] Gyro0GyroX
                                  [66] Gyro0GyroY
                                  [67] Gyro0GyroZ
                                  [68] Meteors
                                  [69] SunAz
                                  [70] SunAlt
                                  [71] MoonAz
                                  [72] MoonAlt
                                  [73] MoonPhase
                                  [74] MercuryAz
                                  [75] MercuryAlt
                                  [76] VenusAz
                                  [77] VenusAlt
                                  [78] MarsAz
                                  [79] MarsAlt
                                  [80] JupiterAz
                                  [81] JupiterAlt
                                  [82] SaturnAz
                                  [83] SaturnAlt
                                  [84] UranusAz
                                  [85] UranusAlt
                                  [86] NeptuneAz
                                  [87] NeptuneAlt
                                  [88] ADMPlex0
                                  [89] MappedValue
                                  [90] SDCARDInserted
                                  [91] SDCARDMounted

      matrix -fx n                Set function -f value x for switch -s.
      matrix -fy n                Set function -f value y for switch -s.
      matrix -fz n                Set function -f value z for switch -s.
      matrix -fi n                Set function -f logic inverted for switch -s.

      matrix -fo n                Set function -f operator for switch -s.
                                  [0] None
                                  [1] Equal
                                  [2] Over
                                  [3] Under
                                  [4] Range
                                  
      matrix --pwm0 n             Set switch -s uS time off period (0uS = remain on)
      matrix --pwm1 n             Set switch -s uS time on period  (0uS = remain off after on)
      matrix --flux n             Set switch -s output fluctuation threshold.
      matrix --oride n            Override switch -s output values.
      matrix --computer-assist n  Enable/disable computer assist for switch -s.
      matrix --omode n            Set switch -s output mode: (0 : matrix logic) (1 : mapped value analog/digital).
      matrix --map-slot n         Set switch -s output as map slot n value.

      example set matrix logic 0 function 0:
      matrix -s 0 -f 0 -p 33 -fn 90 -fx 1 -fo 1 --pwm0 1000000 --pwm1 15000 --computer-assist 1
      matrix -s 0 --omode 0
  
  [ Mapping ]

      mapping --save
      mapping --load
      mapping --delete
      mapping -s n       Specify map slot n.
      mapping -m n       Specify slot -s mode. (0 : map min to max) (1 : center map x0) (2 : center map x1)
      mapping -c0 n      Configuration map slot -s  value to map. See available map values.
      mapping -c1 n      Configuration map slot -s. (mode 0 : in_min)  (mode 1 : approximate center value)
      mapping -c2 n      Configuration map slot -s. (mode 0 : in_max)  (mode 1 : Neg_range : 0 to approximate center value)
      mapping -c3 n      Configuration map slot -s. (mode 0 : out_min) (mode 1 : Pos_range : ADC max - neg range)
      mapping -c4 n      Configuration map slot -s. (mode 0 : out_max) (mode 1 : out_max)
      mapping -c5 n      Configuration map slot -s. (mode 1 only : DEADZONE : expected flutuation at center)

      example map analog stick axis x0 on admplex0 channel 0 into map slot 0:
      mapping -s 0 -m 1 -c0 16 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
      Optional: matrix -s 0 --omode 1

      example map analog stick axis x1 on admplex0 channel 1 into map slot 1:
      mapping -s 1 -m 2 -c0 17 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
      Optional: matrix -s 1 --omode 1

  [ INS ]

      ins -m n              Set INS mode n. (0 : Off) (1 : Dynamic, set by gps every 100ms) (2 : Fixed, remains on after conditions met).
      ins -gyro n           INS uses gyro for attitude. (0 : gyro heading) (1 : gps heading).
      ins -p n              Set INS mimimum required gps precision factor to initialize.
      ins -s n              Set INS mimimum required speed to initialize.
      ins -r n              Set INS maximum required heading range difference to initialize (difference between gps heading and gyro heading).
      ins --reset-forced n  Reset INS remains on after conditions met.

  [ Satio ]

      satio --speed-units n  Set displayed units (0 : M/S) (1 : MPH) (2 : KPH) (3 : KTS estimated)
      satio --utc-offset n   Set +-seconds offset time.
      satio --mode-gngga     Use GNGGA data for location.
      satio --mode-gnrmc     Use GNRMC data for location.

  [ Gyro ]

      gyro --calacc        Callibrate the accelerometer.
      gyro --calmag-start  Begin calibrating the magnetometer.
      gyro --calmag-end    End calibrating the magnetometer.

  [ SDCard ]

      sdcard --mount
      sdcard --unmount

  [ Stat ]

      stat -e     Enable print.
      stat -d     Disable print.
      stat -t     Enables/disables serial print stats and counters. Takes arguments -e, -d.
      stat --partition-table      Print partition table.
      stat --memory-ram           Print ram information.
      stat --sdcard               Print matrix information.
      stat --system               Print system configuration.
      stat --matrix               Print matrix configuration.
      stat --matrix n             Print matrix switch n configuration.
      stat --matrix -A            Print configuration of all matrix switches.
      stat --mapping              Print configuration of all mapping slots.
      stat --sentence -A          Print all sentences. Takes arguments -e, -d.
      stat --sentence --satio     Takes arguments -e, -d.
      stat --sentence --ins       Takes arguments -e, -d.
      stat --sentence --gngga     Takes arguments -e, -d.
      stat --sentence --gnrmc     Takes arguments -e, -d.
      stat --sentence --gpatt     Takes arguments -e, -d.
      stat --sentence --matrix    Takes arguments -e, -d.
      stat --sentence --admplex0  Takes arguments -e, -d.
      stat --sentence --gyro0     Takes arguments -e, -d.
      stat --sentence --sun       Takes arguments -e, -d.
      stat --sentence --moon      Takes arguments -e, -d.
      stat --sentence --mercury   Takes arguments -e, -d.
      stat --sentence --venus     Takes arguments -e, -d.
      stat --sentence --mars      Takes arguments -e, -d.
      stat --sentence --jupiter   Takes arguments -e, -d.
      stat --sentence --saturn    Takes arguments -e, -d.
      stat --sentence --uranus    Takes arguments -e, -d.
      stat --sentence --neptune   Takes arguments -e, -d.
      stat --sentence --meteors   Takes arguments -e, -d.
  
  [ Other ]

      -v    Enable verbosoity.
      -vv   Enable extra verbosoity.
      help
  )"
  );
}

void PrintSystemData(void) {
    Serial.println("-----------------------------------------------------");
    Serial.println("[System] ");
    Serial.println("[serial_command] " + String(systemData.serial_command));
    Serial.println("[output_satio_all] " + String(systemData.output_satio_all));
    Serial.println("[output_satio_enabled] " + String(systemData.output_satio_enabled));
    Serial.println("[output_ins_enabled] " + String(systemData.output_ins_enabled));
    Serial.println("[output_gngga_enabled] " + String(systemData.output_gngga_enabled));
    Serial.println("[output_gnrmc_enabled] " + String(systemData.output_gnrmc_enabled));
    Serial.println("[output_gpatt_enabled] " + String(systemData.output_gpatt_enabled));
    Serial.println("[output_matrix_enabled] " + String(systemData.output_matrix_enabled));
    Serial.println("[output_admplex0_enabled] " + String(systemData.output_admplex0_enabled));
    Serial.println("[output_gyro_0_enabled] " + String(systemData.output_gyro_0_enabled));
    Serial.println("[output_sun_enabled] " + String(systemData.output_sun_enabled));
    Serial.println("[output_moon_enabled] " + String(systemData.output_moon_enabled));
    Serial.println("[output_mercury_enabled] " + String(systemData.output_mercury_enabled));
    Serial.println("[output_venus_enabled] " + String(systemData.output_venus_enabled));
    Serial.println("[output_mars_enabled] " + String(systemData.output_mars_enabled));
    Serial.println("[output_jupiter_enabled] " + String(systemData.output_jupiter_enabled));
    Serial.println("[output_saturn_enabled] " + String(systemData.output_saturn_enabled));
    Serial.println("[output_uranus_enabled] " + String(systemData.output_uranus_enabled));
    Serial.println("[output_neptune_enabled] " + String(systemData.output_neptune_enabled));
    Serial.println("[output_meteors_enabled] " + String(systemData.output_meteors_enabled));
    Serial.println("-----------------------------------------------------");
}

void PrintSatIOData(void) {
    Serial.println("-----------------------------------------------------");
    Serial.println("[SatIO] ");
    Serial.println("[coordinate_conversion_mode] " +
      String(satioData.char_coordinate_conversion_mode[satioData.coordinate_conversion_mode]));
    Serial.println("[speed_conversion_mode] " +
      String(satioData.char_speed_conversion_mode[satioData.speed_conversion_mode]));
    Serial.println("[utc_second_offset] " + String(satioData.utc_second_offset));
    Serial.println("[utc_auto_offset_flag] " + String(satioData.utc_auto_offset_flag));
    Serial.println("[set_time_automatically] " + String(satioData.set_time_automatically));
    Serial.println("-----------------------------------------------------");
}

void PrintMappingConfig() {
  Serial.println("-----------------------------------------------------");
  Serial.println("[Available Mapping Values]");
  for (int Mi=0; Mi<MAX_MAPPABLE_VALUES; Mi++)
    {Serial.println("  [" + String(Mi) + "] " + String(mappingData.char_map_value[Mi]));}
}

void PrintMappingData(void) {
  PrintMappingConfig();
  for (int Mi=0; Mi<MAX_MAP_SLOTS; Mi++) {
    Serial.println("-----------------------------------------------------");
    Serial.println("[slot] " + String(Mi));
    Serial.println("[map_mode] " + String(mappingData.map_mode[0][Mi]));
    Serial.println("[map slot idx] " + String(mappingData.index_mapped_value[0][Mi]));
    Serial.println("[map config 0] " + String(mappingData.mapping_config[0][Mi][0]));
    Serial.println("[map config 1] " + String(mappingData.mapping_config[0][Mi][1]));
    Serial.println("[map config 2] " + String(mappingData.mapping_config[0][Mi][2]));
    Serial.println("[map config 3] " + String(mappingData.mapping_config[0][Mi][3]));
    Serial.println("[map config 4] " + String(mappingData.mapping_config[0][Mi][4]));
    Serial.println("[map config 5] " + String(mappingData.mapping_config[0][Mi][5]));
    Serial.println("-----------------------------------------------------");
  }
}

void PrintMatrixConfig() {
    Serial.println("-----------------------------------------------------");
    Serial.println("[load_matrix_on_startup] " + String(matrixData.load_matrix_on_startup));
    Serial.println("[Available Switch Functions]");
    for (int Mi=0; Mi<MAX_MATRIX_FUNCTION_NAMES; Mi++)
      {Serial.println("  [" + String(Mi) + "] " +
        String(matrixData.matrix_function_names[Mi]));}
    Serial.println("[Available Switch Function Operators]");
    for (int Mi=0; Mi<MAX_MATRIX_OPERATORS; Mi++)
      {Serial.println("  [" + String(Mi) + "] " +
        String(matrixData.matrix_function_operator_name[Mi]));}
    Serial.println("-----------------------------------------------------");
}

void PrintMatrixData(void) {
    for (int Mi=0; Mi<MAX_MATRIX_SWITCHES; Mi++) {
    Serial.println("-----------------------------------------------------");
    Serial.println("[matrix switch] " + String(Mi));
    Serial.println("[computer assist] " + String(matrixData.computer_assist[0][Mi]));
    Serial.println("[output mode] " + String(matrixData.output_mode[0][Mi]));
    Serial.println("[map slot] " + String(mappingData.index_mapped_value[0][Mi]));
    Serial.println("[flux] " + String(matrixData.flux_value[0][Mi]));
    Serial.println("[pwm] 0: " + String(matrixData.output_pwm[0][Mi][0]) + " 1: " +
      String(matrixData.output_pwm[0][Mi][1]));
    Serial.println("[port] " + String(matrixData.matrix_port_map[0][Mi]));
    Serial.println("[active] " + String(matrixData.switch_intention[0][Mi]));
    Serial.println("-----------------------------------------------------");
    for (int Fi=0; Fi<MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
      Serial.println("[function " + String(Fi) + " name] " +
        String(matrixData.matrix_function_names[matrixData.matrix_function[0][Mi][Fi]]));
      Serial.println("[function " + String(Fi) + " matrix_function_operator_name] " +
        String(matrixData.matrix_switch_operator_index[0][Mi][Fi]));
      Serial.println("[function " + String(Fi) + " inverted] " +
        String(matrixData.matrix_switch_inverted_logic[0][Mi][Fi]));
      Serial.println("[function " + String(Fi) + " x] " +
        String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]));
      Serial.println("[function " + String(Fi) + " y] " +
        String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]));
      Serial.println("[function " + String(Fi) + " z] " +
        String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]));
      Serial.println("-----------------------------------------------------");
    }
  }
}

void PrintMatrixNData() {
  int Mi=atoi(plainparser.tokens[0]);
  Serial.println("-----------------------------------------------------");
  Serial.println("[matrix switch] " + String(Mi));
  Serial.println("[computer assist] " + String(matrixData.computer_assist[0][Mi]));
  Serial.println("[output mode] " + String(matrixData.output_mode[0][Mi]));
  Serial.println("[flux] " + String(matrixData.flux_value[0][Mi]));
  Serial.println("[pwm] 0: " + String(matrixData.output_pwm[0][Mi][0]) + " 1: " +
    String(matrixData.output_pwm[0][Mi][1]));
  Serial.println("[port] " + String(matrixData.matrix_port_map[0][Mi]));
  Serial.println("[active] " + String(matrixData.switch_intention[0][Mi]));
  Serial.println("-----------------------------------------------------");
  for (int Fi=0; Fi<MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
    Serial.println("[function " + String(Fi) + " name] " +
      String(matrixData.matrix_function_names[matrixData.matrix_function[0][Mi][Fi]]));
    Serial.println("[function " + String(Fi) + " matrix_function_operator_name] " +
      String(matrixData.matrix_switch_operator_index[0][Mi][Fi]));
    Serial.println("[function " + String(Fi) + " inverted] " +
      String(matrixData.matrix_switch_inverted_logic[0][Mi][Fi]));
    Serial.println("[function " + String(Fi) + " x] " +
      String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]));
    Serial.println("[function " + String(Fi) + " y] " +
      String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]));
    Serial.println("[function " + String(Fi) + " z] " +
      String(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]));
    Serial.println("-----------------------------------------------------");
  }
}

void PrintSDCardInformation() {
  Serial.println("sdcard inserted:    " + String(sdcardData.sdcard_inserted));
  Serial.println("sdcard mounted:     " + String(sdcardData.sdcard_mounted));
  Serial.println("sdcard mount point: " + String(sdcardData.sdcard_mountpoint));
  Serial.println("sdcard type:        " +
    String(sdcardData.sdcard_type_names[sdcardData.sdcard_type]) +
    " (type: " + String(sdcardData.sdcard_type) + ").");
  Serial.println("sdcard card size:   " + String(sdcardData.sdcard_card_size));
  Serial.println("sdcard total bytes: " + String(sdcardData.sdcard_total_bytes));
  Serial.println("sdcard used bytes:  " + String(sdcardData.sdcard_used_bytes));
  Serial.println("sdcard sector size: " + String(sdcardData.sdcard_sector_size));
}

void setAllSentenceOutput(bool enable) {
  systemData.output_satio_enabled=enable;
  systemData.output_gngga_enabled=enable;
  systemData.output_gnrmc_enabled=enable;
  systemData.output_gpatt_enabled=enable;
  systemData.output_ins_enabled=enable;
  systemData.output_matrix_enabled=enable;
  systemData.output_admplex0_enabled=enable;
  systemData.output_gyro_0_enabled=enable;
  systemData.output_sun_enabled=enable;
  systemData.output_moon_enabled=enable;
  systemData.output_mercury_enabled=enable;
  systemData.output_venus_enabled=enable;
  systemData.output_mars_enabled=enable;
  systemData.output_jupiter_enabled=enable;
  systemData.output_saturn_enabled=enable;
  systemData.output_uranus_enabled=enable;
  systemData.output_neptune_enabled=enable;
  systemData.output_meteors_enabled=enable;
}

void setMatrixPort(int switch_idx, signed int port_n) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && port_n>=-1 && port_n<MAX_MATRIX_SWITCHES) {
    matrixData.matrix_port_map[0][switch_idx]=port_n;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixFunction(int switch_idx, int func_idx, int func_n) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 &&
      func_idx<MAX_MATRIX_SWITCH_FUNCTIONS &&
      func_n>=0 &&
      func_n<MAX_MATRIX_FUNCTION_NAMES) {
    matrixData.matrix_function[0][switch_idx][func_idx]=func_n;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixX(int switch_idx, int func_idx, double func_x) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES &&
      func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS &&
      func_x>=DBL_MIN &&
      func_x<DBL_MAX) {
    matrixData.matrix_function_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_X]=func_x;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixY(int switch_idx, int func_idx, double func_y) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 &&
      func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_y>=DBL_MIN &&
      func_y<DBL_MAX) {
    matrixData.matrix_function_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_Y]=func_y;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixZ(int switch_idx, int func_idx, double func_z) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 &&
      func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_z>=DBL_MIN && func_z<DBL_MAX) {
    matrixData.matrix_function_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_Z]=func_z;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixInverted(int switch_idx, int func_idx, int func_i) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 &&
      func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_i>=0 && func_i<=1) {
    matrixData.matrix_switch_inverted_logic[0][switch_idx][func_idx]=func_i;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixOperator(int switch_idx, int func_idx, int func_o) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 &&
      func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_o>=0 && func_o<MAX_MATRIX_OPERATORS) {
    matrixData.matrix_switch_operator_index[0][switch_idx][func_idx]=func_o;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixModulation(int switch_idx, uint32_t pwm0, uint32_t pwm1) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && pwm0>=0 &&
      pwm0<UINT32_MAX && pwm1>=0 && pwm1<UINT32_MAX) {
    matrixData.output_pwm[0][switch_idx][0]=pwm0;
    matrixData.output_pwm[0][switch_idx][1]=pwm1;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setFlux(int switch_idx, uint32_t flux) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES &&
      flux>=0 && flux<LONG_MAX) {
    matrixData.flux_value[0][switch_idx]=flux;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setOutputMode(int switch_idx, int output_mode) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES &&
      output_mode>=0 && output_mode<MAX_MATRIX_OUTPUT_MODES) {
    matrixData.output_mode[0][switch_idx]=output_mode;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setOverrideOutputValue(int switch_idx, uint32_t override_value) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES &&
      override_value>=LONG_MIN && override_value<LONG_MAX) {
    matrixData.computer_assist[0][switch_idx]=false;
    matrixData.override_output_value[0][switch_idx]=override_value;
    long i_retry;
    while (matrixData.computer_assist[0][switch_idx]!=false) {
      matrixData.computer_assist[0][switch_idx]=false;
      i_retry++;
      if (i_retry==MAX_MATRIX_OVERRIDE_TIME)
        {Serial.println("WARNING! Could not override computer_assist!"); break;}
      delayMicroseconds(1);
    }
    i_retry=0;
    while (matrixData.override_output_value[0][switch_idx]!=override_value) {
      matrixData.override_output_value[0][switch_idx]=override_value;
      i_retry++;
      if (i_retry==MAX_MATRIX_OVERRIDE_TIME)
        {Serial.println("WARNING! Could not override override_output_value!"); break;}
      delayMicroseconds(1);
    }
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

char *cmd_proc_xyzptr;

void setComputerAssist(int switch_idx, bool computer_assist) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && computer_assist>=0 && computer_assist<=1) {
    matrixData.computer_assist[0][switch_idx]=computer_assist;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setINSMode(int ins_mode) {
  if (ins_mode>=0 && ins_mode <MAX_INS_MODE)
    {insData.INS_MODE=ins_mode;}
}

void setINSGPSPrecision(double ins_precision) {
  if (ins_precision>=0 && ins_precision<DBL_MAX)
    {insData.INS_REQ_GPS_PRECISION=ins_precision;}
}

void setINSMinSpeed(double ins_min_speed) {
  if (ins_min_speed>=0 && ins_min_speed<DBL_MAX)
    {insData.INS_REQ_MIN_SPEED=ins_min_speed;}
}

void setINSHeadingRangeDiff(double ins_range_diff) {
  if (ins_range_diff>=0 && ins_range_diff<DBL_MAX)
    {insData.INS_REQ_HEADING_RANGE_DIFF=ins_range_diff;}
}

void setSpeedUnits(int speed_units) {
  if (speed_units>=0 && speed_units <MAX_SPEED_CONVERSION_MODES)
    {satioData.speed_conversion_mode=speed_units;}
}

void setUTCSecondOffset(int64_t seconds) {
  if (seconds>=LONG_LONG_MIN && seconds<LONG_LONG_MAX)
    {satioData.utc_second_offset=seconds;}
}

void setMapConfig(int map_slot,
                  int map_mode,
                  signed long c0,
                  signed long c1,
                  signed long c2,
                  signed long c3,
                  signed long c4,
                  signed long c5) {
  if (map_slot>=0 && map_slot<MAX_MAP_SLOTS &&
      map_mode>=0 && map_mode<MAX_MAP_MODES &&
      c0>=INT32_MIN && c0<INT32_MAX &&
      c1>=INT32_MIN && c1<INT32_MAX &&
      c2>=INT32_MIN && c2<INT32_MAX &&
      c3>=LONG_MIN && c3<INT32_MAX &&
      c4>=INT32_MIN && c4<INT32_MAX &&
      c5>=INT32_MIN && c5<INT32_MAX) {
    mappingData.mapping_config[0][map_slot][0]=c0;
    mappingData.mapping_config[0][map_slot][1]=c1;
    mappingData.mapping_config[0][map_slot][2]=c2;
    mappingData.mapping_config[0][map_slot][3]=c3;
    mappingData.mapping_config[0][map_slot][4]=c4;
    mappingData.mapping_config[0][map_slot][5]=c5;
    mappingData.map_mode[0][map_slot]=map_mode;
    matrixData.matrix_switch_write_required[0][map_slot]=true;
  }
}

void setMapSlot(int matrix_switch,
                int map_slot) {
  if (matrix_switch>=0 && matrix_switch<MAX_MAP_SLOTS &&
      map_slot>=0 && map_slot<=1) {
    mappingData.index_mapped_value[0][matrix_switch]=map_slot;
    matrixData.matrix_switch_write_required[0][map_slot]=true;
  }
}

void saveMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_SLOTS) {
    memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath));
    strcpy(satioFileData.current_matrix_filepath, satioFileData.matix_filepaths[matrix_file_slot]);
    sdmmcFlagData.save_matrix=true;
  }
}

void loadMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_SLOTS) {
    memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath));
    strcpy(satioFileData.current_matrix_filepath, satioFileData.matix_filepaths[matrix_file_slot]);
    sdmmcFlagData.load_matrix=true;
  }
}

void deleteMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_SLOTS) {
    memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath));
    strcpy(satioFileData.current_matrix_filepath, satioFileData.matix_filepaths[matrix_file_slot]);
    sdmmcFlagData.delete_matrix=true;
  }
}

void star_nav() {
  // star sirius test: starnav 6 45 8.9 -16 42 58.0
  // ngc test:         starnav 2 20 35.0 -23 7 0.0
  // ic test:          starnav 17 46 18 5 43 00
  // other obj test:   starnav 1 36 0 61 17 0
  // messier test:     starnav 16 41 40 36 28 0
  // caldwel test:     starnav 1 19 32.6 58 17 27
  // Herschel test:    starnav 0 29 56.0 60 14 0.0
  // new stars test    starnav 0 02 07.2 -14 40 34
  // caldwel test:     starnav 00 13 0 72 32 0
  // non-h400          starnav 7 38 3 -14 52 3
  //                   starnav 1 33 9 30 39 0
  simple_argparser_init_from_buffer(&plainparser, serial0Data.BUFFER, 1);
  if ((str_is_int32(plainparser.tokens[0])==true) &&
      (str_is_int32(plainparser.tokens[1])==true) &&
      (str_is_float(plainparser.tokens[2])==true) &&
      (str_is_int32(plainparser.tokens[3])==true) &&
      (str_is_int32(plainparser.tokens[4])==true) &&
      (str_is_float(plainparser.tokens[5])==true)
    )
  {
    Serial.println("attempting to identify object..");
    // this is identify (so first identify object)
    IdentifyObject(
      atoi(plainparser.tokens[0]),
      atoi(plainparser.tokens[1]),
      atof(plainparser.tokens[2]),
      atoi(plainparser.tokens[3]),
      atoi(plainparser.tokens[4]),
      atof(plainparser.tokens[5])
    );
    //
    /*
      Once identified we can track object (requires modified SiderealObjects lib).
    */
    trackObject(satioData.degrees_latitude, satioData.degrees_longitude,
      satioData.rtc_year, satioData.rtc_month, satioData.rtc_mday,
      satioData.rtc_hour, satioData.rtc_minute, satioData.rtc_second,
      satioData.local_hour, satioData.local_minute, satioData.local_second,
      atof(gnggaData.altitude), siderealObjectData.object_table_i, siderealObjectData.object_number);
    Serial.println("---------------------------------------------");
    Serial.println("Table Index:   " + String(siderealObjectData.object_table_i));
    Serial.println("Table:         " + String(siderealObjectData.object_table_name));
    Serial.println("Number:        " + String(siderealObjectData.object_number));
    Serial.println("Name:          " + String(siderealObjectData.object_name));
    Serial.println("Type:          " + String(siderealObjectData.object_type));
    Serial.println("Constellation: " + String(siderealObjectData.object_con));
    Serial.println("Distance:      " + String(siderealObjectData.object_dist));
    Serial.println("Azimuth:       " + String(siderealObjectData.object_az));
    Serial.println("Altitude:      " + String(siderealObjectData.object_alt));
    Serial.println("Rise:          " + String(siderealObjectData.object_r));
    Serial.println("Set:           " + String(siderealObjectData.object_s));
    Serial.println("---------------------------------------------");
  }
  else {Serial.println("identify object: bad input data");}
}

void unmountSDCard() {
  sdmmcFlagData.no_delay_flag=true;
  sdmmcFlagData.unmount_sdcard_flag=true;
}

void mountSDCard() {
  sdmmcFlagData.no_delay_flag=true;
  sdmmcFlagData.mount_sdcard_flag=true;
}

/*
  Debug ArgParse.
  Expected behaviour:
  command: foo -a -b -c
  flags:   a b c
  command: foo -a 1 -b 2 -c 3
  flags:   a="1" b="2" c="3"
  Note:
    - For best practice only use ArgParser if flags are required, else use PlainArgParser for simple tokenization.
    - Use PlainArgParser if processing negative numbers.
    - short flags: 1-3 alphanumeric chars. example: -a, -a1, -a12, -abc.
    - long flags: 1-256 alphanumeric chars. example: --foobar, --foo-bar, --foobar123.
    - see ArgParser for more details.
*/
size_t pos_count;
const char** pos;
bool verbose;
bool verbose_1;
bool enable;

void printArgParse() {
  Serial.println("-------------------------------------------");
  Serial.print("[debug] First command: ");
  if (pos_count > 0) {Serial.println(pos[0]);}
  else {Serial.println("none");}
  Serial.print("[debug] Positionals (");
  Serial.print(pos_count);
  Serial.print("): ");
  for (size_t j = 0; j < pos_count; ++j)
    {Serial.print(pos[j]); if (j < pos_count - 1) Serial.print(" ");}
  Serial.println();
  Serial.println("----");
  Serial.print("[debug] Flag count: ");
  Serial.println(parser.flag_count);
  Serial.print("[debug] Flags: ");
  for (size_t k = 0; k < parser.flag_count; ++k)
    {Serial.print(parser.flags[k]); const char* val = parser.values[k];
      if (val[0] != '\0') {Serial.print("=\""); Serial.print(val); Serial.print("\"");}
      if (k < parser.flag_count - 1) Serial.print(" ");
  }
  Serial.println();
  Serial.println("-------------------------------------------");
}

// ---------------------------------------------------------------------------------------------------------------
/*
  Serial RXD : Command Process.
*/
// ---------------------------------------------------------------------------------------------------------------
void CmdProcess(void) {
  memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
  while (Serial.available())
    {Serial.readBytesUntil('\n', serial0Data.BUFFER, sizeof(serial0Data.BUFFER)-1);}
  if (strlen(serial0Data.BUFFER)>=2) {
    // Debug Serial Buffer.
    Serial.println("[CmdProcess] " + String(serial0Data.BUFFER));
    // Initialize argparse.
    argparser_reset(&parser);
    if (!argparser_init_from_buffer(&parser, serial0Data.BUFFER))
      {fprintf(stderr, "[cmd] Failed to initialize parser from buffer\n"); return;}
    pos_count=0; pos={}; pos = argparser_get_positionals(&parser, &pos_count);
    // Verbosity.
    verbose=false; verbose_1=false;
    verbose = argparser_get_bool(&parser, "v") || argparser_get_bool(&parser, "verbose");
    verbose_1 = argparser_get_bool(&parser, "vv") || argparser_get_bool(&parser, "verbose1");
    if (verbose_1) {verbose=true;}
    if (verbose==false) {verbose_1=false;}
    Serial.println("[cmd] verbose: " + String(verbose));
    Serial.println("[cmd] verbose1: " + String(verbose_1));
    // Debug Arg Parse.
    printArgParse();
    // Commands.
    if (strcmp(pos[0], "help")==0 || strcmp(pos[0], "h")==0)
      {printf("Usage: [buffer with] [--flag value] [-f value] [positional...]\n");
      if (verbose) {PrintHelp();}
    }
    else if (strcmp(pos[0], "stat")==0) {
      enable=false;
      if (argparser_has_flag(&parser, "disable") || argparser_has_flag(&parser, "d")) {enable=false;}
      else if (argparser_has_flag(&parser, "enable") || argparser_has_flag(&parser, "e")) {enable=true;}
      if (argparser_has_flag(&parser, "t")) {
        if (enable) {systemData.output_stat=enable; systemData.output_stat_v=verbose; systemData.output_stat_vv=verbose_1;}
        else {systemData.output_stat=false; systemData.output_stat_v=false; systemData.output_stat_vv=false;}
      }
      if (argparser_has_flag(&parser, "partition-table")) {print_partition_table();}
      if (argparser_has_flag(&parser, "memory-ram")) {print_ram_info();}
      if (argparser_has_flag(&parser, "sdcard")) {PrintSDCardInformation();}
      if (strcmp(serial0Data.BUFFER, "stat --system")==0) {PrintSystemData();}
      if (strcmp(serial0Data.BUFFER, "stat --matrix\r")==0) {PrintMatrixConfig();}
      if (strncmp(serial0Data.BUFFER, "stat --matrix ", strlen("stat --matrix "))==0) {
        if (argparser_has_flag(&parser, "A")) {PrintMatrixData();}
        else {simple_argparser_init_from_buffer(&plainparser, serial0Data.BUFFER, 2);
          if (val_switch_index(plainparser.tokens[0])) {PrintMatrixNData();}}
      }
      else if (strncmp(serial0Data.BUFFER, "stat --mapping", strlen("stat --mapping"))==0) {PrintMappingData();}
      else if (argparser_has_flag(&parser, "sentence")) {
        if (argparser_has_flag(&parser, "A")) {systemData.output_satio_all=enable; setAllSentenceOutput(enable);}
        if (argparser_has_flag(&parser, "satio")) {systemData.output_satio_enabled=enable;}
        if (argparser_has_flag(&parser, "gngga")) {systemData.output_gngga_enabled=enable;}
        if (argparser_has_flag(&parser, "gnrmc")) {systemData.output_gnrmc_enabled=enable;}
        if (argparser_has_flag(&parser, "gpatt")) {systemData.output_gpatt_enabled=enable;}
        if (argparser_has_flag(&parser, "ins")) {systemData.output_ins_enabled=enable;}
        if (argparser_has_flag(&parser, "matrix")) {systemData.output_matrix_enabled=enable;}
        if (argparser_has_flag(&parser, "admplex0")) {systemData.output_admplex0_enabled=enable;}
        if (argparser_has_flag(&parser, "gyro0")) {systemData.output_gyro_0_enabled=enable;}
        if (argparser_has_flag(&parser, "sun")) {systemData.output_sun_enabled=enable;}
        if (argparser_has_flag(&parser, "moon")) {systemData.output_moon_enabled=enable;}
        if (argparser_has_flag(&parser, "mercury")) {systemData.output_mercury_enabled=enable;}
        if (argparser_has_flag(&parser, "venus")) {systemData.output_venus_enabled=enable;}
        if (argparser_has_flag(&parser, "mars")) {systemData.output_mars_enabled=enable;}
        if (argparser_has_flag(&parser, "jupiter")) {systemData.output_jupiter_enabled=enable;}
        if (argparser_has_flag(&parser, "saturn")) {systemData.output_saturn_enabled=enable;}
        if (argparser_has_flag(&parser, "uranus")) {systemData.output_uranus_enabled=enable;}
        if (argparser_has_flag(&parser, "neptune")) {systemData.output_neptune_enabled=enable;}
        if (argparser_has_flag(&parser, "meteors")) {systemData.output_meteors_enabled=enable;}
      }
    }
    
    else if (strcmp(pos[0], "ls")==0) {
      const char* path_str = argparser_get_path(&parser, "/");
      Serial.println("[cmd] Path: " + String(path_str));
      int maxlevels=0;
      if (argparser_has_flag(&parser, "R")) {maxlevels=-1;}
      else {maxlevels = argparser_get_int8(&parser, "maxlevels", 1);}
      memset(sdmmcArgData.buffer, 0, sizeof(sdmmcArgData.buffer));
      strcpy(sdmmcArgData.buffer, path_str);
      sdmmcArgData.maxlevels=maxlevels;
      sdmmcFlagData.no_delay_flag=true;
      sdmmcFlagData.list_dir_flag=true;
    }

    else if (strcmp(pos[0], "starnav")==0) {star_nav();}
    
    if (systemData.serial_command) {
      if (strcmp(pos[0], "system")==0) {
        if (argparser_has_flag(&parser, "save")) {sdmmcFlagData.save_system=true;}
        else if (argparser_has_flag(&parser, "load")) {sdmmcFlagData.load_system=true;}
        else if (argparser_has_flag(&parser, "restore-defaults")) {restore_system_defaults();}
      }
      else if (strcmp(pos[0], "mapping")==0) {
        if (argparser_has_flag(&parser, "new")) {set_all_mapping_default(); return;}
        else if (argparser_has_flag(&parser, "save")) {sdmmcFlagData.save_mapping=true;}
        else if (argparser_has_flag(&parser, "load")) {sdmmcFlagData.load_mapping=true;}
        else if (argparser_has_flag(&parser, "delete")) {sdmmcFlagData.delete_mapping=true;}
        else {
          int s  = argparser_get_int8(&parser, "s", -1);
          if (s==-1) {return;}
          setMapConfig(s, argparser_get_int8(&parser, "m", mappingData.map_mode[0][s]),
                       argparser_get_int32(&parser, "c0", mappingData.mapping_config[0][s][0]),
                       argparser_get_int32(&parser, "c1", mappingData.mapping_config[0][s][1]),
                       argparser_get_int32(&parser, "c2", mappingData.mapping_config[0][s][2]),
                       argparser_get_int32(&parser, "c3", mappingData.mapping_config[0][s][3]),
                       argparser_get_int32(&parser, "c4", mappingData.mapping_config[0][s][4]),
                       argparser_get_int32(&parser, "c5", mappingData.mapping_config[0][s][5]));}
      }
      else if (strcmp(pos[0], "matrix")==0) {
        if (argparser_has_flag(&parser, "startup-enable")) {matrixData.load_matrix_on_startup=true;}
        else if (argparser_has_flag(&parser, "startup-disable")) {matrixData.load_matrix_on_startup=false;}
        else if (argparser_has_flag(&parser, "new")) {set_all_matrix_default(); return;}
        else if (argparser_has_flag(&parser, "save")) {saveMatrix(argparser_get_int8(&parser, "save", -1));}
        else if (argparser_has_flag(&parser, "load")) {loadMatrix(argparser_get_int8(&parser, "load", -1));}
        else if (argparser_has_flag(&parser, "delete")) {deleteMatrix(argparser_get_int8(&parser, "delete", -1));}
        else {
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "p")) {
            setMatrixPort(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "p", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fn")) {
            setMatrixFunction(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", -1), argparser_get_int8(&parser, "fn", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fx")) {
            setMatrixX(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_double(&parser, "fx", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fy")) {
            setMatrixY(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_double(&parser, "fy", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fz")) {
            setMatrixZ(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_double(&parser, "fz", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fi")) {
            setMatrixInverted(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_bool(&parser, "fi"));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "fo")) {
            setMatrixOperator(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_int8(&parser, "fo", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "pwm0") && argparser_has_flag(&parser, "pwm1")) {
            setMatrixModulation(argparser_get_int8(&parser, "s", -1), argparser_get_uint32(&parser, "pwm0", 0), argparser_get_uint32(&parser, "pwm1", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "flux")) {
            setFlux(argparser_get_int8(&parser, "s", -1), argparser_get_uint32(&parser, "flux", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "oride")) {
            setOverrideOutputValue(argparser_get_int8(&parser, "s", -1), argparser_get_int32(&parser, "oride", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "computer-assist")) {
            setComputerAssist(argparser_get_int8(&parser, "s", -1), argparser_get_bool(&parser, "computer-assist"));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "omode")) {
            setOutputMode(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "omode", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "map-slot")) {
            setMapSlot(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "map-slot", -1));
          }
        }
      }
      else if (strcmp(pos[0], "ins")==0) {
        if (argparser_has_flag(&parser, "m")) {setINSMode(argparser_get_int8(&parser, "m", INS_MODE_DYNAMIC));}
        if (argparser_has_flag(&parser, "gyro")) {insData.INS_USE_GYRO_HEADING=argparser_get_bool(&parser, "gyro");}
        if (argparser_has_flag(&parser, "p")) {setINSGPSPrecision(argparser_get_double(&parser, "p", 0.5));}
        if (argparser_has_flag(&parser, "s")) {setINSMinSpeed(argparser_get_double(&parser, "s", 0.3));}
        if (argparser_has_flag(&parser, "r")) {setINSHeadingRangeDiff(argparser_get_double(&parser, "r", 0));}
        if (argparser_has_flag(&parser, "reset-forced")) {insData.INS_FORCED_ON_FLAG=false;}
      }
      else if (strcmp(pos[0], "satio")==0) {
        if (argparser_has_flag(&parser, "speed-units")) {setSpeedUnits(argparser_get_int8(&parser, "speed-units", 0));}
        if (argparser_has_flag(&parser, "utc-offset")) {setUTCSecondOffset(argparser_get_int64(&parser, "utc-offset", 0));}
        if (argparser_has_flag(&parser, "mode-gngga")) {satioData.coordinate_conversion_mode=0;}
        else if (argparser_has_flag(&parser, "mode-gnrmc")) {satioData.coordinate_conversion_mode=1;}
      }
      else if (strcmp(pos[0], "gyro")==0) {
        if (argparser_has_flag(&parser, "calacc")) {WT901CalAcc();}
        if (argparser_has_flag(&parser, "calmag-start")) {WT901CalMagStart();}
        else if (argparser_has_flag(&parser, "calmag-stop")) {WT901CalMagEnd();}
      }
      else if (strcmp(pos[0], "sdcard")==0) {
        if (argparser_has_flag(&parser, "mount")) {mountSDCard();}
        else if (argparser_has_flag(&parser, "unmount")) {unmountSDCard();}
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
/*
  Serial TXD : Output.
*/
// ---------------------------------------------------------------------------------------------------------------
void outputSentences(void) {
  if (systemData.interval_breach_1_second==true) {outputStat();}

  if (systemData.interval_breach_gps) {
    systemData.interval_breach_gps=0;
    if (systemData.output_gngga_enabled) {Serial.println(gnggaData.outsentence);}
    if (systemData.output_gnrmc_enabled) {Serial.println(gnrmcData.outsentence);}
    if (systemData.output_gpatt_enabled) {Serial.println(gpattData.outsentence);}
    if (systemData.output_satio_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$SATIO,");
      strcat(serial0Data.BUFFER, String(satioData.padded_rtc_time_HHMMSS).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(satioData.padded_rtc_date_DDMMYYYY).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(satioData.padded_rtc_sync_time_HHMMSS).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(satioData.padded_rtc_sync_date_DDMMYYYY).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(satioData.padded_local_time_HHMMSS).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(satioData.padded_local_date_DDMMYYYY).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(systemData.uptime_seconds).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(satioData.degrees_latitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(satioData.degrees_longitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
  }
  if (systemData.interval_breach_ins) {
    systemData.interval_breach_ins = 0;
    if (systemData.output_ins_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$INS,");
      strcat(serial0Data.BUFFER, String(satioData.rtc_unixtime).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.INS_INITIALIZATION_FLAG).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_latitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_longitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_altitude).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_heading).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_speed).c_str());
      strcat(serial0Data.BUFFER, ",");
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
  }
  if (systemData.interval_breach_gyro_0) {
    systemData.interval_breach_gyro_0 = 0;
    if (systemData.output_gyro_0_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$GYRO0,");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_acc_x).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_acc_y).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_acc_z).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_ang_x).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_ang_y).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_ang_z).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_gyr_x).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_gyr_y).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_gyr_z).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_mag_x).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_mag_y).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(gyroData.gyro_0_mag_z).c_str());
      strcat(serial0Data.BUFFER, ",");
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
  }
  if (systemData.interval_breach_track_planets) {
    systemData.interval_breach_track_planets = 0;
    if (systemData.output_sun_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$SUN,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.sun_s + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_moon_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MOON,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_p + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.moon_lum + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_mercury_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MERCURY,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mercury_ecliptic_long + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_venus_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$VENUS,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.venus_ecliptic_long + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_mars_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MARS,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.mars_ecliptic_long + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_jupiter_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$JUPITER,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.jupiter_ecliptic_long + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_saturn_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$SATURN,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.saturn_ecliptic_long + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_uranus_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$URANUS,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.uranus_ecliptic_long + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_neptune_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$NEPTUNE,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_helio_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_helio_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_radius_vector + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_distance + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.neptune_ecliptic_long + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
    if (systemData.output_meteors_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$METEOR,");
      for (int i=0; i<MAX_METEOR_SHOWERS; i++) {
        strcat(serial0Data.BUFFER, String(String(meteor_shower_warning_system[i][0]) + String(",")).c_str());
        strcat(serial0Data.BUFFER, String(String(meteor_shower_warning_system[i][1]) + String(",")).c_str());
      }
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
  }
  // if (systemData.interval_breach_matrix) {
    // systemData.interval_breach_matrix = 0;
    if (systemData.output_matrix_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcpy(serial0Data.BUFFER, "$MATRIX,");
      // append matrix switch state data
      for (int i=0; i < MAX_MATRIX_SWITCHES; i++)
        {strcat(serial0Data.BUFFER, String(String(matrixData.switch_intention[0][i])+",").c_str());}
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
  // }
  if (systemData.interval_breach_mplex) {
    systemData.interval_breach_mplex = 0;
    if (systemData.output_admplex0_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MPLEX0,");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[0]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[1]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[2]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[3]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[4]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[5]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[6]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[7]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[8]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[9]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[10]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[11]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[12]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[13]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[14]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(multiplexerData.ADMPLEX_0_DATA[15]).c_str());
      strcat(serial0Data.BUFFER, ",");
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      Serial.println(serial0Data.BUFFER);
    }
  }
}

void printArray(signed long arr[], int start, int end) {
    for (int i = start; i < end; i++) {
        printf("%-7d", arr[i]); // Left-align with 5-character width
    }
    printf("\n");
}

// uncomment to use
signed long print_index_0[35]={0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                              11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                              21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
                              32, 33, 34};
signed long print_index_1[35]={35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45,
                               46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56,
                               57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
                               68, 69};
char counter_chars_0[15][56]={"Loops p/s",
                              "GPS p/s",
                              "INS p/s",
                              "Gyro0 p/s",
                              "ADMplex0 p/s",
                              "CMD p/s",
                              "Universe p/s",
                              "Matrix p/s",
                              "I/O p/s",
                              "LT uS",
                              "LT Max uS",
                              "Satellites",
                              "GPS Precision"};
double counter_digits_0[15]={};

char counter_chars_1_row_0[9][56]={"Time",
                                   "Date",
                                   "UNIX Time",
                                   "Latitude",
                                   "Longitude",
                                   "Altitude",
                                   "Heading",
                                   "Speed",
                                   "Mileage"};
char counter_chars_1_col_0[5][56]={"GPS",
                                   "RTC",
                                   "RTC Sync",
                                   "System",
                                   "System INS"};
char counter_digits_1_row_N[9][56]={};

void outputStat(void) {
    // ---------------------------------------------------------------------
    //                                                 EXECUTIONS PER SECOND
    // ---------------------------------------------------------------------
    // note that output will not be suitable at too large a font/zoom
    // ---------------------------------------------------------------------
    // System Stat Overview.
    // column width supports numbers of up to 9,999,999.
    // All matrix xyz digits are rounded to fit column width.
    // Column width is 7 (million+space).
    // Any chars/digits wider than 6 chars will overlap/touch next column.
    // For development & diagnostic purposes.
    // ---------------------------------------------------------------------
    if (systemData.output_stat==true || systemData.output_stat_v==true || systemData.output_stat_vv) {
    // printAllTimes();
    // ----------------------------------------------------------------------------------------------------------------------------
    //                                                                                                               PRINT COUNTERS
    // ----------------------------------------------------------------------------------------------------------------------------
    Serial.println();
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    // printf("taskGyro", "Unused stack: %u words\n", watermark_task_gyro);
    // printf("taskUniverse", "Unused stack: %u words\n", watermark_task_universe);
    // printf("taskSwitches", "Unused stack: %u words\n", watermark_task_switches);
    // printf("taskGPS", "Unused stack: %u words\n", watermark_task_gps);
    // printf("taskMultiplexers", "Unused stack: %u words\n", watermark_task_multiplexers);
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    counter_digits_0[0]=systemData.total_loops_a_second;
    counter_digits_0[1]=systemData.total_gps;
    counter_digits_0[2]=systemData.total_ins;
    counter_digits_0[3]=systemData.total_gyro;
    counter_digits_0[4]=systemData.total_mplex;
    counter_digits_0[5]=systemData.total_infocmd;
    counter_digits_0[6]=systemData.total_universe;
    counter_digits_0[7]=systemData.total_matrix;
    counter_digits_0[8]=systemData.total_portcon;
    counter_digits_0[9]=systemData.mainLoopTimeTaken;
    counter_digits_0[10]=systemData.mainLoopTimeTakenMax;
    counter_digits_0[11]=atoi(gnggaData.satellite_count);
    counter_digits_0[12]=atof(gnggaData.gps_precision_factor);
    for (int i = 0; i < 13; i++) {printf("%-16s", counter_chars_0[i]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    for (int i = 0; i < 13; i++) {printf("%-16f", counter_digits_0[i]);}
    printf("\n");
    Serial.println();
    // ----------------------------------------------------------------------------------------------------------------------------
    //                                                                                                           PRINT PRIMARY DATA
    // ----------------------------------------------------------------------------------------------------------------------------
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                   ");
    for (int i = 0; i < 9; i++) {printf("%-19s", counter_chars_1_row_0[i]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");

    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gnrmcData.utc_time).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gnrmcData.utc_date).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); // null
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); strcpy(counter_digits_1_row_N[3], String(gnrmcData.latitude).c_str());
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); strcpy(counter_digits_1_row_N[4], String(gnrmcData.longitude).c_str());
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(gnggaData.altitude, 7).c_str());
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); strcpy(counter_digits_1_row_N[6], String(gnrmcData.ground_heading, 7).c_str());
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); strcpy(counter_digits_1_row_N[7], String(gnrmcData.ground_speed, 7).c_str());
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); strcpy(counter_digits_1_row_N[8], String(gpattData.mileage, 7).c_str());
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");
    
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(satioData.padded_rtc_time_HHMMSS).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(satioData.padded_rtc_date_DDMMYYYY).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(satioData.rtc_unixtime).c_str());
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); // null
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); // null
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); // null
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); // null
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); // null
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); // null
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+1]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");

    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(satioData.padded_rtc_sync_time_HHMMSS).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(satioData.padded_rtc_sync_date_DDMMYYYY).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(satioData.rtcsync_unixtime).c_str());
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); strcpy(counter_digits_1_row_N[3], String(satioData.rtcsync_latitude).c_str());
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); strcpy(counter_digits_1_row_N[4], String(satioData.rtcsync_longitude).c_str());
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(satioData.rtcsync_altitude).c_str());
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); // null
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); // null
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); // null
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+2]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");

    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(satioData.padded_local_time_HHMMSS).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(satioData.padded_local_date_DDMMYYYY).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(satioData.local_unixtime_uS).c_str());
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); strcpy(counter_digits_1_row_N[3], String(satioData.degrees_latitude, 7).c_str());
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); strcpy(counter_digits_1_row_N[4], String(satioData.degrees_longitude, 7).c_str());
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(satioData.altitude).c_str()); // altitude: use srtm for meters above terrain elevation level (make other conversions also)
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); strcpy(counter_digits_1_row_N[6], String(satioData.ground_heading).c_str());
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); strcpy(counter_digits_1_row_N[7], String(String(satioData.speed, 2) + " " + String(satioData.char_speed_conversion_mode[satioData.speed_conversion_mode])).c_str()); // speed (3 dimensional: requires lat, long, alt, microtime)
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); strcpy(counter_digits_1_row_N[8], String(satioData.mileage).c_str()); // make mileage any 3d direction, refactor 'distance'
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+3]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");
    // utc offset
    // gforce.
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(satioData.padded_local_time_HHMMSS).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(satioData.padded_local_date_DDMMYYYY).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(satioData.local_unixtime_uS).c_str());
    memset(counter_digits_1_row_N[3], 0, sizeof(counter_digits_1_row_N[3])); strcpy(counter_digits_1_row_N[3], String(insData.ins_latitude, 7).c_str());
    memset(counter_digits_1_row_N[4], 0, sizeof(counter_digits_1_row_N[4])); strcpy(counter_digits_1_row_N[4], String(insData.ins_longitude, 7).c_str());
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(insData.ins_altitude).c_str());
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); strcpy(counter_digits_1_row_N[6], String(insData.ins_heading).c_str());
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); strcpy(counter_digits_1_row_N[7], String(String(insData.ins_speed, 2) + " " + String(satioData.char_speed_conversion_mode[satioData.speed_conversion_mode])).c_str());
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); strcpy(counter_digits_1_row_N[8], String("pending").c_str());
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+4]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("INS MODE : " + String(insData.INS_MODE) + " (" + String(insData.char_ins_mode[insData.INS_MODE]) + ") | INS FLAG : " + String(insData.INS_INITIALIZATION_FLAG) + "/" + String(MAX_INS_INITIALIZATION_FLAG) + " | INS FORCED ON FLAG : " + String(insData.INS_FORCED_ON_FLAG)
    + " | INS_REQ_GPS_PRECISION : " + String(insData.INS_REQ_GPS_PRECISION) + " | INS_REQ_MIN_SPEED : " + String(insData.INS_REQ_MIN_SPEED) + " | INS_REQ_HEADING_RANGE_DIFF : " + String(insData.INS_REQ_HEADING_RANGE_DIFF));
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("                   X                  Y                  Z");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_ang_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_ang_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_ang_z).c_str());
    Serial.print("Angle              ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_gyr_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_gyr_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_gyr_z).c_str());
    Serial.print("Gyro               ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_acc_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_acc_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_acc_z).c_str());
    Serial.print("Acceleration       ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_mag_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_mag_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_mag_z).c_str());
    Serial.print("Magnetic Field     ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("Weekday int: " + String(satioData.local_wday) + " Weekday Name: " + String(satioData.local_wday_name));
    Serial.print("Meteor Warning: ");
    for (int i = 0; i < MAX_METEOR_SHOWERS; i++) {Serial.print(String(String(meteor_shower_names[i]) + ": " + String(meteor_shower_warning_system[i][0]) + " " + String(meteor_shower_warning_system[i][1]) + " | ").c_str());}
    Serial.println();
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("SDCard mounted: " + String(sdcardData.sdcard_mounted) + " (" + String(sdcardData.sdcard_type_names[sdcardData.sdcard_type]) + " (type: " + String(sdcardData.sdcard_type) + ")");
  }
    // ----------------------------------------------------------------------------------------------------------------------------
    //                                                                                                      PRINT PROGRAMMABLE DATA
    // ----------------------------------------------------------------------------------------------------------------------------
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                      ");
    printArray(print_index_0, 0, 35);
    Serial.print("Computer Assist    :  ");
    for (int i=0;i<35;i++) {Serial.print(String(matrixData.computer_assist[0][i]) + "      ");}
    Serial.println();
    Serial.print("Switch Intention   :  ");
    for (int i=0;i<35;i++) {Serial.print(String(matrixData.switch_intention[0][i]) + "      ");}
    Serial.println();
    Serial.print("Computer Intention :  ");
    for (int i=0;i<35;i++) {Serial.print(String(matrixData.computer_intention[0][i]) + "      ");}
    Serial.println();
    Serial.print("Output Value       :  ");
    printArray(matrixData.output_value[0], 0, 35);
    }
    if (systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 0  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][0]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 1  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][1]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 2  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][2]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 3  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][3]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 4  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][4]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 0  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][5]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 6  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][6]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 7  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][7]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 8  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][8]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 9  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][9]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    }
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                      ");
    printArray(print_index_1, 0, 35);
    Serial.print("Computer Assist    :  ");
    for (int i=35;i<70;i++) {Serial.print(String(matrixData.computer_assist[0][i]) + "      ");}
    Serial.println();
    Serial.print("Switch Intention   :  ");
    for (int i=35;i<70;i++) {Serial.print(String(matrixData.switch_intention[0][i]) + "      ");}
    Serial.println();
    Serial.print("Computer Intention :  ");
    for (int i=35;i<70;i++) {Serial.print(String(matrixData.computer_intention[0][i]) + "      ");}
    Serial.println();
    Serial.print("Output Value       :  ");
    printArray(matrixData.output_value[0], 35, 70);
    }
    if (systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 0  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][0]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 1  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][1]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 2  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][2]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 3  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][3]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 4  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][4]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 0  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][5]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 6  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][6]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 7  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][7]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 8  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][8]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Switch Function 9  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][9]);}
    printf("\n");
    Serial.print("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    Serial.print("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    Serial.print("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    }
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                      ");
    printArray(print_index_0, 0, 35);
    Serial.print("Mapped Values      :  ");
    printArray(mappingData.mapped_value[0], 0, 35);
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("                      ");
    printArray(print_index_1, 0, 35);
    Serial.print("Mapped Values      :  ");
    printArray(mappingData.mapped_value[0], 35, 70);
    }
}
