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
// #include "esp32_helper.h"
#include "sdcard_helper.h"
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
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "task_handler.h"
#include "config.h"
#include "system_data.h"
#include "multiplexers.h"


bool debug_bool=true;
void debug(String debug_str) {if (debug_bool==true) {printf("%s\n", debug_str.c_str());}}

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
  printf("%s\n", data);
  if (str_is_int8(data)!=true) {return false;}
  if (atoi(data)<MAX_MAPPABLE_VALUES) {return true;} 
  return false;
}

bool val_map_slot_index(const char * data) {
  printf("%s\n", data);
  if (str_is_int8(data)!=true) {return false;}
  if (atoi(data)<MAX_MAP_SLOTS) {return true;} 
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
  puts(
  R"(
  [ System ]

      system --save               Takes no further arguments.
      system --load               Takes no further arguments.
      system --restore-defaults   Takes no further arguments.
      system -log                 Automatically log data to disk (See performance for timing). Takes arguments -e, -d.

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
      matrix -fn n                Set function -f for switch -s. Primary Comparitors:
                                  [0] NONE
                                  [1] ON
                                  [2] Switch Link
                                  [3] Time HHMMSS
                                  [4] Week Day
                                  [5] Month Day
                                  [6] Month
                                  [7] Year
                                  [8] SatIO Deg Lat
                                  [9] SatIO Deg Lon
                                  [10] SatIO INS Lat
                                  [11] SatIO INS Lon
                                  [12] SatIO INS Heading
                                  [13] SatIO INS Alt
                                  [14] GNGGA Status
                                  [15] GNGGA Sat Count
                                  [16] GNGGA Prescion
                                  [17] GNGGA Altitude
                                  [18] GNRMC Ground Speed
                                  [19] GNRMC Heading
                                  [20] GPATT Line
                                  [21] GPATT Static
                                  [22] GPATT Run State
                                  [23] GPATT INS
                                  [24] GPATT Mileage
                                  [25] GPATT GST
                                  [26] GPATT Yaw
                                  [27] GPATT Roll
                                  [28] GPATT Pitch
                                  [29] GNGGA Valid CS
                                  [30] GNRMC Valid CS
                                  [31] GPATT Valid CS
                                  [32] GNGGA Valid CD
                                  [33] GNRMC Valid CD
                                  [34] GPATT Valid CD
                                  [35] GNRMC Pos Stat A
                                  [36] GNRMC Pos Stat V
                                  [37] GNRMC Mode Ind A
                                  [38] GNRMC Mode Ind D
                                  [39] GNRMC Mode Ind E
                                  [40] GNRMC Mode Ind N
                                  [41] GNRMC Hemi North
                                  [42] GNRMC Hemi South
                                  [43] GNRMC Hemi East
                                  [44] GNRMC Hemi West
                                  [45] G0 G-Force X
                                  [46] G0 G-Force Y
                                  [47] G0 G-Force Z
                                  [48] G0 Incline X
                                  [49] G0 Incline Y
                                  [50] G0 Incline Z
                                  [51] G0 Mag Field X
                                  [52] G0 Mag Field Y
                                  [53] G0 Mag Field Z
                                  [54] G0 Velocity X
                                  [55] G0 Velocity Y
                                  [56] G0 Velocity Z
                                  [57] Meteor
                                  [58] Sun Azimuth
                                  [59] Sun Altitude
                                  [60] Sun Helio Ecl Lat
                                  [61] Sun Helio Ecl Lon
                                  [62] Luna Azimuth
                                  [63] Luna Altitude
                                  [64] Luna Phase
                                  [65] Mercury Azimuth
                                  [66] Mercury Altitude
                                  [67] Mercury H.Ecliptic Lat
                                  [68] Mercury H.Ecliptic Lon
                                  [69] Mercury Ecliptic Lat
                                  [70] Mercury Ecliptic Lon
                                  [71] Venus Azimuth
                                  [72] Venus Altitude
                                  [73] Venus H.Ecliptic Lat
                                  [74] Venus H.Ecliptic Lon
                                  [75] Venus Ecliptic Lat
                                  [76] Venus Ecliptic Lon
                                  [77] Earth Ecliptic Lon
                                  [78] Mars Azimuth
                                  [79] Mars Altitude
                                  [80] Mars H.Ecliptic Lat
                                  [81] Mars H.Ecliptic Lon
                                  [82] Mars Ecliptic Lat
                                  [83] Mars Ecliptic Lon
                                  [84] Jupiter Azimuth
                                  [85] jupiter Altitude
                                  [86] Jupiter H.Ecliptic Lat
                                  [87] Jupiter H.Ecliptic Lon
                                  [88] Jupiter Ecliptic Lat
                                  [89] Jupiter Ecliptic Lon
                                  [90] Saturn Azimuth
                                  [91] Saturn Altitude
                                  [92] Saturn H.Ecliptic Lat
                                  [93] Saturn H.Ecliptic Lon
                                  [94] Saturn Ecliptic Lat
                                  [95] Saturn Ecliptic Lon
                                  [96] Uranus Azimuth
                                  [97] Uranus Altitude
                                  [98] Uranus H.Ecliptic Lat
                                  [99] Uranus H.Ecliptic Lon
                                  [100] Uranus Ecliptic Lat
                                  [101] Uranus Ecliptic Lon
                                  [102] Neptune Azimuth
                                  [103] Neptune Altitude
                                  [104] Neptune H.Ecliptic Lat
                                  [105] Neptune H.Ecliptic Lon
                                  [106] Neptune Ecliptic Lat
                                  [107] Neptune Ecliptic Lon
                                  [108] AD Multiplexer 0
                                  [109] Map Slot
                                  [110] SD Card Inserted
                                  [111] SD Card Mounted
                                  [112] Port Con 0
      matrix --xyz-mode-x         Specify function comparitor mode. Used with -s and -f.
      matrix --xyz-mode-y         Specify function comparitor mode. Used with -s and -f.
      matrix --xyz-mode-z         Specify function comparitor mode. Used with -s and -f.
                                  [0] User Value. A value that is set by the user.
                                  [1] System Value. A value that is set by the system. Allows primary comparitors to be compared to other primary comparitors.
                                  In mode 0, user defined value is compared to a primary comparitor.
                                  In mode 1, primary comparitors are compared to each other. 
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

      example set matrix logic 0 function 0 to detect sdcard mounted:
      matrix -s 0 -f 0 -p 33 -fn 85 -fx 1 -fo 1 --pwm0 1000000 --pwm1 15000 --computer-assist 1
      matrix -s 0 --omode 0

  [ Mapping ]

      map --new      Clears all mapping in memory.
      map --save
      map --load
      map --delete
      map -s n       Specify map slot n.
      map -m n       Specify slot -s mode. (0 : map min to max) (1 : center map x0) (2 : center map x1)
      map -c0 n      Configuration map slot -s value to map. See available map values.
                     [0] Digital
                     [1] YawGPATT
                     [2] RollGPATT
                     [3] PitchGPATT
                     [4] Gyro0AccX
                     [5] Gyro0AccY
                     [6] Gyro0AccZ
                     [7] Gyro0AngX
                     [8] Gyro0AngY
                     [9] Gyro0AngZ
                     [10] Gyro0MagX
                     [11] Gyro0MagY
                     [12] Gyro0MagZ
                     [13] Gyro0GyroX
                     [14] Gyro0GyroY
                     [15] Gyro0GyroZ
                     [16] ADMPlex0_0
                     [17] ADMPlex0_1
                     [18] ADMPlex0_2
                     [19] ADMPlex0_3
                     [20] ADMPlex0_4
                     [21] ADMPlex0_5
                     [22] ADMPlex0_6
                     [23] ADMPlex0_7
                     [24] ADMPlex0_8
                     [25] ADMPlex0_9
                     [26] ADMPlex0_10
                     [27] ADMPlex0_11
                     [28] ADMPlex0_12
                     [29] ADMPlex0_13
                     [30] ADMPlex0_14
                     [31] ADMPlex0_15
      map -c1 n      Configuration map slot -s. (mode 0 : in_min)  (mode 1 : approximate center value)
      map -c2 n      Configuration map slot -s. (mode 0 : in_max)  (mode 1 : Neg_range : 0 to approximate center value)
      map -c3 n      Configuration map slot -s. (mode 0 : out_min) (mode 1 : Pos_range : ADC max - neg range)
      map -c4 n      Configuration map slot -s. (mode 0 : out_max) (mode 1 : out_max)
      map -c5 n      Configuration map slot -s. (mode 1 only : DEADZONE : expected flutuation at center)

      example map analog stick axis x0 on admplex0 channel 0 into map slot 0:
      map -s 0 -m 1 -c0 16 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
      matrix -s 0 --map-slot 0
      Optional: matrix -s 0 --omode 1

      example map analog stick axis x1 on admplex0 channel 1 into map slot 1:
      map -s 1 -m 2 -c0 17 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
      matrix -s 1 --map-slot 1
      Optional: matrix -s 1 --omode 1

  [ INS ]

      ins -m n              Set INS mode n. (0 : Off) (1 : Dynamic, set by gps every 100ms) (2 : Fixed, remains on after conditions met).
      ins -gyro n           INS uses gyro for attitude. (0 : gyro heading) (1 : gps heading).
      ins -p n              Set INS minmum required gps precision factor to initialize (higher requires less gps precision).
      ins -s n              Set INS minmum required speed to initialize (lower requires less speed).
      ins -r n              Set INS minmum required range difference difference between gps heading and gyro heading (higher allows more difference).
      ins --reset-forced    Reset INS remains on after conditions met.

  [ Satio ]

      [ Time ]
      satio --utc-offset n             Set +-seconds offset time.
      satio --auto-datetime-on         Enable set datetime automatically  (--auto-datetime-on overrides any datetime -set).
      satio --auto-datetime-off        Disable set datetime automatically (ensure --auto-datetime-off before using -set time).
      satio --set-datetime --year n --month n --mday n --hour n --minute n --second n  (expects UTC +- 0).

      [ Location ]
      satio --coord-value-mode-gps             Use GPS coordinates.
      satio --coord-value-mode-user            User user defined coordinates.
      satio --set-coord -lat n -lon n          Set degrees latitude and longitude.

      [ Speed ]
      satio --speed-value-mode-gps             Use GPS speed.
      satio --speed-value-mode-user            User user defined speed.
      satio --set-speed n                      Set speed in meters per second.

      [ Altitude ]
      satio --altitude-value-mode-gps          Use GPS altitude values.
      satio --altitude-value-mode-user         User user defined altitude.
      satio --set-altitude n                   Set altitude in meters.
      
      [ Ground Heading ]
      satio --ground-heading-value-mode-gps    Use GPS ground heading values.
      satio --ground-heading-value-mode-user   User user defined ground heading.
      satio --set-ground-heading               Set ground heading in degrees.

  [ Gyro ]

      gyro --calacc        Callibrate the accelerometer.
      gyro --calmag-start  Begin calibrating the magnetometer.
      gyro --calmag-end    End calibrating the magnetometer.

  [ SDCard ]

      sdcard --mount
      sdcard --unmount

  [ PERFORMANCE ]

      powercfg --ultimate-performance  Sets power configuration to ultimate performance mode.
      powercfg --power-saving          Sets power configuration to low power consumption mode.
      settick -e                       Enable tick delay for specified args.
      settick -d                       Disable tick delay for specified args (enables millisecond delay).
      settick --admplex0               Takes arguments -e, -d.
      settick --gyro0                  Takes arguments -e, -d.
      settick --universe               Takes arguments -e, -d.
      settick --gps                    Takes arguments -e, -d.
      settick --matrix                 Takes arguments -e, -d.
      settick --pcinput                Takes arguments -e, -d.
      settick --log                    Takes arguments -e, -d.

      example: settick -e --admplex0 --gyro0 --gps

      setdelay --admplex0               Specify ticks/milliseconds delay.
      setdelay --gyro0                  Specify ticks/milliseconds delay.
      setdelay --universe               Specify ticks/milliseconds delay.
      setdelay --gps                    Specify ticks/milliseconds delay.
      setdelay --matrix                 Specify ticks/milliseconds delay.
      setdelay --pcinput                Specify ticks/milliseconds delay.
      setdelay --log                    Specify ticks/milliseconds delay.

      example: setdelay --admplex0 1 --gyro0 1 --gps 1

  [ StarNav ]

      starnav RA_HOUR RA_MIN RA_SEC DEC_D DEC_M DEC_S
    
      example: starnav 6 45 8.9 -16 42 58.0

  [ Stat ]

      stat -e     Enable print.
      stat -d     Disable print.
      stat -t     Enables/disables serial print stats and counters. Takes arguments -e, -d.
      stat --partition-table      Print partition table.
      stat --memory-ram           Print ram information.
      stat --sdcard               Print SDCard information.
      stat --system               Print system configuration.
      stat --matrix n             Print matrix switch n configuration.
      stat --matrix -A            Print configuration of all matrix switches.
      stat --map n                Print map slot n data.
      stat --map -A               Print all map slot data.
      stat --sentence -A          Print all sentences. Takes arguments -e, -d.
      stat --sentence --satio     Takes arguments -e, -d.
      stat --sentence --ins       Takes arguments -e, -d.
      stat --sentence --gngga     Takes arguments -e, -d.
      stat --sentence --gnrmc     Takes arguments -e, -d.
      stat --sentence --gpatt     Takes arguments -e, -d.
      stat --sentence --matrix    Takes arguments -e, -d.
      stat --sentence --pcinput   Takes arguments -e, -d.
      stat --sentence --admplex0  Takes arguments -e, -d.
      stat --sentence --gyro0     Takes arguments -e, -d.
      stat --sentence --sun       Takes arguments -e, -d.
      stat --sentence --luna      Takes arguments -e, -d.
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
    printf("-----------------------------------------------------\n");
    printf("[System] \n");
    printf("[serial_command] %d\n", systemData.serial_command);
    printf("[output_satio_all] %d\n", systemData.output_satio_all);
    printf("[output_satio_enabled] %d\n", systemData.output_satio_enabled);
    printf("[output_ins_enabled] %d\n", systemData.output_ins_enabled);
    printf("[output_gngga_enabled] %d\n", systemData.output_gngga_enabled);
    printf("[output_gnrmc_enabled] %d\n", systemData.output_gnrmc_enabled);
    printf("[output_gpatt_enabled] %d\n", systemData.output_gpatt_enabled);
    printf("[output_matrix_enabled] %d\n", systemData.output_matrix_enabled);
    printf("[output_input_portcontroller] %d\n", systemData.output_input_portcontroller);
    printf("[output_admplex0_enabled] %d\n", systemData.output_admplex0_enabled);
    printf("[output_gyro_0_enabled] %d\n", systemData.output_gyro_0_enabled);
    printf("[output_sun_enabled] %d\n", systemData.output_sun_enabled);
    printf("[output_luna_enabled] %d\n", systemData.output_luna_enabled);
    printf("[output_mercury_enabled] %d\n", systemData.output_mercury_enabled);
    printf("[output_venus_enabled] %d\n", systemData.output_venus_enabled);
    printf("[output_mars_enabled] %d\n", systemData.output_mars_enabled);
    printf("[output_jupiter_enabled] %d\n", systemData.output_jupiter_enabled);
    printf("[output_saturn_enabled] %d\n", systemData.output_saturn_enabled);
    printf("[output_uranus_enabled] %d\n", systemData.output_uranus_enabled);
    printf("[output_neptune_enabled] %d\n", systemData.output_neptune_enabled);
    printf("[output_meteors_enabled] %d\n", systemData.output_meteors_enabled);
    printf("-----------------------------------------------------\n");
}

void PrintSatIOData(void) {
    printf("-----------------------------------------------------\n");
    printf("[SatIO] \n");
    printf("[utc_second_offset] %lld\n", satioData.utc_second_offset);
    printf("[utc_auto_offset_flag] %d\n", satioData.utc_auto_offset_flag);
    printf("[set_time_automatically] %d\n", satioData.set_time_automatically);
    printf("-----------------------------------------------------\n");
}

void PrintMappingNData(int map_slot) {
  if (map_slot>=0 && map_slot<MAX_MAP_SLOTS) {
    printf("-----------------------------------------------------\n");
    printf("[slot] %d\n", map_slot);
    printf("[map_mode] %d\n", mappingData.map_mode[0][map_slot]);
    printf("[map slot idx] %d\n", matrixData.index_mapped_value[0][map_slot]);
    printf("[map config 0] %ld\n", mappingData.mapping_config[0][map_slot][0]);
    printf("[map config 1] %ld\n", mappingData.mapping_config[0][map_slot][1]);
    printf("[map config 2] %ld\n", mappingData.mapping_config[0][map_slot][2]);
    printf("[map config 3] %ld\n", mappingData.mapping_config[0][map_slot][3]);
    printf("[map config 4] %ld\n", mappingData.mapping_config[0][map_slot][4]);
    printf("[map config 5] %ld\n", mappingData.mapping_config[0][map_slot][5]);
    printf("-----------------------------------------------------\n");
  }
}

void PrintMappingData(void) {
  for (int Mi=0; Mi<MAX_MAP_SLOTS; Mi++) {PrintMappingNData(Mi);}
}

void PrintMatrixNData(int matrix_index) {
  if (matrix_index>=0 && matrix_index<MAX_MATRIX_SWITCHES) {
    printf("-----------------------------------------------------\n");
    printf("[matrix switch] %d\n", matrix_index);
    printf("[computer assist] %d\n", matrixData.computer_assist[0][matrix_index]);
    printf("[output mode] %d\n", matrixData.output_mode[0][matrix_index]);
    printf("[flux] %lu\n", matrixData.flux_value[0][matrix_index]);
    printf("[pwm] 0: %lu 1: %lu\n",
      matrixData.output_pwm[0][matrix_index][0],
      matrixData.output_pwm[0][matrix_index][1]);
    printf("[port] %d\n", matrixData.matrix_port_map[0][matrix_index]);
    printf("[active] %d\n", matrixData.switch_intention[0][matrix_index]);
    printf("-----------------------------------------------------\n");
    for (int Fi=0; Fi<MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
      //                                         matrixData.matrix_function[0][switch_idx][func_idx]
      printf("[function %d name] %s\n",
        Fi, matrixData.matrix_function_names[ matrixData.matrix_function[0][matrix_index][Fi] ]);
      printf("[function %d matrix_function_operator_name] %d\n",
        Fi, matrixData.matrix_switch_operator_index[0][matrix_index][Fi]);
      printf("[function %d inverted] %d\n",
        Fi, matrixData.matrix_switch_inverted_logic[0][matrix_index][Fi]);
      printf("[function %d x] %f\n",
        Fi, matrixData.matrix_function_xyz[0][matrix_index][Fi][INDEX_MATRIX_FUNTION_X]);
      printf("[function %d y] %f\n",
        Fi, matrixData.matrix_function_xyz[0][matrix_index][Fi][INDEX_MATRIX_FUNTION_Y]);
      printf("[function %d z] %f\n",
        Fi, matrixData.matrix_function_xyz[0][matrix_index][Fi][INDEX_MATRIX_FUNTION_Z]);
      printf("-----------------------------------------------------\n");
    }
  }
}

void PrintMatrixData(void) {
    for (int Mi=0; Mi<MAX_MATRIX_SWITCHES; Mi++) {PrintMatrixNData(Mi);}
}

// void PrintSDCardInformation() {
//   Serial.println("sdcard inserted:    " + String(sdcardData.sdcard_inserted));
//   Serial.println("sdcard mounted:     " + String(sdcardData.sdcard_mounted));
//   Serial.println("sdcard mount point: " + String(sdcardData.sdcard_mountpoint));
//   Serial.println("sdcard type:        " +
//     String(sdcardData.sdcard_type_names[sdcardData.sdcard_type]) +
//     " (type: " + String(sdcardData.sdcard_type) + ").");
//   Serial.println("sdcard card size:   " + String(sdcardData.sdcard_card_size));
//   Serial.println("sdcard total bytes: " + String(sdcardData.sdcard_total_bytes));
//   Serial.println("sdcard used bytes:  " + String(sdcardData.sdcard_used_bytes));
//   Serial.println("sdcard sector size: " + String(sdcardData.sdcard_sector_size));
// }

void setAllSentenceOutput(bool enable) {
  systemData.output_satio_all = enable;
  systemData.output_satio_enabled=enable;
  systemData.output_gngga_enabled=enable;
  systemData.output_gnrmc_enabled=enable;
  systemData.output_gpatt_enabled=enable;
  systemData.output_ins_enabled=enable;
  systemData.output_matrix_enabled=enable;
  systemData.output_input_portcontroller=enable;
  systemData.output_admplex0_enabled=enable;
  systemData.output_gyro_0_enabled=enable;
  systemData.output_sun_enabled=enable;
  systemData.output_mercury_enabled=enable;
  systemData.output_venus_enabled=enable;
  systemData.output_earth_enabled=enable;
  systemData.output_luna_enabled=enable;
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
  printf("func_n %d\n", func_n);
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 &&
      func_idx<MAX_MATRIX_SWITCH_FUNCTIONS &&
      func_n>=0 &&
      func_n<MAX_MATRIX_FUNCTION_NAMES) {
    matrixData.matrix_function[0][switch_idx][func_idx]=func_n;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
  printf("matrix_function %d\n", matrixData.matrix_function[0][switch_idx][func_idx]);
}

void setMatrixFunctionXComparitorMode(int switch_idx, int func_idx, int comparitor_mode) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES &&
      func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS)
  {
    if (comparitor_mode>=0 && comparitor_mode<MAX_MATRIX_FUNCTION_XYZ_MODES) {
      matrixData.matrix_function_mode_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_X]=comparitor_mode;
      matrixData.matrix_switch_write_required[0][switch_idx]=true;
    }
  }
}

void setMatrixFunctionYComparitorMode(int switch_idx, int func_idx, int comparitor_mode) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES &&
      func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS)
  {
    if (comparitor_mode>=0 && comparitor_mode<MAX_MATRIX_FUNCTION_XYZ_MODES) {
      matrixData.matrix_function_mode_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_Y]=comparitor_mode;
      matrixData.matrix_switch_write_required[0][switch_idx]=true;
    }
  }
}

void setMatrixFunctionZComparitorMode(int switch_idx, int func_idx, int comparitor_mode) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES &&
      func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS)
  {
    if (comparitor_mode>=0 && comparitor_mode<MAX_MATRIX_FUNCTION_XYZ_MODES) {
      matrixData.matrix_function_mode_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_Z]=comparitor_mode;
      matrixData.matrix_switch_write_required[0][switch_idx]=true;
    }
  }
}

void setMatrixX(int switch_idx, int func_idx, double func_x) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES &&
      func_idx>=0 && func_idx<MAX_MATRIX_SWITCH_FUNCTIONS &&
      func_x>=-DBL_MAX &&
      func_x<DBL_MAX) {
    matrixData.matrix_function_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_X]=func_x;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixY(int switch_idx, int func_idx, double func_y) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 &&
      func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_y>=-DBL_MAX &&
      func_y<DBL_MAX) {
    matrixData.matrix_function_xyz[0][switch_idx][func_idx][INDEX_MATRIX_FUNTION_Y]=func_y;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
}

void setMatrixZ(int switch_idx, int func_idx, double func_z) {
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES && func_idx>=0 &&
      func_idx<MAX_MATRIX_SWITCH_FUNCTIONS && func_z>=-DBL_MAX && func_z<DBL_MAX) {
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
  printf("[setOverrideOutputValue] attempting to override switch %d\n", switch_idx);
  if (switch_idx < 0) {printf("[setOverrideOutputValue] switch_idx must be >= 0.\n"); return;}
  if (switch_idx >= MAX_MATRIX_SWITCHES) {printf("[setOverrideOutputValue] switch_idx must be < %d.\n", MAX_MATRIX_SWITCHES); return;}
  if (override_value < 0) {printf("[setOverrideOutputValue] override_value must be >= 0.\n"); return;}
  if (override_value > LONG_MAX) {printf("[setOverrideOutputValue] override_value must be <= %ld.\n", LONG_MAX); return;}

  matrixData.computer_assist[0][switch_idx]=false;
  matrixData.override_output_value[0][switch_idx]=override_value;
  long i_retry=0;
  while (matrixData.computer_assist[0][switch_idx]!=false) {
    matrixData.computer_assist[0][switch_idx]=false;
    i_retry+=1;
    if (i_retry==MAX_MATRIX_OVERRIDE_TIME)
      {printf("WARNING! Could not override computer_assist!\n"); break;}
    delayMicroseconds(1);
  }
  i_retry=0;
  while (matrixData.override_output_value[0][switch_idx]!=override_value) {
    matrixData.override_output_value[0][switch_idx]=override_value;
    i_retry+=1;
    if (i_retry==MAX_MATRIX_OVERRIDE_TIME)
      {printf("WARNING! Could not override override_output_value!\n"); break;}
    delayMicroseconds(1);
  }
  matrixData.matrix_switch_write_required[0][switch_idx]=true;
  printf("[setOverrideOutputValue] computer_assist %d\n", matrixData.computer_assist[0][switch_idx]);
  printf("[setOverrideOutputValue] override_output_value %ld\n", matrixData.override_output_value[0][switch_idx]);
  printf("[setOverrideOutputValue] matrix_switch_write_required %d\n", matrixData.matrix_switch_write_required[0][switch_idx]);
}

char *cmd_proc_xyzptr;

void setComputerAssist(int switch_idx, bool computer_assist) {
  debug("[setComputerAssist] switch_idx:" + String(switch_idx) + "  computer_assist: " + String(computer_assist));
  debug("[setComputerAssist] current computer_assist: " + String(matrixData.computer_assist[0][switch_idx]));
  if (switch_idx>=0 && switch_idx<MAX_MATRIX_SWITCHES) {
    matrixData.computer_assist[0][switch_idx]=computer_assist;
    matrixData.matrix_switch_write_required[0][switch_idx]=true;
  }
  else {debug("[setComputerAssist] failed to change computer assist!");}
  debug("[setComputerAssist] computer_assist: " + String(matrixData.computer_assist[0][switch_idx]));
}

void setINSMode(int ins_mode) {
  if (ins_mode>=0 && ins_mode <MAX_INS_MODE)
    {insData.INS_MODE=ins_mode;}
}

void setINSUseGyroHeading(int use_gyro_heading) {
  if (use_gyro_heading>=0 && use_gyro_heading<=1)
    {insData.INS_USE_GYRO_HEADING=use_gyro_heading;}
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
  // Serial.println("[setMapSlot] matrix_switch:" + String(matrix_switch) + "  map_slot:" + String(map_slot));
  if (matrix_switch>=0 && matrix_switch<MAX_MAP_SLOTS &&
      map_slot>=0 && map_slot<=1) {
    matrixData.index_mapped_value[0][matrix_switch]=map_slot;
    matrixData.matrix_switch_write_required[0][map_slot]=true;
  }
}

void saveMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_FILE_SLOTS) {
    satioFileData.i_current_matrix_file_path=matrix_file_slot;
    sdcardFlagData.save_matrix=true;
  }
}

void loadMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_FILE_SLOTS) {
    satioFileData.i_current_matrix_file_path=matrix_file_slot;
    sdcardFlagData.load_matrix=true;
  }
}

void deleteMatrix(int matrix_file_slot) {
  if (matrix_file_slot>=0 && matrix_file_slot<MAX_MATRIX_FILE_SLOTS) {
    satioFileData.i_current_matrix_file_path=matrix_file_slot;
    sdcardFlagData.delete_matrix=true;
  }
}

void datetimeSetRTC(uint16_t year, uint8_t month, uint8_t mday,
                            uint8_t hour, uint8_t min, uint8_t sec) {
  /*
  satio --utc-offset n       Set +-seconds offset time.
  satio --auto-datetime-on   Enable set datetime automatically
  satio --auto-datetime-off  Disable set datetime automatically (-set requires auto off prior to calling set)
  satio --set-datetime --year n --month n --mday n --hour n --minute n --second n  (must be UTC except if utc offset 0)
  satio --set-datetime --year 25 --month 10 --mday 16 --hour 5 --minute 0 --second 0
  */
  if (year>=0  && year <UINT16_MAX &&
      month>=0 && month<UINT8_MAX  &&
      mday>=0  && mday <UINT8_MAX  &&
      hour>=0  && hour <UINT8_MAX  &&
      min>=0   && min  <UINT8_MAX  &&
      sec>=0   && sec  <UINT8_MAX) {
    satioData.tmp_year_int=year;
    satioData.tmp_month_int=month;
    satioData.tmp_day_int=mday;
    satioData.tmp_hour_int=hour;
    satioData.tmp_minute_int=min;
    satioData.tmp_second_int=sec;
    satioData.set_rtc_datetime_flag=true;
  }
}

void datetimeSetDTAuto(bool set_dt_auto) {
  if (set_dt_auto==true) {satioData.sync_rtc_immediately_flag=true; satioData.set_time_automatically=true;}
  else {satioData.set_time_automatically=false;}
}

void setCoordinatesDegrees(double latitude, double longitude) {
  if (latitude>=-90 && latitude<=90 && longitude>=-180 && longitude<=180) {
    satioData.user_degrees_latitude=latitude;
    satioData.user_degrees_longitude=longitude;
  }
}

void setAltitude(double altitude) {
  if (altitude>=DBL_MIN && altitude<DBL_MAX && altitude!=NAN) {
    satioData.user_altitude=altitude;
  }
}

void setSpeed(double speed) {
  if (speed>=0 && speed<DBL_MAX && speed!=NAN) {
    satioData.user_speed=speed;
  }
}

void setGroundHeading(double heading) {
  if (heading>=-360 && heading<=360 && heading!=NAN) {
    satioData.user_ground_heading=heading;
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
  simple_argparser_init_from_buffer(&plainparser, serial0Data.BUFFER, 1);
  if ((str_is_int8(plainparser.tokens[0])==true) &&
      (str_is_int8(plainparser.tokens[1])==true) &&
      (str_is_float(plainparser.tokens[2])==true) &&
      (str_is_int8(plainparser.tokens[3])==true) &&
      (str_is_int8(plainparser.tokens[4])==true) &&
      (str_is_float(plainparser.tokens[5])==true)
    )
  {
    printf("attempting to identify object..\n");
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
    printf("---------------------------------------------\n");
    printf("Table Index:   %d\n", siderealObjectData.object_table_i);
    printf("Table:         %s\n", siderealObjectData.object_table_name);
    printf("Number:        %d\n", siderealObjectData.object_number);
    printf("Name:          %s\n", siderealObjectData.object_name);
    printf("Type:          %s\n", siderealObjectData.object_type);
    printf("Constellation: %s\n", siderealObjectData.object_con);
    printf("Distance:      %f\n", siderealObjectData.object_dist);
    printf("Azimuth:       %f\n", siderealObjectData.object_az);
    printf("Altitude:      %f\n", siderealObjectData.object_alt);
    printf("Rise:          %f\n", siderealObjectData.object_r);
    printf("Set:           %f\n", siderealObjectData.object_s);
    printf("---------------------------------------------\n");
  }
  else {printf("identify object: bad input data\n");}
}

void unmountSDCard() {
  sdcardFlagData.no_delay_flag=true;
  sdcardFlagData.unmount_sdcard_flag=true;
}

void mountSDCard() {
  sdcardFlagData.no_delay_flag=true;
  sdcardFlagData.mount_sdcard_flag=true;
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

// void printArgParse() {
//   Serial.println("-------------------------------------------");
//   Serial.print("[debug] First command: ");
//   if (pos_count > 0) {Serial.println(pos[0]);}
//   else {Serial.println("none");}
//   Serial.print("[debug] Positionals (");
//   Serial.print(pos_count);
//   Serial.print("): ");
//   for (size_t j = 0; j < pos_count; ++j)
//     {Serial.print(pos[j]); if (j < pos_count - 1) Serial.print(" ");}
//   Serial.println();
//   Serial.println("----");
//   Serial.print("[debug] Flag count: ");
//   Serial.println(parser.flag_count);
//   Serial.print("[debug] Flags: ");
//   for (size_t k = 0; k < parser.flag_count; ++k)
//     {Serial.print(parser.flags[k]); const char* val = parser.values[k];
//       if (val[0] != '\0') {Serial.print("=\""); Serial.print(val); Serial.print("\"");}
//       if (k < parser.flag_count - 1) Serial.print(" ");
//   }
//   Serial.println();
//   Serial.println("-------------------------------------------");
// }

void printArgParse() {
  printf("-------------------------------------------");
  printf("[debug] First command: ");
  if (pos_count > 0) {printf("%s\n", pos[0]);}
  else {printf("none\n");}
  printf("[debug] Positionals (");
  printf("%zu", pos_count);
  printf("): ");
  for (size_t j = 0; j < pos_count; ++j)
    {printf("%s", pos[j]); if (j < pos_count - 1) printf(" ");}
  printf("\n");
  printf("----\n");
  printf("[debug] Flag count: ");
  printf("%zu\n", parser.flag_count);
  printf("[debug] Flags: ");
  for (size_t k = 0; k < parser.flag_count; ++k)
    {printf("%s", parser.flags[k]); const char* val = parser.values[k];
      if (val[0] != '\0') {printf("=\"%s\"", val);}
      if (k < parser.flag_count - 1) printf(" ");
  }
  printf("\n");
  printf("-------------------------------------------\n");
}

// ---------------------------------------------------------------------------------------------------------------
/*
  Serial RXD : Command Process.
*/
// ---------------------------------------------------------------------------------------------------------------

void CmdProcess() {
  if (strlen(serial0Data.BUFFER)>=2) {
    // Debug Serial Buffer.
    printf("[CmdProcess] Received data: %s\n", serial0Data.BUFFER);
    // return;
    // Initialize argparse.
    argparser_reset(&parser);
    if (!argparser_init_from_buffer(&parser, serial0Data.BUFFER))
      {printf("[cmd] Failed to initialize parser from buffer\n"); return;}
    pos_count=0; pos={}; pos = argparser_get_positionals(&parser, &pos_count);
    // Verbosity.
    verbose=false; verbose_1=false;
    verbose = argparser_get_bool(&parser, "v") || argparser_get_bool(&parser, "verbose");
    verbose_1 = argparser_get_bool(&parser, "vv") || argparser_get_bool(&parser, "verbose1");
    if (verbose_1) {verbose=true;}
    if (verbose==false) {verbose_1=false;}
    printf("[cmd] verbose: %d\n", verbose);
    printf("[cmd] verbose1: %d\n", verbose_1);
    // Enable/Disable
    enable=false;
    if (argparser_has_flag(&parser, "disable") || argparser_has_flag(&parser, "d")) {enable=false;}
    else if (argparser_has_flag(&parser, "enable") || argparser_has_flag(&parser, "e")) {enable=true;}
    // Debug Arg Parse.
    printArgParse();
    // Check if command provided.
    if (pos_count==0) {printf("[cmd] No command provided.\n"); return;}
    // Commands. help -v
    if (strcmp(pos[0], "help")==0 || strcmp(pos[0], "h")==0)
      {printf("Usage: [buffer with] [--flag value] [-f value] [positional...]\n");
      if (verbose) {PrintHelp();}
    }
    else if (strcmp(pos[0], "stat")==0) {

      if (argparser_has_flag(&parser, "t")) {
        if (enable) {systemData.output_stat=enable; systemData.output_stat_v=verbose; systemData.output_stat_vv=verbose_1;}
        else {systemData.output_stat=false; systemData.output_stat_v=false; systemData.output_stat_vv=false;}
      }
      // if (argparser_has_flag(&parser, "partition-table")) {print_partition_table();}

      // if (argparser_has_flag(&parser, "memory-ram")) {print_ram_info();}

      // if (argparser_has_flag(&parser, "sdcard")) {PrintSDCardInformation();}

      if (strcmp(serial0Data.BUFFER, "stat --system")==0) {PrintSystemData();}

      if (strncmp(serial0Data.BUFFER, "stat --matrix ", strlen("stat --matrix "))==0) {
        if (argparser_has_flag(&parser, "A")) {PrintMatrixData();}
        else {PrintMatrixNData(argparser_get_int8(&parser, "matrix", -1));}
      }

      else if (strncmp(serial0Data.BUFFER, "stat -map ", strlen("stat -map "))==0) {
        if (argparser_has_flag(&parser, "A")) {PrintMappingData();}
        else {PrintMappingNData(argparser_get_int8(&parser, "map", -1));}
      }

      else if (argparser_has_flag(&parser, "sentence")) {
        if (argparser_has_flag(&parser, "A")) {systemData.output_satio_all=enable; setAllSentenceOutput(enable);}
        if (argparser_has_flag(&parser, "satio")) {systemData.output_satio_enabled=enable; printf("setting satio output enabled: %d\n", systemData.output_satio_enabled);}
        if (argparser_has_flag(&parser, "gngga")) {systemData.output_gngga_enabled=enable; printf("setting gngga output enabled: %d\n", systemData.output_gngga_enabled);}
        if (argparser_has_flag(&parser, "gnrmc")) {systemData.output_gnrmc_enabled=enable; printf("setting gnrmc output enabled: %d\n", systemData.output_gnrmc_enabled);}
        if (argparser_has_flag(&parser, "gpatt")) {systemData.output_gpatt_enabled=enable; printf("setting gpatt output enabled: %d\n", systemData.output_gpatt_enabled);}
        if (argparser_has_flag(&parser, "ins")) {systemData.output_ins_enabled=enable; printf("setting ins output enabled: %d\n", systemData.output_ins_enabled);}
        if (argparser_has_flag(&parser, "matrix")) {systemData.output_matrix_enabled=enable; printf("setting matrix output enabled: %d\n", systemData.output_matrix_enabled);}
        if (argparser_has_flag(&parser, "pcinput")) {systemData.output_input_portcontroller=enable; printf("setting input_portcontroller output enabled: %d\n", systemData.output_input_portcontroller);}
        if (argparser_has_flag(&parser, "admplex0")) {systemData.output_admplex0_enabled=enable; printf("setting admplex0 output enabled: %d\n", systemData.output_admplex0_enabled);}
        if (argparser_has_flag(&parser, "gyro0")) {systemData.output_gyro_0_enabled=enable; printf("setting gyro_0 output enabled: %d\n", systemData.output_gyro_0_enabled);}
        if (argparser_has_flag(&parser, "sun")) {systemData.output_sun_enabled=enable; printf("setting sun output enabled: %d\n", systemData.output_sun_enabled);}
        if (argparser_has_flag(&parser, "earth")) {systemData.output_sun_enabled=enable; printf("setting earth output enabled: %d\n", systemData.output_earth_enabled);}
        if (argparser_has_flag(&parser, "luna")) {systemData.output_luna_enabled=enable; printf("setting luna output enabled: %d\n", systemData.output_luna_enabled);}
        if (argparser_has_flag(&parser, "mercury")) {systemData.output_mercury_enabled=enable; printf("setting mercury output enabled: %d\n", systemData.output_mercury_enabled);}
        if (argparser_has_flag(&parser, "venus")) {systemData.output_venus_enabled=enable; printf("setting venus output enabled: %d\n", systemData.output_venus_enabled);}
        if (argparser_has_flag(&parser, "mars")) {systemData.output_mars_enabled=enable; printf("setting mars output enabled: %d\n", systemData.output_mars_enabled);}
        if (argparser_has_flag(&parser, "jupiter")) {systemData.output_jupiter_enabled=enable; printf("setting jupiter output enabled: %d\n", systemData.output_jupiter_enabled);}
        if (argparser_has_flag(&parser, "saturn")) {systemData.output_saturn_enabled=enable; printf("setting saturn output enabled: %d\n", systemData.output_saturn_enabled);}
        if (argparser_has_flag(&parser, "uranus")) {systemData.output_uranus_enabled=enable; printf("setting uranus output enabled: %d\n", systemData.output_uranus_enabled);}
        if (argparser_has_flag(&parser, "neptune")) {systemData.output_neptune_enabled=enable; printf("setting neptune output enabled: %d\n", systemData.output_neptune_enabled);}
        if (argparser_has_flag(&parser, "meteors")) {systemData.output_meteors_enabled=enable; printf("setting meteors output enabled: %d\n", systemData.output_meteors_enabled);}
        if (argparser_has_flag(&parser, "xmatrix")) {systemData.output_config_matrix_enabled=enable; printf("setting config_matrix output enabled: %d\n", systemData.output_config_matrix_enabled);}
        if (argparser_has_flag(&parser, "xmap")) {systemData.output_config_mapping_enabled=enable; printf("setting config_mapping output enabled: %d\n", systemData.output_config_mapping_enabled);}
      }
    }
    
    // else if (strcmp(pos[0], "ls")==0) {
    //   const char* path_str = argparser_get_path(&parser, "/");
    //   Serial.println("[cmd] Path: " + String(path_str));
    //   int maxlevels=0;
    //   if (argparser_has_flag(&parser, "R")) {maxlevels=-1;}
    //   else {maxlevels = argparser_get_int8(&parser, "maxlevels", 1);}
    //   memset(sdmmcArgData.buffer, 0, sizeof(sdmmcArgData.buffer));
    //   strcpy(sdmmcArgData.buffer, path_str);
    //   sdmmcArgData.maxlevels=maxlevels;
    //   sdcardFlagData.no_delay_flag=true;
    //   sdcardFlagData.list_dir_flag=true;
    // }

    // else if (strcmp(pos[0], "starnav")==0) {star_nav();}
    
    if (systemData.serial_command) {

      if (strcmp(pos[0], "system")==0) {
        if (argparser_has_flag(&parser, "save")) {sdcardFlagData.save_system=true;}
        else if (argparser_has_flag(&parser, "load")) {sdcardFlagData.load_system=true;}
        else if (argparser_has_flag(&parser, "restore-defaults")) {restore_system_defaults();}
        else if (argparser_has_flag(&parser, "log")) {
          printf("setting log enabled: %d\n", enable);
          systemData.logging_enabled=enable;}
      }
      else if (strcmp(pos[0], "map")==0) {
        if (argparser_has_flag(&parser, "new")) {set_all_mapping_default(); {return;}}
        else if (argparser_has_flag(&parser, "save")) {sdcardFlagData.save_mapping=true;}
        else if (argparser_has_flag(&parser, "load")) {sdcardFlagData.load_mapping=true;}
        else if (argparser_has_flag(&parser, "delete")) {sdcardFlagData.delete_mapping=true;}
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
        else if (argparser_has_flag(&parser, "new")) {override_all_computer_assists(); set_all_matrix_default(); return;}
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
            setMatrixInverted(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", 0), argparser_get_int8(&parser, "fi", 0));
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
            setComputerAssist(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "computer-assist", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "omode")) {
            setOutputMode(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "omode", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "map-slot")) {
            setMapSlot(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "map-slot", -1));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "xyz-mode-x")) {
            setMatrixFunctionXComparitorMode(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", -1), argparser_get_int8(&parser, "xyz-mode-x", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "xyz-mode-y")) {
            setMatrixFunctionYComparitorMode(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", -1), argparser_get_int8(&parser, "xyz-mode-y", 0));
          }
          if (argparser_has_flag(&parser, "s") && argparser_has_flag(&parser, "f") && argparser_has_flag(&parser, "xyz-mode-z")) {
            setMatrixFunctionZComparitorMode(argparser_get_int8(&parser, "s", -1), argparser_get_int8(&parser, "f", -1), argparser_get_int8(&parser, "xyz-mode-z", 0));
          }
        }
      }

      else if (strcmp(pos[0], "ins")==0) {
        if (argparser_has_flag(&parser, "m")) {setINSMode(argparser_get_int8(&parser, "m", -1));}
        if (argparser_has_flag(&parser, "gyro")) {setINSUseGyroHeading(argparser_get_int8(&parser, "gyro", -1));}
        if (argparser_has_flag(&parser, "p")) {setINSGPSPrecision(argparser_get_double(&parser, "p", -1));}
        if (argparser_has_flag(&parser, "s")) {setINSMinSpeed(argparser_get_double(&parser, "s", -1));}
        if (argparser_has_flag(&parser, "r")) {setINSHeadingRangeDiff(argparser_get_double(&parser, "r", -1));}
        if (argparser_has_flag(&parser, "reset-forced")) {insData.INS_FORCED_ON_FLAG=false;}
      }

      else if (strcmp(pos[0], "satio")==0) {
        // time
        if (argparser_has_flag(&parser, "utc-offset")) {setUTCSecondOffset(argparser_get_int64(&parser, "utc-offset", 0));}
        if (argparser_has_flag(&parser, "auto-datetime-on")) {datetimeSetDTAuto(true);}
        if (argparser_has_flag(&parser, "auto-datetime-off")) {datetimeSetDTAuto(false);}
        if (argparser_has_flag(&parser, "set-datetime")) {
          datetimeSetRTC(argparser_get_uint16(&parser, "year", -1),
                                 argparser_get_uint8(&parser,  "month", -1),
                                 argparser_get_uint8(&parser,  "mday", -1),
                                 argparser_get_uint8(&parser,  "hour", -1),
                                 argparser_get_uint8(&parser,  "minute", -1),
                                 argparser_get_uint8(&parser,  "second", -1));}
        // location
        if (argparser_has_flag(&parser, "set-coord") && argparser_has_flag(&parser, "lat") && argparser_has_flag(&parser, "lon")) {
          setCoordinatesDegrees(argparser_get_double(&parser, "lat", NAN), argparser_get_double(&parser, "lon", NAN));
        }
        if (argparser_has_flag(&parser, "coord-value-mode-gps")) {satioData.location_value_mode=SATIO_MODE_GPS;}
        if (argparser_has_flag(&parser, "coord-value-mode-user")) {satioData.location_value_mode=SATIO_MODE_USER;}

        // speed
        if (argparser_has_flag(&parser, "set-speed")) {
          setSpeed(argparser_get_double(&parser, "set-speed", NAN));
        }
        if (argparser_has_flag(&parser, "speed-value-mode-gps")) {satioData.speed_value_mode=SATIO_MODE_GPS;}
        if (argparser_has_flag(&parser, "speed-value-mode-user")) {satioData.speed_value_mode=SATIO_MODE_USER;}

        // altitude
        if (argparser_has_flag(&parser, "set-altitude")) {
          setAltitude(argparser_get_double(&parser, "set-altitude", NAN));
        }
        if (argparser_has_flag(&parser, "altitude-value-mode-gps")) {satioData.altitude_value_mode=SATIO_MODE_GPS;}
        if (argparser_has_flag(&parser, "altitude-value-mode-user")) {satioData.altitude_value_mode=SATIO_MODE_USER;}

        // ground heading
        if (argparser_has_flag(&parser, "set-ground-heading")) {
          setGroundHeading(argparser_get_double(&parser, "set-ground-heading", NAN));
        }
        if (argparser_has_flag(&parser, "ground-heading-value-mode-gps")) {satioData.ground_heading_value_mode=SATIO_MODE_GPS;}
        if (argparser_has_flag(&parser, "ground-heading-value-mode-user")) {satioData.ground_heading_value_mode=SATIO_MODE_USER;}
      }

      else if (strcmp(pos[0], "gyro")==0) {
        if (argparser_has_flag(&parser, "calacc")) {WT901CalAcc();}
        if (argparser_has_flag(&parser, "calmag-start")) {WT901CalMagStart();}
        else if (argparser_has_flag(&parser, "calmag-stop")) {WT901CalMagEnd();}
      }

      // else if (strcmp(pos[0], "sdcard")==0) {
      //   if (argparser_has_flag(&parser, "mount")) {mountSDCard();}
      //   else if (argparser_has_flag(&parser, "unmount")) {unmountSDCard();}
      // }

      // else if (strcmp(pos[0], "powercfg")==0) {
      //   // The intention is to further develop powercfg in regards to sleep modes.
      //   // powercfg --ultimate-performance
      //   if (argparser_has_flag(&parser, "ultimate-performance")) {setTasksDelayUltimatePerformance();}
      //   // powercfg --power-saving 
      //   else if (argparser_has_flag(&parser, "power-saving")) {setTasksDelayPowerSaving();}
      // }

      // else if (strcmp(pos[0], "settick")==0) {
      //   // if (argparser_has_flag(&parser, "infocmd"))
      //   //   {setTick(TaskSerialInfoCMD, &TICK_DELAY_TASK_SERIAL_INFOCMD, enable);}

      //   if (argparser_has_flag(&parser, "admplex0"))
      //     {setTick(TaskMultiplexers, &TICK_DELAY_TASK_MULTIPLEXERS, enable);}

      //   if (argparser_has_flag(&parser, "gyro0"))
      //     {setTick(TaskGyro, &TICK_DELAY_TASK_GYRO0, enable);}

      //   if (argparser_has_flag(&parser, "universe"))
      //     {setTick(TaskUniverse, &TICK_DELAY_TASK_UNIVERSE, enable);}

      //   if (argparser_has_flag(&parser, "gps"))
      //     {setTick(TaskGPS, &TICK_DELAY_TASK_GPS, enable);}

      //   if (argparser_has_flag(&parser, "matrix"))
      //     {setTick(TaskSwitches, &TICK_DELAY_TASK_SWITCHES, enable);}

      //   if (argparser_has_flag(&parser, "pcinput"))
      //     {setTick(TaskPortControllerInput, &TICK_DELAY_TASK_PORTCONTROLLER_INPUT, enable);}
        
      //   if (argparser_has_flag(&parser, "log"))
      //     {setTick(TaskLogging, &TICK_DELAY_TASK_LOGGING, enable);}

      //   // if (argparser_has_flag(&parser, "storage"))
      //   //   {setTick(TaskStorage, &TICK_DELAY_TASK_STORAGE, enable);}
      // }
      // else if (strcmp(pos[0], "setdelay")==0) {
      //   // if (argparser_has_flag(&parser, "infocmd"))
      //   //   setDelay(TaskSerialInfoCMD, &DELAY_TASK_SERIAL_INFOCMD, argparser_get_int32(&parser, "infocmd", DELAY_TASK_SERIAL_INFOCMD));

      //   if (argparser_has_flag(&parser, "admplex0"))
      //     {setDelay(TaskMultiplexers, &DELAY_TASK_MULTIPLEXERS, argparser_get_int32(&parser, "admplex0", DELAY_TASK_MULTIPLEXERS));}

      //   if (argparser_has_flag(&parser, "gyro0"))
      //     {setDelay(TaskGyro, &DELAY_TASK_GYRO0, argparser_get_int32(&parser, "gyro0", DELAY_TASK_GYRO0));}

      //   if (argparser_has_flag(&parser, "universe"))
      //     {setDelay(TaskUniverse, &DELAY_TASK_UNIVERSE, argparser_get_int32(&parser, "universe", DELAY_TASK_UNIVERSE));}

      //   if (argparser_has_flag(&parser, "gps"))
      //     {setDelay(TaskGPS, &DELAY_TASK_GPS, argparser_get_int32(&parser, "gps", DELAY_TASK_GPS));}

      //   if (argparser_has_flag(&parser, "matrix"))
      //     {setDelay(TaskSwitches, &DELAY_TASK_SWITCHES, argparser_get_int32(&parser, "matrix", DELAY_TASK_SWITCHES));}

      //   if (argparser_has_flag(&parser, "pcinput"))
      //     {setDelay(TaskPortControllerInput, &DELAY_TASK_PORTCONTROLLER_INPUT, argparser_get_int32(&parser, "pcinput", DELAY_TASK_PORTCONTROLLER_INPUT));}

      //   if (argparser_has_flag(&parser, "log"))
      //     {setDelay(TaskLogging, &DELAY_TASK_LOGGING, argparser_get_int32(&parser, "log", DELAY_TASK_LOGGING));}
          
        // if (argparser_has_flag(&parser, "storage"))
        //   setDelay(TaskStorage, &DELAY_TASK_STORAGE, argparser_get_int32(&parser, "storage", DELAY_TASK_STORAGE));
      // }
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
/*
  Serial TXD : Output.
*/
// ---------------------------------------------------------------------------------------------------------------
long i_output_config_matrix = 0;

void outputSentences(void) {
  if (systemData.interval_breach_1_second_output) {
    systemData.interval_breach_1_second_output=false;
    outputStat();
  }
  // printf("outputSentences: interval_breach_gps: %d\n", systemData.interval_breach_gps);
  if (systemData.interval_breach_gps) {
    systemData.interval_breach_gps=false;
    if (systemData.output_gngga_enabled) {printf("%s\n", gnggaData.outsentence);}
    if (systemData.output_gnrmc_enabled) {printf("%s\n", gnrmcData.outsentence);}
    if (systemData.output_gpatt_enabled) {printf("%s\n", gpattData.outsentence);}
    if (systemData.output_satio_enabled) {
      // printf("outputting SATIO sentence:\n");
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
      
      strcat(serial0Data.BUFFER, String(insData.ins_latitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_longitude, 7).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_altitude).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.ins_heading).c_str());
      
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.INS_INITIALIZATION_FLAG).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.INS_MODE).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.INS_FORCED_ON_FLAG).c_str());
      strcat(serial0Data.BUFFER, ",");

      strcat(serial0Data.BUFFER, String(insData.INS_REQ_GPS_PRECISION).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.INS_REQ_HEADING_RANGE_DIFF).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.INS_REQ_MIN_SPEED).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(insData.INS_USE_GYRO_HEADING).c_str());
      strcat(serial0Data.BUFFER, ",");

      strcat(serial0Data.BUFFER, String(insData.INS_ENABLED).c_str());
      strcat(serial0Data.BUFFER, ",");

      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      printf("%s\n", serial0Data.BUFFER);
    }
  }
  if (systemData.interval_breach_gyro_0) {
    systemData.interval_breach_gyro_0=false;
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
      printf("%s\n", serial0Data.BUFFER);
    }
  }
  if (systemData.interval_breach_track_planets) {
    systemData.interval_breach_track_planets=false;
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
      printf("%s\n", serial0Data.BUFFER);
    }
    if (systemData.output_earth_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$EARTH,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.earth_ecliptic_lat + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.earth_ecliptic_long + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(satioData.altitude + String(",")).c_str()); // distance to earth sea level
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      printf("%s\n", serial0Data.BUFFER);
    }
    if (systemData.output_luna_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$LUNA,");
      strcat(serial0Data.BUFFER, String(siderealPlanetData.luna_ra + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.luna_dec + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.luna_az + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.luna_alt + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.luna_r + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.luna_s + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.luna_p + String(",")).c_str());
      strcat(serial0Data.BUFFER, String(siderealPlanetData.luna_lum + String(",")).c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      printf("%s\n", serial0Data.BUFFER);
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
      printf("%s\n", serial0Data.BUFFER);
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
      printf("%s\n", serial0Data.BUFFER);
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
      printf("%s\n", serial0Data.BUFFER);
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
      printf("%s\n", serial0Data.BUFFER);
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
      printf("%s\n", serial0Data.BUFFER);
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
      printf("%s\n", serial0Data.BUFFER);
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
      printf("%s\n", serial0Data.BUFFER);
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
      printf("%s\n", serial0Data.BUFFER);
    }
  }

  // if (systemData.interval_breach_matrix) {
  // systemData.interval_breach_matrix=false;
  if (systemData.output_matrix_enabled) {
    memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
    strcpy(serial0Data.BUFFER, "$MATRIX,");
    // append matrix switch state data
    for (int i=0; i < MAX_MATRIX_SWITCHES; i++)
      {strcat(serial0Data.BUFFER, String(String(matrixData.switch_intention[0][i])+",").c_str());}
    for (int i=0; i < MAX_MATRIX_SWITCHES; i++)
      {strcat(serial0Data.BUFFER, String(String(matrixData.computer_intention[0][i])+",").c_str());}
    for (int i=0; i < MAX_MATRIX_SWITCHES; i++)
      {strcat(serial0Data.BUFFER, String(String(matrixData.output_value[0][i])+",").c_str());}
    createChecksumSerial0(serial0Data.BUFFER);
    strcat(serial0Data.BUFFER, "*");
    strcat(serial0Data.BUFFER, serial0Data.checksum);
    printf("%s\n", serial0Data.BUFFER);
  }

  if (systemData.output_input_portcontroller) {
    memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
    strcpy(serial0Data.BUFFER, "$PCINPT,");
    // append matrix switch state data
    for (int i=0; i < MAX_MATRIX_SWITCHES; i++)
      {strcat(serial0Data.BUFFER, String(String(matrixData.input_value[0][i])+",").c_str());}
    createChecksumSerial0(serial0Data.BUFFER);
    strcat(serial0Data.BUFFER, "*");
    strcat(serial0Data.BUFFER, serial0Data.checksum);
    printf("%s\n", serial0Data.BUFFER);
  }

  /*
  COMPREHENSIVE MATRIX CONFIGURATION (SENTENCE PER SWITCH)
  */
  if (systemData.output_config_matrix_enabled) {
    // for (int Mi=0; i_output_config_matrix < MAX_MATRIX_SWITCHES; Mi++) { // uncomment to dump all sentences at once
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcpy(serial0Data.BUFFER, "$XMATRIX,");
      strcat(serial0Data.BUFFER, String(String(i_output_config_matrix)+",").c_str());
      for (int Fi=0; Fi < MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
        strcat(serial0Data.BUFFER, String(String(matrixData.matrix_function[0][i_output_config_matrix][Fi])+",").c_str());
      }
      for (int Fi=0; Fi < MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
        strcat(serial0Data.BUFFER, String(String(matrixData.matrix_switch_operator_index[0][i_output_config_matrix][Fi])+",").c_str());
      }
      for (int Fi=0; Fi < MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
        strcat(serial0Data.BUFFER, String(String(matrixData.matrix_switch_inverted_logic[0][i_output_config_matrix][Fi])+",").c_str());
      }
      for (int Fi=0; Fi < MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
        strcat(serial0Data.BUFFER, String(String(matrixData.matrix_function_xyz[0][i_output_config_matrix][Fi][INDEX_MATRIX_FUNTION_X])+",").c_str());
      }
      for (int Fi=0; Fi < MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
        strcat(serial0Data.BUFFER, String(String(matrixData.matrix_function_xyz[0][i_output_config_matrix][Fi][INDEX_MATRIX_FUNTION_Y])+",").c_str());
      }
      for (int Fi=0; Fi < MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
        strcat(serial0Data.BUFFER, String(String(matrixData.matrix_function_xyz[0][i_output_config_matrix][Fi][INDEX_MATRIX_FUNTION_Z])+",").c_str());
      }
      strcat(serial0Data.BUFFER, String(String(matrixData.flux_value[0][i_output_config_matrix])+",").c_str());
      strcat(serial0Data.BUFFER, String(String(matrixData.output_pwm[0][i_output_config_matrix][0])+",").c_str());
      strcat(serial0Data.BUFFER, String(String(matrixData.output_pwm[0][i_output_config_matrix][1])+",").c_str());
      strcat(serial0Data.BUFFER, String(String(matrixData.output_mode[0][i_output_config_matrix])+",").c_str());
      strcat(serial0Data.BUFFER, String(String(matrixData.index_mapped_value[0][i_output_config_matrix])+",").c_str());
      strcat(serial0Data.BUFFER, String(String(matrixData.computer_assist[0][i_output_config_matrix])+",").c_str());
      strcat(serial0Data.BUFFER, String(String(matrixData.matrix_port_map[0][i_output_config_matrix])+",").c_str());
      // strcat(serial0Data.BUFFER, String(String(matrixData.switch_intention[0][i_output_config_matrix])+",").c_str());
      // strcat(serial0Data.BUFFER, String(String(matrixData.computer_intention[0][i_output_config_matrix])+",").c_str());
      // strcat(serial0Data.BUFFER, String(String(matrixData.output_value[0][i_output_config_matrix])+",").c_str());
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      printf("%s\n", serial0Data.BUFFER);
      i_output_config_matrix++;
      if (i_output_config_matrix>=MAX_MATRIX_SWITCHES) {i_output_config_matrix=0;}
    // }
  }
  /*
  COMPREHENSIVE MAPPING CONFIGURATION
  */
  if (systemData.output_config_mapping_enabled) {
    for (int Mi=0; Mi < MAX_MAP_SLOTS; Mi++) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcpy(serial0Data.BUFFER, "$XMAPP,");
      strcat(serial0Data.BUFFER, String(String(Mi)+",").c_str());
      for (int Fi=0; Fi < MAX_MAPPING_PARAMETERS; Fi++) {
        strcat(serial0Data.BUFFER, String(String(mappingData.mapping_config[0][Mi][Fi])+",").c_str());
      }
      strcat(serial0Data.BUFFER, String(String(mappingData.map_mode[0][Mi])+",").c_str());
      strcat(serial0Data.BUFFER, String(String(mappingData.mapped_value[0][Mi])+",").c_str());

      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      printf("%s\n", serial0Data.BUFFER);
    }
  }

  if (systemData.interval_breach_mplex_0) {
    systemData.interval_breach_mplex_0=false;
    if (systemData.output_admplex0_enabled) {
      memset(serial0Data.BUFFER, 0, sizeof(serial0Data.BUFFER));
      strcat(serial0Data.BUFFER, "$MPLEX0,");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[0]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[1]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[2]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[3]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[4]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[5]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[6]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[7]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[8]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[9]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[10]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[11]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[12]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[13]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[14]).c_str());
      strcat(serial0Data.BUFFER, ",");
      strcat(serial0Data.BUFFER, String(ad_mux_0.data[15]).c_str());
      strcat(serial0Data.BUFFER, ",");
      createChecksumSerial0(serial0Data.BUFFER);
      strcat(serial0Data.BUFFER, "*");
      strcat(serial0Data.BUFFER, serial0Data.checksum);
      printf("%s\n", serial0Data.BUFFER);
    }
  }
}

void printArray(signed long arr[], int start, int end) {
    for (int i = start; i < end; i++) {
        printf("%-7ld", arr[i]); // Left-align with 5-character width
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
char counter_chars_0[16][56]={"Loops p/s",
                              "GPS p/s",
                              "INS p/s",
                              "Gyro0 p/s",
                              "ADMplex0 p/s",
                              "CMD p/s",
                              "Universe p/s",
                              "Matrix p/s",
                              "I/O p/s",
                              "PConInput p/s",
                              "LT uS",
                              "LT Max uS",
                              "Satellites",
                              "GPS Precision"};
double counter_digits_0[16]={};

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
    printf("\n");
    printf("[ %llu ]\n", satioData.local_unixtime_uS);
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    // printf("taskGyro", "Unused stack: %u words\n", watermark_task_gyro);
    // printf("taskUniverse", "Unused stack: %u words\n", watermark_task_universe);
    // printf("taskSwitches", "Unused stack: %u words\n", watermark_task_switches);
    // printf("taskGPS", "Unused stack: %u words\n", watermark_task_gps);
    // printf("taskMultiplexers", "Unused stack: %u words\n", watermark_task_multiplexers);
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    for (int i = 0; i < 14; i++) {printf("%-16f", counter_digits_0[i]);}
    printf("\n");
    printf("\n");
    // ----------------------------------------------------------------------------------------------------------------------------
    //                                                                                                           PRINT PRIMARY DATA
    // ----------------------------------------------------------------------------------------------------------------------------
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("                   ");
    for (int i = 0; i < 9; i++) {printf("%-19s", counter_chars_1_row_0[i]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");

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
    memset(counter_digits_1_row_N[5], 0, sizeof(counter_digits_1_row_N[5])); strcpy(counter_digits_1_row_N[5], String(String(satioData.altitude)).c_str());
    memset(counter_digits_1_row_N[6], 0, sizeof(counter_digits_1_row_N[6])); strcpy(counter_digits_1_row_N[6], String(satioData.ground_heading).c_str());
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); strcpy(counter_digits_1_row_N[7], String(String(satioData.speed, 2)).c_str());
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
    memset(counter_digits_1_row_N[7], 0, sizeof(counter_digits_1_row_N[7])); strcpy(counter_digits_1_row_N[7], String("N/A").c_str());
    memset(counter_digits_1_row_N[8], 0, sizeof(counter_digits_1_row_N[8])); strcpy(counter_digits_1_row_N[8], String("pending").c_str());
    for (int i = 0; i < 10; i++) {if (i==0) {printf("%-19s", counter_chars_1_col_0[i+4]);} else {printf("%-19s", counter_digits_1_row_N[i-1]);}}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("INS MODE : %d (%s) | INS INIT FLAG : %d/%d | INS FORCED ON FLAG : %d | INS_REQ_GPS_PRECISION : %f | INS_REQ_MIN_SPEED : %f | INS_REQ_HEADING_RANGE_DIFF : %f\n",
      insData.INS_MODE, insData.char_ins_mode[insData.INS_MODE], insData.INS_INITIALIZATION_FLAG, MAX_INS_INITIALIZATION_FLAG, insData.INS_FORCED_ON_FLAG,
      insData.INS_REQ_GPS_PRECISION, insData.INS_REQ_MIN_SPEED, insData.INS_REQ_HEADING_RANGE_DIFF);
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("                   X                  Y                  Z\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_ang_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_ang_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_ang_z).c_str());
    printf("Angle              ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_gyr_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_gyr_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_gyr_z).c_str());
    printf("Gyro               ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_acc_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_acc_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_acc_z).c_str());
    printf("Acceleration       ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    memset(counter_digits_1_row_N[0], 0, sizeof(counter_digits_1_row_N[0])); strcpy(counter_digits_1_row_N[0], String(gyroData.gyro_0_mag_x).c_str());
    memset(counter_digits_1_row_N[1], 0, sizeof(counter_digits_1_row_N[1])); strcpy(counter_digits_1_row_N[1], String(gyroData.gyro_0_mag_y).c_str());
    memset(counter_digits_1_row_N[2], 0, sizeof(counter_digits_1_row_N[2])); strcpy(counter_digits_1_row_N[2], String(gyroData.gyro_0_mag_z).c_str());
    printf("Magnetic Field     ");
    for (int i = 0; i < 3; i++) {printf("%-19s", counter_digits_1_row_N[i]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Weekday int: %d Weekday Name: %s\n", satioData.local_wday, satioData.local_wday_name);
    printf("Meteor Warning: ");
    for (int i = 0; i < MAX_METEOR_SHOWERS; i++) {printf("%s: %d %d | ", meteor_shower_names[i], meteor_shower_warning_system[i][0], meteor_shower_warning_system[i][1]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    // printf("SDCard mounted: %d (%s (type: %d)\n", sdcardData.sdcard_mounted, sdcardData.sdcard_type_names[sdcardData.sdcard_type], sdcardData.sdcard_type);
  }
    // ----------------------------------------------------------------------------------------------------------------------------
    //                                                                                                      PRINT PROGRAMMABLE DATA
    // ----------------------------------------------------------------------------------------------------------------------------
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("                      ");
    printArray(print_index_0, 0, 35);
    printf("Computer Assist    :  ");
    for (int i=0;i<35;i++) {printf("%d      ", matrixData.computer_assist[0][i]);}
    printf("\n");
    printf("Switch Intention   :  ");
    for (int i=0;i<35;i++) {printf("%d      ", matrixData.switch_intention[0][i]);}
    printf("\n");
    printf("Computer Intention :  ");
    for (int i=0;i<35;i++) {printf("%d      ", matrixData.computer_intention[0][i]);}
    printf("\n");
    printf("Output Value       :  ");
    printArray(matrixData.output_value[0], 0, 35);
    }
    if (systemData.output_stat_vv) {
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 0  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][0]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 1  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][1]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 2  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][2]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 3  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][3]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 4  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][4]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 0  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][5]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 6  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][6]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 7  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][7]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 8  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][8]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 9  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7d", matrixData.matrix_function[0][i][9]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 0; i < 35; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    }
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("                      ");
    printArray(print_index_1, 0, 35);
    printf("Computer Assist    :  ");
    for (int i=35;i<70;i++) {printf("%d      ", matrixData.computer_assist[0][i]);}
    printf("\n");
    printf("Switch Intention   :  ");
    for (int i=35;i<70;i++) {printf("%d      ", matrixData.switch_intention[0][i]);}
    printf("\n");
    printf("Computer Intention :  ");
    for (int i=35;i<70;i++) {printf("%d      ", matrixData.computer_intention[0][i]);}
    printf("\n");
    printf("Output Value       :  ");
    printArray(matrixData.output_value[0], 35, 70);
    }
    if (systemData.output_stat_vv) {
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 0  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][0]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][0][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 1  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][1]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][1][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 2  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][2]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][2][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 3  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][3]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][3][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 4  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][4]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][4][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 0  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][5]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][5][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 6  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][6]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][6][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 7  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][7]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][7][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 8  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][8]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][8][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("Switch Function 9  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7d", matrixData.matrix_function[0][i][9]);}
    printf("\n");
    printf("Switch Function X  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_X]);}
    printf("\n");
    printf("Switch Function Y  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Y]);}
    printf("\n");
    printf("Switch Function Z  :  ");
    for (int i = 35; i < 70; i++) {printf("%-7.0f", matrixData.matrix_function_xyz[0][i][9][INDEX_MATRIX_FUNTION_Z]);}
    printf("\n");
    }
    if (systemData.output_stat_v==true || systemData.output_stat_vv) {
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("                      ");
    printArray(print_index_0, 0, 35);
    printf("Mapped Values      :  ");
    printArray(mappingData.mapped_value[0], 0, 35);
    printf("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    printf("                      ");
    printArray(print_index_1, 0, 35);
    printf("Mapped Values      :  ");
    printArray(mappingData.mapped_value[0], 35, 70);
    }
}