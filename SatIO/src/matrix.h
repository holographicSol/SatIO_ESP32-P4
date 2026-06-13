/*
  Matrix Library. Written by Benjamin Jack Cullen.

*/

#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "i2c_helper.h"

/**
 * @struct MatrixStruct
 */
struct MatrixStruct {
  // Count matrix executions per second.
  int64_t i_count_matrix;
  // Load matrix file automatically every time the system starts.
  bool load_matrix_on_startup;
  // Count computer assist enabled.
  int i_computer_assist_enabled;
  // Count computer assist disabled.
  int i_computer_assist_disabled;
  // Count intention high
  int i_switch_intention_high;
  // Count intention low
  int i_switch_intention_low;
  // Count intention high
  int i_computer_intention_high;
  // Count intention low
  int i_computer_intention_low;
  // Checksummed sentence
  char matrix_sentence[MAX_GLOBAL_SERIAL_BUFFER_SIZE];

  // Enable/disable computer assist.
  bool computer_assist[1][MAX_MATRIX_SWITCHES];

  // Final switch high/low intention (true/false).
  bool switch_intention[1][MAX_MATRIX_SWITCHES];
  bool prev_switch_intention[1][MAX_MATRIX_SWITCHES];

  // Computer high/low intention (true/false). Is switch logic true or false.
  bool computer_intention[1][MAX_MATRIX_SWITCHES];

  // Matrix switch ports. Values should correspond to pins on the port controller.
  int16_t matrix_port_map[1][MAX_MATRIX_SWITCHES];

  // Output values. Values that will be sent to the port controller (digital/mapped).
  int32_t output_value[1][MAX_MATRIX_SWITCHES];
  int32_t prev_output_value[1][MAX_MATRIX_SWITCHES];

  // Fluctuation threshold. No output unless threshold breached+- beyond previous output value.
  uint32_t flux_value[1][MAX_MATRIX_SWITCHES]={};

  // Override output values (computer assist should never ammend these values).
  signed long override_output_value[1][MAX_MATRIX_SWITCHES];
  signed long override_prev_output_value[1][MAX_MATRIX_SWITCHES];

  /**
   * Output mode.
   * 
   * 0 : matrix logic (digital) sets output_value as switch_intention value.
   * 1 : mapped value (analog/digital) sets output_value as mapped value.
   */
  int output_mode[1][MAX_MATRIX_SWITCHES];

  /**
   * Output mode names.
   * 
   * 0 : matrix logic (digital) sets output_value as switch_intention value.
   * 1 : mapped value (analog/digital) sets output_value as mapped value.
   */
  char output_mode_names[MAX_MATRIX_OUTPUT_MODES][MAX_GLOBAL_ELEMENT_SIZE]={};

  /**
   * Map slot index.
   *
   * Map slot index order is aligned with matrix slot order.
   * 
   * Each slot contains an index number to access any map slot (for potential shareing).
   * 
   */
  int index_mapped_value[1][MAX_MAP_SLOTS];

  // Matrix switch write required.
  bool matrix_switch_write_required[1][MAX_MATRIX_SWITCHES];

  /**
   * Output Pulse Width Modulation.
   * 
   * 0 : uS time off period (0uS = remain on).
   * 1 : uS time on period  (0uS = remain off).
   */
  uint32_t output_pwm[1][MAX_MATRIX_SWITCHES][2];

  /**
   * Inverted logic Names.
   * 
   * 0: Standard
   * 1: Inverted
   */
  char inverted_logic_names[MAX_MATRIX_FUNCTION_INVERTED_LOGIC_MODES][MAX_GLOBAL_ELEMENT_SIZE]={};

  /**
   * Inverted logic.
   * 
   * If true then matrix switch function logic return true if false, false if true. 
   */
  bool matrix_switch_inverted_logic[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS];
  

  // Matrix switch function name index (default off = 0).
  int matrix_function[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS];

  /**
   * Matrix function values.
   * 
   * 0 : Value X
   * 1 : Value Y
   * 2 : Value Z
   */
  double matrix_function_xyz[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS][3];

  /**
   * Matrix function value mode.
   * 
   * 0 : Value X : Mode=0 User Value  Mode=1 System Value
   * 1 : Value Y : Mode=0 User Value  Mode=1 System Value
   * 2 : Value Z : Mode=0 User Value  Mode=1 System Value
   */
  int matrix_function_mode_xyz[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS][3];

  /**
   * Matrix Comparator Modes
   * 
   * 0: User
   * 1: System
   */
  char matrix_function_mode_xyz_name[MAX_MATRIX_FUNCTION_XYZ_MODES][MAX_GLOBAL_ELEMENT_SIZE]={};

  /**
   * Matrix switch function operators.
   * 
   * 0 : None
   * 1 : Equal
   * 2 : Over
   * 3 : Under
   * 4 : In Range
   */
  int matrix_switch_operator_index[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS];

  /**
   * Matrix switch function operator names.
   * 
   * 0 : None
   * 1 : Equal
   * 2 : Over
   * 3 : Under
   * 4 : In Range
   */
  char matrix_function_operator_name[MAX_MATRIX_OPERATORS][MAX_GLOBAL_ELEMENT_SIZE]={};

  /**
   * Matrix switch function names.
   * 
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
   */
  char matrix_function_names[MAX_MATRIX_FUNCTION_NAMES][MAX_GLOBAL_ELEMENT_SIZE]={};
  
  double input_value[1][MAX_MATRIX_SWITCHES];
  signed int input_port_map[1][MAX_MATRIX_SWITCHES];
};
extern struct MatrixStruct matrixData;

/**
 * Matrix switch calculates all logic accross all switches.
 * @return Returns true each completion
 */
bool matrixSwitch(void);

/**
 * @brief Get Matrix Switch Funstion Comparator.
 * 
 * @param index Specify target matrix value comparitor.
 */
String get_matrix_function_comparitor(int index_matrix_value_comparitor);

/**
 * Count switch related stats.
 * @return Returns None
 */
void SwitchStat(void);

/**
 * Clear all matrix switch logic.
 * @return Returns None
 */
void set_all_matrix_default(void);

/**
 * Determines which output values will be set according to output_mode.
 * @return Returns None
 */
void setOutputValues();

/**
 * Override all computer assist.
 * This can be csequentially different from disabling all computer assist. 
 * @return Returns None
 */
void override_all_computer_assists();

/**
 * Instruct portcontroller to clear all stored instructions.
 * @param wire I2C bus instance.
 * @param address I2C address of output portcontroller.
 * @return Returns None
 */
void writeOutputPortControllerClear(TwoWire &wire, int address);

/**
 * @brief Write all stored instructions to output portcontroller.
 * @param wire I2C bus instance.
 * @param address I2C address of output portcontroller.
 * @return Returns None
 */
void writeOutputPortControllerSetPins(TwoWire &wire, int address);

/**
 * @brief Read all input states from input portcontroller.
 * @param wire I2C bus instance.
 * @param address I2C address of input portcontroller.
 * @return Returns true if successful.
 */
bool readInputPortControllerReadPins(TwoWire &wire,
                                 int address);

#endif