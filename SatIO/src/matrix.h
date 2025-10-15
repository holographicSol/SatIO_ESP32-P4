/*
  Matrix Library. Written by Benjamin Jack Cullen.

*/

#ifndef MATRIX_H
#define MATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#define MAX_MATRIX_OUTPUT_MODES 2

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
  bool computer_assist[1][MAX_MATRIX_SWITCHES]={{}};

  // Final switch high/low intention (true/false).
  bool switch_intention[1][MAX_MATRIX_SWITCHES]={{}};
  bool prev_switch_intention[1][MAX_MATRIX_SWITCHES]={{}};

  // Computer high/low intention (true/false). Is switch logic true or false.
  bool computer_intention[1][MAX_MATRIX_SWITCHES]={{}};

  // Matrix switch ports. Values should correspond to pins on the port controller.
  signed int matrix_port_map[1][MAX_MATRIX_SWITCHES]={{}};

  // Output values. Values that will be sent to the port controller (digital/mapped).
  int32_t output_value[1][MAX_MATRIX_SWITCHES]={};
  int32_t prev_output_value[1][MAX_MATRIX_SWITCHES]={};

  // Fluctuation threshold. No output unless threshold breached+- beyond previous output value.
  uint32_t flux_value[1][MAX_MATRIX_SWITCHES]={};

  // Override output values (computer assist should never ammend these values).
  signed long override_output_value[1][MAX_MATRIX_SWITCHES]={};
  signed long override_prev_output_value[1][MAX_MATRIX_SWITCHES]={};

  /**
   * Output mode.
   * 
   * 0 : matrix logic (digital) sets output_value as switch_intention value.
   * 1 : mapped value (analog/digital) sets output_value as mapped value.
   */
  int output_mode[1][MAX_MATRIX_SWITCHES]={};

  // Matrix switch write required.
  bool matrix_switch_write_required[1][MAX_MATRIX_SWITCHES]={{}};

  /**
   * Output Pulse Width Modulation.
   * 
   * 0 : uS time off period (0uS = remain on).
   * 1 : uS time on period  (0uS = remain off).
   */
  uint32_t output_pwm[1][MAX_MATRIX_SWITCHES][2]={{}};

  /**
   * Inverted logic.
   * 
   * If true then matrix switch function logic return true if false, false if true. 
   */
  bool matrix_switch_inverted_logic[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS]={{}};

  // Matrix switch function name index (default off = 0).
  int matrix_function[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS]={{}};

  /**
   * Matrix function values.
   * 
   * 0 : Value X
   * 1 : Value Y
   * 2 : Value Z
   */
  double matrix_function_xyz[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS][3]={{}};

  /**
   * Matrix switch function operators.
   * 
   * 0 : None
   * 1 : Equal
   * 2 : Over
   * 3 : Under
   * 4 : In Range
   */
  int matrix_switch_operator_index[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS]={{}};

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
   * 0 None
   * 1 On
   * 2 SwitchLink
   * 3 LocalTime
   * 4 Weekday
   * 5 DateDayX
   * 6 DateMonthX
   * 7 DateYearX
   * 8 DegLat
   * 9 DegLon
   * 10 DegLatLon
   * 11 INSLat
   * 12 INSLon
   * 13 INSLatLon
   * 14 INSHeading
   * 15 INSSpeed
   * 16 INSAltitude
   * 17 UTCTimeGNGGA
   * 18 PosStatusGNGGA
   * 19 SatCount
   * 20 HemiGNGGANorth
   * 21 HemiGNGGASouth
   * 22 HemiGNGGAEast
   * 23 HemiGNGGAWest
   * 24 GPSPrecision
   * 25 AltGNGGA
   * 26 UTCTimeGNRMC
   * 27 PosStatusGNRMCA
   * 28 PosStatusGNRMCV
   * 29 ModeGNRMCA
   * 30 ModeGNRMCD
   * 31 ModeGNRMCE
   * 32 ModeGNRMCN
   * 33 HemiGNRMCNorth
   * 34 HemiGNRMCSouth
   * 35 HemiGNRMCEast
   * 36 HemiGNRMCWest
   * 37 GSpeedGNRMC
   * 38 HeadingGNRMC
   * 39 UTCDateGNRMC
   * 40 LFlagGPATT
   * 41 SFlagGPATT
   * 42 RSFlagGPATT
   * 43 INSGPATT
   * 44 SpeedNumGPATT
   * 45 MileageGPATT
   * 46 GSTDataGPATT
   * 47 YawGPATT
   * 48 RollGPATT
   * 49 PitchGPATT
   * 50 GNGGAValidCS
   * 51 GNRMCValidCS
   * 52 GPATTValidCS
   * 53 GNGGAValidCD
   * 54 GNRMCValidCD
   * 55 GPATTValidCD
   * 56 Gyro0AccX
   * 57 Gyro0AccY
   * 58 Gyro0AccZ
   * 59 Gyro0AngX
   * 60 Gyro0AngY
   * 61 Gyro0AngZ
   * 62 Gyro0MagX
   * 63 Gyro0MagY
   * 64 Gyro0MagZ
   * 65 Gyro0GyroX
   * 66 Gyro0GyroY
   * 67 Gyro0GyroZ
   * 68 Meteors
   * 69 SunAz
   * 70 SunAlt
   * 71 MoonAz
   * 72 MoonAlt
   * 73 MoonPhase
   * 74 MercuryAz
   * 75 MercuryAlt
   * 76 VenusAz
   * 77 VenusAlt
   * 78 MarsAz
   * 79 MarsAlt
   * 80 JupiterAz
   * 81 JupiterAlt
   * 82 SaturnAz
   * 83 SaturnAlt
   * 84 UranusAz
   * 85 UranusAlt
   * 86 NeptuneAz
   * 87 NeptuneAlt
   * 88 ADMPlex0
   * 89 MappedValue
   * 90 SDCARDInserted
   * 91 SDCARDMounted
   */
  char matrix_function_names[MAX_MATRIX_FUNCTION_NAMES][MAX_GLOBAL_ELEMENT_SIZE]={};
};
extern struct MatrixStruct matrixData;

/**
 * Matrix switch calculates all logic accross all switches.
 * @return Returns true each completion
 */
bool matrixSwitch(void);

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
 * Instruct portcontroller to clear all stored instructions.
 * @return Returns None
 */
void writePortControllerM0(void);

/**
 * Instruct portcontroller in mode 1 (pin config/output/pwm).
 * @return Returns None
 */
void writePortControllerM1();

/**
 * Determines if intructions need to be sent to the port controller.
 * @return Returns None
 */
bool portControllerWriteRequired(int idx);

/**
 * Writes buffer over I2C.
 * @return Returns None
 */
void writeI2C(int I2C_Address);

/**
 * Override all computer assist.
 * This can be csequentially different from disabling all computer assist. 
 * @return Returns None
 */
void override_all_computer_assists();

#ifdef __cplusplus
}
#endif

#endif