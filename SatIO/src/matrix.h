/*
  Matrix Library. Written by Benjamin Jack Cullen.

*/

#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/**
 * @struct MatrixStruct
 */
struct MatrixStruct {
  // Count matrix executions per second.
  volatile int64_t i_count_matrix;
  // Load matrix file automatically every time the system starts.
  volatile bool load_matrix_on_startup;
  // Count computer assist enabled.
  volatile int i_computer_assist_enabled;
  // Count computer assist disabled.
  volatile int i_computer_assist_disabled;
  // Count intention high
  volatile int i_switch_intention_high;
  // Count intention low
  volatile int i_switch_intention_low;
  // Count intention high
  volatile int i_computer_intention_high;
  // Count intention low
  volatile int i_computer_intention_low;
  // Checksummed sentence
  char matrix_sentence[MAX_GLOBAL_SERIAL_BUFFER_SIZE];

  // Enable/disable computer assist.
  volatile bool computer_assist[1][MAX_MATRIX_SWITCHES];

  // Final switch high/low intention (true/false).
  volatile bool switch_intention[1][MAX_MATRIX_SWITCHES];
  volatile bool prev_switch_intention[1][MAX_MATRIX_SWITCHES];

  // Computer high/low intention (true/false). Is switch logic true or false.
  bool computer_intention[1][MAX_MATRIX_SWITCHES];

  // Matrix switch ports. Values should correspond to pins on the port controller.
  volatile signed int matrix_port_map[1][MAX_MATRIX_SWITCHES];

  // Output values. Values that will be sent to the port controller (digital/mapped).
  int32_t output_value[1][MAX_MATRIX_SWITCHES];
  int32_t prev_output_value[1][MAX_MATRIX_SWITCHES];

  // Fluctuation threshold. No output unless threshold breached+- beyond previous output value.
  volatile uint32_t flux_value[1][MAX_MATRIX_SWITCHES]={};

  // Override output values (computer assist should never ammend these values).
  volatile signed long override_output_value[1][MAX_MATRIX_SWITCHES];
  volatile signed long override_prev_output_value[1][MAX_MATRIX_SWITCHES];

  /**
   * Output mode.
   * 
   * 0 : matrix logic (digital) sets output_value as switch_intention value.
   * 1 : mapped value (analog/digital) sets output_value as mapped value.
   */
  volatile int output_mode[1][MAX_MATRIX_SWITCHES];

  // Matrix switch write required.
  volatile bool matrix_switch_write_required[1][MAX_MATRIX_SWITCHES];

  /**
   * Output Pulse Width Modulation.
   * 
   * 0 : uS time off period (0uS = remain on).
   * 1 : uS time on period  (0uS = remain off).
   */
  volatile uint32_t output_pwm[1][MAX_MATRIX_SWITCHES][2];

  /**
   * Inverted logic.
   * 
   * If true then matrix switch function logic return true if false, false if true. 
   */
  volatile bool matrix_switch_inverted_logic[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS];

  // Matrix switch function name index (default off = 0).
  volatile int matrix_function[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS];

  /**
   * Matrix function values.
   * 
   * 0 : Value X
   * 1 : Value Y
   * 2 : Value Z
   */
  volatile double matrix_function_xyz[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS][3];

  /**
   * Matrix switch function operators.
   * 
   * 0 : None
   * 1 : Equal
   * 2 : Over
   * 3 : Under
   * 4 : In Range
   */
  volatile int matrix_switch_operator_index[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS];

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
    * 0 NONE
    * 1 ON
    * 2 SWITCHLINK
    * 3 LOCALTIME
    * 4 WEEKDAY
    * 5 DATEDAYX
    * 6 DATEMONTHX
    * 7 DATEYEARX
    * 8 DEGLAT
    * 9 DEGLON
    * 10 INSLAT
    * 11 INSLON
    * 12 INSHEADING
    * 13 INSALTITUDE
    * 14 POSSTATUSGNGGA
    * 15 SATCOUNT
    * 16 GPSPRECISION
    * 17 ALTGNGGA
    * 18 GSPEEDGNRMC
    * 19 HEADINGGNRMC
    * 20 LFLAGGPATT
    * 21 SFLAGGPATT
    * 22 RSFLAGGPATT
    * 23 INSGPATT
    * 24 SPEEDNUMGPATT
    * 25 MILEAGEGPATT
    * 26 GSTDATAGPATT
    * 27 YAWGPATT
    * 28 ROLLGPATT
    * 29 PITCHGPATT
    * 30 GNGGAVALIDCS
    * 31 GNRMCVALIDCS
    * 32 GPATTVALIDCS
    * 33 GNGGAVALIDCD
    * 34 GNRMCVALIDCD
    * 35 GPATTVALIDCD
    * 36 GYRO0ACCX
    * 37 GYRO0ACCY
    * 38 GYRO0ACCZ
    * 39 GYRO0ANGX
    * 40 GYRO0ANGY
    * 41 GYRO0ANGZ
    * 42 GYRO0MAGX
    * 43 GYRO0MAGY
    * 44 GYRO0MAGZ
    * 45 GYRO0GYROX
    * 46 GYRO0GYROY
    * 47 GYRO0GYROZ
    * 48 METEORS
    * 49 SUNAZ
    * 50 SUNALT
    * 51 MOONAZ
    * 52 MOONALT
    * 53 MOONPHASE
    * 54 MERCURYAZ
    * 55 MERCURYALT
    * 56 VENUSAZ
    * 57 VENUSALT
    * 58 MARSAZ
    * 59 MARSALT
    * 60 JUPITERAZ
    * 61 JUPITERALT
    * 62 SATURNAZ
    * 63 SATURNALT
    * 64 URANUSAZ
    * 65 URANUSALT
    * 66 NEPTUNEAZ
    * 67 NEPTUNEALT
    * 68 HEMIGNGGANORTH
    * 69 HEMIGNGGASOUTH
    * 70 HEMIGNGGAEAST
    * 71 HEMIGNGGAWEST
    * 72 POSSTATUSGNRMCA
    * 73 POSSTATUSGNRMCV
    * 74 MODEGNRMCA
    * 75 MODEGNRMCD
    * 76 MODEGNRMCE
    * 77 MODEGNRMCN
    * 78 HEMIGNRMCNORTH
    * 79 HEMIGNRMCSOUTH
    * 80 HEMIGNRMCEAST
    * 81 HEMIGNRMCWEST
    * 82 ADMPLEX0
    * 83 MAPPEDVALUE
    * 84 SDCARDINSERTED
    * 85 SDCARDMOUNTED
   */
  char matrix_function_names[MAX_MATRIX_FUNCTION_NAMES][MAX_GLOBAL_ELEMENT_SIZE]={};
  
  volatile double input_value[1][MAX_MATRIX_SWITCHES];
  volatile signed int input_port_map[1][MAX_MATRIX_SWITCHES];
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
 * Override all computer assist.
 * This can be csequentially different from disabling all computer assist. 
 * @return Returns None
 */
void override_all_computer_assists();

/**
 * Instruct portcontroller to clear all stored instructions.
 * @return Returns None
 */
void writeOutputPortControllerM0(TwoWire &iic_bus);

/**
 * Write mode 1 instructions to portcontroller.
 * @return Returns None
 */
void writeOutputPortControllerM1(TwoWire &iic_bus);

bool readInputPortControllerM1(TwoWire &iic_bus);

#endif