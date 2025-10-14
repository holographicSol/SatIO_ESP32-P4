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

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DATA: MATRIX
// ------------------------------------------------------------------------------------------------------------------------------

struct MatrixStruct {

  int64_t i_count_matrix;

  bool load_matrix_on_startup;

  int i_computer_assist_enabled; // count computer assist enabled 
  int i_computer_assist_disabled; // count computer assist disabled

  int i_switch_intention_high; // count intention high
  int i_switch_intention_low; // count intention low
  
  int i_computer_intention_high; // count intention high
  int i_computer_intention_low; // count intention low

  char matrix_sentence[MAX_GLOBAL_SERIAL_BUFFER_SIZE]; // an NMEA inspired sentence reflecting matrix switch states

  // -------------------------------------------------------------------------------------------------------
  // Enable/disable computer. Allow system to control pins on the port controller.
  // -------------------------------------------------------------------------------------------------------
  bool computer_assist[1][MAX_MATRIX_SWITCHES]={{}};
  // -------------------------------------------------------------------------------------------------------
  // Switch high/low intention (digital). Set pins on port controller high/low/(analog if true).
  // -------------------------------------------------------------------------------------------------------
  bool switch_intention[1][MAX_MATRIX_SWITCHES]={{}};
  bool prev_switch_intention[1][MAX_MATRIX_SWITCHES]={{}};
  // -------------------------------------------------------------------------------------------------------
  // Computer high/low intention (digital). Is switch logic true or false.
  // -------------------------------------------------------------------------------------------------------
  bool computer_intention[1][MAX_MATRIX_SWITCHES]={{}};
  // -------------------------------------------------------------------------------------------------------
  // Matrix switch ports. Should correspond to pins on the port controller.
  // -------------------------------------------------------------------------------------------------------
  signed int matrix_port_map[1][MAX_MATRIX_SWITCHES]={{}};
  // -------------------------------------------------------------------------------------------------------
  // Computer output values. Values that will be sent to the port controller (digital/mapped).
  // -------------------------------------------------------------------------------------------------------
  int32_t output_value[1][MAX_MATRIX_SWITCHES]={};
  int32_t prev_output_value[1][MAX_MATRIX_SWITCHES]={};
  // -------------------------------------------------------------------------------------------------------
  // Fluctuation threshold. No output unless threshold breached+- beyond previous output value.
  // -------------------------------------------------------------------------------------------------------
  uint32_t flux_value[1][MAX_MATRIX_SWITCHES]={};
  // -------------------------------------------------------------------------------------------------------
  // Override output values (computer assist should never ammend these values).
  // -------------------------------------------------------------------------------------------------------
  signed long override_output_value[1][MAX_MATRIX_SWITCHES]={};
  signed long override_prev_output_value[1][MAX_MATRIX_SWITCHES]={};
  // -------------------------------------------------------------------------------------------------------
  // Output mode.
  // key 0 : matrix logic (digital)
  // key 1 : mapped value (analog/digital)
  // -------------------------------------------------------------------------------------------------------
  int output_mode[1][MAX_MATRIX_SWITCHES]={};
  // -------------------------------------------------------------------------------------------------------
  // Matrix switch write required (per switch).
  // -------------------------------------------------------------------------------------------------------
  bool matrix_switch_write_required[1][MAX_MATRIX_SWITCHES]={{}};
  // -------------------------------------------------------------------------------------------------------
  // Output Pulse Width Modulation.
  // 0 : uS time off period (0uS = remain on).
  // 1 : uS time on period  (0uS = remain off).
  // 2 : uS previous time (set automatically).
  // Example: if 0,1 both 0uS then remain on.
  // Example: if 0=>0uS and 1=0uS then pulse, remain off.
  // Example: if 0=>0uS and 1>0uS then keep modulating.
  // Allows for multiple scenarios while remaining simple.
  // -------------------------------------------------------------------------------------------------------
  uint32_t output_pwm[1][MAX_MATRIX_SWITCHES][2]={{}};
  // -------------------------------------------------------------------------------------------------------
  // Matrix switch standard/inverted logic (default standard).
  // -------------------------------------------------------------------------------------------------------
  bool matrix_switch_inverted_logic[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS]={{}};
  // -------------------------------------------------------------------------------------------------------
  // Matrix switch function operator index (default none = 0).
  // -------------------------------------------------------------------------------------------------------
  int matrix_switch_operator_index[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS]={{}};
  // -------------------------------------------------------------------------------------------------------
  // Matrix switch function name index (default off = 0).
  // -------------------------------------------------------------------------------------------------------
  int matrix_function[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS]={{}};
  // -------------------------------------------------------------------------------------------------------
  /*
    Store up to 3 values for each matrix switch function:

                                      0     1     2     
                                      X     Y     Z    
                              {  {   0.0,  0.0,  0.0   } }
    note: - consider matrix_function_xyz type changed to float from double.
          - float has 7 digits of precision and should be faster on ESP32 than use of doubles.
          - would require careful consideration and refactoring.
          - the highest precision floating point numbers are currently various latitide and longitude values.
  */
  // -------------------------------------------------------------------------------------------------------
  double matrix_function_xyz[1][MAX_MATRIX_SWITCHES][MAX_MATRIX_SWITCH_FUNCTIONS][3]={{}};

  // -------------------------------------------------------------------------------------------------------
  // Function operators.
  // -------------------------------------------------------------------------------------------------------
  char matrix_function_operator_name[MAX_MATRIX_OPERATORS][MAX_GLOBAL_ELEMENT_SIZE]={};

  // -------------------------------------------------------------------------------------------------------
  // Function names for function name matrix.
  // -------------------------------------------------------------------------------------------------------
  char matrix_function_names[MAX_MATRIX_FUNCTION_NAMES][MAX_GLOBAL_ELEMENT_SIZE]={};
};
extern struct MatrixStruct matrixData;

// ------------------------------------------------------------------------
// Function Prototypes.
// ------------------------------------------------------------------------
bool matrixSwitch(void);
void SwitchStat(void);
void zero_matrix(void);
void setMatrixDefault(void);
// void setOutputValues(int Mi);
// void writePortControllerM1(int Mi);
void setOutputValues();
void writePortControllerM1();
void writePortControllerM0(void);
bool portControllerWriteRequired(int idx);
void writeI2C(int I2C_Address);

#ifdef __cplusplus
}
#endif

#endif