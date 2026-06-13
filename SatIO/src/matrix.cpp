/*
  Matrix Library. Written by Benjamin Jack Cullen.

*/

#include "matrix.h"
#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include "wtgps300p.h"
#include "wt901.h"
#include "multiplexers.h"
#include "custommapping.h"
#include "sidereal_helper.h"
#include "satio.h"
#include "ins.h"
#include "meteors.h"
#include "serial_infocmd.h"
#include "system_data.h"
#include "sdcard_helper.h"
#include "i2c_helper.h"

// ----------------------------------------------------------------------------------------
//  MATRIX
// ----------------------------------------------------------------------------------------
struct MatrixStruct matrixData = {
  .i_count_matrix=0,
  .load_matrix_on_startup=0,
  .i_computer_assist_enabled=0,
  .i_computer_assist_disabled=0,
  .i_switch_intention_high=0,
  .i_switch_intention_low=0,
  .i_computer_intention_high=0,
  .i_computer_intention_low=0,
  .matrix_sentence={0},
  .computer_assist={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false
    }
  },
  .switch_intention={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false
    }
  },
  .prev_switch_intention={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false
    }
  },
  .computer_intention={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false
    }
  },
  .matrix_port_map={
    {
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 0-9
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 10-19
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 20-29
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 30-9
      -1, -1, -1, -1, -1
    }
  },
  .output_value={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
    0, 0, 0, 0, 0
  },
  .prev_output_value={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
    0, 0, 0, 0, 0
  },
  .flux_value={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
    0, 0, 0, 0, 0
  },
  .override_output_value={},
  .override_prev_output_value={},
  .output_mode={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
    0, 0, 0, 0, 0
  },
  .output_mode_names=
  {
    "Digital", // 0
    "Mapped"   // 1
  },
  .index_mapped_value={
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-39
      0, 0, 0, 0, 0
  },
  .matrix_switch_write_required={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false
    }
  },
  .output_pwm={
    {
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 0-9
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 10-19
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 20-29
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 30-39
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}
    }
  },
  .inverted_logic_names=
  {
    "Standard", // 0
    "Inverted"   // 1
  },
  .matrix_switch_inverted_logic={
    {
      {false, false, false, false, false, false, false, false, false, false}, // 0
      {false, false, false, false, false, false, false, false, false, false}, // 1
      {false, false, false, false, false, false, false, false, false, false}, // 2
      {false, false, false, false, false, false, false, false, false, false}, // 3
      {false, false, false, false, false, false, false, false, false, false}, // 4
      {false, false, false, false, false, false, false, false, false, false}, // 5
      {false, false, false, false, false, false, false, false, false, false}, // 6
      {false, false, false, false, false, false, false, false, false, false}, // 7
      {false, false, false, false, false, false, false, false, false, false}, // 8
      {false, false, false, false, false, false, false, false, false, false}, // 9
      {false, false, false, false, false, false, false, false, false, false}, // 10
      {false, false, false, false, false, false, false, false, false, false}, // 11
      {false, false, false, false, false, false, false, false, false, false}, // 12
      {false, false, false, false, false, false, false, false, false, false}, // 13
      {false, false, false, false, false, false, false, false, false, false}, // 14
      {false, false, false, false, false, false, false, false, false, false}, // 15
      {false, false, false, false, false, false, false, false, false, false}, // 16
      {false, false, false, false, false, false, false, false, false, false}, // 17
      {false, false, false, false, false, false, false, false, false, false}, // 18
      {false, false, false, false, false, false, false, false, false, false}, // 19
      {false, false, false, false, false, false, false, false, false, false}, // 20
      {false, false, false, false, false, false, false, false, false, false}, // 21
      {false, false, false, false, false, false, false, false, false, false}, // 22
      {false, false, false, false, false, false, false, false, false, false}, // 23
      {false, false, false, false, false, false, false, false, false, false}, // 24
      {false, false, false, false, false, false, false, false, false, false}, // 25
      {false, false, false, false, false, false, false, false, false, false}, // 26
      {false, false, false, false, false, false, false, false, false, false}, // 27
      {false, false, false, false, false, false, false, false, false, false}, // 28
      {false, false, false, false, false, false, false, false, false, false}, // 29
      {false, false, false, false, false, false, false, false, false, false}, // 30
      {false, false, false, false, false, false, false, false, false, false}, // 31
      {false, false, false, false, false, false, false, false, false, false}, // 32
      {false, false, false, false, false, false, false, false, false, false}, // 33
      {false, false, false, false, false, false, false, false, false, false}, // 34
      {false, false, false, false, false, false, false, false, false, false}, // 35
      {false, false, false, false, false, false, false, false, false, false}, // 36
      {false, false, false, false, false, false, false, false, false, false}, // 37
      {false, false, false, false, false, false, false, false, false, false}, // 38
      {false, false, false, false, false, false, false, false, false, false}, // 39
      {false, false, false, false, false, false, false, false, false, false}, // 40
      {false, false, false, false, false, false, false, false, false, false}, // 41
      {false, false, false, false, false, false, false, false, false, false}, // 42
      {false, false, false, false, false, false, false, false, false, false}, // 43
      {false, false, false, false, false, false, false, false, false, false}, // 44
    }
  },
  .matrix_function={
    {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 0
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 1
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 2
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 3
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 4
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 5
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 6
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 7
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 8
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 9
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 10
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 11
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 12
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 13
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 14
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 15
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 16
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 17
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 18
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 19
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 20
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 21
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 22
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 23
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 24
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 25
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 26
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 27
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 28
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 29
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 30
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 31
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 32
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 33
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 34
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 35
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 36
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 37
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 38
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 39
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 40
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 41
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 42
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 43
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 44
    }
  },
  .matrix_function_xyz={
    {
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 0
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 1
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 2
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 3
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 4
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 5
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 6
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 7
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 8
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 9
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 10
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 11
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 12
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 13
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 14
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 15
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 16
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 17
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 18
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 19
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 20
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 21
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 22
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 23
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 24
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 25
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 26
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 27
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 28
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 29
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 30
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 31
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 32
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 33
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 34
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 35
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 36
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 37
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 38
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 39
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 40
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 41
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 42
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 43
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 44
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      }
    }
  },
  .matrix_function_mode_xyz={
    {
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 0
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 1
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 2
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 3
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 4
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 5
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 6
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 7
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 8
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 9
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 10
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 11
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 12
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 13
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 14
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 15
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 16
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 17
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 18
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 19
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 20
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 21
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 22
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 23
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 24
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 25
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 26
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 27
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 28
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 29
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 30
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 31
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 32
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 33
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 34
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 35
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 36
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 37
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 38
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 39
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 40
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 41
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 42
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 43
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      },
      {
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // 44
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
      }
    }
  },
  .matrix_function_mode_xyz_name={
    "User",   // 0
    "System", // 1
  },
  .matrix_switch_operator_index={
    {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 0
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 1
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 2
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 3
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 4
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 5
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 6
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 7
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 8
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 9
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 10
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 11
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 12
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 13
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 14
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 15
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 16
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 17
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 18
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 19
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 20
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 21
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 22
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 23
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 24
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 25
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 26
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 27
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 28
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 29
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 30
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 31
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 32
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 33
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 34
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 35
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 36
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 37
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 38
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 39
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 40
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 41
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 42
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 43
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 44
    }
  },
  .matrix_function_operator_name =
  {
    "None",  // 0
    "Equal", // 1 
    "Over",  // 2
    "Under", // 3
    "Range", // 4
  },
  .matrix_function_names =
  {
    "NONE", //0
    "ON", //1
    "Switch Link", //2
    "Time HHMMSS", //3
    "Week Day", //4
    "Month Day", //5
    "Month", //6
    "Year", //7
    "SatIO Deg Lat", //8
    "SatIO Deg Lon", //9
    "SatIO INS Lat", //10
    "SatIO INS Lon", //11
    "SatIO INS Heading", //12
    "SatIO INS Alt", //13
    "GNGGA Status", //14
    "GNGGA Sat Count", //15
    "GNGGA Prescion", //16
    "GNGGA Altitude", //17
    "GNRMC Ground Speed", //18
    "GNRMC Heading", //19
    "GPATT Line", //20
    "GPATT Static", //21
    "GPATT Run State", //22
    "GPATT INS", //23
    "GPATT Mileage", //24
    "GPATT GST", //25
    "GPATT Yaw", //26
    "GPATT Roll", //27
    "GPATT Pitch", //28
    "GNGGA Valid CS", //29
    "GNRMC Valid CS", //30
    "GPATT Valid CS", //31
    "GNGGA Bad CD", //32
    "GNRMC Bad CD", //33
    "GPATT Bad CD", //34
    "GNRMC Pos Stat A", //35
    "GNRMC Pos Stat V", //36
    "GNRMC Mode Ind A", //37
    "GNRMC Mode Ind D", //38
    "GNRMC Mode Ind E", //39
    "GNRMC Mode Ind N", //40
    "GNRMC Hemi North", //41
    "GNRMC Hemi South", //42
    "GNRMC Hemi East", //43
    "GNRMC Hemi West", //44
    "G0 G-Force X", //45
    "G0 G-Force Y", //46
    "G0 G-Force Z", //47
    "G0 Incline X", //48
    "G0 Incline Y", //49
    "G0 Incline Z", //50
    "G0 Mag Field X", //51
    "G0 Mag Field Y", //52
    "G0 Mag Field Z", //53
    "G0 Velocity X", //54
    "G0 Velocity Y", //55
    "G0 Velocity Z", //56
    "Meteor", //57
    "Sun Azimuth", //58
    "Sun Altitude", //59
    "Sun Helio Ecl Lat", //60
    "Sun Helio Ecl Lon", //61
    "Luna Azimuth", //62
    "Luna Altitude", //63
    "Luna Phase", //64
    "Mercury Azimuth", //65
    "Mercury Altitude", //66
    "Mercury H.Ecliptic Lat", //67
    "Mercury H.Ecliptic Lon", //68
    "Mercury Ecliptic Lat", //69
    "Mercury Ecliptic Lon", //70
    "Venus Azimuth", //71
    "Venus Altitude", //72
    "Venus H.Ecliptic Lat", //73
    "Venus H.Ecliptic Lon", //74
    "Venus Ecliptic Lat", //75
    "Venus Ecliptic Lon", //76
    "Earth Ecliptic Lon", //77
    "Mars Azimuth", //78
    "Mars Altitude", //79
    "Mars H.Ecliptic Lat", //80
    "Mars H.Ecliptic Lon", //81
    "Mars Ecliptic Lat", //82
    "Mars Ecliptic Lon", //83
    "Jupiter Azimuth", //84
    "jupiter Altitude", //85
    "Jupiter H.Ecliptic Lat", //86
    "Jupiter H.Ecliptic Lon", //87
    "Jupiter Ecliptic Lat", //88
    "Jupiter Ecliptic Lon", //89
    "Saturn Azimuth", //90
    "Saturn Altitude", //91
    "Saturn H.Ecliptic Lat", //92
    "Saturn H.Ecliptic Lon", //93
    "Saturn Ecliptic Lat", //94
    "Saturn Ecliptic Lon", //95
    "Uranus Azimuth", //96
    "Uranus Altitude", //97
    "Uranus H.Ecliptic Lat", //98
    "Uranus H.Ecliptic Lon", //99
    "Uranus Ecliptic Lat", //100
    "Uranus Ecliptic Lon", //101
    "Neptune Azimuth", //102
    "Neptune Altitude", //103
    "Neptune H.Ecliptic Lat", //104
    "Neptune H.Ecliptic Lon", //105
    "Neptune Ecliptic Lat", //106
    "Neptune Ecliptic Lon", //107
    "AD Multiplexer 0", //108
    "Map Slot", //109
    "SD Card Inserted", //110
    "SD Card Mounted", //111
    "Port Con 0", //112
  },
  .input_value = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-39
    0, 0, 0, 0, 0
  },
  .input_port_map={
    {
      // --------------------------------------------
      // digital
      // --------------------------------------------
      // 0, 1. 2, 3, 4, 5, 6, 7, 8, 9,
      // 10,11,12,13,14,15,16,17,18,19,
      // 20,21,22,23,24,
      25,26,27,28,29,
      30,31,32,33,34,35,36,37,38,39,
      40,41,42,43,44,45,46,47,48,49,
      50,51,52,53,
      // --------------------------------------------
      // analog
      // --------------------------------------------
      54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,

    }
  },
};

bool in_range_check_true(double n0, double n1, double r) {
  // Serial.println(
  //   "in_range_check_true: (n0 " +
  //   String(n0) +
  //   " >= n1 (" +
  //   String(n1) +
  //   " - r/2 " +
  //   String(r/2) +
  //   ")) && (n0 " +
  //   String(n0) +
  //   " <= n1 (" +
  //   String(n1) +
  //   " + r/2 " +
  //   String(r/2) +
  //   "))");
  if ((n0  >=  n1 - r/2) && (n0  <= n1 + r/2)) {return true;}
  return false;
}

bool in_range_check_false(double n0, double n1, double r) {
  // Serial.println(
  //   "in_range_check_false: (n0 " +
  //   String(n0) +
  //   " >= n1 (" +
  //   String(n1) +
  //   " - r/2 " +
  //   String(r/2) +
  //   ")) && (n0 " +
  //   String(n0) +
  //   " <= n1 (" +
  //   String(n1) +
  //   " + r/2 " +
  //   String(r/2) +
  //   "))");
  if ((n0  >=  n1 - r/2) && (n0  <= n1 + r/2)) {return false;}
  return true;
}

bool in_square_range_check_true(double x0, double x1, double y0, double y1, double r) {
  if (in_range_check_true(x0, x1, r)==true) {
    if (in_range_check_true(y0, y1, r)==true) {return true;} else return false;}
  return false;
}

bool in_square_range_check_false(double x0, double x1, double y0, double y1, double r) {
  if (in_range_check_true(x0, x1, r)==true) {
    if (in_range_check_true(y0, y1, r)==true) {return false;} else return true;}
  return true;
}

bool check_over_true(double n0, double n1) {
  // Serial.println("check_over_true: n0 " + String(n0) + " > n1 " + String(n1));
  if (n0 > n1) {return true;}
  return false;
}

bool check_over_false(double n0, double n1) {
  // Serial.println("check_over_false: n0 " + String(n0) + " > n1 " + String(n1));
  if (n0 > n1) {return false;}
  return true;
}

bool check_under_true(double n0, double n1) {
  // Serial.println("check_under_true: n0 " + String(n0) + " < n1 " + String(n1));
  if (n0 < n1) {return true;}
  else return false;
}

bool check_under_false(double n0, double n1) {
  // Serial.println("check_under_false: n0 " + String(n0) + " < n1 " + String(n1));
  if (n0 < n1) {return false;}
  return true;
}

bool check_equal_true(double n0, double n1) {
  // Serial.println("check_equal_true: n0 " + String(n0) + "==n1 " + String(n1));
  if (n0==n1) {return true;}
  return false;
}

bool check_equal_false(double n0, double n1) {
  // Serial.println("check_equal_false: n0 " + String(n0) + "==n1 " + String(n1));s
  if (n0 != n1) {return true;}
  return false;
}

bool check_ge_and_le_true(double n0, double n1, double n2) {
  // Serial.println(
  //   "check_ge_and_le_true: n0 " +
  //   String(n0) +
  //   " >= n1 " +
  //   String(n1) +
  //   " && n0 " +
  //   String(n0) +
  //   " <= " +
    // String(n2));
  if ((n0 >= n1) && (n0 <= n2)) {return true;}
  return false;
}

bool check_ge_and_le_false(double n0, double n1, double n2) {
  // Serial.println(
  //   "check_ge_and_le_false: n0 " +
  //   String(n0) +
  //   " >= n1 " +
  //   String(n1) +
  //   " && n0 " +
  //   String(n0) +
  //   " <= " +
  //   String(n2));
  if ((n0 >= n1) && (n0 <= n2)) {return false;}
  return true;
}

bool check_strncmp_true(const char * c0, const char * c1, int n) {
  // Serial.println("check_strncmp_true: c0 " + String(c0) + "==c1 " + String(c1) + " (n=" + String(n) + ")");
  if (strncmp(c0, c1, n)==0) {return true;}
  return false;
}

bool check_strncmp_false(const char * c0, const char * c1, int n) {
  // Serial.println("check_strncmp_false: c0 " + String(c0) + "==c1 " + String(c1) + " (n=" + String(n) + ")");
  if (strncmp(c0, c1, n)==0) {return false;}
  return true;
}

bool check_bool_true(bool _bool) {
  // Serial.println("check_bool_true: " + String(_bool));
  if (_bool==true) {return true;}
  return false;
}

bool check_bool_false(bool _bool) {
  // Serial.println("check_bool_false: " + String(_bool));
  if (_bool==false) {return true;}
  return false;
}

bool final_bool=true;
String temp_string_x="";
String temp_string_y="";
String temp_string_z="";
char *xyzptr;
double tmp_x=0;
double tmp_y=0;
double tmp_z=0;
bool handle_char=false;
bool handle_digit=false;
int final_counter=0;

bool matrixSwitch(void) {

  // Iterate over each matrix switch.
  for (int Mi=0; Mi < MAX_MATRIX_SWITCHES; Mi++) {

    // Temporary switch is zero (each switch has 0 to 10 functions that must all be true for the switch to turn high/low).
    bool tmp_matrix[10]={};
    final_counter=0;
    
    // Iterate over each function in the current matrix switch.
    for (int Fi=0; Fi < MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
      handle_char=false;
      handle_digit=false;

      // uncomment to debug
      // Serial.println("-----------------------------------------------" + String());
      // Serial.println("[Mi] " + String(Mi));
      // Serial.println("[Fi] " + String(Fi));
      // Serial.println("[matrixData.matrix_function[0][Mi][Fi]] " + String(matrixData.matrix_function[0][Mi][Fi]));

      /*
        TODO: CONVERT THE FOLLOWING TO SWITCH STATEMENT: SWITCH matrixData.matrix_function[0][Mi][Fi]
      */

      // Perfromance prefers adding function names in matrix from index zero, so if function index zero is 'none' then break.
      if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NONE) && (Fi==0)) {tmp_matrix[Fi]=false; break;}

      // Put true in temporary matrix for 'none' at non-zero function indices (allows for 1-N functions to be set).
      // requires functions set consecutively from index 0-N.
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NONE) && (Fi!=0))  {
        for (int i=Fi; i < MAX_MATRIX_SWITCH_FUNCTIONS; i++) {tmp_matrix[i]=true;}
        break;
      }

      // Put true in temporary matrix if switch is function name is set to Enabled (return true with no further logic required).
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_ON) && (Fi==0)) {
        for (int i=0; i < MAX_MATRIX_SWITCH_FUNCTIONS; i++) {tmp_matrix[i]=true;}
        break;
      }

      // A special function that allows stacking matrix switch logic (specify matrix switch n to link).
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SWITCH_LINK) && (Fi==0)) {
        if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false) {
          tmp_matrix[Fi]=check_equal_true(
            matrixData.switch_intention[0][(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]],
            true);
        }
        else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true) {
          tmp_matrix[Fi]=check_equal_false(
            matrixData.switch_intention[0][(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]],
            true);
        }
      }

      // Check digits.
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_TIME_HHMMSS)) {
        tmp_x = atol(satioData.padded_local_time_HHMMSS);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_WEEK_DAY)) {
        tmp_x = satioData.local_wday;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MONTH_DAY)) {
        tmp_x = satioData.local_mday;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MONTH)) {
        tmp_x = satioData.local_month;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_YEAR)) {
        tmp_x = satioData.local_year;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATIO_DEG_LAT)) {
        tmp_x = satioData.degrees_latitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATIO_DEG_LON)) {
        tmp_x = satioData.degrees_longitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_LAT)) {
        tmp_x = insData.ins_latitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_LON)) {
        tmp_x = insData.ins_longitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_HEADING)) {
        tmp_x = insData.ins_heading;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_ALT)) {
        tmp_x = insData.ins_altitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_STATUS)) {
        tmp_x = atoi(gnggaData.solution_status);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_SAT_COUNT)) {
        tmp_x = atol(gnggaData.satellite_count);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_PRESCION)) {
        tmp_x = atof(gnggaData.gps_precision_factor);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_ALTITUDE)) {
        tmp_x = atof(gnggaData.altitude);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_GROUND_SPEED)) {
        tmp_x = atof(gnrmcData.ground_speed);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEADING)) {
        tmp_x = strtod(gnrmcData.ground_heading, NULL);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_LINE)) {
        tmp_x = atoi(gpattData.line_flag);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_STATIC)) {
        tmp_x = atoi(gpattData.static_flag);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_RUN_STATE)) {
        tmp_x = atoi(gpattData.run_state_flag);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_INS)) {
        tmp_x = atoi(gpattData.ins);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_MILEAGE)) {
        tmp_x = strtod(gpattData.mileage, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_GST)) {
        tmp_x = strtod(gpattData.gst_data, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_YAW)) {
        tmp_x = strtod(gpattData.yaw, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_ROLL)) {
        tmp_x = strtod(gpattData.roll, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_PITCH)) {
        tmp_x = strtod(gpattData.pitch, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_VALID_CS)) {
        tmp_x = gnggaData.valid_checksum;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_VALID_CS)) {
        tmp_x = gnrmcData.valid_checksum;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_VALID_CS)) {
        tmp_x = gpattData.valid_checksum;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_BAD_CD)) {
        tmp_x = (long)gnggaData.bad_element_count;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_BAD_CD)) {
        tmp_x = (long)gnrmcData.bad_element_count;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATT_BAD_CD)) {
        tmp_x = (long)gnrmcData.bad_element_count;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_X)) {
        tmp_x = gyroData.gyro_0_acc_x;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_Y)) {
        tmp_x = gyroData.gyro_0_acc_y;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_Z)) {
        tmp_x = gyroData.gyro_0_acc_z;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_X)) {
        tmp_x = gyroData.gyro_0_ang_x;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_Y)) {
        tmp_x = gyroData.gyro_0_ang_y;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_Z)) {
        tmp_x = gyroData.gyro_0_ang_z;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_X)) {
        tmp_x = gyroData.gyro_0_mag_x;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_Y)) {
        tmp_x = gyroData.gyro_0_mag_y;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_Z)) {
        tmp_x = gyroData.gyro_0_mag_z;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_X)) {
        tmp_x = gyroData.gyro_0_gyr_x;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_Y)) {
        tmp_x = gyroData.gyro_0_gyr_y;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_Z)) {
        tmp_x = gyroData.gyro_0_gyr_z;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_METEOR)) {
        tmp_x = meteor_shower_warning_system
                [(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]]
                [(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]];
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Sun
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SUN_AZIMUTH)) {
        tmp_x = siderealPlanetData.sun_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SUN_ALTITUDE)) {
        tmp_x = siderealPlanetData.sun_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SUN_HELIO_ECL_LAT)) {
        tmp_x = siderealPlanetData.sun_helio_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SUN_HELIO_ECL_LON)) {
        tmp_x = siderealPlanetData.saturn_helio_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Luna
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_LUNA_AZIMUTH)) {
        tmp_x = siderealPlanetData.luna_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_LUNA_ALTITUDE)) {
        tmp_x = siderealPlanetData.luna_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_LUNA_PHASE)) {
        tmp_x = siderealPlanetData.luna_p;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Mercury
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_AZIMUTH)) {
        tmp_x = siderealPlanetData.mercury_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ALTITUDE)) {
        tmp_x = siderealPlanetData.mercury_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_H_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.mercury_helio_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_H_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.mercury_helio_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.mercury_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.mercury_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Venus
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_VENUS_AZIMUTH)) {
        tmp_x = siderealPlanetData.venus_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_VENUS_ALTITUDE)) {
        tmp_x = siderealPlanetData.venus_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_VENUS_H_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.venus_helio_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_VENUS_H_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.venus_helio_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_VENUS_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.venus_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_EARTH_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.venus_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Earth
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_EARTH_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.earth_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Mars
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MARS_AZIMUTH)) {
        tmp_x = siderealPlanetData.mars_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MARS_ALTITUDE)) {
        tmp_x = siderealPlanetData.mars_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MARS_H_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.mars_helio_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MARS_H_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.mars_helio_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MARS_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.mars_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MARS_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.mars_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Jupiter
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_AZIMUTH)) {
        tmp_x = siderealPlanetData.jupiter_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ALTITUDE)) {
        tmp_x = siderealPlanetData.jupiter_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_H_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.jupiter_helio_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_H_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.jupiter_helio_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.jupiter_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.jupiter_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Saturn
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATURN_AZIMUTH)) {
        tmp_x = siderealPlanetData.saturn_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ALTITUDE)) {
        tmp_x = siderealPlanetData.saturn_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATURN_H_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.saturn_helio_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATURN_H_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.saturn_helio_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.saturn_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.saturn_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Uranus
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_URANUS_AZIMUTH)) {
        tmp_x = siderealPlanetData.uranus_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ALTITUDE)) {
        tmp_x = siderealPlanetData.uranus_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_URANUS_H_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.uranus_helio_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_URANUS_H_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.uranus_helio_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.uranus_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.uranus_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Neptune
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_AZIMUTH)) {
        tmp_x = siderealPlanetData.neptune_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ALTITUDE)) {
        tmp_x = siderealPlanetData.neptune_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_H_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.neptune_helio_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_H_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.neptune_helio_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ECLIPTIC_LAT)) {
        tmp_x = siderealPlanetData.neptune_ecliptic_lat;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ECLIPTIC_LON)) {
        tmp_x = siderealPlanetData.neptune_ecliptic_long;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_AD_MULTIPLEXER_0)) {
        tmp_x = ad_mux_0.data
                [(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]];
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MAP_SLOT)) {
        tmp_x = mappingData.mapped_value
                [0][(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]];
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      // else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SD_CARD_INSERTED)) {
      //   tmp_x = sdcardData.sdcard_inserted;
      //   tmp_y = 0;
      //   tmp_z = 0;
      //   handle_digit=true;
      // }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SD_CARD_MOUNTED)) {
        tmp_x = sdcardData.sdcard_mounted;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_PORT_CON_0)) {
        tmp_x = matrixData.input_value[0][(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]];
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }

      // Handle digits.
      if (handle_digit==true) {

        if ((matrixData.matrix_switch_operator_index[0][Mi][Fi]==INDEX_MATRIX_SWITCH_OPERATOR_OVER)) {
          if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false) {
            // User X: Compare System Value to Users X Value
            if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==0) {
              tmp_matrix[Fi]=check_over_true(tmp_x,
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
            }
            // System X: Compare System Value to System Value Using X as Index To System Value
            else if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==1) {
              tmp_matrix[Fi]=check_over_true(tmp_x,
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]).c_str(), NULL));
            }
          }
          else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true) {
            // User X: Compare System Value to Users X Value
            if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==0) {
              tmp_matrix[Fi]=check_over_false(tmp_x,
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
            }
            // System X: Compare System Value to System Value Using X as Index To System Value
            else if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==1) {
              tmp_matrix[Fi]=check_over_false(tmp_x,
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]).c_str(), NULL));
            }
          }
        }

        else if ((matrixData.matrix_switch_operator_index[0][Mi][Fi]==INDEX_MATRIX_SWITCH_OPERATOR_UNDER)) {
          if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false) {
            // User X: Compare System Value to Users X Value
            if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==0) {
              tmp_matrix[Fi]=check_under_true(tmp_x,
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
            }
            // System X: Compare System Value to System Value Using X as Index To System Value
            else if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==1) {
              tmp_matrix[Fi]=check_under_true(tmp_x,
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]).c_str(), NULL));
            }
          }
          else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true) {
            // User X: Compare System Value to Users X Value
            if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==0) {
              tmp_matrix[Fi]=check_under_false(tmp_x,
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
            }
            // System X: Compare System Value to System Value Using X as Index To System Value
            else if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==1) {
              tmp_matrix[Fi]=check_under_false(tmp_x,
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]).c_str(), NULL));
            }
          }
        }

        else if ((matrixData.matrix_switch_operator_index[0][Mi][Fi]==INDEX_MATRIX_SWITCH_OPERATOR_EQUAL)) {
            // User X: Compare System Value to Users X Value
            if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==0) {
              tmp_matrix[Fi]=check_equal_true(tmp_x,
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
            }
            // System X: Compare System Value to System Value Using X as Index To System Value
            else if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==1) {
              tmp_matrix[Fi]=check_equal_true(tmp_x,
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]).c_str(), NULL));
            }
        }

        else if ((matrixData.matrix_switch_operator_index[0][Mi][Fi]==INDEX_MATRIX_SWITCH_OPERATOR_RANGE)) {
          if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false) {
            // User X: Compare System Value to Users X Value
            if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==0) {
              tmp_matrix[Fi]=check_ge_and_le_true(tmp_x,
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X],
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]);
            }
            else if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==1) {
              tmp_matrix[Fi]=check_ge_and_le_true(tmp_x,
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]).c_str(), NULL),
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]).c_str(), NULL));
            }
          }
          else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true) {
            // User X: Compare System Value to Users X Value
            if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==0) {
              tmp_matrix[Fi]=check_ge_and_le_false(tmp_x,
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X],
              matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]);
            }
            else if (matrixData.matrix_function_mode_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]==1) {
              tmp_matrix[Fi]=check_ge_and_le_false(tmp_x,
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]).c_str(), NULL),
              strtod(get_matrix_function_comparitor(matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]).c_str(), NULL));
            }
          }
        }
      }
      // Check chars.
      else {
        if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_POS_STAT_A))
          {temp_string_x="A"; temp_string_y=String(gnrmcData.positioning_status); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_POS_STAT_V))
          {temp_string_x="V"; temp_string_y=String(gnrmcData.positioning_status); handle_char=true;}

        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_A))
          {temp_string_x="A"; temp_string_y=String(gnrmcData.mode_indication); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_D))
          {temp_string_x="D"; temp_string_y=String(gnrmcData.mode_indication); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_E))
          {temp_string_x="E"; temp_string_y=String(gnrmcData.mode_indication); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_N))
          {temp_string_x="N"; temp_string_y=String(gnrmcData.mode_indication); handle_char=true;}

        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_NORTH))
          {temp_string_x="N"; temp_string_y=String(gnrmcData.latitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_SOUTH))
          {temp_string_x="S"; temp_string_y=String(gnrmcData.latitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_EAST))
          {temp_string_x="E"; temp_string_y=String(gnrmcData.longitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_WEST))
          {temp_string_x="W"; temp_string_y=String(gnrmcData.longitude_hemisphere); handle_char=true;}
        if (handle_char==true) {
          // Handle Chars.
          if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false)
            {tmp_matrix[Fi]=check_strncmp_true(temp_string_x.c_str(),
                                               temp_string_y.c_str(),
                                               strlen(temp_string_x.c_str()));}
          else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true)
            {tmp_matrix[Fi]=check_strncmp_false(temp_string_x.c_str(),
                                                temp_string_y.c_str(),
                                                strlen(temp_string_x.c_str()));}
        }
      }
    } // End function iteration for this switch.

    // ##############################################################################################################
    // FINAL SWITCH START
    // ##############################################################################################################

    /**
     * Set Final Bool.
     */
    final_bool=true;
    for (int FC=0; FC < MAX_MATRIX_SWITCH_FUNCTIONS; FC++)
      {if (tmp_matrix[FC]==false) {final_bool=false; break;}}

    /**
     * If computer_assist enabled:
     * - Computer_intention true/false is set.
     * - Switch_intention true/false is set.
     * 
     * If computer_assist disabled:
     * - Computer_intention true/false is set.
     * - Switch_intention true/false is not set.
     */

    // Computer Intent
    matrixData.computer_intention[0][Mi]=final_bool; // computer intention always set

    // Computer Assist
    if (matrixData.computer_assist[0][Mi]==true) {
      
      // Switch Intent
      matrixData.switch_intention[0][Mi]=final_bool;
    }
    else {
      matrixData.switch_intention[0][Mi]=false;
    }
    
    // ##############################################################################################################
    // FINAL SWITCH END
    // ##############################################################################################################

  } // End switch iteration
  return true;
}

/**
 * @brief Get Matrix Switch Funstion Comparator.
 * 
 * @param index Specify target matrix value comparitor.
 */
String get_matrix_function_comparitor(int index_matrix_value_comparitor) {
  String result = "NAN";
  switch (index_matrix_value_comparitor) {

    case INDEX_MATRIX_SWITCH_FUNCTION_TIME_HHMMSS:
      result = String(satioData.padded_local_time_HHMMSS);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_WEEK_DAY:
      result = String(satioData.local_wday);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MONTH_DAY:
      result = String(satioData.local_mday);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MONTH:
      result = String(satioData.local_month);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_YEAR:
      result = String(satioData.local_year);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATIO_DEG_LAT:
      result = String(satioData.degrees_latitude);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATIO_DEG_LON:
      result = String(satioData.degrees_longitude);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_LAT:
      result = String(insData.ins_latitude);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_LON:
      result = String(insData.ins_longitude);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_HEADING:
      result = String(insData.ins_heading);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATIO_INS_ALT:
      result = String(insData.ins_altitude);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_STATUS:
      result = String(gnggaData.solution_status);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_SAT_COUNT:
      result = String(gnggaData.satellite_count);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_PRESCION:
      result = String(gnggaData.gps_precision_factor);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_ALTITUDE:
      result = String(gnggaData.altitude);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_GROUND_SPEED:
      result = String(gnrmcData.ground_speed);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEADING: {
      result = String(gnrmcData.ground_heading);
      break;
    }

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_LINE:
      result = String(gpattData.line_flag);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_STATIC:
      result = String(gpattData.static_flag);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_RUN_STATE:
      result = String(gpattData.run_state_flag);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_INS:
      result = String(gpattData.ins);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_MILEAGE: {
      result = String(gpattData.mileage);
      break;
    }

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_GST: {
      result = String(gpattData.gst_data);
      break;
    }

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_YAW: {
      result = String(gpattData.yaw);
      break;
    }

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_ROLL: {
      result = String(gpattData.roll);
      break;
    }

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_PITCH: {
      result = String(gpattData.pitch);
      break;
    }

    case INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_VALID_CS:
      result = String(gnggaData.valid_checksum);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_VALID_CS:
      result = String(gnrmcData.valid_checksum);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_VALID_CS:
      result = String(gpattData.valid_checksum);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNGGA_BAD_CD:
      result = String((long)gnggaData.bad_element_count);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_BAD_CD:
      result = String((long)gnrmcData.bad_element_count);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GPATT_BAD_CD:
      result = String((long)gnrmcData.bad_element_count);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_X:
      result = String(gyroData.gyro_0_acc_x);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_Y:
      result = String(gyroData.gyro_0_acc_y);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_G_FORCE_Z:
      result = String(gyroData.gyro_0_acc_z);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_X:
      result = String(gyroData.gyro_0_ang_x);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_Y:
      result = String(gyroData.gyro_0_ang_y);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_INCLINE_Z:
      result = String(gyroData.gyro_0_ang_z);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_X:
      result = String(gyroData.gyro_0_mag_x);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_Y:
      result = String(gyroData.gyro_0_mag_y);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_MAG_FIELD_Z:
      result = String(gyroData.gyro_0_mag_z);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_X:
      result = String(gyroData.gyro_0_gyr_x);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_Y:
      result = String(gyroData.gyro_0_gyr_y);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_G0_VELOCITY_Z:
      result = String(gyroData.gyro_0_gyr_z);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_METEOR:
      result = String(
        meteor_shower_warning_system
          [(int)matrixData.matrix_function_xyz[0][index_matrix_value_comparitor][0][INDEX_MATRIX_FUNTION_X]]
          [(int)matrixData.matrix_function_xyz[0][index_matrix_value_comparitor][0][INDEX_MATRIX_FUNTION_Y]]
      );
      break;

    // Sun

    case INDEX_MATRIX_SWITCH_FUNCTION_SUN_AZIMUTH:
      result = String(siderealPlanetData.sun_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SUN_ALTITUDE:
      result = String(siderealPlanetData.sun_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SUN_HELIO_ECL_LAT:
      result = String(siderealPlanetData.sun_helio_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SUN_HELIO_ECL_LON:
      result = String(siderealPlanetData.saturn_helio_ecliptic_long);
      break;

    // Luna

    case INDEX_MATRIX_SWITCH_FUNCTION_LUNA_AZIMUTH:
      result = String(siderealPlanetData.luna_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_LUNA_ALTITUDE:
      result = String(siderealPlanetData.luna_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_LUNA_PHASE:
      result = String(siderealPlanetData.luna_p);
      break;

    // Mercury

    case INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_AZIMUTH:
      result = String(siderealPlanetData.mercury_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ALTITUDE:
      result = String(siderealPlanetData.mercury_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_H_ECLIPTIC_LAT:
      result = String(siderealPlanetData.mercury_helio_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_H_ECLIPTIC_LON:
      result = String(siderealPlanetData.mercury_helio_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ECLIPTIC_LAT:
      result = String(siderealPlanetData.mercury_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MERCURY_ECLIPTIC_LON:
      result = String(siderealPlanetData.mercury_ecliptic_long);
      break;

    // Venus

    case INDEX_MATRIX_SWITCH_FUNCTION_VENUS_AZIMUTH:
      result = String(siderealPlanetData.venus_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_VENUS_ALTITUDE:
      result = String(siderealPlanetData.venus_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_VENUS_H_ECLIPTIC_LAT:
      result = String(siderealPlanetData.venus_helio_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_VENUS_H_ECLIPTIC_LON:
      result = String(siderealPlanetData.venus_helio_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_VENUS_ECLIPTIC_LAT:
      result = String(siderealPlanetData.venus_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_VENUS_ECLIPTIC_LON:
      result = String(siderealPlanetData.venus_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_EARTH_ECLIPTIC_LON:
      result = String(siderealPlanetData.earth_ecliptic_long);
      break;

    // Mars

    case INDEX_MATRIX_SWITCH_FUNCTION_MARS_AZIMUTH:
      result = String(siderealPlanetData.mars_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MARS_ALTITUDE:
      result = String(siderealPlanetData.mars_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MARS_H_ECLIPTIC_LAT:
      result = String(siderealPlanetData.mars_helio_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MARS_H_ECLIPTIC_LON:
      result = String(siderealPlanetData.mars_helio_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MARS_ECLIPTIC_LAT:
      result = String(siderealPlanetData.mars_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MARS_ECLIPTIC_LON:
      result = String(siderealPlanetData.mars_ecliptic_long);
      break;

    // Jupiter

    case INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_AZIMUTH:
      result = String(siderealPlanetData.jupiter_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ALTITUDE:
      result = String(siderealPlanetData.jupiter_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_H_ECLIPTIC_LAT:
      result = String(siderealPlanetData.jupiter_helio_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_H_ECLIPTIC_LON:
      result = String(siderealPlanetData.jupiter_helio_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ECLIPTIC_LAT:
      result = String(siderealPlanetData.jupiter_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_JUPITER_ECLIPTIC_LON:
      result = String(siderealPlanetData.jupiter_ecliptic_long);
      break;

    // Saturn

    case INDEX_MATRIX_SWITCH_FUNCTION_SATURN_AZIMUTH:
      result = String(siderealPlanetData.saturn_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ALTITUDE:
      result = String(siderealPlanetData.saturn_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATURN_H_ECLIPTIC_LAT:
      result = String(siderealPlanetData.saturn_helio_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATURN_H_ECLIPTIC_LON:
      result = String(siderealPlanetData.saturn_helio_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ECLIPTIC_LAT:
      result = String(siderealPlanetData.saturn_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SATURN_ECLIPTIC_LON:
      result = String(siderealPlanetData.saturn_ecliptic_long);
      break;

    // Uranus

    case INDEX_MATRIX_SWITCH_FUNCTION_URANUS_AZIMUTH:
      result = String(siderealPlanetData.uranus_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ALTITUDE:
      result = String(siderealPlanetData.uranus_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_URANUS_H_ECLIPTIC_LAT:
      result = String(siderealPlanetData.uranus_helio_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_URANUS_H_ECLIPTIC_LON:
      result = String(siderealPlanetData.uranus_helio_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ECLIPTIC_LAT:
      result = String(siderealPlanetData.uranus_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_URANUS_ECLIPTIC_LON:
      result = String(siderealPlanetData.uranus_ecliptic_long);
      break;

    // Neptune

    case INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_AZIMUTH:
      result = String(siderealPlanetData.neptune_az);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ALTITUDE:
      result = String(siderealPlanetData.neptune_alt);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_H_ECLIPTIC_LAT:
      result = String(siderealPlanetData.neptune_helio_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_H_ECLIPTIC_LON:
      result = String(siderealPlanetData.neptune_helio_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ECLIPTIC_LAT:
      result = String(siderealPlanetData.neptune_ecliptic_lat);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNE_ECLIPTIC_LON:
      result = String(siderealPlanetData.neptune_ecliptic_long);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_AD_MULTIPLEXER_0:
      result = String(
        ad_mux_0.data[
          (int)matrixData.matrix_function_xyz[0][index_matrix_value_comparitor][0][INDEX_MATRIX_FUNTION_Z]
        ]
      );
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_MAP_SLOT:
      result = String(
        mappingData.mapped_value[0][
          (int)matrixData.matrix_function_xyz[0][index_matrix_value_comparitor][0][INDEX_MATRIX_FUNTION_Z]
        ]
      );
      break;

    // The SD_CARD_INSERTED block is commented out in your digit code.
    case INDEX_MATRIX_SWITCH_FUNCTION_SD_CARD_INSERTED:
      result = String("");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_SD_CARD_MOUNTED:
      result = String(sdcardData.sdcard_mounted);
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_PORT_CON_0:
      result = String(
        matrixData.input_value[0][
          (int)matrixData.matrix_function_xyz[0][index_matrix_value_comparitor][0][INDEX_MATRIX_FUNTION_Z]
        ]
      );
      break;

    // Char-comparison cases – return the “constant” side you compare against

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_POS_STAT_A:
      result = String("A");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_POS_STAT_V:
      result = String("V");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_A:
      result = String("A");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_D:
      result = String("D");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_E:
      result = String("E");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_MODE_IND_N:
      result = String("N");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_NORTH:
      result = String("N");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_SOUTH:
      result = String("S");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_EAST:
      result = String("E");
      break;

    case INDEX_MATRIX_SWITCH_FUNCTION_GNRMC_HEMI_WEST:
      result = String("W");
      break;

    default:
      result = String("NAN");
      break;
  }
  return result;
}

void SwitchStat(void) {
  int tmp_i_computer_assist_enabled=0;
  int tmp_i_computer_assist_disabled=0;

  int tmp_i_switch_intention_high=0;
  int tmp_i_switch_intention_low=0;

  int tmp_i_computer_intention_high=0;
  int tmp_i_computer_intention_low=0;

  for (int Mi=0; Mi < MAX_MATRIX_SWITCHES; Mi++) {
    if (matrixData.computer_assist[0][Mi]==true)
      {tmp_i_computer_assist_enabled++;} else {tmp_i_computer_assist_disabled++;}
    if (matrixData.switch_intention[0][Mi]==true)
      {tmp_i_switch_intention_high++;} else {tmp_i_switch_intention_low++;}
    if (matrixData.computer_intention[0][Mi]==true)
      {tmp_i_computer_intention_high++;} else {tmp_i_computer_intention_low++;}
  }

  matrixData.i_computer_assist_enabled=tmp_i_computer_assist_enabled;
  matrixData.i_computer_assist_disabled=tmp_i_computer_assist_disabled;

  matrixData.i_switch_intention_high=tmp_i_switch_intention_high;
  matrixData.i_switch_intention_low=tmp_i_switch_intention_low;

  matrixData.i_computer_intention_high=tmp_i_computer_intention_high;
  matrixData.i_computer_intention_low=tmp_i_computer_intention_low;
}

void matrixZeroX() {
}
void matrixZeroY() {
}
void matrixZeroZ() {
}
void matrixZeroFunctionName() {
}
void matrixZeroPort() {
}
void matrixZeroInverted() {
}

void override_all_computer_assists() {
  for (int Mi=0; Mi < MAX_MATRIX_SWITCHES; Mi++)
    {matrixData.computer_assist[0][Mi]=false;
     matrixData.override_output_value[0][Mi]=0;
     matrixData.matrix_switch_write_required[0][Mi]=true;}
}

void set_matrix_default(int matrix_switch) {
  matrixData.matrix_port_map[0][matrix_switch]=-1;
  matrixData.output_mode[0][matrix_switch]=0;
  matrixData.output_pwm[0][matrix_switch][0]=0;
  matrixData.output_pwm[0][matrix_switch][1]=0;
  matrixData.flux_value[0][matrix_switch]=0;
  matrixData.switch_intention[0][matrix_switch]=0;
  matrixData.computer_intention[0][matrix_switch]=0;
  matrixData.index_mapped_value[0][matrix_switch]=0;
  for (int Fi=0; Fi < MAX_MATRIX_SWITCH_FUNCTIONS; Fi++) {
    matrixData.matrix_function[0][matrix_switch][Fi]=0;
    matrixData.matrix_function_xyz[0][matrix_switch][Fi][INDEX_MATRIX_FUNTION_X]=0.0;
    matrixData.matrix_function_xyz[0][matrix_switch][Fi][INDEX_MATRIX_FUNTION_Y]=0.0;
    matrixData.matrix_function_xyz[0][matrix_switch][Fi][INDEX_MATRIX_FUNTION_Z]=0.0;
    matrixData.matrix_switch_operator_index[0][matrix_switch][Fi]=0;
    matrixData.matrix_switch_inverted_logic[0][matrix_switch][Fi]=false;
  }
}

void set_all_matrix_default(void) {
  for (int Mi=0; Mi < MAX_MATRIX_SWITCHES; Mi++) {set_matrix_default(Mi);}
  Serial.println("$MATRIXNEW");
}

void setAllMatrixSwitchesStateFalse() {
  for (int i=0; i<MAX_MATRIX_SWITCHES; i++) {matrixData.switch_intention[0][i]=false;}
}

void setAllMatrixSwitchesStateTrue() {
  for (int i=0; i<MAX_MATRIX_SWITCHES; i++) {matrixData.switch_intention[0][i]=true;}
}

void setOutputValues(void) {
  for (int Mi=0; Mi<MAX_MATRIX_SWITCHES; Mi++) {

    int32_t oval = 0;

    // Off
    if (matrixData.matrix_function[0][Mi][0]==INDEX_MATRIX_SWITCH_FUNCTION_NONE) {oval=0;}

    else {

      // A : Default output value: switch intention (digital 0/1).
      if (matrixData.output_mode[0][Mi]==0) {
        oval=matrixData.switch_intention[0][Mi];
      }

      // B : Mapped values.
      else if ((matrixData.output_mode[0][Mi]==1)) {
          oval=mappingData.mapped_value[0][matrixData.index_mapped_value[0][Mi]];
          // printf("Mi=%d  Imv=%d  Ov=%ld\n", Mi, matrixData.index_mapped_value[0][Mi], mappingData.mapped_value[0][matrixData.index_mapped_value[0][Mi]]);
      }

    }

    // Override according to switch intention
    if (matrixData.switch_intention[0][Mi]==true) {matrixData.output_value[0][Mi]=oval;}
    else {matrixData.output_value[0][Mi]=0;}

    // Set write required
    if ( (matrixData.output_value[0][Mi] > matrixData.prev_output_value[0][Mi] + matrixData.flux_value[0][Mi]) || 
        (matrixData.output_value[0][Mi] < matrixData.prev_output_value[0][Mi] - matrixData.flux_value[0][Mi]))
    {
      matrixData.prev_output_value[0][Mi] = matrixData.output_value[0][Mi];
      matrixData.matrix_switch_write_required[0][Mi]=true;
    }
  }
}

// ----------------------------------------------------------------------------------------
//  MATRIX I/O
// ----------------------------------------------------------------------------------------
IICLink IICLinkPCI; // IIC link data structure for input port controller
IICLink IICLinkPCO; // IIC link data structure for output port controller

void writeOutputPortControllerClear(TwoWire &wire, int address) {
  clearI2CLinkOutputPacket(IICLinkPCO);
  write_uint8_ToPacket(IICLinkPCO.OUTPUT_PACKET, 0, 0x0A); // command 10
  writeI2CToSlaveBin(wire, IICLinkPCO, address, 1, 0, "writeOutputPortControllerClear");
  systemData.i_count_port_controller_output++;
}

void writeOutputPortControllerSetPins(TwoWire &wire, int address) {
  for (int Mi = 0; Mi < MAX_MATRIX_SWITCHES; Mi++) {
    if (matrixData.matrix_switch_write_required[0][Mi]) {
      clearI2CLinkOutputPacket(IICLinkPCO);

      // Select value to send as either override value or output value 
      int32_t value_to_send = matrixData.computer_assist[0][Mi]
                        ? matrixData.output_value[0][Mi]
                        : (int32_t)matrixData.override_output_value[0][Mi];
      
      uint32_t off_time = matrixData.output_pwm[0][Mi][0];
      uint32_t on_time = matrixData.output_pwm[0][Mi][1];
      
      // Build binary packet with human readable helper functions
      write_uint8_ToPacket(IICLinkPCO.OUTPUT_PACKET, 0, 0x14); // command 20
      write_uint8_ToPacket(IICLinkPCO.OUTPUT_PACKET, 1, (uint8_t)Mi);
      write_int8_ToPacket(IICLinkPCO.OUTPUT_PACKET, 2, (int8_t)matrixData.matrix_port_map[0][Mi]);
      write_int32_ToPacket(IICLinkPCO.OUTPUT_PACKET, 3, value_to_send);
      write_uint32_ToPacket(IICLinkPCO.OUTPUT_PACKET, 7, off_time);
      write_uint32_ToPacket(IICLinkPCO.OUTPUT_PACKET, 11, on_time);
      // Write to slave
      writeI2CToSlaveBin(wire, IICLinkPCO, address, 15, 0, "writeOutputPortControllerM1");

      // Clear flag after successful send (or retry logic if needed)
      matrixData.matrix_switch_write_required[0][Mi] = false;
    }
  }
  systemData.i_count_port_controller_output++;
}

bool readInputPortControllerReadPins(TwoWire &wire, int address) {

  // Send event ID once to set a mode on receiver
  clearI2CLinkOutputPacket(IICLinkPCO);
  IICLinkPCO.OUTPUT_PACKET[0] = 0x1E; // command 30
  writeI2CToSlaveBin(wire, IICLinkPCO, address, 1, 0, "readInputPortControllerM1");

  // Custom request function: Keep making requests until complete
  for (uint8_t i = 0; i < MAX_MATRIX_SWITCHES; i++) {
    // Send request
    if (requestFromSlaveBinNoID(wire,
                           IICLinkPCO,
                           address,
                           5,
                           0,
                           "readInputPortControllerM1"))
    {
      // Populate value
      uint8_t pin;
      read_uint8_FromWire(wire, pin);
      float value;
      read_float_FromWire(wire, value);
      if (pin < MAX_MATRIX_SWITCHES) {
        matrixData.input_value[0][pin] = (double)value;
        // Serial.printf("[BIN] P%d = %.6f\n", pin, matrixData.input_value[0][pin]);
      }
    }
  }
  return true;
}
