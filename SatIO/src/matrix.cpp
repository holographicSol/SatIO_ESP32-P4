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
#include "sdmmc_helper.h"

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
      false, false, false, false, false, false, false, false, false, false, // 40-49
      false, false, false, false, false, false, false, false, false, false, // 50-59
      false, false, false, false, false, false, false, false, false, false, // 60-69
    }
  },
  .switch_intention={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false, false, false, false, false, false, // 40-49
      false, false, false, false, false, false, false, false, false, false, // 50-59
      false, false, false, false, false, false, false, false, false, false, // 60-69
    }
  },
  .prev_switch_intention={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false, false, false, false, false, false, // 40-49
      false, false, false, false, false, false, false, false, false, false, // 50-59
      false, false, false, false, false, false, false, false, false, false, // 60-69
    }
  },
  .computer_intention={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false, false, false, false, false, false, // 40-49
      false, false, false, false, false, false, false, false, false, false, // 50-59
      false, false, false, false, false, false, false, false, false, false, // 60-69
    }
  },
  .matrix_port_map={
    {
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 0-9
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 10-19
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 20-29
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 30-9
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 40-49
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 50-59
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 60-69
    }
  },
  .output_value={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
  },
  .prev_output_value={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
  },
  .flux_value={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
  },
  .override_output_value={},
  .override_prev_output_value={},
  .output_mode={
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
  },
  .matrix_switch_write_required={
    {
      false, false, false, false, false, false, false, false, false, false, // 0-9
      false, false, false, false, false, false, false, false, false, false, // 10-19
      false, false, false, false, false, false, false, false, false, false, // 20-29
      false, false, false, false, false, false, false, false, false, false, // 30-39
      false, false, false, false, false, false, false, false, false, false, // 40-49
      false, false, false, false, false, false, false, false, false, false, // 50-59
      false, false, false, false, false, false, false, false, false, false, // 60-69
    }
  },
  .output_pwm={
    {
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 0-9
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 10-19
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 20-29
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 30-39
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 40-49
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, // 50-59
      {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}  // 60-69
    }
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
      {false, false, false, false, false, false, false, false, false, false}, // 45
      {false, false, false, false, false, false, false, false, false, false}, // 46
      {false, false, false, false, false, false, false, false, false, false}, // 47
      {false, false, false, false, false, false, false, false, false, false}, // 48
      {false, false, false, false, false, false, false, false, false, false}, // 49
      {false, false, false, false, false, false, false, false, false, false}, // 50
      {false, false, false, false, false, false, false, false, false, false}, // 51
      {false, false, false, false, false, false, false, false, false, false}, // 52
      {false, false, false, false, false, false, false, false, false, false}, // 53
      {false, false, false, false, false, false, false, false, false, false}, // 54
      {false, false, false, false, false, false, false, false, false, false}, // 55
      {false, false, false, false, false, false, false, false, false, false}, // 56
      {false, false, false, false, false, false, false, false, false, false}, // 57
      {false, false, false, false, false, false, false, false, false, false}, // 58
      {false, false, false, false, false, false, false, false, false, false}, // 59
      {false, false, false, false, false, false, false, false, false, false}, // 60
      {false, false, false, false, false, false, false, false, false, false}, // 61
      {false, false, false, false, false, false, false, false, false, false}, // 62
      {false, false, false, false, false, false, false, false, false, false}, // 63
      {false, false, false, false, false, false, false, false, false, false}, // 64
      {false, false, false, false, false, false, false, false, false, false}, // 65
      {false, false, false, false, false, false, false, false, false, false}, // 66
      {false, false, false, false, false, false, false, false, false, false}, // 67
      {false, false, false, false, false, false, false, false, false, false}, // 68
      {false, false, false, false, false, false, false, false, false, false}, // 69
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
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 45
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 46
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 47
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 48
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 49
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 50
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 51
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 52
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 53
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 54
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 55
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 56
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 57
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 58
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 59
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 60
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 61
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 62
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 63
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 64
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 65
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 66
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 67
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 68
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 69
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
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 45
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 46
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 47
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 48
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 49
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 50
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 51
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 52
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 53
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 54
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 55
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 56
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 57
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 58
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 59
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 60
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 61
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 62
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 63
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 64
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 65
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 66
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 67
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 68
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
      {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 69
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
      },
    }
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
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 45
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 46
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 47
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 48
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 49
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 50
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 51
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 52
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 53
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 54
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 55
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 56
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 57
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 58
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 59
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 60
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 61
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 62
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 63
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 64
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 65
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 66
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 67
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 68
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 69
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
    "None",  // 0
    "On",  // 1
    "SwitchLink",  // 2
    "LocalTime",  // 3
    "Weekday",  // 4
    "DateDayX",  // 5
    "DateMonthX",  // 6
    "DateYearX",  // 7
    "DegLat",  // 8
    "DegLon",  // 9
    "DegLatLon",  // 10
    "INSLat",  // 11
    "INSLon",  // 12
    "INSLatLon",  // 13
    "INSHeading",  // 14
    "INSAltitude",  // 15
    "UTCTimeGNGGA",  // 16
    "PosStatusGNGGA",  // 17
    "SatCount",  // 18
    "HemiGNGGANorth",  // 19
    "HemiGNGGASouth",  // 20
    "HemiGNGGAEast",  // 21
    "HemiGNGGAWest",  // 22
    "GPSPrecision",  // 23
    "AltGNGGA",  // 24
    "UTCTimeGNRMC",  // 25
    "PosStatusGNRMCA",  // 26
    "PosStatusGNRMCV",  // 27
    "ModeGNRMCA",  // 28
    "ModeGNRMCD",  // 29
    "ModeGNRMCE",  // 30
    "ModeGNRMCN",  // 31
    "HemiGNRMCNorth",  // 32
    "HemiGNRMCSouth",  // 33
    "HemiGNRMCEast",  // 34
    "HemiGNRMCWest",  // 35
    "GSpeedGNRMC",  // 36
    "HeadingGNRMC",  // 37
    "UTCDateGNRMC",  // 38
    "LFlagGPATT",  // 39
    "SFlagGPATT",  // 40
    "RSFlagGPATT",  // 41
    "INSGPATT",  // 42
    "SpeedNumGPATT",  // 43
    "MileageGPATT",  // 44
    "GSTDataGPATT",  // 45
    "YawGPATT",  // 46
    "RollGPATT",  // 47
    "PitchGPATT",  // 48
    "GNGGAValidCS",  // 49
    "GNRMCValidCS",  // 50
    "GPATTValidCS",  // 51
    "GNGGAValidCD",  // 52
    "GNRMCValidCD",  // 53
    "GPATTValidCD",  // 54
    "Gyro0AccX",  // 55
    "Gyro0AccY",  // 56
    "Gyro0AccZ",  // 57
    "Gyro0AngX",  // 58
    "Gyro0AngY",  // 59
    "Gyro0AngZ",  // 60
    "Gyro0MagX",  // 61
    "Gyro0MagY",  // 62
    "Gyro0MagZ",  // 63
    "Gyro0GyroX",  // 64
    "Gyro0GyroY",  // 65
    "Gyro0GyroZ",  // 66
    "Meteors",  // 67
    "SunAz",  // 68
    "SunAlt",  // 69
    "MoonAz",  // 70
    "MoonAlt",  // 71
    "MoonPhase",  // 72
    "MercuryAz",  // 73
    "MercuryAlt",  // 74
    "VenusAz",  // 75
    "VenusAlt",  // 76
    "MarsAz",  // 77
    "MarsAlt",  // 78
    "JupiterAz",  // 79
    "JupiterAlt",  // 80
    "SaturnAz",  // 81
    "SaturnAlt",  // 82
    "UranusAz",  // 83
    "UranusAlt",  // 84
    "NeptuneAz",  // 85
    "NeptuneAlt",  // 86
    "ADMPlex0",  // 87
    "MappedValue",  // 88
    "SDCARDInserted",  // 89
    "SDCARDMounted",  // 90
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
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SWITCHLINK) && (Fi==0)) {
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
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_LOCALTIME)) {
        tmp_x = atol(satioData.padded_local_time_HHMMSS);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_WEEKDAY)) {
        tmp_x = satioData.local_wday;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_DATEDAYX)) {
        tmp_x = satioData.local_mday;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_DATEMONTHX)) {
        tmp_x = satioData.local_month;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_DATEYEARX)) {
        tmp_x = satioData.local_year;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_DEGLAT)) {
        tmp_x = satioData.degrees_latitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_DEGLON)) {
        tmp_x = satioData.degrees_longitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_INSLAT)) {
        tmp_x = insData.ins_latitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_INSLON)) {
        tmp_x = insData.ins_longitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_INSHEADING)) {
        tmp_x = insData.ins_heading;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_INSALTITUDE)) {
        tmp_x = insData.ins_altitude;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_POSSTATUSGNGGA)) {
        tmp_x = atoi(gnggaData.solution_status);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATCOUNT)) {
        tmp_x = atol(gnggaData.satellite_count);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPSPRECISION)) {
        tmp_x = atof(gnggaData.gps_precision_factor);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_ALTGNGGA)) {
        tmp_x = atof(gnggaData.altitude);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GSPEEDGNRMC)) {
        tmp_x = atof(gnrmcData.ground_speed);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEADINGGNRMC)) {
        tmp_x = strtod(gnrmcData.ground_heading, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_LFLAGGPATT)) {
        tmp_x = atoi(gpattData.line_flag);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SFLAGGPATT)) {
        tmp_x = atoi(gpattData.static_flag);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_RSFLAGGPATT)) {
        tmp_x = atoi(gpattData.run_state_flag);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_INSGPATT)) {
        tmp_x = atoi(gpattData.ins);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SPEEDNUMGPATT)) {
        tmp_x = atoi(gpattData.speed_num);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MILEAGEGPATT)) {
        tmp_x = strtod(gpattData.mileage, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GSTDATAGPATT)) {
        tmp_x = strtod(gpattData.gst_data, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_YAWGPATT)) {
        tmp_x = strtod(gpattData.yaw, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_ROLLGPATT)) {
        tmp_x = strtod(gpattData.roll, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_PITCHGPATT)) {
        tmp_x = strtod(gpattData.pitch, &xyzptr);
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNGGAVALIDCS)) {
        tmp_x = gnggaData.valid_checksum;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMCVALIDCS)) {
        tmp_x = gnrmcData.valid_checksum;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATTVALIDCS)) {
        tmp_x = gpattData.valid_checksum;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNGGAVALIDCD)) {
        tmp_x = (long)gnggaData.bad_element_count;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GNRMCVALIDCD)) {
        tmp_x = (long)gnrmcData.bad_element_count;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GPATTVALIDCD)) {
        tmp_x = (long)gnrmcData.bad_element_count;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ACCX)) {
        tmp_x = gyroData.gyro_0_acc_x;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ACCY)) {
        tmp_x = gyroData.gyro_0_acc_y;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ACCZ)) {
        tmp_x = gyroData.gyro_0_acc_z;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ANGX)) {
        tmp_x = gyroData.gyro_0_ang_x;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ANGY)) {
        tmp_x = gyroData.gyro_0_ang_y;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0ANGZ)) {
        tmp_x = gyroData.gyro_0_ang_z;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0MAGX)) {
        tmp_x = gyroData.gyro_0_mag_x;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0MAGY)) {
        tmp_x = gyroData.gyro_0_mag_y;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0MAGZ)) {
        tmp_x = gyroData.gyro_0_mag_z;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0GYROX)) {
        tmp_x = gyroData.gyro_0_gyr_x;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0GYROY)) {
        tmp_x = gyroData.gyro_0_gyr_y;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_GYRO0GYROZ)) {
        tmp_x = gyroData.gyro_0_gyr_z;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_METEORS)) {
        tmp_x = meteor_shower_warning_system
                [(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]]
                [(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]];
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SUNAZ)) {
        tmp_x = siderealPlanetData.sun_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SUNALT)) {
        tmp_x = siderealPlanetData.sun_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MOONAZ)) {
        tmp_x = siderealPlanetData.moon_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MOONALT)) {
        tmp_x = siderealPlanetData.moon_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MOONPHASE)) {
        tmp_x = siderealPlanetData.moon_p;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MERCURYAZ)) {
        tmp_x = siderealPlanetData.mercury_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MERCURYALT)) {
        tmp_x = siderealPlanetData.mercury_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_VENUSAZ)) {
        tmp_x = siderealPlanetData.venus_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_VENUSALT)) {
        tmp_x = siderealPlanetData.venus_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MARSAZ)) {
        tmp_x = siderealPlanetData.mars_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MARSALT)) {
        tmp_x = siderealPlanetData.mars_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_JUPITERAZ)) {
        tmp_x = siderealPlanetData.jupiter_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_JUPITERALT)) {
        tmp_x = siderealPlanetData.jupiter_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATURNAZ)) {
        tmp_x = siderealPlanetData.saturn_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SATURNALT)) {
        tmp_x = siderealPlanetData.saturn_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_URANUSAZ)) {
        tmp_x = siderealPlanetData.uranus_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_URANUSALT)) {
        tmp_x = siderealPlanetData.uranus_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNEAZ)) {
        tmp_x = siderealPlanetData.uranus_az;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_NEPTUNEALT)) {
        tmp_x = siderealPlanetData.uranus_alt;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_ADMPLEX0)) {
        tmp_x = multiplexerData.ADMPLEX_0_DATA
                [(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]];
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MAPPEDVALUE)) {
        tmp_x = mappingData.mapped_value
                [0][(int)matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Z]];
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SDCARDINSERTED)) {
        tmp_x = sdcardData.sdcard_inserted;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_SDCARDMOUNTED)) {
        tmp_x = sdcardData.sdcard_mounted;
        tmp_y = 0;
        tmp_z = 0;
        handle_digit=true;
      }
      // Handle digits.
      if (handle_digit==true) {
        if ((matrixData.matrix_switch_operator_index[0][Mi][Fi]==INDEX_MATRIX_SWITCH_OPERATOR_OVER)) {
          if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false) {
            tmp_matrix[Fi]=check_over_true(tmp_x,
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
          }
          else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true) {
            tmp_matrix[Fi]=check_over_false(tmp_x,
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
          }
        }
        else if ((matrixData.matrix_switch_operator_index[0][Mi][Fi]==INDEX_MATRIX_SWITCH_OPERATOR_UNDER)) {
          if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false) {
            tmp_matrix[Fi]=check_under_true(tmp_x,
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
          }
          else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true) {
            tmp_matrix[Fi]=check_under_false(tmp_x,
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
          }
        }
        else if ((matrixData.matrix_switch_operator_index[0][Mi][Fi]==INDEX_MATRIX_SWITCH_OPERATOR_EQUAL)) {
          if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false) {
            tmp_matrix[Fi]=check_equal_true(tmp_x,
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
          }
          else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true) {
            tmp_matrix[Fi]=check_equal_false(tmp_x,
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X]);
          }
        }
        else if ((matrixData.matrix_switch_operator_index[0][Mi][Fi]==INDEX_MATRIX_SWITCH_OPERATOR_RANGE)) {
          if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==false) {
            tmp_matrix[Fi]=check_ge_and_le_true(tmp_x,
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X],
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]);
          }
          else if (matrixData.matrix_switch_inverted_logic[0][Mi][Fi]==true) {
            tmp_matrix[Fi]=check_ge_and_le_false(tmp_x,
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_X],
            matrixData.matrix_function_xyz[0][Mi][Fi][INDEX_MATRIX_FUNTION_Y]);
          }
        }
      }
      // Check chars.
      else {
        if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNGGANORTH))
          {temp_string_x="N"; temp_string_y=String(gnggaData.latitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNGGASOUTH))
          {temp_string_x="S"; temp_string_y=String(gnggaData.latitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNGGAEAST))
          {temp_string_x="E"; temp_string_y=String(gnggaData.longitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNGGAWEST))
          {temp_string_x="W"; temp_string_y=String(gnggaData.longitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_POSSTATUSGNRMCA))
          {temp_string_x="A"; temp_string_y=String(gnrmcData.positioning_status); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_POSSTATUSGNRMCV))
          {temp_string_x="V"; temp_string_y=String(gnrmcData.positioning_status); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MODEGNRMCA))
          {temp_string_x="A"; temp_string_y=String(gnrmcData.mode_indication); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MODEGNRMCD))
          {temp_string_x="D"; temp_string_y=String(gnrmcData.mode_indication); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MODEGNRMCE))
          {temp_string_x="E"; temp_string_y=String(gnrmcData.mode_indication); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_MODEGNRMCN))
          {temp_string_x="N"; temp_string_y=String(gnrmcData.mode_indication); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNRMCNORTH))
          {temp_string_x="N"; temp_string_y=String(gnrmcData.latitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNRMCSOUTH))
          {temp_string_x="S"; temp_string_y=String(gnrmcData.latitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNRMCEAST))
          {temp_string_x="E"; temp_string_y=String(gnrmcData.longitude_hemisphere); handle_char=true;}
        else if ((matrixData.matrix_function[0][Mi][Fi]==INDEX_MATRIX_SWITCH_FUNCTION_HEMIGNRMCWEST))
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

    /**
     * Summarize results per switch.
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
    matrixData.switch_intention[0][Mi]=false; // switch intention default false
    if (matrixData.computer_assist[0][Mi]==true) {matrixData.switch_intention[0][Mi]=final_bool;}
    matrixData.computer_intention[0][Mi]=final_bool; // computer intention always set
    // 1 : Check state changed before making write required
    if (matrixData.prev_switch_intention[0][Mi]!=matrixData.switch_intention[0][Mi]) {
      matrixData.matrix_switch_write_required[0][Mi]=true;
      matrixData.prev_switch_intention[0][Mi]=matrixData.switch_intention[0][Mi];
    }
    // 2 : Check output value changed before making write required
    if (final_bool==true) {
      if ( (matrixData.output_value[0][Mi] > matrixData.prev_output_value[0][Mi] + matrixData.flux_value[0][Mi]) || 
           (matrixData.output_value[0][Mi] < matrixData.prev_output_value[0][Mi] - matrixData.flux_value[0][Mi]))
      {
        matrixData.prev_output_value[0][Mi]=matrixData.output_value[0][Mi];
        matrixData.matrix_switch_write_required[0][Mi]=true;
        matrixData.prev_switch_intention[0][Mi]=matrixData.switch_intention[0][Mi];
      }
    }

  } // End switch iteration
  return true;
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
}

void setAllMatrixSwitchesStateFalse() {
  for (int i=0; i<MAX_MATRIX_SWITCHES; i++) {matrixData.switch_intention[0][i]=false;}
}

void setAllMatrixSwitchesStateTrue() {
  for (int i=0; i<MAX_MATRIX_SWITCHES; i++) {matrixData.switch_intention[0][i]=true;}
}

struct I2CLinkStruct {
  char * token;
  int i_token=0;
  byte OUTPUT_BUFFER[MAX_IIC_BUFFER_SIZE]; // bytes to be sent
  char INPUT_BUFFER[MAX_IIC_BUFFER_SIZE];  // chars received
  char TMP_BUFFER_0[MAX_IIC_BUFFER_SIZE];  // chars of bytes to be sent
  char TMP_BUFFER_1[MAX_IIC_BUFFER_SIZE];  // some space for type conversions
};
I2CLinkStruct I2CLink;

void writeI2C(int I2C_Address) {
  // Compile bytes array.
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  for (byte i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++)
    {I2CLink.OUTPUT_BUFFER[i]=(byte)I2CLink.TMP_BUFFER_0[i];}
  // Begin.
  Wire.beginTransmission(I2C_Address);
  // Write bytes array.
  Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
  // End.
  Wire.endTransmission();
}

bool portControllerWriteRequired(int idx) {
  if (matrixData.matrix_switch_write_required[0][idx]==true)
    {matrixData.matrix_switch_write_required[0][idx]=false; return true;}
  return false;
}

void writePortControllerM0(void) {
  // Tag.
  memset(I2CLink.TMP_BUFFER_0, 0, sizeof(I2CLink.TMP_BUFFER_0));
  strcpy(I2CLink.TMP_BUFFER_0, "M0");
  // Write instruction.
  writeI2C(I2C_ADDR_PORTCONTROLLER_0);
  systemData.i_count_port_controller++;
}

void writePortControllerM1(void) {
  for (int Mi=0; Mi<MAX_MATRIX_SWITCHES; Mi++) {
    // Check for change.
    if (portControllerWriteRequired(Mi)==true) {
      // Tag.
      memset(I2CLink.TMP_BUFFER_0, 0, sizeof(I2CLink.TMP_BUFFER_0));
      strcpy(I2CLink.TMP_BUFFER_0, "M1,");
      // Index.
      memset(I2CLink.TMP_BUFFER_1, 0, sizeof(I2CLink.TMP_BUFFER_1));
      itoa(Mi, I2CLink.TMP_BUFFER_1, 10);
      strcat(I2CLink.TMP_BUFFER_0, I2CLink.TMP_BUFFER_1);
      strcat(I2CLink.TMP_BUFFER_0, ",");
      // Port.
      memset(I2CLink.TMP_BUFFER_1, 0, sizeof(I2CLink.TMP_BUFFER_1));
      itoa(matrixData.matrix_port_map[0][Mi], I2CLink.TMP_BUFFER_1, 10);
      strcat(I2CLink.TMP_BUFFER_0, I2CLink.TMP_BUFFER_1);
      strcat(I2CLink.TMP_BUFFER_0, ",");
      // Output value.
      memset(I2CLink.TMP_BUFFER_1, 0, sizeof(I2CLink.TMP_BUFFER_1));
      // Serial.println(matrixData.output_value[0][Mi]);
      if (matrixData.computer_assist[0][Mi]==true)
        {ltoa(matrixData.output_value[0][Mi], I2CLink.TMP_BUFFER_1, 10);}
      else {ltoa(matrixData.override_output_value[0][Mi], I2CLink.TMP_BUFFER_1, 10);}
      strcat(I2CLink.TMP_BUFFER_0, I2CLink.TMP_BUFFER_1);
      strcat(I2CLink.TMP_BUFFER_0, ",");
      // Modulation time.
      strcat(I2CLink.TMP_BUFFER_0, String(matrixData.output_pwm[0][Mi][INDEX_MATRIX_MOD_0]).c_str());
      strcat(I2CLink.TMP_BUFFER_0, ",");
      strcat(I2CLink.TMP_BUFFER_0, String(matrixData.output_pwm[0][Mi][INDEX_MATRIX_MOD_1]).c_str());
      strcat(I2CLink.TMP_BUFFER_0, ",");
      // Write instruction.
      writeI2C(I2C_ADDR_PORTCONTROLLER_0);
      // Debug.
      // Serial.println("[SND] " + String(I2CLink.TMP_BUFFER_0));
    }
  }
  systemData.i_count_port_controller++;
}

void setOutputValues(void) {
  for (int Mi=0; Mi<MAX_MATRIX_SWITCHES; Mi++) {
    // A : Default output value: switch intention (digital 0/1).
    if (matrixData.output_mode[0][Mi]==0) {
      matrixData.output_value[0][Mi]=matrixData.switch_intention[0][Mi];
    }
    // B : Mapped values.
    else if ((matrixData.output_mode[0][Mi]==1)) {
      if ( (mappingData.mapped_value[0][Mi] > matrixData.output_value[0][Mi] + matrixData.flux_value[0][Mi]) || 
           (mappingData.mapped_value[0][Mi] < matrixData.output_value[0][Mi] - matrixData.flux_value[0][Mi]))
      {
        matrixData.output_value[0][Mi]=mappingData.mapped_value[0][mappingData.index_mapped_value[0][Mi]];
      }
    }
    // A : Update and write (passthrough).
    if (matrixData.matrix_function[0][Mi][0]==0) {
      if (matrixData.output_value[0][Mi]!=matrixData.prev_output_value[0][Mi])
        {matrixData.prev_output_value[0][Mi]=matrixData.output_value[0][Mi];
         matrixData.matrix_switch_write_required[0][Mi]=true;}
    }
    // B : Just update & let matrix passthrough if required.
    else {matrixData.output_value[0][Mi];}
  }
}