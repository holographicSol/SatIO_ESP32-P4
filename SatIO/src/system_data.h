/*
    System Data. Written by Benjamin Jack Cullen.
*/

#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "config.h"

/**
 * @struct System data containing flags, counters, and statistics for system monitoring and control.
 */
struct systemStruct {
  bool interval_breach_gps;
  bool interval_breach_ins;
  bool interval_breach_gyro_0;
  bool interval_breach_mplex_0;
  bool interval_breach_matrix;
  bool interval_breach_track_planets;
  bool interval_breach_logging;
  bool interval_breach_1_second_output;
  bool interval_breach_1_second;

  bool debug;
  bool output_stat;
  bool output_stat_v;
  bool output_stat_vv;
  bool serial_command;
  bool logging_enabled;

  bool output_satio_all;
  bool output_satio_enabled;
  bool output_gngga_enabled;
  bool output_gnrmc_enabled;
  bool output_gpatt_enabled;
  bool output_ins_enabled;
  bool output_matrix_enabled;
  bool output_input_portcontroller;
  bool output_config_matrix_enabled;
  bool output_config_mapping_enabled;
  bool output_admplex0_enabled;
  bool output_gyro_0_enabled;
  bool output_sun_enabled;
  bool output_moon_enabled;
  bool output_mercury_enabled;
  bool output_venus_enabled;
  bool output_mars_enabled;
  bool output_jupiter_enabled;
  bool output_saturn_enabled;
  bool output_uranus_enabled;
  bool output_neptune_enabled;
  bool output_meteors_enabled;

  // Counters
  int32_t mainLoopTimeTaken;
  int32_t mainLoopTimeStart;
  int32_t mainLoopTimeTakenMax;
  int32_t uptime_seconds;
  int32_t interval_timer_1_second;

  int32_t i_count_read_gps;
  int32_t prev_i_count_read_gps;

  int32_t i_count_read_ins;
  int32_t prev_i_count_read_ins;

  int32_t i_count_read_gyro_0;
  int32_t prev_i_count_read_gyro_0;

  int32_t i_count_read_mplex_0;
  int32_t prev_i_count_read_mplex_0;

  int32_t i_count_matrix;
  int32_t prev_i_count_matrix;

  int32_t i_count_portcontroller_input;
  int32_t prev_i_count_portcontroller_input;

  int32_t i_count_port_controller_output;
  int32_t prev_i_count_port_controller_output;

  int32_t i_count_track_planets;
  int32_t prev_i_count_track_planets;

  int32_t i_count_read_serial_commands;
  int32_t prev_i_count_read_serial_commands;

  int32_t i_count_logging;
  int32_t prev_i_count_logging;

  int32_t i_count_display;
  int32_t prev_i_count_display;

  int32_t loops_a_second;
  int32_t total_loops_a_second;

  // Additional totals (from intervalBreach1Second)
  int32_t total_gps;
  int32_t total_ins;
  int32_t total_gyro_0;
  int32_t total_mplex_0;
  int32_t total_matrix;
  int32_t total_universe;
  int32_t total_infocmd;
  int32_t total_portcontroller_output;
  int32_t total_portcontroller_input;
  int32_t total_display;
};
extern struct systemStruct systemData;

/**
 * Restores the system to default configuration values.
 */
void restore_system_defaults(void);

#endif