/*
    System Data. Written by Benjamin Jack Cullen.
*/

#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "config.h"

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    SYSTEM TIME
// ------------------------------------------------------------------------------------------------------------------------------

// Bitpacked Struct
struct systemStruct {
  // Packed breach flags (6 bits)
  unsigned interval_breach_gps : 1;
  unsigned interval_breach_ins : 1;
  unsigned interval_breach_gyro_0 : 1;
  unsigned interval_breach_mplex : 1;
  unsigned interval_breach_matrix : 1;
  unsigned interval_breach_track_planets : 1;
  unsigned interval_breach_1_second : 1;

  // Packed debug/config flags (5 bits)
  unsigned debug : 1;
  unsigned output_stat : 1;
  unsigned output_stat_v : 1;
  unsigned output_stat_vv : 1;
  unsigned serial_command : 1;

  // Packed output enables (18 bits for the main ones + satio_all = 19 bits)
  unsigned output_satio_all : 1;
  unsigned output_satio_enabled : 1;
  unsigned output_ins_enabled : 1;
  unsigned output_gngga_enabled : 1;
  unsigned output_gnrmc_enabled : 1;
  unsigned output_gpatt_enabled : 1;
  unsigned output_matrix_enabled : 1;
  unsigned output_admplex0_enabled : 1;
  unsigned output_gyro_0_enabled : 1;
  unsigned output_sun_enabled : 1;
  unsigned output_moon_enabled : 1;
  unsigned output_mercury_enabled : 1;
  unsigned output_venus_enabled : 1;
  unsigned output_mars_enabled : 1;
  unsigned output_jupiter_enabled : 1;
  unsigned output_saturn_enabled : 1;
  unsigned output_uranus_enabled : 1;
  unsigned output_neptune_enabled : 1;
  unsigned output_meteors_enabled : 1;

  // Counters (assuming uint32_t; adjust types as per original)
  unsigned mainLoopTimeTaken : 32;
  unsigned mainLoopTimeStart : 32;
  unsigned mainLoopTimeTakenMax : 32;
  unsigned uptime_seconds : 32;
  unsigned interval_timer_1_second : 32;  // Assuming int32_t

  unsigned i_count_read_gps : 32;
  unsigned prev_i_count_read_gps : 32;

  unsigned i_count_read_ins : 32;
  unsigned prev_i_count_read_ins : 32;

  unsigned i_count_read_gyro_0 : 32;
  unsigned prev_i_count_read_gyro_0 : 32;

  unsigned i_count_read_mplex : 32;
  unsigned prev_i_count_read_mplex : 32;

  unsigned i_count_matrix : 32;
  unsigned prev_i_count_matrix : 32;

  unsigned i_count_port_controller : 32;
  unsigned prev_i_count_port_controller : 32;

  unsigned i_count_track_planets : 32;
  unsigned prev_i_count_track_planets : 32;

  unsigned i_count_read_serial_commands : 32;
  unsigned prev_i_count_read_serial_commands : 32;

  unsigned loops_a_second : 32;
  unsigned total_loops_a_second : 32;

  // Additional totals (from intervalBreach1Second)
  unsigned total_gps : 32;
  unsigned total_ins : 32;
  unsigned total_gyro : 32;
  unsigned total_mplex : 32;
  unsigned total_matrix : 32;
  unsigned total_portcon : 32;
  unsigned total_universe : 32;
  unsigned total_infocmd : 32;
};
extern struct systemStruct systemData;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
void systemIntervalCheck(void);
void intervalBreach1Second(void);
void restore_system_defaults(void);

#ifdef __cplusplus
}
#endif

#endif