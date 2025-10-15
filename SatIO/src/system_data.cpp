/*
    System Data. Written by Benjamin Jack Cullen.
*/

#include "system_data.h"
#include <Arduino.h>
#include "satio.h"
#include "serial_infocmd.h"

/**
 * @struct System data containing flags, counters, and statistics for system monitoring and control.
 * @warning This struct is bitpacked.
 */
struct systemStruct systemData = {
  .interval_breach_gps = 0,
  .interval_breach_ins = 0,
  .interval_breach_gyro_0 = 0,
  .interval_breach_mplex = 0,
  .interval_breach_matrix = 0,
  .interval_breach_track_planets = 0,
  .interval_breach_1_second = 0,

  .debug = 0,
  .output_stat = 0,
  .output_stat_v = 0,
  .output_stat_vv = 0,
  .serial_command = 1,

  .output_satio_all = 0,
  .output_satio_enabled = 0,
  .output_ins_enabled = 0,
  .output_gngga_enabled = 0,
  .output_gnrmc_enabled = 0,
  .output_gpatt_enabled = 0,
  .output_matrix_enabled = 0,
  .output_admplex0_enabled = 0,
  .output_gyro_0_enabled = 0,
  .output_sun_enabled = 0,
  .output_moon_enabled = 0,
  .output_mercury_enabled = 0,
  .output_venus_enabled = 0,
  .output_mars_enabled = 0,
  .output_jupiter_enabled = 0,
  .output_saturn_enabled = 0, 
  .output_uranus_enabled = 0, 
  .output_neptune_enabled = 0,
  .output_meteors_enabled = 0,

  .mainLoopTimeTaken = 0,
  .mainLoopTimeStart = 0,
  .mainLoopTimeTakenMax = 0,
  .uptime_seconds = 0,
  .interval_timer_1_second = 0,

  .i_count_read_gps = 0,
  .prev_i_count_read_gps = 0,

  .i_count_read_ins = 0,
  .prev_i_count_read_ins = 0,

  .i_count_read_gyro_0 = 0,
  .prev_i_count_read_gyro_0 = 0,

  .i_count_read_mplex = 0,
  .prev_i_count_read_mplex = 0,

  .i_count_matrix = 0,
  .prev_i_count_matrix = 0,

  .i_count_port_controller = 0,
  .prev_i_count_port_controller = 0,

  .i_count_track_planets = 0,
  .prev_i_count_track_planets = 0,

  .i_count_read_serial_commands = 0,
  .prev_i_count_read_serial_commands = 0,

  .loops_a_second = 0,
  .total_loops_a_second = 0,

  .total_gps = 0,
  .total_ins = 0,
  .total_gyro = 0,
  .total_mplex = 0,
  .total_matrix = 0,
  .total_portcon = 0,
  .total_universe = 0,
  .total_infocmd = 0,
};

int64_t prev_tv_sec;

void systemIntervalCheck(void) {
  systemData.interval_breach_1_second = 0; // set default
  // Second interval: ensure safe by detecting a bad prev_tv_sec.
  if (prev_tv_sec > tv_now.tv_sec || prev_tv_sec < tv_now.tv_sec - 2) {
    Serial.println("correcting prev_tv_sec");
    prev_tv_sec = tv_now.tv_sec - 1;
    systemData.interval_breach_1_second = 1;
  }
  // Second interval: check using system time.
  if (tv_now.tv_sec - prev_tv_sec >= 1) {
    prev_tv_sec = tv_now.tv_sec;
    systemData.interval_breach_1_second = 1;
  }
}

void intervalBreach1Second(void) {
  if (systemData.interval_breach_1_second) {
    // store system time
    storeLocalTime();
    // store rtc time
    storeRTCTime();
    // set loop counter
    systemData.total_loops_a_second = systemData.loops_a_second;
    systemData.loops_a_second = 0;
    // set gps counters
    systemData.total_gps = systemData.i_count_read_gps;
    systemData.i_count_read_gps = 0;
    // set ins counters
    systemData.total_ins = systemData.i_count_read_ins;
    systemData.i_count_read_ins = 0;
    // set gyro counters
    systemData.total_gyro = systemData.i_count_read_gyro_0;
    systemData.i_count_read_gyro_0 = 0;
    // set mplex counters
    systemData.total_mplex = systemData.i_count_read_mplex;
    systemData.i_count_read_mplex = 0;
    // set mplex counters
    systemData.total_matrix = systemData.i_count_matrix;
    systemData.i_count_matrix = 0;
    // set mplex counters
    systemData.total_portcon = systemData.i_count_port_controller;
    systemData.i_count_port_controller = 0;
    // set mplex counters
    systemData.total_universe = systemData.i_count_track_planets;
    systemData.i_count_track_planets = 0;
    // set mplex counters
    systemData.total_infocmd = systemData.i_count_read_serial_commands;
    systemData.i_count_read_serial_commands = 0;
    // set second flags
    systemData.interval_breach_track_planets = 1;
    // set uptime
    systemData.uptime_seconds++;
    if (systemData.uptime_seconds >= LONG_MAX - 2)
      {systemData.uptime_seconds = 0;
        Serial.println("[reset uptime_seconds] " + String(systemData.uptime_seconds));
      }
  }
}

void restore_system_defaults(void) {
  systemData.debug = 0;
  systemData.output_satio_all = 0;
  systemData.output_satio_enabled = 0; 
  systemData.output_ins_enabled = 0;
  systemData.output_gngga_enabled = 0;
  systemData.output_gnrmc_enabled = 0;
  systemData.output_gpatt_enabled = 0;
  systemData.output_matrix_enabled = 0;
  systemData.output_admplex0_enabled = 0;
  systemData.output_gyro_0_enabled = 0;
  systemData.output_sun_enabled = 0; 
  systemData.output_moon_enabled = 0;
  systemData.output_mercury_enabled = 0;
  systemData.output_venus_enabled = 0;
  systemData.output_mars_enabled = 0;
  systemData.output_jupiter_enabled = 0;
  systemData.output_saturn_enabled = 0; 
  systemData.output_uranus_enabled = 0;
  systemData.output_neptune_enabled = 0;
  systemData.output_meteors_enabled = 0;
}