/*
    System Data. Written by Benjamin Jack Cullen.
*/

#include "system_data.h"
#include <Arduino.h>
#include "satio.h"
#include "serial_infocmd.h"

// Bitpacking in struct
struct systemStruct systemData = {
  // Breach flags default to 0 (false)
  .interval_breach_gps = 0,
  .interval_breach_ins = 0,
  .interval_breach_gyro_0 = 0,
  .interval_breach_mplex = 0,
  .interval_breach_matrix = 0,
  .interval_breach_track_planets = 0,
  .interval_breach_1_second = 0,

  .debug = 0,          // print verbose information over serial
  .output_stat = 0,    // prints system information
  .output_stat_v = 0,  // prints system information
  .output_stat_vv = 0, // prints verbose system information (may require fullscreen)
  .serial_command = 1,  // allows commands to be received over serial (strongly recommend firmware default false unless you know what you are doing)

  .output_satio_all = 0,        // enables/disables all following output sentences
  .output_satio_enabled = 0,    // enables/disables output SatIO sentence over serial
  .output_ins_enabled = 0,
  .output_gngga_enabled = 0,    // enables/disables output GPS sentence over serial
  .output_gnrmc_enabled = 0,    // enables/disables output GPS sentence over serial
  .output_gpatt_enabled = 0,    // enables/disables output GPS sentence over serial
  .output_matrix_enabled = 0,   // enables/disables output matrix switch active/inactive states sentence over serial
  .output_admplex0_enabled = 0, // enables/disables output of analog/digital multiplexer data sentence over serial
  .output_gyro_0_enabled = 0,   // enables/disables output of gyro data sentence over serial
  .output_sun_enabled = 0,      // enables/disables output sentence over serial
  .output_moon_enabled = 0,     // enables/disables output sentence over serial
  .output_mercury_enabled = 0,  // enables/disables output sentence over serial
  .output_venus_enabled = 0,    // enables/disables output sentence over serial
  .output_mars_enabled = 0,     // enables/disables output sentence over serial
  .output_jupiter_enabled = 0,  // enables/disables output sentence over serial
  .output_saturn_enabled = 0,   // enables/disables output sentence over serial
  .output_uranus_enabled = 0,   // enables/disables output sentence over serial
  .output_neptune_enabled = 0,  // enables/disables output sentence over serial
  .output_meteors_enabled = 0,  // enables/disables output sentence over serial

  // Counters (assuming uint32_t; adjust types as per original)
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

  // Additional totals (from intervalBreach1Second)
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
  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                            COUNTER INTERVALS
  // ----------------------------------------------------------------------------------------------------------------------------
  // Currently used to determine if something has ran successfully so that something can be done efficiently by 'done interval'.
  // ----------------------------------------------------------------------------------------------------------------------------
  // -------------------------------------
  // Default each loop.
  // -------------------------------------
  systemData.interval_breach_1_second = 0;
  // --------------------------------------------------------------
  // Second interval: ensure safe by detecting a bad prev_tv_sec.
  // --------------------------------------------------------------
  if (prev_tv_sec > tv_now.tv_sec || prev_tv_sec < tv_now.tv_sec - 2) {
    Serial.println("correcting prev_tv_sec");
    prev_tv_sec = tv_now.tv_sec - 1;
    systemData.interval_breach_1_second = 1;
  }
  // --------------------------------------------------------------
  // Second interval: check using system time.
  // --------------------------------------------------------------
  if (tv_now.tv_sec - prev_tv_sec >= 1) {
    prev_tv_sec = tv_now.tv_sec;
    systemData.interval_breach_1_second = 1;
  }
}

void intervalBreach1Second(void) {
  if (systemData.interval_breach_1_second) {
    // store system time
    storeLocalTime();
    // Serial.println("unixtime inter: " + String(satioData.local_unixtime_uS)); // ideally this time is as early as possible within each second (<20uS). uncomment to check
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
    if (systemData.uptime_seconds >= LONG_MAX - 2) {systemData.uptime_seconds = 0; Serial.println("[reset uptime_seconds] " + String(systemData.uptime_seconds));}
  }
}

void restore_system_defaults(void) {
  systemData.debug = 0;             // print verbose information over serial
  // systemData.output_stat = 0;    // prints system information
  // systemData.output_stat_v = 0;  // prints system information
  // systemData.output_stat_vv = 0; // prints verbose system information (may require fullscreen)
  // systemData.serial_command = 1; // allows commands to be received over serial (strongly recommend firmware default false unless you know what you are doing)
  systemData.output_satio_all = 0;        // enables/disables all following output sentences
  systemData.output_satio_enabled = 0;    // enables/disables output SatIO sentence over serial
  systemData.output_ins_enabled = 0;
  systemData.output_gngga_enabled = 0;    // enables/disables output GPS sentence over serial
  systemData.output_gnrmc_enabled = 0;    // enables/disables output GPS sentence over serial
  systemData.output_gpatt_enabled = 0;    // enables/disables output GPS sentence over serial
  systemData.output_matrix_enabled = 0;   // enables/disables output matrix switch active/inactive states sentence over serial
  systemData.output_admplex0_enabled = 0; // enables/disables output of analog/digital multiplexer data sentence over serial
  systemData.output_gyro_0_enabled = 0;   // enables/disables output of gyro data sentence over serial
  systemData.output_sun_enabled = 0;      // enables/disables output sentence over serial
  systemData.output_moon_enabled = 0;     // enables/disables output sentence over serial
  systemData.output_mercury_enabled = 0;  // enables/disables output sentence over serial
  systemData.output_venus_enabled = 0;    // enables/disables output sentence over serial
  systemData.output_mars_enabled = 0;     // enables/disables output sentence over serial
  systemData.output_jupiter_enabled = 0;  // enables/disables output sentence over serial
  systemData.output_saturn_enabled = 0;   // enables/disables output sentence over serial
  systemData.output_uranus_enabled = 0;   // enables/disables output sentence over serial
  systemData.output_neptune_enabled = 0;  // enables/disables output sentence over serial
  systemData.output_meteors_enabled = 0;  // enables/disables output sentence over serial
}