/*
    System Data. Written by Benjamin Jack Cullen.
*/

#include "system_data.h"
#include <Arduino.h>
#include "satio.h"
#include "serial_infocmd.h"
#include "ins.h"

/**
 * @struct System data containing flags, counters, and statistics for system monitoring and control.
 * @warning This struct is bitpacked.
 */
struct systemStruct systemData = {
  .interval_breach_gps = false,
  .interval_breach_ins = false,
  .interval_breach_gyro_0 = false,
  .interval_breach_mplex_0 = false,
  .interval_breach_matrix = false,
  .interval_breach_track_planets = false,
  .interval_breach_logging = false,
  .interval_breach_1_second_output = false,
  .interval_breach_1_second = false,

  .debug = false,
  .output_stat = false,
  .output_stat_v = false,
  .output_stat_vv = false,
  .serial_command = true,
  .logging_enabled=false,

  .output_satio_all = false,
  .output_satio_enabled = false,
  .output_gngga_enabled = false,
  .output_gnrmc_enabled = false,
  .output_gpatt_enabled = false,
  .output_ins_enabled = false,
  .output_matrix_enabled = false,
  .output_input_portcontroller = false,
  .output_config_matrix_enabled = false,
  .output_config_mapping_enabled = false,
  .output_admplex0_enabled = false,
  .output_gyro_0_enabled = false,
  .output_sun_enabled = false,
  .output_moon_enabled = false,
  .output_mercury_enabled = false,
  .output_venus_enabled = false,
  .output_mars_enabled = false,
  .output_jupiter_enabled = false,
  .output_saturn_enabled = false,
  .output_uranus_enabled = false, 
  .output_neptune_enabled = false,
  .output_meteors_enabled = false,

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

  .i_count_read_mplex_0 = 0,
  .prev_i_count_read_mplex_0 = 0,

  .i_count_matrix = 0,
  .prev_i_count_matrix = 0,

  .i_count_portcontroller_input = 0,
  .prev_i_count_portcontroller_input = 0,

  .i_count_port_controller_output = 0,
  .prev_i_count_port_controller_output = 0,

  .i_count_track_planets = 0,
  .prev_i_count_track_planets = 0,

  .i_count_read_serial_commands = 0,
  .prev_i_count_read_serial_commands = 0,

  .i_count_logging = 0,
  .prev_i_count_logging = 0,

  .i_count_display = 0,
  .prev_i_count_display = 0,

  .loops_a_second = 0,
  .total_loops_a_second = 0,

  .total_gps = 0,
  .total_ins = 0,
  .total_gyro_0 = 0,
  .total_mplex_0 = 0,
  .total_matrix = 0,
  .total_universe = 0,
  .total_infocmd = 0,
  .total_portcontroller_output = 0,
  .total_portcontroller_input = 0,
  .total_display = 0,
};

void restore_system_defaults(void) {
  systemData.debug = false;
  systemData.output_satio_all = false;
  systemData.output_satio_enabled = false; 
  systemData.output_gngga_enabled = false;
  systemData.output_gnrmc_enabled = false;
  systemData.output_gpatt_enabled = false;
  systemData.output_ins_enabled = false;
  systemData.output_matrix_enabled = false;
  systemData.output_input_portcontroller = false;
  systemData.output_config_matrix_enabled = false;
  systemData.output_config_mapping_enabled = false;
  systemData.output_admplex0_enabled = false;
  systemData.output_gyro_0_enabled = false;
  systemData.output_sun_enabled = false;
  systemData.output_moon_enabled = false;
  systemData.output_mercury_enabled = false;
  systemData.output_venus_enabled = false;
  systemData.output_mars_enabled = false;
  systemData.output_jupiter_enabled = false;
  systemData.output_saturn_enabled = false;
  systemData.output_uranus_enabled = false;
  systemData.output_neptune_enabled = false;
  systemData.output_meteors_enabled = false;

  satioData.utc_second_offset = 0;
  satioData.utc_auto_offset_flag = false;
  satioData.set_time_automatically = true;
  satioData.sync_rtc_immediately_flag = true;
  satioData.coordinate_conversion_mode = COORDINATE_CONVERSION_MODE_GPS;
  satioData.altitude_unit_mode = ALTITUDE_UNIT_MODE_METERS;
  satioData.altitude_conversion_mode = ALTITUDE_CONVERSION_MODE_GPS;
  satioData.speed_unit_mode = SPEED_UNIT_MODE_KTS;
  satioData.speed_conversion_mode = SPEED_CONVERSION_MODE_GPS;
  satioData.ground_heading_mode = GROUND_HEADING_MODE_GPS;

  insData.INS_REQ_GPS_PRECISION = 0.5;
  insData.INS_REQ_MIN_SPEED = 0.1;
  insData.INS_REQ_HEADING_RANGE_DIFF = 1;
  insData.INS_USE_GYRO_HEADING = true;
  insData.INS_MODE = INS_MODE_DYNAMIC;
  
  Serial.println("$SYSTEMNEW");
}