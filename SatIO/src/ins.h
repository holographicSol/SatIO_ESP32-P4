/*
    INS Library. Written by Benjamin Jack Cullen.

    Estimate location using gyro data and or dead reckoning.
*/

#ifndef INS_H
#define INS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/**
 * @struct InsData
 */
struct InsData {
  // Master INS enable (INS will still continue to update initialization level)
  bool   INS_ENABLED;

  // Set required minimum GPS precision for INS.
  double INS_REQ_GPS_PRECISION;

  // Set required minimum speed for INS.
  double INS_REQ_MIN_SPEED;

  // Set maximum allowed difference between GPS heading and gyro heading.
  double INS_REQ_HEADING_RANGE_DIFF;

  // Determines INS uses gyro or GPS heading
  bool   INS_USE_GYRO_HEADING;

  // This flag is set on successfull mode 2 initialization
  bool   INS_FORCED_ON_FLAG;

  /** INS Moded
   * 
   *  0 : Disabled :
   *        - continually set initialization flag (with INS_ENABLED false).
   *        - only set ins data as gps data.
   *        - no estimation/precision required.
   * 
   *  1 : Dynamic :
   *        - continually set initialization flag (with INS_ENABLED true).
   *        - set ins data as gps data every 100ms and estimate in between.
   *        - sane normal ins use.
   * 
   *  2 : Hold the Line (forced remain on) :
   *        - continually set initialization flag (with INS_ENABLED true).
   *        - set ins data as gps data once then pure ins until further notice.
   *        - for spinning out of the death star.
  */
  int    INS_MODE;
  char   char_ins_mode[MAX_INS_MODE][MAX_GLOBAL_ELEMENT_SIZE] =
         {"INS OFF", "INS DYNAMIC", "INS FORCED ON"};
  
  // Current INS initialization level
  int    INS_INITIALIZATION_FLAG;
  int    tmp_ins_initialization_flag;

  // Current INS data
  double ins_latitude; // GPS/estimated latitude in degrees
  double ins_longitude; // GPS/estimated longitude in degrees
  double ins_altitude; // GPS/estimated altitude in meters
  double ins_heading; // GPS/estimated heading in degrees
  int64_t ins_dt_prev;
};
extern struct InsData insData;

/**
 * Estimates current position using current gyro and gps data.
 * @param pitch Gyro pitch
 * @param yaw Gyro yaw
 * @param gps_ground_heading GPS ground heading in degrees
 * @param gps_ground_speed GPS ground speed
 * @param dt Current unix uS datetime
 * @return Returns bool
 */
bool ins_estimate_position(double pitch,
                           double yaw,
                           double gps_ground_heading,
                           double gps_ground_speed,
                           int64_t dt);

/**
 * Compares angle1 to angle2 to within a specified range.
 * @param angle1 Angle that will be compared to angle2
 * @param angle2 Angle that will be compared to angle1
 * @param range Preferred range difference
 * @return Returns true if angles compare within range difference
 */
bool angles_are_close(double angle1,
                      double angle2,
                      double range);

/**
 * Attempts to initialize the INS according to initialization rules.
 * @param gps_precision_factor GPS precision factor
 * @param gps_ground_heading GPS ground heading in degrees
 * @param gps_ground_speed GPS ground speed
 * @param gyro_heading Gyro heading in degrees
 * @return Returns None
 */
void ins_init(double gps_precision_factor,
              double gps_ground_heading,
              double gps_ground_speed,
              double gyro_heading);

/**
 * Sets the INS data as either GPS data or estimated position.
 * INS mode determines how and when the INS data should be set.
 * @param gps_latitude GPS latitude degrees
 * @param gps_longitude GPS longitude degrees
 * @param gps_altitude GPS Altitude in meters
 * @param gps_ground_heading GPS ground heading in degrees
 * @param gps_ground_speed GPS ground speed
 * @param gps_precision_factor GPS precision factor
 * @param gyro_heading Gyro heading in degrees
 * @return Returns None
 */
void set_ins(double gps_latitude,
             double gps_longitude,
             double gps_altitude,
             double gps_ground_heading,
             double gps_ground_speed,
             double gps_precision_factor,
             double gyro_heading);

#ifdef __cplusplus
}
#endif

#endif