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

// ------------------------------------------------------------------------
// INS Data Structure.
// ------------------------------------------------------------------------
struct InsData {
  double INS_REQ_GPS_PRECISION;
  double INS_REQ_MIN_SPEED;
  int INS_MODE;
  char char_ins_mode[MAX_INS_MODE][MAX_GLOBAL_ELEMENT_SIZE] = {"INS OFF", "INS DYNAMIC", "INS FORCED ON"};
  int INS_INITIALIZATION_FLAG;
  int tmp_ins_initialization_flag;
  bool INS_FORCED_ON_FLAG;
  bool INS_ENABLED;
  bool INS_USE_GYRO_HEADING;
  double INS_REQ_HEADING_RANGE_DIFF;
  double ins_latitude; // Latitude in degrees
  double ins_longitude; // Longitude in degrees
  double ins_altitude; // Altitude in meters
  double ins_heading; // Heading in degrees
  double ins_speed; // Speed in meters per second
  int64_t ins_dt_prev;
};

extern struct InsData insData;

// ------------------------------------------------------------------------
// Function Prototypes.
// ------------------------------------------------------------------------
bool GetINSPosition(double pitch, double yaw, double gps_ground_heading, double gps_ground_speed, int64_t dt);
bool anglesAreClose(double angle1, double angle2, double range);
void setINSInitializationFlag(double gps_precision_factor, double gps_ground_heading, double gps_ground_speed, double gyro_0_ang_z);
void setINS(double gps_latitude, double gps_longitude, double gps_altitude, double gps_ground_heading, double gps_ground_speed, double gps_precision_factor,
    double gyro_0_ang_z);

#ifdef __cplusplus
}
#endif

#endif