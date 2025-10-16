/*
    INS Library. Written by Benjamin Jack Cullen.

    Estimate location using gyro data and or dead reckoning.
*/

#include <ins.h>
#include <Arduino.h>
#include "satio.h"

/**
 * @struct InsData
 */
struct InsData insData = {
  .INS_ENABLED = true,
  .INS_REQ_GPS_PRECISION = 0.5,
  .INS_REQ_MIN_SPEED = 0.3,
  .INS_REQ_HEADING_RANGE_DIFF=0.5,
  .INS_USE_GYRO_HEADING = true,
  .INS_FORCED_ON_FLAG = false,
  .INS_MODE = INS_MODE_DYNAMIC,
  .char_ins_mode = {"INS OFF", "INS DYNAMIC", "INS FORCED ON"},
  .INS_INITIALIZATION_FLAG = INS_INITIALIZATION_FLAG_0,
  .ins_latitude = 0.0,
  .ins_longitude = 0.0,
  .ins_altitude = 0.0,
  .ins_heading = 0.0,
  .ins_dt_prev = 0,
};

bool angles_are_close(double angle1,
                      double angle2,
                      double range) {
  // -------------------------------------------------------------------------------
  // Normalize angles to [0, 360).
  // -------------------------------------------------------------------------------
  angle1 = fmodf(angle1, 360.0f);
  angle2 = fmodf(angle2, 360.0f);
  if (angle1 < 0) angle1 += 360.0f;
  if (angle2 < 0) angle2 += 360.0f;
  // -------------------------------------------------------------------------------
  // Calculate the absolute difference.
  // -------------------------------------------------------------------------------
  double diff = fabsf(angle1 - angle2);
  // -------------------------------------------------------------------------------
  // Consider wraparound (e.g., 0 and 360 are close).
  // -------------------------------------------------------------------------------
  if (diff > 180.0f) {diff = 360.0f - diff;}
  // -------------------------------------------------------------------------------
  // Check if difference is within the specified range.
  // -------------------------------------------------------------------------------
  return diff <= range;
}

void ins_init(double gps_precision_factor,
              double gps_ground_heading,
              double gps_ground_speed,
              double gyro_heading) {
  // -------------------------------------------------------------------------------
  // 0 : Default.
  // -------------------------------------------------------------------------------
  insData.tmp_ins_initialization_flag=0;
  // -------------------------------------------------------------------------------
  // 1 : Check GPS precsion.
  // -------------------------------------------------------------------------------
  if (gps_precision_factor<=insData.INS_REQ_GPS_PRECISION)
    {insData.tmp_ins_initialization_flag++;
    //  Serial.println("INS precision flag: true");
    }
  // -----------------------------------------------------------------------------
  // 2 : Check Speed.
  // -----------------------------------------------------------------------------
  if (gps_ground_speed>=insData.INS_REQ_MIN_SPEED)
  {insData.tmp_ins_initialization_flag++;
  //  Serial.println("INS speed flag: true");
  }
  // ---------------------------------------------------------------------------
  // 3 : Check Heading.
  // ---------------------------------------------------------------------------
  if ((angles_are_close(gps_ground_heading,
      gyro_heading,
      insData.INS_REQ_HEADING_RANGE_DIFF)==true) ||
      (insData.INS_USE_GYRO_HEADING==false))
    {insData.tmp_ins_initialization_flag++;
    //  Serial.println("INS gyro flag: true");
    }
  // -------------------------------------------------------------------------
  // 4 : Check enabled.
  // -------------------------------------------------------------------------
  if (insData.INS_ENABLED==true)
    {insData.tmp_ins_initialization_flag++;
    //  Serial.println("INS enabled: true");
    }
  // -------------------------------------------------------------------------
  // Set INS initialization
  // -------------------------------------------------------------------------
  insData.INS_INITIALIZATION_FLAG=insData.tmp_ins_initialization_flag;
}

void set_ins_data_as_gps(double gps_latitude,
                         double gps_longitude,
                         double gps_altitude,
                         double gps_ground_heading) {
  insData.ins_latitude=gps_latitude;
  insData.ins_longitude=gps_longitude;
  insData.ins_altitude=gps_altitude;
  insData.ins_heading=gps_ground_heading;
}

void set_ins(double gps_latitude,
             double gps_longitude,
             double gps_altitude,
             double gps_ground_heading,
             double gps_ground_speed,
             double gps_precision_factor,
             double gyro_heading) {
  // -------------------------------------------------------------------------------
  // 0 : Off.
  // -------------------------------------------------------------------------------
  if (insData.INS_MODE==0) {
    insData.INS_ENABLED=false;
    set_ins_data_as_gps(gps_latitude, gps_longitude, gps_altitude, gps_ground_heading);
    ins_init(gps_precision_factor, gps_ground_heading, gps_ground_speed, gyro_heading);
  }
  // -------------------------------------------------------------------------------
  // 1 : Dynamic.
  // -------------------------------------------------------------------------------
  else if (insData.INS_MODE==1) {
    insData.INS_ENABLED=true;
    set_ins_data_as_gps(gps_latitude, gps_longitude, gps_altitude, gps_ground_heading);
    ins_init(gps_precision_factor, gps_ground_heading, gps_ground_speed, gyro_heading);
  }
  // -------------------------------------------------------------------------------
  // 2 : Forced.
  // -------------------------------------------------------------------------------
  else if (insData.INS_MODE==2) {
    insData.INS_ENABLED=true;
    // Check initialization flag because INS_FORCED_ON_FLAG overrides.
    if (insData.INS_INITIALIZATION_FLAG==MAX_INS_INITIALIZATION_FLAG &&
        insData.INS_FORCED_ON_FLAG==false) {
      ins_init(gps_precision_factor, gps_ground_heading, gps_ground_speed, gyro_heading);
      // Continue once or try again next time.
      if (insData.INS_FORCED_ON_FLAG==false) {
        set_ins_data_as_gps(gps_latitude, gps_longitude, gps_altitude, gps_ground_heading);
        insData.INS_FORCED_ON_FLAG=true;
      }
    }
  }
}

double temp_lat;
double temp_lon;
bool ins_estimate_position(double pitch,
                           double yaw,
                           double gps_ground_heading,
                           double gps_ground_speed,
                           int64_t dt) {
  // -------------------------------------------------------------------------------
  // uncomment to force values for testing purposes
  // -------------------------------------------------------------------------------
  // pitch=0; // force pitch
  // yaw=90; // force heading
  // gps_ground_speed=500000; // force speed
  // INS_INITIALIZATION_FLAG=MAX_INS_INITIALIZATION_FLAG;

  if (insData.INS_INITIALIZATION_FLAG==MAX_INS_INITIALIZATION_FLAG ||
      insData.INS_FORCED_ON_FLAG==true) {
    // -------------------------------------------------------------------------------
    // Calculate time interval in seconds from microseconds dt.
    // -------------------------------------------------------------------------------
    double dt_interval = (double)(dt - insData.ins_dt_prev) / 1000000.0;
    // Serial.println(dt_interval);
    // -------------------------------------------------------------------------------
    // Ensure positive time interval; fallback to 0.001s if invalid.
    // -------------------------------------------------------------------------------
    if (dt_interval <= 0.0) {dt_interval = 0.001;}
    // -------------------------------------------------------------------------------
    // Normalize yaw to [0, 360°).
    // -------------------------------------------------------------------------------
    while (yaw >= 360.0) yaw -= 360.0;
    while (yaw < 0.0) yaw += 360.0;
    // -------------------------------------------------------------------------------
    // Convert angles to radians.
    // -------------------------------------------------------------------------------
    double yaw_rad = yaw * DEG_TO_RAD;
    double pitch_rad = pitch * DEG_TO_RAD;
    // -------------------------------------------------------------------------------
    // Horizontal and vertical speed components.
    // -------------------------------------------------------------------------------
    double v_horizontal = gps_ground_speed * cos(pitch_rad);
    double v_vertical = gps_ground_speed * sin(pitch_rad);
    // -------------------------------------------------------------------------------
    // Calculate horizontal distance traveled.
    // -------------------------------------------------------------------------------
    double distance_horizontal = v_horizontal * dt_interval;
    // -------------------------------------------------------------------------------
    // North-South and East-West displacements.
    // -------------------------------------------------------------------------------
    double delta_y = distance_horizontal * cos(yaw_rad); // North-South (m)
    double delta_x = distance_horizontal * sin(yaw_rad); // East-West (m)
    // -------------------------------------------------------------------------------
    // Altitude change.
    // -------------------------------------------------------------------------------
    double delta_alt = v_vertical * dt_interval;
    // -------------------------------------------------------------------------------
    // Convert initial latitude to radians.
    // -------------------------------------------------------------------------------
    double lat0_rad = insData.ins_latitude * DEG_TO_RAD;
    // -------------------------------------------------------------------------------
    // Latitude change: Δφ = Δy / R.
    // -------------------------------------------------------------------------------
    double delta_lat = (delta_y / EARTH_MEAN_RADIUS) * RAD_TO_DEG;
    // -------------------------------------------------------------------------------
    // Longitude change: Δλ = Δx / (R * cos(φ₀)).
    // -------------------------------------------------------------------------------
    double delta_lon = (delta_x / (EARTH_MEAN_RADIUS * cos(lat0_rad))) * RAD_TO_DEG;
    // -------------------------------------------------------------------------------
    // normalize and update lat & long.
    // -------------------------------------------------------------------------------
    temp_lat = insData.ins_latitude + delta_lat;
    temp_lon = insData.ins_longitude + delta_lon;
    // -------------------------------------------------------------------------------
    // Normalize latitude to [-90°, 90°], reflecting at poles.
    // -------------------------------------------------------------------------------
    while (temp_lat > 90.0) temp_lat = 180.0 - temp_lat;
    while (temp_lat < -90.0) temp_lat = -180.0 - temp_lat;
    // -------------------------------------------------------------------------------
    // Normalize longitude to [-180°, 180°].
    // -------------------------------------------------------------------------------
    temp_lon = fmod(temp_lon + 180.0, 360.0) - 180.0;
    // -------------------------------------------------------------------------------
    // Update INS data with normalized values.
    // -------------------------------------------------------------------------------
    insData.ins_latitude = temp_lat;
    insData.ins_longitude = temp_lon;
    // -------------------------------------------------------------------------------
    // update other INS data.
    // -------------------------------------------------------------------------------
    insData.ins_altitude = insData.ins_altitude + delta_alt;
    if (insData.INS_USE_GYRO_HEADING==true) {insData.ins_heading=yaw;}
    else {insData.ins_heading=gps_ground_heading;}
    // -------------------------------------------------------------------------------
    // Ensure presvious datetime is set to current datetime.
    // -------------------------------------------------------------------------------
    insData.ins_dt_prev=dt;
    return true;
  }
  return false;
}
