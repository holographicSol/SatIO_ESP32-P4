/*
    INS Library. Written by Benjamin Jack Cullen.

    Estimate location using gyro data and or dead reckoning.
*/

#include <ins.h>
#include <Arduino.h>

// ------------------------------------------------------------------------
// Define INS data.
// ------------------------------------------------------------------------
struct InsData insData = {
  .INS_REQ_GPS_PRECISION = 0.5,
  .INS_REQ_MIN_SPEED = 0.3,
  .INS_MODE = INS_MODE_DYNAMIC,
  .char_ins_mode = {"INS OFF", "INS DYNAMIC", "INS FORCED ON"},
  .INS_INITIALIZATION_FLAG = INS_INITIALIZATION_FLAG_0,
  .INS_FORCED_ON_FLAG = false,
  .INS_ENABLED = true,
  .INS_USE_GYRO_HEADING = true,
  .INS_REQ_HEADING_RANGE_DIFF=0.5,
  .ins_latitude = 0.0,
  .ins_longitude = 0.0,
  .ins_altitude = 0.0,
  .ins_heading = 0.0,
  .ins_speed = 0.0,
  .ins_dt_prev = 0,
};

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                      SIMPLE INS (EXPERIMENTAL)
// ------------------------------------------------------------------------------------------------------------------------------
/**
 * basic principle:
 *  INS location/speed/heading primary set = set by gps.
 *  INS location/speed/heading secondary set (in between primary sets).
 *  INS data should intend to be a safe and primary source for positional data 100% of the time.
 * 
 * requirements:
 *    - gyro calibration.
 *    - stable gyro data: data subject to magnetic anomolies/EMI.
 * 
 * imrovements:
 *    - dipole 9 axis gyro and complimenting algorithm to set potentially more stable, accurate gyro data.
 *    - 'real' hunk of junk gyro (not MEMS).
 *    - factor in changes in speed during estimation period.
 * 
 * considerations:
 *    - changes in speed during estimation period may be negligable for many applications where capacity/possibility of accelerating
 *      up/down within estimation time period is relatively low/moderate (moves/moved <= some negligable speed threshold).
 * 

    set ins mode n                              Set INS mode n.
                                                0 : Do not use INS
                                                1 : Dynamic. Estimate position between primary GPS sets.
                                                2 : Hold The Line. Do not allow GPS to set INS until further notice.
    reset ins forced flag                       Manually reset special flag set in INS mode 2.
    set ins heading gyro                        Always use gyro heading for INS heading.
                                                Gyro must be properly calibrated.
    set ins heading gps                         Always use GPS heading for INS heading.
                                                Only recommended if INS does not require changes in heading while estimating.
    set ins gps precision n                     Set required minimum GPS precision for INS.
    set ins min speed n                         Set required minimum speed for INS.
    set ins heading range diff n                Set required max range difference between gyro heading and GPS heading.

    following command arguments recommended only for testing purposes:
    set ins gps precision 1
    set ins min speed 0
    set ins heading range diff 360
    set ins mode 2
*/

double temp_lat;
double temp_lon;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                              ESTIMATE POSITION
// ------------------------------------------------------------------------------------------------------------------------------
bool GetINSPosition(double pitch, double yaw, double gps_ground_heading, double gps_ground_speed, int64_t dt) {

  // uncomment to force values for testing purposes
  // pitch=0; // force pitch
  // yaw=90; // force heading
  // gps_ground_speed=500000; // force speed
  // INS_INITIALIZATION_FLAG=MAX_INS_INITIALIZATION_FLAG;

  // uncomment to condition the INS
  if (insData.INS_INITIALIZATION_FLAG==MAX_INS_INITIALIZATION_FLAG || insData.INS_FORCED_ON_FLAG==true) {

    // ---------------------------------------------------------
    // Calculate time interval in seconds from microseconds dt.
    // ---------------------------------------------------------
    double dt_interval = (double)(dt - insData.ins_dt_prev) / 1000000.0;
    // Serial.println(dt_interval);

    // ---------------------------------------------------------
    // EXPERIMENTAL: INS SPEED.
    // ---------------------------------------------------------
    // loc_point2_ins={ins_latitude, ins_longitude, ins_altitude, satioData.local_unixtime_uS-((uint64_t)dt_interval*1000000)};

    // ---------------------------------------------------------
    // Ensure positive time interval; fallback to 0.001s if invalid.
    // ---------------------------------------------------------
    if (dt_interval <= 0.0) {dt_interval = 0.001;}
    
    // ---------------------------------------------------------
    // Normalize yaw to [0, 360°).
    // ---------------------------------------------------------
    while (yaw >= 360.0) yaw -= 360.0;
    while (yaw < 0.0) yaw += 360.0;
    
    // ---------------------------------------------------------
    // Convert angles to radians.
    // ---------------------------------------------------------
    double yaw_rad = yaw * DEG_TO_RAD;
    double pitch_rad = pitch * DEG_TO_RAD;
    
    // ---------------------------------------------------------
    // Horizontal and vertical speed components.
    // ---------------------------------------------------------
    double v_horizontal = gps_ground_speed * cos(pitch_rad);
    double v_vertical = gps_ground_speed * sin(pitch_rad);
    
    // ---------------------------------------------------------
    // Calculate horizontal distance traveled.
    // ---------------------------------------------------------
    double distance_horizontal = v_horizontal * dt_interval;
    
    // ---------------------------------------------------------
    // North-South and East-West displacements.
    // ---------------------------------------------------------
    double delta_y = distance_horizontal * cos(yaw_rad); // North-South (m)
    double delta_x = distance_horizontal * sin(yaw_rad); // East-West (m)
    
    // ---------------------------------------------------------
    // Altitude change.
    // ---------------------------------------------------------
    double delta_alt = v_vertical * dt_interval;
    
    // ---------------------------------------------------------
    // Convert initial latitude to radians.
    // ---------------------------------------------------------
    double lat0_rad = insData.ins_latitude * DEG_TO_RAD;
    
    // ---------------------------------------------------------
    // Latitude change: Δφ = Δy / R.
    // ---------------------------------------------------------
    double delta_lat = (delta_y / EARTH_MEAN_RADIUS) * RAD_TO_DEG;
    
    // ---------------------------------------------------------
    // Longitude change: Δλ = Δx / (R * cos(φ₀)).
    // ---------------------------------------------------------
    double delta_lon = (delta_x / (EARTH_MEAN_RADIUS * cos(lat0_rad))) * RAD_TO_DEG;

    // ---------------------------------------------------------
    // normalize and update lat & long.
    // ---------------------------------------------------------
    temp_lat = insData.ins_latitude + delta_lat;
    temp_lon = insData.ins_longitude + delta_lon;
    // ---------------------------------------------------------
    // Normalize latitude to [-90°, 90°], reflecting at poles.
    // ---------------------------------------------------------
    while (temp_lat > 90.0) temp_lat = 180.0 - temp_lat;
    while (temp_lat < -90.0) temp_lat = -180.0 - temp_lat;
    // ---------------------------------------------------------
    // Normalize longitude to [-180°, 180°].
    // ---------------------------------------------------------
    temp_lon = fmod(temp_lon + 180.0, 360.0) - 180.0;
    // ---------------------------------------------------------
    // Update INS data with normalized values.
    // ---------------------------------------------------------
    insData.ins_latitude = temp_lat;
    insData.ins_longitude = temp_lon;
    // ---------------------------------------------------------
    // update other INS data.
    // ---------------------------------------------------------
    insData.ins_altitude = insData.ins_altitude + delta_alt;
    if (insData.INS_USE_GYRO_HEADING==true) {insData.ins_heading=yaw;}
    else {insData.ins_heading=gps_ground_heading;}

    // ---------------------------------------------------------
    // EXPERIMENTAL: Update INS SPEED
    // Should approximately reflect actual speed assuming speed
    // does not change during estimation period.
    // Currently only useful for checking INS position estimation
    // until/unless changes in speed are factored into the INS.
    // ---------------------------------------------------------
    // satioData.ins_speed=calculate_speed_from_location_data(loc_point1_ins, loc_point2_ins);
    // satioData.ins_speed=convertSpeedUnits(satioData.ins_speed);
    // loc_point1_ins={satioData.ins_latitude, satioData.ins_longitude, satioData.ins_altitude, satioData.local_unixtime_uS-((uint64_t)dt_interval*1000000)};

    // ---------------------------------------------------------
    // Ensure presvious datetime is set to current datetime.
    // ---------------------------------------------------------
    insData.ins_dt_prev=dt;
    // ---------------------------------------------------------
    // Inform caller of execution.
    // ---------------------------------------------------------
    return true;
  }
  return false;
  // (else dont touch the INS data. INS data remains as GPS data )
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                        ANGLES COMPARE IN RANGE
// ------------------------------------------------------------------------------------------------------------------------------
bool anglesAreClose(double angle1, double angle2, double range) {
  // ---------------------------------------------------------
  // Normalize angles to [0, 360).
  // ---------------------------------------------------------
  angle1 = fmodf(angle1, 360.0f);
  angle2 = fmodf(angle2, 360.0f);
  if (angle1 < 0) angle1 += 360.0f;
  if (angle2 < 0) angle2 += 360.0f;
  // ---------------------------------------------------------
  // Calculate the absolute difference.
  // ---------------------------------------------------------
  double diff = fabsf(angle1 - angle2);
  // ---------------------------------------------------------
  // Consider wraparound (e.g., 0 and 360 are close).
  // ---------------------------------------------------------
  if (diff > 180.0f) {diff = 360.0f - diff;}
  // ---------------------------------------------------------
  // Check if difference is within the specified range.
  // ---------------------------------------------------------
  return diff <= range;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                    SET INS INITIALIZATION FLAG
// ------------------------------------------------------------------------------------------------------------------------------
void setINSInitializationFlag(double gps_precision_factor, double gps_ground_heading, double gps_ground_speed, double gyro_0_ang_z) {
  // ---------------------------------------------------------
  // critical: set ins initialization flag.
  // ---------------------------------------------------------
  /**
    * experimental! values subject to experimentation.
    * condition 0 : gps precision
    * condition 1 : speed
    * condition 2 : gps heading = +- gyro heading
    * condition x : consider further conditions
  */
  // ---------------------------------------------------------
  // 0 : Default.
  // ---------------------------------------------------------
  insData.tmp_ins_initialization_flag=INS_INITIALIZATION_FLAG_0;
  // ---------------------------------------------------------
  // 1 : GPS precsion.
  // ---------------------------------------------------------
  if (gps_precision_factor<=insData.INS_REQ_GPS_PRECISION) {
    insData.tmp_ins_initialization_flag=INS_INITIALIZATION_FLAG_1;
    // -------------------------------------------------------
    // 2 : Speed.
    // -------------------------------------------------------
    if (gps_ground_speed>=insData.INS_REQ_MIN_SPEED) {
      insData.tmp_ins_initialization_flag=INS_INITIALIZATION_FLAG_2;
      // -----------------------------------------------------
      // 3 : Heading.
      // -----------------------------------------------------
      if ((anglesAreClose(gps_ground_heading, gyro_0_ang_z, insData.INS_REQ_HEADING_RANGE_DIFF)==true) || (insData.INS_USE_GYRO_HEADING==false)) {
        insData.tmp_ins_initialization_flag=INS_INITIALIZATION_FLAG_3;
        // ---------------------------------------------------
        // 4 : Check enabled.
        // ---------------------------------------------------
        if (insData.INS_ENABLED==true) {
          insData.tmp_ins_initialization_flag=INS_INITIALIZATION_FLAG_4;
        }
      }
    }
  }
  insData.INS_INITIALIZATION_FLAG=insData.tmp_ins_initialization_flag;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                       SET INS DATA AS GPS DATA
// ------------------------------------------------------------------------------------------------------------------------------
void setINSDataAsGPS(double gps_latitude, double gps_longitude, double gps_altitude, double gps_ground_heading) {
  // ---------------------------------------------------------
  // Critical: Set INS data as GPS data.
  // ---------------------------------------------------------
  insData.ins_latitude=gps_latitude;
  insData.ins_longitude=gps_longitude;
  insData.ins_altitude=gps_altitude;
  insData.ins_heading=gps_ground_heading;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SET INS
// ------------------------------------------------------------------------------------------------------------------------------
void setINS(double gps_latitude, double gps_longitude, double gps_altitude, double gps_ground_heading, double gps_ground_speed, double gps_precision_factor,
    double gyro_0_ang_z) {
  // -------------------------------------------------------------------
  /** EXPERIMENTAL
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
  // -------------------------------------------------------------------
  // 0 : Off.
  // -------------------------------------------------------------------
  if (insData.INS_MODE==0) {
    insData.INS_ENABLED=false;
    setINSDataAsGPS(gps_latitude, gps_longitude, gps_altitude, gps_ground_heading);
    setINSInitializationFlag(gps_precision_factor, gps_ground_heading, gps_ground_speed, gyro_0_ang_z);
  }
  // -------------------------------------------------------------------
  // 1 : Dynamic.
  // -------------------------------------------------------------------
  else if (insData.INS_MODE==1) {
    insData.INS_ENABLED=true;
    setINSDataAsGPS(gps_latitude, gps_longitude, gps_altitude, gps_ground_heading);
    setINSInitializationFlag(gps_precision_factor, gps_ground_heading, gps_ground_speed, gyro_0_ang_z);
  }
  // -------------------------------------------------------------------
  // 2 : Forced.
  // -------------------------------------------------------------------
  else if (insData.INS_MODE==2) {
    insData.INS_ENABLED=true;
    // -----------------------------------------------------------------
    // Check initialization flag because INS_FORCED_ON_FLAG overrides.
    // -----------------------------------------------------------------
    if (insData.INS_INITIALIZATION_FLAG==MAX_INS_INITIALIZATION_FLAG && insData.INS_FORCED_ON_FLAG==false) {
      setINSInitializationFlag(gps_precision_factor, gps_ground_heading, gps_ground_speed, gyro_0_ang_z);
      // -----------------------------------------------------------------
      // Continue once or try again next time.
      // -----------------------------------------------------------------
      if (insData.INS_FORCED_ON_FLAG==false) {
        setINSDataAsGPS(gps_latitude, gps_longitude, gps_altitude, gps_ground_heading);
        insData.INS_FORCED_ON_FLAG=true;
      }
    }
  }
}