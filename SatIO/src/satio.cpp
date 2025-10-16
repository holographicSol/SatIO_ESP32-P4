/*
    SATIO Library. Written by Benjamin Jack Cullen.

*/

#include "satio.h"
#include <Arduino.h>
#include <RTClib.h>  // https://github.com/adafruit/RTClib
#include "wtgps300p.h"

// ----------------------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------------------
RTC_DS3231 rtc;

struct SATIOStruct satioData = {
    .satio_sentence = {0},
    .coordinate_conversion_mode = 0,
    .char_coordinate_conversion_mode = {"GNGGA", "GNRMC"},
    .latitude_meter = 0.0000100,
    .longitude_meter = 0.0000100,
    .latitude_mile = 0.0000100 * 1609.34,
    .longitude_mile = 0.0000100 * 1609.34,
    .abs_latitude_gngga_0 = 0.0,
    .abs_longitude_gngga_0 = 0.0,
    .abs_latitude_gnrmc_0 = 0.0,
    .abs_longitude_gnrmc_0 = 0.0,
    .temp_latitude_gngga = 0.0,
    .temp_longitude_gngga = 0.0,
    .temp_latitude_gnrmc = 0.0,
    .temp_longitude_gnrmc = 0.0,
    .minutesLat = 0.0,
    .minutesLong = 0.0,
    .secondsLat = 0.0,
    .secondsLong = 0.0,
    .millisecondsLat = 0.0,
    .millisecondsLong = 0.0,
    .degrees_latitude = 0.0,
    .degrees_longitude = 0.0,
    .degreesLat = 0.0,
    .degreesLong = 0.0,
    .speed_gps = 0.0,
    .speed = 0.0,
    .speed_conversion_mode = 0,
    .char_speed_conversion_mode = {"M/S", "MPH", "KPH", "KTS"},
    .ground_heading = {0},
    .altitude = "pending",
    .mileage = "pending",
    .tmp_year_int = 0,
    .tmp_month_int = 0,
    .tmp_day_int = 0,
    .tmp_hour_int = 0,
    .tmp_minute_int = 0,
    .tmp_second_int = 0,
    .tmp_millisecond_int = 0,
    .tmp_year = {0},
    .tmp_month = {0},
    .tmp_day = {0},
    .tmp_hour = {0},
    .tmp_minute = {0},
    .tmp_second = {0},
    .tmp_millisecond = {0},
    .week_day_names = {
        "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday", // todo
    },
    .local_hour = 0,
    .local_minute = 0,
    .local_second = 0,
    .local_year = 2000,
    .local_month = 1,
    .local_mday = 1,
    .local_yday = 1,
    .local_wday = 1,
    .local_wday_name = {0},
    .formatted_local_time = "00:00:00",
    .formatted_local_date = "00/00/00",
    .padded_local_time_HHMMSS = "000000",
    .padded_local_date_DDMMYYYY = "00000000",
    .local_unixtime_uS = 0,
    .rtcsync_hour = 0,
    .rtcsync_minute = 0,
    .rtcsync_second = 0,
    .rtcsync_year = 0,
    .rtcsync_month = 0,
    .rtcsync_day = 0,
    .formatted_rtc_sync_time = "00:00:00",
    .formatted_rtc_sync_date = "00/00/00",
    .padded_rtc_sync_time_HHMMSS = "000000",
    .padded_rtc_sync_date_DDMMYYYY = "00000000",
    .rtcsync_unixtime = 0,
    .rtcsync_latitude = "0.0",
    .rtcsync_longitude = "0.0",
    .rtcsync_altitude = "0.0",
    .rtc_hour = 0,
    .rtc_minute = 0,
    .rtc_second = 0,
    .rtc_year = 2000,
    .rtc_month = 1,
    .rtc_mday = 1,
    .rtc_wday = 1,
    .rtc_wday_name = {0},
    .formatted_rtc_time = "00:00:00",
    .formatted_rtc_date = "00/00/00",
    .padded_rtc_time_HHMMSS = "000000",
    .padded_rtc_date_DDMMYYYY = "00000000",
    .rtc_unixtime = 0,
    .utc_second_offset = 0,
    .utc_auto_offset_flag = false,
    .set_time_automatically = true,
    .set_rtc_datetime_flag = false,
    .sync_rtc_immediately_flag = true, // default true to attempt sync immediately on starttup
};

LocPoint loc_point1_gps = {0.0, 0.0, 0.0, 0};
LocPoint loc_point2_gps = {0.0, 0.0, 0.0, 0};
LocPoint loc_point1_ins = {0.0, 0.0, 0.0, 0};
LocPoint loc_point2_ins = {0.0, 0.0, 0.0, 0};

struct SpeedStruct speedData = {
    .lat1_rad = 0.0,
    .lon1_rad = 0.0,
    .lat2_rad = 0.0,
    .lon2_rad = 0.0,
    .delta_lat = 0.0,
    .delta_lon = 0.0,
    .delta_alt = 0.0,
    .a = 0.0,
    .c = 0.0,
    .distance_2d = 0.0,
    .distance_3d = 0.0,
    .delta_time = 0.0,
    .speed = 0.0
};

// -------------------------------------------------------------------------------
/**
   * @brief Calculates the speed between two GPS points in any direction.
   *
   * This function first calculates the great-circle distance on the Earth's
   * surface using the Haversine formula, then accounts for altitude change
   * to find the total 3D distance. Finally, it divides this distance by the
   * elapsed time to determine the average speed.
   *
   * @param p1 The first GPS point (latitude, longitude, altitude, time).
   * @param p2 The second GPS point.
   * @return The calculated speed in meters per second (m/s).
   * 
   * This function has no Kalman filter.
 */
// -------------------------------------------------------------------------------
 double calculateSpeedFromLocationData(LocPoint p1, LocPoint p2) {
    // -------------------------------------------------------------------------------
    // Convert latitude and longitude from degrees to radians for calculations.
    // -------------------------------------------------------------------------------
    double lat1_rad = p1.latitude * M_PI / 180.0;
    double lon1_rad = p1.longitude * M_PI / 180.0;
    double lat2_rad = p2.latitude * M_PI / 180.0;
    double lon2_rad = p2.longitude * M_PI / 180.0;
    // -------------------------------------------------------------------------------
    // Calculate the change in coordinates.
    // -------------------------------------------------------------------------------
    double delta_lat = lat2_rad - lat1_rad;
    double delta_lon = lon2_rad - lon1_rad;
    // -------------------------------------------------------------------------------
    // Calculate the change in altitude.
    // -------------------------------------------------------------------------------
    double delta_alt = p2.altitude - p1.altitude;
    // -------------------------------------------------------------------------------
    // Haversine formula to calculate the 2D distance.
    // -------------------------------------------------------------------------------
    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    // -------------------------------------------------------------------------------
    // Calculate the 2D distance (great-circle distance).
    // -------------------------------------------------------------------------------
    double distance_2d = EARTH_MEAN_RADIUS * c;
    // -------------------------------------------------------------------------------
    // Calculate the total 3D distance using the altitude change.
    // -------------------------------------------------------------------------------
    double distance_3d = sqrt(distance_2d * distance_2d + delta_alt * delta_alt);
    // -------------------------------------------------------------------------------
    // Calculate the change in time in seconds.
    // -------------------------------------------------------------------------------
    double delta_time = (p2.time - p1.time) / 1000000.0;
    // -------------------------------------------------------------------------------
    // Handle the case of zero time difference to avoid division by zero.
    // -------------------------------------------------------------------------------
    double speed=0;
    if (delta_time == 0.0) {speed=0.0;}
    // -------------------------------------------------------------------------------
    // The result is in meters per second, as distance is in meters and time is in seconds.
    // -------------------------------------------------------------------------------
    else {speed = distance_3d / delta_time;}
    return speed;
}

// -----------------------------------------------------------------------
// convertSpeedUnits.
// -----------------------------------------------------------------------
double convertSpeedUnits(double speed) {
  double new_speed=speed;
  if      (satioData.speed_conversion_mode==SPEED_CONVERSION_MODE_METERS_A_SECOND) {new_speed=speed;}
  else if (satioData.speed_conversion_mode==SPEED_CONVERSION_MODE_MPH) {new_speed=speed*METERS_TO_MPH_RATIO;}
  else if (satioData.speed_conversion_mode==SPEED_CONVERSION_MODE_KPH) {new_speed=speed*METERS_TO_KPH_RATIO;}
  else if (satioData.speed_conversion_mode==SPEED_CONVERSION_MODE_KTS) {new_speed=speed*METERS_TO_KTS_RATIO;}
  return new_speed;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                         CONVERT COORDINTE DATA
// ------------------------------------------------------------------------------------------------------------------------------
void calculateLocation(){
  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                  GNGGA COORDINATE CONVERSION
  // ----------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------
  // Convert GNGGA latitude & longitude strings to decimal degrees and format into hours, minutes, seconds, milliseconds.
  // ----------------------------------------------------------------------------------------------------------------------------
  if (satioData.coordinate_conversion_mode==0) {
    // -----------------------------------------------------------------------------------------
    // Extract absolute latitude value from GNGGA data as decimal degrees.
    // -----------------------------------------------------------------------------------------
    satioData.abs_latitude_gngga_0=atof(String(gnggaData.latitude).c_str());
    // -----------------------------------------------------------------------------------------
    // Store absolute latitude in temporary variable for further processing.
    // -----------------------------------------------------------------------------------------
    satioData.temp_latitude_gngga=satioData.abs_latitude_gngga_0;
    // -----------------------------------------------------------------------------------------
    // Separate the integer degrees value from the fractional part.
    // -----------------------------------------------------------------------------------------
    satioData.degreesLat=trunc(satioData.temp_latitude_gngga / 100);
    // -----------------------------------------------------------------------------------------
    // Calculate minutes and seconds values based on remaining fractional part.
    // -----------------------------------------------------------------------------------------
    satioData.minutesLat=satioData.temp_latitude_gngga - (satioData.degreesLat * 100);
    // -----------------------------------------------------------------------------------------
    // Convert excess fractional part to seconds.
    // -----------------------------------------------------------------------------------------
    satioData.secondsLat=(satioData.minutesLat - trunc(satioData.minutesLat)) * 60;
    // -----------------------------------------------------------------------------------------
    // Convert excess seconds to milliseconds.
    // -----------------------------------------------------------------------------------------
    satioData.millisecondsLat=(satioData.secondsLat - trunc(satioData.secondsLat)) * 1000;
    // -----------------------------------------------------------------------------------------
    // Round off minutes and seconds values to nearest integer.
    // -----------------------------------------------------------------------------------------
    satioData.minutesLat=trunc(satioData.minutesLat);
    satioData.secondsLat=trunc(satioData.secondsLat);
    // -----------------------------------------------------------------------------------------
    // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
    // -----------------------------------------------------------------------------------------
    satioData.degrees_latitude =
    satioData.degreesLat + satioData.minutesLat / 60 + satioData.secondsLat / 3600 + satioData.millisecondsLat / 3600000;
    // -----------------------------------------------------------------------------------------
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    // -----------------------------------------------------------------------------------------
    if (strcmp(gnggaData.latitude_hemisphere, "S")==0) {
      satioData.degrees_latitude=0 - satioData.degrees_latitude;
    }
    // -----------------------------------------------------------------------------------------
    // Save formatted latitude value as a string for later use.
    // -----------------------------------------------------------------------------------------
    scanf("%f17", &satioData.degrees_latitude);
    // -----------------------------------------------------------------------------------------
    // Extract absolute longitude value from GNGGA data as decimal degrees.
    // -----------------------------------------------------------------------------------------
    satioData.abs_longitude_gngga_0=atof(String(gnggaData.longitude).c_str());
    // -----------------------------------------------------------------------------------------
    // Store absolute latitude in temporary variable for further processing.
    // -----------------------------------------------------------------------------------------
    satioData.temp_longitude_gngga=satioData.abs_longitude_gngga_0;
    // -----------------------------------------------------------------------------------------
    // Separate the integer degrees value from the fractional part.
    // -----------------------------------------------------------------------------------------
    satioData.degreesLong=trunc(satioData.temp_longitude_gngga / 100);
    // -----------------------------------------------------------------------------------------
    // Calculate minutes and seconds values based on remaining fractional part.
    // -----------------------------------------------------------------------------------------
    satioData.minutesLong=satioData.temp_longitude_gngga - (satioData.degreesLong * 100);
    // -----------------------------------------------------------------------------------------
    // Convert excess fractional part to seconds.
    // -----------------------------------------------------------------------------------------
    satioData.secondsLong=(satioData.minutesLong - trunc(satioData.minutesLong)) * 60;
    // -----------------------------------------------------------------------------------------
    // Convert excess seconds to milliseconds.
    // -----------------------------------------------------------------------------------------
    satioData.millisecondsLong=(satioData.secondsLong - trunc(satioData.secondsLong)) * 1000;
    // -----------------------------------------------------------------------------------------
    // Round off minutes and seconds values to nearest integer.
    // -----------------------------------------------------------------------------------------
    satioData.minutesLong=trunc(satioData.minutesLong);
    satioData.secondsLong=trunc(satioData.secondsLong);
    // -----------------------------------------------------------------------------------------
    // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
    // -----------------------------------------------------------------------------------------
    satioData.degrees_longitude =
    satioData.degreesLong + satioData.minutesLong / 60 + satioData.secondsLong / 3600 + satioData.millisecondsLong / 3600000;
    // -----------------------------------------------------------------------------------------
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    // -----------------------------------------------------------------------------------------
    if (strcmp(gnggaData.longitude_hemisphere, "W")==0) {
      satioData.degrees_longitude=0 - satioData.degrees_longitude;
    }
    // -----------------------------------------------------------------------------------------
    // Save formatted latitude value as a string for later use.
    // -----------------------------------------------------------------------------------------
    scanf("%f17", &satioData.degrees_longitude);
  }
  // ------------------------------------------------------------------------------------------------------------------------
  //                                                                                              GNRMC COORDINATE CONVERSION
  // ------------------------------------------------------------------------------------------------------------------------
  // ------------------------------------------------------------------------------------------------------------------------
  // Convert GNRMC latitude & longitude strings to decimal degrees and format into hours, minutes, seconds, milliseconds.
  // ------------------------------------------------------------------------------------------------------------------------
  else if (satioData.coordinate_conversion_mode==1) {
    // -----------------------------------------------------------------------------------------
    // Extract absolute latitude value from GNGGA data as decimal degrees.
    // -----------------------------------------------------------------------------------------
    satioData.abs_latitude_gnrmc_0=atof(String(gnrmcData.latitude).c_str());
    // -----------------------------------------------------------------------------------------
    // Store absolute latitude in temporary variable for further processing.
    // -----------------------------------------------------------------------------------------
    satioData.temp_latitude_gnrmc=satioData.abs_latitude_gnrmc_0;
    // -----------------------------------------------------------------------------------------
    // Separate the integer degrees value from the fractional part.
    // -----------------------------------------------------------------------------------------
    satioData.degreesLat=trunc(satioData.temp_latitude_gnrmc / 100);
    // -----------------------------------------------------------------------------------------
    // Calculate minutes and seconds values based on remaining fractional part.
    // -----------------------------------------------------------------------------------------
    satioData.minutesLat=satioData.temp_latitude_gnrmc - (satioData.degreesLat * 100);
    // -----------------------------------------------------------------------------------------
    // Convert excess fractional part to seconds.
    // -----------------------------------------------------------------------------------------
    satioData.secondsLat=(satioData.minutesLat - (satioData.minutesLat)) * 60;
    // -----------------------------------------------------------------------------------------
    // Convert excess seconds to milliseconds.
    // -----------------------------------------------------------------------------------------
    satioData.millisecondsLat=(satioData.secondsLat - trunc(satioData.secondsLat)) * 1000;
    // -----------------------------------------------------------------------------------------
    // Round off minutes and seconds values to nearest integer.
    // -----------------------------------------------------------------------------------------
    satioData.minutesLat=trunc(satioData.minutesLat);
    satioData.secondsLat=trunc(satioData.secondsLat);
    // -----------------------------------------------------------------------------------------
    // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
    // -----------------------------------------------------------------------------------------
    satioData.degrees_latitude =
    satioData.degreesLat + satioData.minutesLat / 60 + satioData.secondsLat / 3600 + satioData.millisecondsLat / 3600000;
    // -----------------------------------------------------------------------------------------
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    // -----------------------------------------------------------------------------------------
    if (strcmp(gnrmcData.latitude_hemisphere, "S")==0) {
      satioData.degrees_latitude=0 - satioData.degrees_latitude;
    }
    // -----------------------------------------------------------------------------------------
    // Save formatted latitude value as a string for later use.
    // -----------------------------------------------------------------------------------------
    scanf("%f17", &satioData.degrees_latitude);
    // -----------------------------------------------------------------------------------------
    // Extract absolute latitude value from GNGGA data as decimal degrees.
    // -----------------------------------------------------------------------------------------
    satioData.abs_longitude_gnrmc_0=atof(String(gnrmcData.longitude).c_str());
    // -----------------------------------------------------------------------------------------
    // Store absolute latitude in temporary variable for further processing.
    // -----------------------------------------------------------------------------------------
    satioData.temp_longitude_gnrmc=satioData.abs_longitude_gnrmc_0;
    // -----------------------------------------------------------------------------------------
    // Separate the integer degrees value from the fractional part.
    // -----------------------------------------------------------------------------------------
    satioData.degreesLong=trunc(satioData.temp_longitude_gnrmc / 100);
    // -----------------------------------------------------------------------------------------
    // Calculate minutes and seconds values based on remaining fractional part.
    // -----------------------------------------------------------------------------------------
    satioData.minutesLong=satioData.temp_longitude_gnrmc - (satioData.degreesLong * 100);
    // -----------------------------------------------------------------------------------------
    // Convert excess fractional part to seconds.
    // -----------------------------------------------------------------------------------------
    satioData.secondsLong=(satioData.minutesLong - trunc(satioData.minutesLong)) * 60;
    // -----------------------------------------------------------------------------------------
    // Convert excess seconds to milliseconds.
    // -----------------------------------------------------------------------------------------
    satioData.millisecondsLong=(satioData.secondsLong - trunc(satioData.secondsLong)) * 1000;
    // -----------------------------------------------------------------------------------------
    // Round off minutes and seconds values to nearest integer.
    // -----------------------------------------------------------------------------------------
    satioData.minutesLong=trunc(satioData.minutesLong);
    satioData.secondsLong=trunc(satioData.secondsLong);
    // -----------------------------------------------------------------------------------------
    // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
    // -----------------------------------------------------------------------------------------
    satioData.degrees_longitude =
    satioData.degreesLong + satioData.minutesLong / 60 + satioData.secondsLong / 3600 + satioData.millisecondsLong / 3600000;
    // -----------------------------------------------------------------------------------------
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    // -----------------------------------------------------------------------------------------
    if (strcmp(gnrmcData.longitude_hemisphere, "W")==0) {
      satioData.degrees_longitude=0 - satioData.degrees_longitude;
    }
    // -----------------------------------------------------------------------------------------
    // Save formatted latitude value as a string for later use.
    // -----------------------------------------------------------------------------------------
    scanf("%f17", &satioData.degrees_longitude);
  }
}

// ----------------------------------------------------------------------------------------
// groundHeadingDegreesToNESW.
// ----------------------------------------------------------------------------------------
String groundHeadingDegreesToNESW(float num) {
  if (num == 0 || num == 360)      {return String("N");}
  else if (num > 0 && num < 45)    {return String("NNE");}
  else if (num == 45)              {return String("NE");}
  else if (num > 45 && num < 90)   {return String("ENE");}
  else if (num == 90)              {return String("E");}
  else if (num > 90 && num < 135)  {return String("ESE");}
  else if (num == 135)             {return String("SE");}
  else if (num > 135 && num < 180) {return String("SSE");}
  else if (num == 180)             {return String("S");}
  else if (num > 180 && num < 225) {return String("SSW");}
  else if (num == 225)             {return String("SW");}
  else if (num > 225 && num < 270) {return String("WSW");}
  else if (num == 270)             {return String("W");}
  else if (num > 270 && num < 315) {return String("WNW");}
  else if (num == 315)             {return String("NW");}
  else if (num > 315 && num < 360) {return String("NNW");}
  return String("");
}
void setGroundHeadingName(float num) {
  memset(satioData.ground_heading, 0, sizeof(satioData.ground_heading));
  strcpy(satioData.ground_heading, groundHeadingDegreesToNESW(num).c_str());
}

// ----------------------------------------------------------------------------------------
// printAllTimes.
// ----------------------------------------------------------------------------------------
struct tm *timeinfo;
struct timeval tv_now;

void printAllTimes(void) {
  /*     UTC     */
  Serial.println("-----------------------------------------");
  Serial.println("[gnrmcData.utc_date]      " + String(gnrmcData.utc_date));
  Serial.println("[gnrmcData.utc_time]      " + String(gnrmcData.utc_time));
  Serial.println("-----------------------------------------");
  Serial.println("[satioData.tmp_year_int]    " + String(satioData.tmp_year_int));
  Serial.println("[satioData.tmp_month_int]   " + String(satioData.tmp_month_int));
  Serial.println("[satioData.tmp_day_int]     " + String(satioData.tmp_day_int));
  Serial.println("[satioData.tmp_hour_int]    " + String(satioData.tmp_hour_int));
  Serial.println("[satioData.tmp_minute_int]  " + String(satioData.tmp_minute_int));
  Serial.println("[satioData.tmp_second_int]  " + String(satioData.tmp_second_int));
  Serial.println("[satioData.tmp_msecond_int] " + String(satioData.tmp_millisecond_int));
  Serial.println("-----------------------------------------");
  Serial.println("[satioData.rtc_year]        " + String(satioData.rtc_year));
  Serial.println("[satioData.rtc_month]       " + String(satioData.rtc_month));
  Serial.println("[satioData.rtc_mday]        " + String(satioData.rtc_mday));
  Serial.println("[satioData.rtc_hour]        " + String(satioData.rtc_hour));
  Serial.println("[satioData.rtc_minute]      " + String(satioData.rtc_minute));
  Serial.println("[satioData.rtc_second]      " + String(satioData.rtc_second));
  Serial.println("[satioData.rtc_unixtime]    " + String(satioData.rtc_unixtime));
  Serial.println("[satioData.rtc wday]        " + String(rtc.now().dayOfTheWeek()));
  Serial.println("-----------------------------------------");
  Serial.println("[satioData.rtcsync_year]    " + String(satioData.rtcsync_year));
  Serial.println("[satioData.rtcsync_month]   " + String(satioData.rtcsync_month));
  Serial.println("[satioData.rtc_mday]        " + String(satioData.rtc_mday));
  Serial.println("[satioData.rtcsync_hour]    " + String(satioData.rtcsync_hour));
  Serial.println("[satioData.rtcsync_minute]  " + String(satioData.rtcsync_minute));
  Serial.println("[satioData.rtcsync_second]  " + String(satioData.rtcsync_second));
  /*    SYSTEM/LOCAL    */
  Serial.println("-----------------------------------------");
  Serial.println("[timeinfo->tm_year+last_epoch] " + String(timeinfo->tm_year+LAST_EPOCH));
  Serial.println("[timeinfo->tm_mon+1]           " + String(timeinfo->tm_mon+1));
  Serial.println("[timeinfo->tm_mday]            " + String(timeinfo->tm_mday));
  Serial.println("[timeinfo->tm_hour]            " + String(timeinfo->tm_hour));
  Serial.println("[timeinfo->tm_min]             " + String(timeinfo->tm_min));
  Serial.println("[timeinfo->tm_sec]             " + String(timeinfo->tm_sec));
  Serial.println("[tv_now.tv_sec]                " + String(tv_now.tv_sec));
  Serial.println("[tv_now.tv_usec]               " + String(tv_now.tv_usec));
  Serial.println("[timeinfo->wday]               " + String(timeinfo->tm_wday));
  Serial.println("[timeinfo->mday]               " + String(timeinfo->tm_mday));
  Serial.println("[timeinfo->yday]               " + String(timeinfo->tm_yday));
  Serial.println("-----------------------------------------");

}

// ----------------------------------------------------------------------------------------
// padDigitsZero.
// ----------------------------------------------------------------------------------------
void padDigitsZero(int digits, char* output, size_t output_size) {
    // Prepends a zero to pad a string of digits evenly
    memset(output, 0, output_size);
    if (digits < 10) {
        strcpy(output, "0");
    }
    char temp[12]; // Enough for int32_t in base 10
    itoa(digits, temp, 10);
    strncat(output, temp, output_size - strlen(output) - 1);
}

// ----------------------------------------------------------------------------------------
// storeRTCTime.
// ----------------------------------------------------------------------------------------
void storeRTCTime(void) {
    // Store RTC time (UTC) to avoid multiple calls to rtc.now()
    satioData.rtc_hour = rtc.now().hour();
    satioData.rtc_minute = rtc.now().minute();
    satioData.rtc_second = rtc.now().second();
    satioData.rtc_year = rtc.now().year();
    satioData.rtc_month = rtc.now().month();
    satioData.rtc_wday = rtc.now().dayOfTheWeek();
    satioData.rtc_mday = rtc.now().day();
    satioData.rtc_unixtime = rtc.now().unixtime();

    // Debug output without String
    // char debug_str[MAX_GLOBAL_ELEMENT_SIZE];
    // snprintf(debug_str, MAX_GLOBAL_ELEMENT_SIZE, "RTC: %d", satioData.rtc_hour);
    // Serial.println(debug_str);

    // Copy weekday name
    memset(satioData.rtc_wday_name, 0, sizeof(satioData.rtc_wday_name));
    strcpy(satioData.rtc_wday_name, satioData.week_day_names[satioData.rtc_wday]);

    // Format time (HH:MM:SS)
    char hour_str[MAX_GLOBAL_ELEMENT_SIZE], min_str[MAX_GLOBAL_ELEMENT_SIZE], sec_str[MAX_GLOBAL_ELEMENT_SIZE];
    padDigitsZero(satioData.rtc_hour, hour_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.rtc_minute, min_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.rtc_second, sec_str, MAX_GLOBAL_ELEMENT_SIZE);

    memset(satioData.formatted_rtc_time, 0, sizeof(satioData.formatted_rtc_time));
    snprintf(satioData.formatted_rtc_time, MAX_GLOBAL_ELEMENT_SIZE, "%s:%s:%s", hour_str, min_str, sec_str);

    // Debug formatted time
    // snprintf(debug_str, MAX_GLOBAL_ELEMENT_SIZE, "formatted_rtc_time: %s", satioData.formatted_rtc_time);
    // Serial.println(debug_str);

    // Format date (DD/MM/YYYY)
    char day_str[MAX_GLOBAL_ELEMENT_SIZE], month_str[MAX_GLOBAL_ELEMENT_SIZE], year_str[MAX_GLOBAL_ELEMENT_SIZE];
    padDigitsZero(satioData.rtc_mday, day_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.rtc_month, month_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.rtc_year, year_str, MAX_GLOBAL_ELEMENT_SIZE);

    memset(satioData.formatted_rtc_date, 0, sizeof(satioData.formatted_rtc_date));
    snprintf(satioData.formatted_rtc_date, MAX_GLOBAL_ELEMENT_SIZE, "%s/%s/%s", day_str, month_str, year_str);

    // Format padded time (HHMMSS)
    memset(satioData.padded_rtc_time_HHMMSS, 0, sizeof(satioData.padded_rtc_time_HHMMSS));
    snprintf(satioData.padded_rtc_time_HHMMSS, MAX_GLOBAL_ELEMENT_SIZE, "%s%s%s", hour_str, min_str, sec_str);

    // Debug padded time
    // snprintf(debug_str, MAX_GLOBAL_ELEMENT_SIZE, "padded_rtc_time_HHMMSS: %s", satioData.padded_rtc_time_HHMMSS);
    // Serial.println(debug_str);

    // Format padded date (DDMMYYYY)
    memset(satioData.padded_rtc_date_DDMMYYYY, 0, sizeof(satioData.padded_rtc_date_DDMMYYYY));
    snprintf(satioData.padded_rtc_date_DDMMYYYY, MAX_GLOBAL_ELEMENT_SIZE, "%s%s%s", day_str, month_str, year_str);
}

// ----------------------------------------------------------------------------------------
// storeLocalTime.
// ----------------------------------------------------------------------------------------
void storeLocalTime(void) {
    // Store system time (Local Time) to avoid multiple calls to timeinfo
    satioData.local_hour = timeinfo->tm_hour;
    satioData.local_minute = timeinfo->tm_min;
    satioData.local_second = timeinfo->tm_sec;
    satioData.local_year = timeinfo->tm_year + LAST_EPOCH; // Adjust from timeinfo's year (since 1900)
    satioData.local_month = timeinfo->tm_mon + 1;    // Adjust from 0-based to 1-based
    satioData.local_wday = timeinfo->tm_wday;
    satioData.local_mday = timeinfo->tm_mday;
    satioData.local_yday = timeinfo->tm_yday;

    // Copy weekday name
    memset(satioData.local_wday_name, 0, sizeof(satioData.local_wday_name));
    strcpy(satioData.local_wday_name, satioData.week_day_names[satioData.local_wday]);

    // Format time (HH:MM:SS)
    char hour_str[MAX_GLOBAL_ELEMENT_SIZE], min_str[MAX_GLOBAL_ELEMENT_SIZE], sec_str[MAX_GLOBAL_ELEMENT_SIZE];
    padDigitsZero(satioData.local_hour, hour_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.local_minute, min_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.local_second, sec_str, MAX_GLOBAL_ELEMENT_SIZE);

    memset(satioData.formatted_local_time, 0, sizeof(satioData.formatted_local_time));
    snprintf(satioData.formatted_local_time, MAX_GLOBAL_ELEMENT_SIZE, "%s:%s:%s", hour_str, min_str, sec_str);

    // Format date (DD/MM/YYYY)
    char day_str[MAX_GLOBAL_ELEMENT_SIZE], month_str[MAX_GLOBAL_ELEMENT_SIZE], year_str[MAX_GLOBAL_ELEMENT_SIZE];
    padDigitsZero(satioData.local_mday, day_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.local_month, month_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.local_year, year_str, MAX_GLOBAL_ELEMENT_SIZE);
    memset(satioData.formatted_local_date, 0, sizeof(satioData.formatted_local_date));
    snprintf(satioData.formatted_local_date, MAX_GLOBAL_ELEMENT_SIZE, "%s/%s/%s", day_str, month_str, year_str);

    // Format padded time (HHMMSS)
    memset(satioData.padded_local_time_HHMMSS, 0, sizeof(satioData.padded_local_time_HHMMSS));
    snprintf(satioData.padded_local_time_HHMMSS, MAX_GLOBAL_ELEMENT_SIZE, "%s%s%s", hour_str, min_str, sec_str);

    // Format padded date (DDMMYYYY)
    memset(satioData.padded_local_date_DDMMYYYY, 0, sizeof(satioData.padded_local_date_DDMMYYYY));
    snprintf(satioData.padded_local_date_DDMMYYYY, MAX_GLOBAL_ELEMENT_SIZE, "%s%s%s", day_str, month_str, year_str);
}

// ----------------------------------------------------------------------------------------
// storeRTCSYNCTime.
// ----------------------------------------------------------------------------------------
void storeRTCSYNCTime(void) {
    // Store RTC sync time (based on local time and RTC)
    satioData.rtcsync_hour = satioData.local_hour;
    satioData.rtcsync_minute = satioData.local_minute;
    satioData.rtcsync_second = satioData.local_second;
    satioData.rtcsync_year = satioData.local_year;
    satioData.rtcsync_month = satioData.local_month;
    satioData.rtcsync_day = satioData.local_mday;
    satioData.rtcsync_unixtime = rtc.now().unixtime();

    // Format sync time (HH:MM:SS)
    char hour_str[MAX_GLOBAL_ELEMENT_SIZE], min_str[MAX_GLOBAL_ELEMENT_SIZE], sec_str[MAX_GLOBAL_ELEMENT_SIZE];
    padDigitsZero(satioData.rtcsync_hour, hour_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.rtcsync_minute, min_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.rtcsync_second, sec_str, MAX_GLOBAL_ELEMENT_SIZE);

    memset(satioData.formatted_rtc_sync_time, 0, sizeof(satioData.formatted_rtc_sync_time));
    snprintf(satioData.formatted_rtc_sync_time, MAX_GLOBAL_ELEMENT_SIZE, "%s:%s:%s", hour_str, min_str, sec_str);

    // Format sync date (DD/MM/YYYY)
    char day_str[MAX_GLOBAL_ELEMENT_SIZE], month_str[MAX_GLOBAL_ELEMENT_SIZE], year_str[MAX_GLOBAL_ELEMENT_SIZE];
    padDigitsZero(satioData.rtcsync_day, day_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.rtcsync_month, month_str, MAX_GLOBAL_ELEMENT_SIZE);
    padDigitsZero(satioData.rtcsync_year, year_str, MAX_GLOBAL_ELEMENT_SIZE);

    memset(satioData.formatted_rtc_sync_date, 0, sizeof(satioData.formatted_rtc_sync_date));
    snprintf(satioData.formatted_rtc_sync_date, MAX_GLOBAL_ELEMENT_SIZE, "%s/%s/%s", day_str, month_str, year_str);

    // Format padded sync time (HHMMSS)
    memset(satioData.padded_rtc_sync_time_HHMMSS, 0, sizeof(satioData.padded_rtc_sync_time_HHMMSS));
    snprintf(satioData.padded_rtc_sync_time_HHMMSS, MAX_GLOBAL_ELEMENT_SIZE, "%s%s%s", hour_str, min_str, sec_str);

    // Format padded sync date (DDMMYYYY)
    memset(satioData.padded_rtc_sync_date_DDMMYYYY, 0, sizeof(satioData.padded_rtc_sync_date_DDMMYYYY));
    snprintf(satioData.padded_rtc_sync_date_DDMMYYYY, MAX_GLOBAL_ELEMENT_SIZE, "%s%s%s", day_str, month_str, year_str);

    // RTC sync location
    char temp_str[MAX_GLOBAL_ELEMENT_SIZE];
    snprintf(temp_str, MAX_GLOBAL_ELEMENT_SIZE, "%.7f", satioData.degrees_latitude);
    memset(satioData.rtcsync_latitude, 0, sizeof(satioData.rtcsync_latitude));
    strcpy(satioData.rtcsync_latitude, temp_str);

    snprintf(temp_str, MAX_GLOBAL_ELEMENT_SIZE, "%.7f", satioData.degrees_longitude);
    memset(satioData.rtcsync_longitude, 0, sizeof(satioData.rtcsync_longitude));
    strcpy(satioData.rtcsync_longitude, temp_str);

    memset(satioData.rtcsync_altitude, 0, sizeof(satioData.rtcsync_altitude));
    strcpy(satioData.rtcsync_altitude, gnggaData.altitude);
}

// ----------------------------------------------------------------------------------------
// setSystemTime.
// ----------------------------------------------------------------------------------------
void setSystemTime(long usec) {
  // -----------------------------------------------------
  // System time = Local Time.
  // -----------------------------------------------------
  struct tm tmpti = {0};
  memset(&tmpti, 0, sizeof(tmpti));
  tmpti.tm_year = int(rtc.now().year()) - LAST_EPOCH; // Years since 1900 (since last epoch)
  tmpti.tm_mon = rtc.now().month() - 1; // Months 0-11
  tmpti.tm_mday = rtc.now().day();
  tmpti.tm_hour = rtc.now().hour();
  tmpti.tm_min = rtc.now().minute();
  tmpti.tm_sec = rtc.now().second();
  tmpti.tm_isdst = -1; // No DST
  time_t now = mktime(&tmpti);
  tv_now = {
      .tv_sec = now + satioData.utc_second_offset, // negative utc_second_offset will be deducted.
      .tv_usec = usec
  };
  if (settimeofday(&tv_now, NULL) == 0) {}
  else {Serial.println("[settimeofday] failed");}
}

// ----------------------------------------------------------------------------------------
// getSystemTime.
// ----------------------------------------------------------------------------------------
void getSystemTime(void) {
  // --------------------------------------------------------
  // System time = Local Time
  // This function must be called in order to update timeinfo
  // Only use when required and alternatively, use other
  // stored times when a lower time resolution is required.
  // --------------------------------------------------------
  gettimeofday(&tv_now, NULL);
  timeinfo = localtime(&tv_now.tv_sec); // Assumes localtime works
  // --------------------------------------------------------
  // Keep this function quick by only storing unixtime uS.
  // --------------------------------------------------------
  satioData.local_unixtime_uS = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                              SYNC RTC FROM GPS
// ------------------------------------------------------------------------------------------------------------------------------
void extractDateTimeFromGPSData(void) {
  satioData.tmp_day[0]=gnrmcData.utc_date[0];
  satioData.tmp_day[1]=gnrmcData.utc_date[1];
  satioData.tmp_month[0]=gnrmcData.utc_date[2];
  satioData.tmp_month[1]=gnrmcData.utc_date[3];
  satioData.tmp_year[0]=gnrmcData.utc_date[4];
  satioData.tmp_year[1]=gnrmcData.utc_date[5];
  satioData.tmp_hour[0]=gnrmcData.utc_time[0];
  satioData.tmp_hour[1]=gnrmcData.utc_time[1];
  satioData.tmp_minute[0]=gnrmcData.utc_time[2];
  satioData.tmp_minute[1]=gnrmcData.utc_time[3];
  satioData.tmp_day_int=atoi(satioData.tmp_day);
  satioData.tmp_month_int=atoi(satioData.tmp_month);
  satioData.tmp_year_int=atoi(satioData.tmp_year);
  satioData.tmp_hour_int=atoi(satioData.tmp_hour);
  satioData.tmp_minute_int=atoi(satioData.tmp_minute);
  satioData.tmp_second[0]=gnrmcData.utc_time[4];
  satioData.tmp_second[1]=gnrmcData.utc_time[5];
  satioData.tmp_millisecond[0]=gnrmcData.utc_time[7];
  satioData.tmp_millisecond[1]=gnrmcData.utc_time[8];
  satioData.tmp_second_int=atoi(satioData.tmp_second);
  satioData.tmp_millisecond_int=atoi(satioData.tmp_millisecond);
}

void setRTCDateTime() {
  rtc.adjust(DateTime((uint16_t)satioData.tmp_year_int, (uint8_t)satioData.tmp_month_int, (uint8_t)satioData.tmp_day_int,
  (uint8_t)satioData.tmp_hour_int, (uint8_t)satioData.tmp_minute_int, (uint8_t)satioData.tmp_second_int));
  setSystemTime(0);
  storeRTCSYNCTime();
  satioData.sync_rtc_immediately_flag=false;
}

void syncRTC() {
  /**
   * Manually set RTC datetime.
   * 
   * (1) Set set_time_automatically false.
   * (1) Set temporary datetime values.
   * (2) Set satioData.set_rtc_datetime_flag true.
   */
  if (satioData.set_time_automatically==false && satioData.set_rtc_datetime_flag==true)
    {setRTCDateTime(); satioData.set_rtc_datetime_flag=false; Serial.println("[rtc] sync 2: " + String(rtc.now().timestamp()));}

  /**
   * Automatically set RTC datetime with GPS data.
   */
  else if (satioData.set_time_automatically==true) {
    // ----------------------------------------------------------------------------------------------
    /*                                 SYNC RTC TIME & DATE FROM GPS                               */
    // ----------------------------------------------------------------------------------------------
    // Serial.println("[satellite_count]      " + String(gnggaData.satellite_count));
    // Serial.println("[gps_precision_factor] " + String(gnggaData.gps_precision_factor));
    if ((atoi(gnggaData.satellite_count)>3) && (atoi(gnggaData.gps_precision_factor)<=3)) {
      // ----------------------------------------------------------------------------
      // Extract just what we need to perform a timing check.
      // ----------------------------------------------------------------------------
      extractDateTimeFromGPSData();

      if (satioData.sync_rtc_immediately_flag==true) {
        // ----------------------------------------------------------------------------
        // Sync within the first 100 milliseconds of any second.
        // ----------------------------------------------------------------------------
        if (satioData.tmp_millisecond_int==0) {
          // --------------------------------------------------------------------------
          // Sync RTC to UTC.
          // --------------------------------------------------------------------------
          // Serial.println("[rtc] sync 0: " + String(rtc.now().timestamp()));
          setRTCDateTime();
          Serial.println("[rtc] sync 0: " + String(rtc.now().timestamp()));
        }
      }
      else {
        // ----------------------------------------------------------------------------
        // Sync within the first 100 milliseconds of any minute.
        // ----------------------------------------------------------------------------
        if ((satioData.tmp_second_int==0) && (satioData.tmp_millisecond_int==0)) {
          // --------------------------------------------------------------------------
          // Sync RTC to UTC.
          // --------------------------------------------------------------------------
          // Serial.println("[rtc] sync 0: " + String(rtc.now().timestamp()));
          setRTCDateTime();
          Serial.println("[rtc] sync 1: " + String(rtc.now().timestamp()));
        }
      }
    }
  }
}

// ----------------------------------------------------------------------------------------
// setSatIOData.
// ----------------------------------------------------------------------------------------
void setSatIOData(void) {
      syncRTC();
      calculateLocation();
      setGroundHeadingName(atof(gnrmcData.ground_heading));
}

// ----------------------------------------------------------------------------------------
// initSystemTime.
// ----------------------------------------------------------------------------------------
void initSystemTime(void) {
  int rtc_second_now=rtc.now().second();
  Serial.println("[SYNC] synchronizing system time with RTC");
  getSystemTime();
  while (rtc_second_now==rtc.now().second()) // wait to sync
  setSystemTime(0);
  getSystemTime();
  storeLocalTime();
  storeRTCTime();
  Serial.println("[SYNC] RTC datetime:    " + String(satioData.padded_rtc_time_HHMMSS) + " " + String(satioData.padded_rtc_date_DDMMYYYY));
  Serial.println("[SYNC] system datetime: " + String(satioData.padded_local_time_HHMMSS) + " " + String(satioData.padded_local_date_DDMMYYYY) +
                 " (+- offset seconds " + String(satioData.utc_second_offset) + ")");
}

void initRTC(void) {
  rtc.begin();
}