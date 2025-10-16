/*
    SATIO Library. Written by Benjamin Jack Cullen.

    1 : From main call initRTC().
    2 : From main call initSystemTime().
    3 : From main call setSatIOData() when required to sync RTC with GPS data and set all other data.
    4 : All satio data will be available in satioData.
*/

#ifndef SATIO_H
#define SATIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "config.h"

extern struct tm *timeinfo;
extern struct timeval tv_now;
extern bool sync_rtc_bool;

// ----------------------------------------------------------------------------------------
// SATIO Struct.
// ----------------------------------------------------------------------------------------
struct SATIOStruct {
    char satio_sentence[MAX_GLOBAL_SERIAL_BUFFER_SIZE]; // Buffer for processed data
    double latitude_meter;                              // One meter (tunable)
    double longitude_meter;                             // One meter (tunable)
    double latitude_mile;                               // One mile (tunable)
    double longitude_mile;                              // One mile (tunable)
    double abs_latitude_gngga_0;                        // Absolute latitude from GNGGA
    double abs_longitude_gngga_0;                       // Absolute longitude from GNGGA
    double abs_latitude_gnrmc_0;                        // Absolute latitude from GNRMC
    double abs_longitude_gnrmc_0;                       // Absolute longitude from GNRMC
    double temp_latitude_gngga;                         // Temp latitude for GNGGA
    double temp_longitude_gngga;                        // Temp longitude for GNGGA
    double temp_latitude_gnrmc;                         // Temp latitude for GNRMC
    double temp_longitude_gnrmc;                        // Temp longitude for GNRMC
    double minutesLat;                                  // Minutes for latitude conversion
    double minutesLong;                                 // Minutes for longitude conversion
    double secondsLat;                                  // Seconds for latitude conversion
    double secondsLong;                                 // Seconds for longitude conversion
    double millisecondsLat;                             // Milliseconds for latitude conversion
    double millisecondsLong;                            // Milliseconds for longitude conversion
    double degrees_latitude;                            // Converted latitude in degrees
    double degrees_longitude;                           // Converted longitude in degrees
    double degreesLat;                                  // Degrees for latitude conversion
    double degreesLong;                                 // Degrees for longitude conversion
    uint16_t tmp_year_int;                              // Temp year
    uint8_t tmp_month_int;                              // Temp month
    uint8_t tmp_day_int;                                // Temp day
    uint8_t tmp_hour_int;                               // Temp hour
    uint8_t tmp_minute_int;                             // Temp minute
    uint8_t tmp_second_int;                             // Temp second
    uint8_t tmp_millisecond_int;                        // Temp millisecond
    char tmp_year[MAX_GLOBAL_ELEMENT_SIZE];             // Temp year string
    char tmp_month[MAX_GLOBAL_ELEMENT_SIZE];            // Temp month string
    char tmp_day[MAX_GLOBAL_ELEMENT_SIZE];              // Temp day string
    char tmp_hour[MAX_GLOBAL_ELEMENT_SIZE];             // Temp hour string
    char tmp_minute[MAX_GLOBAL_ELEMENT_SIZE];           // Temp minute string
    char tmp_second[MAX_GLOBAL_ELEMENT_SIZE];           // Temp second string
    char tmp_millisecond[MAX_GLOBAL_ELEMENT_SIZE];      // Temp millisecond string
    char week_day_names[MAX_DAYS_OF_THE_WEEK][MAX_GLOBAL_ELEMENT_SIZE]; // Weekday names
    uint8_t local_hour;                                 // Local hour
    uint8_t local_minute;                               // Local minute
    uint8_t local_second;                               // Local second
    uint16_t local_year;                                // Local year
    uint8_t local_month;                                // Local month
    uint8_t local_mday;                                 // Local day of month
    uint8_t local_yday;                                 // Local day of year
    uint8_t local_wday;                                 // Local day of week
    char local_wday_name[MAX_GLOBAL_ELEMENT_SIZE];      // Local weekday name
    char formatted_local_time[MAX_GLOBAL_ELEMENT_SIZE];  // Formatted local time
    char formatted_local_date[MAX_GLOBAL_ELEMENT_SIZE];  // Formatted local date
    char padded_local_time_HHMMSS[MAX_GLOBAL_ELEMENT_SIZE]; // Padded local time (HHMMSS)
    char padded_local_date_DDMMYYYY[MAX_GLOBAL_ELEMENT_SIZE]; // Padded local date
    uint64_t local_unixtime_uS;                         // Local Unix time in microseconds
    uint8_t rtcsync_hour;                               // RTC sync hour
    uint8_t rtcsync_minute;                             // RTC sync minute
    uint8_t rtcsync_second;                             // RTC sync second
    uint16_t rtcsync_year;                              // RTC sync year
    uint8_t rtcsync_month;                              // RTC sync month
    uint8_t rtcsync_day;                                // RTC sync day
    char formatted_rtc_sync_time[MAX_GLOBAL_ELEMENT_SIZE]; // Formatted RTC sync time
    char formatted_rtc_sync_date[MAX_GLOBAL_ELEMENT_SIZE]; // Formatted RTC sync date
    char padded_rtc_sync_time_HHMMSS[MAX_GLOBAL_ELEMENT_SIZE]; // Padded RTC sync time (HHMMSS)
    char padded_rtc_sync_date_DDMMYYYY[MAX_GLOBAL_ELEMENT_SIZE]; // Padded RTC sync date
    uint32_t rtcsync_unixtime;                          // RTC sync Unix time
    char rtcsync_latitude[MAX_GLOBAL_ELEMENT_SIZE];      // RTC sync latitude
    char rtcsync_longitude[MAX_GLOBAL_ELEMENT_SIZE];     // RTC sync longitude
    char rtcsync_altitude[MAX_GLOBAL_ELEMENT_SIZE];      // RTC sync altitude
    uint8_t rtc_hour;                                   // RTC current hour
    uint8_t rtc_minute;                                 // RTC current minute
    uint8_t rtc_second;                                 // RTC current second
    uint16_t rtc_year;                                  // RTC current year
    uint8_t rtc_month;                                  // RTC current month
    uint8_t rtc_mday;                                   // RTC current day
    uint8_t rtc_wday;                                   // RTC current day of week
    char rtc_wday_name[MAX_GLOBAL_ELEMENT_SIZE];         // RTC weekday name
    char formatted_rtc_time[MAX_GLOBAL_ELEMENT_SIZE];    // Formatted RTC time
    char formatted_rtc_date[MAX_GLOBAL_ELEMENT_SIZE];    // Formatted RTC date
    char padded_rtc_time_HHMMSS[MAX_GLOBAL_ELEMENT_SIZE]; // Padded RTC time (HHMMSS)
    char padded_rtc_date_DDMMYYYY[MAX_GLOBAL_ELEMENT_SIZE]; // Padded RTC date
    uint32_t rtc_unixtime;                              // RTC Unix time

    int64_t utc_second_offset;                 // UTC offset in seconds
    bool utc_auto_offset_flag;                          // Auto offset flag
    bool set_time_automatically;                        // Auto time setting flag
    bool set_rtc_datetime_flag;
    bool sync_rtc_immediately_flag;

    int  coordinate_conversion_mode; // 0: STATIC, 1: GNGGA, 2: GNRMC
    char char_coordinate_conversion_mode[MAX_COORDINATE_CONVERSION_CONVERSION_MODES][10];

    double altitude; // should always be meters altitude.
    double altitude_converted;
    int    altitude_unit_mode; // 0: Meters, 1: Miles, 2: Km, add more
    char   char_altitude_unit_mode[MAX_ALTITUDE_UNIT_MODES][10];
    int  altitude_conversion_mode; // 0: STATIC, 1: GNGGA
    char char_altitude_conversion_mode[MAX_ALTITUDE_CONVERSION_MODES][10];

    double speed; // should always be meters a second.
    double speed_converted;
    int    speed_unit_mode; // 0: M/S, 1: MPH, 2: KPH, 3: KTS add more
    char   char_speed_unit_mode[MAX_SPEED_UNIT_MODES][10]; // Speed mode names
    int  speed_conversion_mode; // 0: STATIC, 1: GNRMC
    char char_speed_conversion_mode[MAX_COORDINATE_CONVERSION_CONVERSION_MODES][10];

    char ground_heading[MAX_GLOBAL_ELEMENT_SIZE]; // Ground heading (e.g., N, NNE)
    char mileage[MAX_GLOBAL_ELEMENT_SIZE];        // Mileage (pending processing)
};
extern struct SATIOStruct satioData;

/**
 * @struct LocPoint
 */
typedef struct {
    double latitude;   // Latitude in degrees
    double longitude;  // Longitude in degrees
    double altitude;   // Altitude in meters
    uint64_t time;     // Time in microseconds
} LocPoint;
extern LocPoint loc_point1_gps;
extern LocPoint loc_point2_gps;
extern LocPoint loc_point1_ins;
extern LocPoint loc_point2_ins;

/**
 * @struct SpeedStruct
 */
struct SpeedStruct {
    double lat1_rad;    // Latitude 1 in radians
    double lon1_rad;    // Longitude 1 in radians
    double lat2_rad;    // Latitude 2 in radians
    double lon2_rad;    // Longitude 2 in radians
    double delta_lat;   // Change in latitude
    double delta_lon;   // Change in longitude
    double delta_alt;   // Change in altitude
    double a;           // Haversine intermediate
    double c;           // Haversine intermediate
    double distance_2d; // 2D distance
    double distance_3d; // 3D distance
    double delta_time;  // Time difference
    double speed;       // Calculated speed
};

extern struct SpeedStruct speedData;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
double convertSpeedUnits(double speed);
void calculateLocation(void);
void setGroundHeadingName(float num);
char* padDigitsZero(int digits);
void printAllTimes(void);
void storeRTCTime(void);
void storeLocalTime(void);
void storeRTCSYNCTime(void) ;
void extractDateTimeFromGPSData(void);
void setSystemTime(long usec);
void getSystemTime(void);
void syncRTC(void);
void setSatIOData(void);
void initRTC(void);
void initSystemTime(void);

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
 double calculateSpeedFromLocationData(LocPoint p1, LocPoint p2);

#ifdef __cplusplus
}
#endif

#endif