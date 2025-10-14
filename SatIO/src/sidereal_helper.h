/*
    Sidereal Helper. Written by Benjamin Jack Cullen.
*/

#ifndef SIDEREAL_HELPER_H
#define SIDEREAL_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

// ----------------------------------------------------------------------------------------
// Planet Data Structure.
// ----------------------------------------------------------------------------------------
struct SiderealPlantetsStruct {
    double earth_ecliptic_lat;
    double earth_ecliptic_long;
    double sun_ra;
    double sun_dec;
    double sun_az;
    double sun_alt;
    double sun_r;
    double sun_s;
    double sun_helio_ecliptic_lat;
    double sun_helio_ecliptic_long;
    double sun_radius_vector;
    double sun_distance;
    double sun_ecliptic_lat;
    double sun_ecliptic_long;
    double moon_ra;
    double moon_dec;
    double moon_az;
    double moon_alt;
    double moon_r;
    double moon_s;
    double moon_p;
    char moon_p_name[8][MAX_GLOBAL_ELEMENT_SIZE];
    double moon_lum;
    double mercury_ra;
    double mercury_dec;
    double mercury_az;
    double mercury_alt;
    double mercury_r;
    double mercury_s;
    double mercury_helio_ecliptic_lat;
    double mercury_helio_ecliptic_long;
    double mercury_radius_vector;
    double mercury_distance;
    double mercury_ecliptic_lat;
    double mercury_ecliptic_long;
    double venus_ra;
    double venus_dec;
    double venus_az;
    double venus_alt;
    double venus_r;
    double venus_s;
    double venus_helio_ecliptic_lat;
    double venus_helio_ecliptic_long;
    double venus_radius_vector;
    double venus_distance;
    double venus_ecliptic_lat;
    double venus_ecliptic_long;
    double mars_ra;
    double mars_dec;
    double mars_az;
    double mars_alt;
    double mars_r;
    double mars_s;
    double mars_helio_ecliptic_lat;
    double mars_helio_ecliptic_long;
    double mars_radius_vector;
    double mars_distance;
    double mars_ecliptic_lat;
    double mars_ecliptic_long;
    double jupiter_ra;
    double jupiter_dec;
    double jupiter_az;
    double jupiter_alt;
    double jupiter_r;
    double jupiter_s;
    double jupiter_helio_ecliptic_lat;
    double jupiter_helio_ecliptic_long;
    double jupiter_radius_vector;
    double jupiter_distance;
    double jupiter_ecliptic_lat;
    double jupiter_ecliptic_long;
    double saturn_ra;
    double saturn_dec;
    double saturn_az;
    double saturn_alt;
    double saturn_r;
    double saturn_s;
    double saturn_helio_ecliptic_lat;
    double saturn_helio_ecliptic_long;
    double saturn_radius_vector;
    double saturn_distance;
    double saturn_ecliptic_lat;
    double saturn_ecliptic_long;
    double uranus_ra;
    double uranus_dec;
    double uranus_az;
    double uranus_alt;
    double uranus_r;
    double uranus_s;
    double uranus_helio_ecliptic_lat;
    double uranus_helio_ecliptic_long;
    double uranus_radius_vector;
    double uranus_distance;
    double uranus_ecliptic_lat;
    double uranus_ecliptic_long;
    double neptune_ra;
    double neptune_dec;
    double neptune_az;
    double neptune_alt;
    double neptune_r;
    double neptune_s;
    double neptune_helio_ecliptic_lat;
    double neptune_helio_ecliptic_long;
    double neptune_radius_vector;
    double neptune_distance;
    double neptune_ecliptic_lat;
    double neptune_ecliptic_long;
    char sentence[MAX_GLOBAL_SERIAL_BUFFER_SIZE];
};

extern struct SiderealPlantetsStruct siderealPlanetData;

// ----------------------------------------------------------------------------------------
// Object Data Structure.
// ----------------------------------------------------------------------------------------
struct SiderealObjectStruct {
    char object_name[MAX_GLOBAL_ELEMENT_SIZE];
    char object_table_name[MAX_GLOBAL_ELEMENT_SIZE];
    signed int object_number;
    signed int object_table_i;
    double object_ra;
    double object_dec;
    double object_az;
    double object_alt;
    double object_mag;
    double object_r;
    double object_s;
    char object_table[7][MAX_GLOBAL_ELEMENT_SIZE];
    int object_s_value;
    char object_type[MAX_GLOBAL_ELEMENT_SIZE];
    char object_con[MAX_GLOBAL_ELEMENT_SIZE];
    char object_desc[MAX_GLOBAL_ELEMENT_SIZE];
    double object_dist;
};

extern struct SiderealObjectStruct siderealObjectData;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
void trackObject(double latitude, double longitude,
    double utc_year, double utc_month, double utc_mday,
    double utc_hour, double utc_minute, double utc_second,
    double local_hour, double local_minute, double local_second,
    double altitude, int object_table_i, int object_i);
void IdentifyObject(int ra_hour, int ra_min, float ra_sec, int dec_d, int dec_m, float dec_s);
void trackSun(void);
void trackMoon(void);
void trackMercury(void);
void trackVenus(void);
void trackMars(void);
void trackJupiter(void);
void trackSaturn(void);
void trackUranus(void);
void trackNeptune(void);
void clearSun(void);
void clearMoon(void);
void clearMercury(void);
void clearVenus(void);
void clearMars(void);
void clearJupiter(void);
void clearSaturn(void);
void clearUranus(void);
void clearNeptune(void);
void clearTrackPlanets(void);
void trackPlanets(double latitude, double longitude,
    double utc_year, double utc_month, double utc_mday,
    double utc_hour, double utc_minute, double utc_second,
    double local_hour, double local_minute, double local_second,
    double altitude);
void myAstroBegin(void);
#ifdef __cplusplus
}
#endif

#endif