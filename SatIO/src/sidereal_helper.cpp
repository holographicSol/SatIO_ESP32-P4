/*
    Sidereal Helper. Written by Benjamin Jack Cullen.
*/

#include "sidereal_helper.h"
#include <Arduino.h>
#include <math.h>
#include <SiderealPlanets.h>  // https://github.com/DavidArmstrong/SiderealPlanets
#include <SiderealObjects.h>  // https://github.com/DavidArmstrong/SiderealObjects

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               SIDEREAL PLANETS
// ------------------------------------------------------------------------------------------------------------------------------

SiderealPlanets myAstro;    // for calculating azimuth and altitude
SiderealObjects myAstroObj; // for getting right ascension and declination of objects from star table

struct SiderealPlantetsStruct siderealPlanetData = {
    .earth_ecliptic_lat = 0.0,
    .earth_ecliptic_long = 0.0,
    .sun_ra = NAN,
    .sun_dec = NAN,
    .sun_az = NAN,
    .sun_alt = NAN,
    .sun_r = NAN,
    .sun_s = NAN,
    .sun_helio_ecliptic_lat = NAN,
    .sun_helio_ecliptic_long = NAN,
    .sun_radius_vector = NAN,
    .sun_distance = NAN,
    .sun_ecliptic_lat = NAN,
    .sun_ecliptic_long = NAN,
    .moon_ra = NAN,
    .moon_dec = NAN,
    .moon_az = NAN,
    .moon_alt = NAN,
    .moon_r = NAN,
    .moon_s = NAN,
    .moon_p = NAN,
    .moon_p_name = {
        "New Moon",
        "Waxing Crescent",
        "First Quarter",
        "Waxing Gibbous",
        "Full Moon",
        "Waning Gibbous",
        "Third Quarter",
        "Waning Crescent"
    },
    .moon_lum = NAN,
    .mercury_ra = NAN,
    .mercury_dec = NAN,
    .mercury_az = NAN,
    .mercury_alt = NAN,
    .mercury_r = NAN,
    .mercury_s = NAN,
    .mercury_helio_ecliptic_lat = NAN,
    .mercury_helio_ecliptic_long = NAN,
    .mercury_radius_vector = NAN,
    .mercury_distance = NAN,
    .mercury_ecliptic_lat = NAN,
    .mercury_ecliptic_long = NAN,
    .venus_ra = NAN,
    .venus_dec = NAN,
    .venus_az = NAN,
    .venus_alt = NAN,
    .venus_r = NAN,
    .venus_s = NAN,
    .venus_helio_ecliptic_lat = NAN,
    .venus_helio_ecliptic_long = NAN,
    .venus_radius_vector = NAN,
    .venus_distance = NAN,
    .venus_ecliptic_lat = NAN,
    .venus_ecliptic_long = NAN,
    .mars_ra = NAN,
    .mars_dec = NAN,
    .mars_az = NAN,
    .mars_alt = NAN,
    .mars_r = NAN,
    .mars_s = NAN,
    .mars_helio_ecliptic_lat = NAN,
    .mars_helio_ecliptic_long = NAN,
    .mars_radius_vector = NAN,
    .mars_distance = NAN,
    .mars_ecliptic_lat = NAN,
    .mars_ecliptic_long = NAN,
    .jupiter_ra = NAN,
    .jupiter_dec = NAN,
    .jupiter_az = NAN,
    .jupiter_alt = NAN,
    .jupiter_r = NAN,
    .jupiter_s = NAN,
    .jupiter_helio_ecliptic_lat = NAN,
    .jupiter_helio_ecliptic_long = NAN,
    .jupiter_radius_vector = NAN,
    .jupiter_distance = NAN,
    .jupiter_ecliptic_lat = NAN,
    .jupiter_ecliptic_long = NAN,
    .saturn_ra = NAN,
    .saturn_dec = NAN,
    .saturn_az = NAN,
    .saturn_alt = NAN,
    .saturn_r = NAN,
    .saturn_s = NAN,
    .saturn_helio_ecliptic_lat = NAN,
    .saturn_helio_ecliptic_long = NAN,
    .saturn_radius_vector = NAN,
    .saturn_distance = NAN,
    .saturn_ecliptic_lat = NAN,
    .saturn_ecliptic_long = NAN,
    .uranus_ra = NAN,
    .uranus_dec = NAN,
    .uranus_az = NAN,
    .uranus_alt = NAN,
    .uranus_r = NAN,
    .uranus_s = NAN,
    .uranus_helio_ecliptic_lat = NAN,
    .uranus_helio_ecliptic_long = NAN,
    .uranus_radius_vector = NAN,
    .uranus_distance = NAN,
    .uranus_ecliptic_lat = NAN,
    .uranus_ecliptic_long = NAN,
    .neptune_ra = NAN,
    .neptune_dec = NAN,
    .neptune_az = NAN,
    .neptune_alt = NAN,
    .neptune_r = NAN,
    .neptune_s = NAN,
    .neptune_helio_ecliptic_lat = NAN,
    .neptune_helio_ecliptic_long = NAN,
    .neptune_radius_vector = NAN,
    .neptune_distance = NAN,
    .neptune_ecliptic_lat = NAN,
    .neptune_ecliptic_long = NAN,
    .sentence = {0}
};
struct SiderealObjectStruct siderealObjectData = {
    .object_name = {0},
    .object_table_name = {0},
    .object_number = 0,
    .object_table_i = 0,
    .object_ra = NAN,
    .object_dec = NAN,
    .object_az = NAN,
    .object_alt = NAN,
    .object_mag = NAN,
    .object_r = NAN,
    .object_s = NAN,
    .object_table = {
        "Star Table",
        "NGC Table",
        "IC Table",
        "Messier Table",
        "Caldwell Table",
        "Herschel 400 Table",
        "Other Objects Table",
    },
    .object_s_value = -1,
    .object_type = {0},
    .object_con = {0},
    .object_desc = {0},
    .object_dist = NAN,
};

// ----------------------------------------------------------------------------------------
// Estimate Planet/Object Distance
// ----------------------------------------------------------------------------------------
// a placeholder for a special function that deducts altitude(also considering location) from object distance

// ----------------------------------------------------------------------------------------
// Track Celestial Object.
// ----------------------------------------------------------------------------------------
// Useful for an object that is known and or has been identified.
// ----------------------------------------------------------------------------------------
void trackObject(double latitude, double longitude,
    double utc_year, double utc_month, double utc_mday,
    double utc_hour, double utc_minute, double utc_second,
    double local_hour, double local_minute, double local_second,
    double altitude, int object_table_i, int object_i) {
    // ----------------------------------------------------------------------------------
    // Use degrees latitude & longitude converted from GNGGA/GNRMC data.
    // ----------------------------------------------------------------------------------
    myAstro.setLatLong(latitude, longitude);
    // ----------------------------------------------------------------------------------
    // RTC should be UTC (GMT).
    // ----------------------------------------------------------------------------------
    myAstro.setGMTdate((int)utc_year, (int)utc_month, (int)utc_mday);
    myAstro.setGMTtime((int)utc_hour, (int)utc_minute, (float)utc_second);
    // ----------------------------------------------------------------------------------
    // Set/reject DST.
    // ----------------------------------------------------------------------------------
    // myAstro.rejectDST();
    // myAstro.setDST();
    myAstro.useAutoDST(); // make optional and or use user defined UTC offset time.
    // ----------------------------------------------------------------------------------
    // Local time (RTC+-).
    // ----------------------------------------------------------------------------------
    myAstro.setLocalTime((int)local_hour, (int)local_minute, (float)local_second);
    // ----------------------------------------------------------------------------------
    // Elevation (experimental).
    // ----------------------------------------------------------------------------------
    myAstro.setElevationM(altitude);
    myAstro.spData.DegreesAltitudeOffsetByElevationM = myAstro.inRange90(myAstro.getDegreesAltitudeOffsetByElevationM(altitude));

    if      (object_table_i == INDEX_SIDEREAL_STAR_TABLE) { myAstroObj.selectStarTable(object_i); }
    else if (object_table_i == INDEX_SIDEREAL_NGC_TABLE) { myAstroObj.selectNGCTable(object_i); }
    else if (object_table_i == INDEX_SIDEREAL_IC_TABLE) { myAstroObj.selectICTable(object_i); }
    else if (object_table_i == INDEX_SIDEREAL_MESSIER_TABLE) { myAstroObj.selectMessierTable(object_i); }
    else if (object_table_i == INDEX_SIDEREAL_CALDWELL_TABLE) { myAstroObj.selectCaldwellTable(object_i); }
    else if (object_table_i == INDEX_SIDEREAL_HERSHEL400_TABLE) { myAstroObj.selectHershel400Table(object_i); }
    else if (object_table_i == INDEX_SIDEREAL_OTHER_OBJECTS_TABLE) { myAstroObj.selectOtherObjectsTable(object_i); }

    myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
    myAstro.doRAdec2AltAz();
    siderealObjectData.object_ra = myAstro.getRAdec();
    siderealObjectData.object_dec = myAstro.getDeclinationDec();
    siderealObjectData.object_az = myAstro.getAzimuth();
    siderealObjectData.object_alt = myAstro.getAltitude();
    myAstro.doXRiseSetTimes(0.0); // set 0 for stars. consider non zero values for planets, galaxies, etc.
    siderealObjectData.object_r = myAstro.getRiseTime();
    siderealObjectData.object_s = myAstro.getSetTime();
}

// ----------------------------------------------------------------------------------------
// Set Object Name.
// ----------------------------------------------------------------------------------------
void setObjectStarName() {
    if (myAstroObj.getIdentifiedObjectNumber() <= SObjectsstars_names_num) {
    memset(siderealObjectData.object_name, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_name, myAstroObj.printStarName(myAstroObj.getIdentifiedObjectNumber()));
    }
}
void setObjectMessierName() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectsmessier_names_num) {
    memset(siderealObjectData.object_name, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_name, myAstroObj.printMessierName(myAstroObj.getAltIdentifiedObjectNumber()));
    }
}
void setObjectCaldwellName() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectcaldwell_names_num) {
    memset(siderealObjectData.object_name, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_name, myAstroObj.printCaldwellName(myAstroObj.getAltIdentifiedObjectNumber()));
    }
}
// ----------------------------------------------------------------------------------------
// Set Object Type.
// ----------------------------------------------------------------------------------------
void setObjectStarType() {
    if (myAstroObj.getIdentifiedObjectNumber() <= SObjectsstars_names_num) {
    memset(siderealObjectData.object_type, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_type, myAstroObj.printStarType(myAstroObj.getIdentifiedObjectNumber()));
    }
}
void setObjectNGCType() {
    if (myAstroObj.getIdentifiedObjectNumber() <= SObjectsNGC_names_num) {
    memset(siderealObjectData.object_type, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_type, myAstroObj.printNGCType(myAstroObj.getIdentifiedObjectNumber()));
    }
}
void setObjectICType() {
    if (myAstroObj.getIdentifiedObjectNumber() <= SObjectsIC_names_num) {
    memset(siderealObjectData.object_type, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_type, myAstroObj.printICType(myAstroObj.getIdentifiedObjectNumber()));
    }
}
void setObjectMessierType() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectsmessier_names_num) {
    memset(siderealObjectData.object_type, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_type, myAstroObj.printMessierType(myAstroObj.getAltIdentifiedObjectNumber()));
    }
}
void setObjectCaldwelllType() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectcaldwell_names_num) {
    memset(siderealObjectData.object_type, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_type, myAstroObj.printCaldwellType(myAstroObj.getAltIdentifiedObjectNumber()));
    }
}
void setObjectHerschelType() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectHerschel400_names_num) {
    memset(siderealObjectData.object_type, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_type, myAstroObj.printHerschel400Type(myAstroObj.getAltIdentifiedObjectNumber()));
    }
}
// ----------------------------------------------------------------------------------------
// Set Object Constellation.
// ----------------------------------------------------------------------------------------
// void setObjectStarConstellation() {
//     if (myAstroObj.getIdentifiedObjectNumber() <= SObjectsstars_names_num) {
//     memset(siderealObjectData.object_con, 0, MAX_GLOBAL_ELEMENT_SIZE);
//     strcpy(siderealObjectData.object_con, myAstroObj.printStarCon(myAstroObj.getIdentifiedObjectNumber()));
//     }
// }
void setObjectNGCConstellation() {
    if (myAstroObj.getIdentifiedObjectNumber() <= SObjectsNGC_names_num) {
    memset(siderealObjectData.object_con, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_con, myAstroObj.printNGCCon(myAstroObj.getIdentifiedObjectNumber()));
    }
}
void setObjectICConstellation() {
    if (myAstroObj.getIdentifiedObjectNumber() <= SObjectsIC_names_num) {
    memset(siderealObjectData.object_con, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_con, myAstroObj.printICCon(myAstroObj.getIdentifiedObjectNumber()));
    }
}
void setObjectMessierConstellation() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectsmessier_names_num) {
    memset(siderealObjectData.object_con, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_con, myAstroObj.printMessierCon(myAstroObj.getAltIdentifiedObjectNumber()));
    }
}
void setObjectCaldwellConstellation() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectcaldwell_names_num) {
    memset(siderealObjectData.object_con, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_con, myAstroObj.printCaldwellCon(myAstroObj.getAltIdentifiedObjectNumber()));
    }
}
void setObjectHerschelConstellation() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectHerschel400_names_num) {
    memset(siderealObjectData.object_con, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_con, myAstroObj.printHerschel400Con(myAstroObj.getAltIdentifiedObjectNumber()));
    }
}
// ----------------------------------------------------------------------------------------
// Set Object Distance.
// ----------------------------------------------------------------------------------------
void setObjectStarDist() {
    if (myAstroObj.getIdentifiedObjectNumber() <= SObjectsstars_names_num) {
    siderealObjectData.object_dist = myAstroObj.printStarDist(myAstroObj.getIdentifiedObjectNumber());
    }
}
void setObjectMessierDist() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectsmessier_names_num) {
    siderealObjectData.object_dist = myAstroObj.printMessierDist(myAstroObj.getAltIdentifiedObjectNumber());
    }
}
void setObjectCaldwellDist() {
    if (myAstroObj.getAltIdentifiedObjectNumber() <= SObjectcaldwell_names_num) {
    siderealObjectData.object_dist = myAstroObj.printCaldwellDist(myAstroObj.getAltIdentifiedObjectNumber());
    }
}
// ----------------------------------------------------------------------------------------
// Set Object Table Name.
// ----------------------------------------------------------------------------------------
void setObjectTableName() {
    memset(siderealObjectData.object_table_name, 0, MAX_GLOBAL_ELEMENT_SIZE);
    strcpy(siderealObjectData.object_table_name, siderealObjectData.object_table[siderealObjectData.object_table_i]);
}
// ----------------------------------------------------------------------------------------
// Set Object ID.
// ----------------------------------------------------------------------------------------
void setID() {
    siderealObjectData.object_number = myAstroObj.getIdentifiedObjectNumber();
}
void setAltID() {
    siderealObjectData.object_number = myAstroObj.getAltIdentifiedObjectNumber();
}

void clearAllObjects() {
    siderealObjectData.object_ra = NAN;
    siderealObjectData.object_dec = NAN;
    siderealObjectData.object_az = NAN;
    siderealObjectData.object_alt = NAN;
    siderealObjectData.object_r = NAN;
    siderealObjectData.object_s = NAN;
    siderealObjectData.object_dist = NAN;
    memset(siderealObjectData.object_table_name, 0, MAX_GLOBAL_ELEMENT_SIZE);
    memset(siderealObjectData.object_name, 0, MAX_GLOBAL_ELEMENT_SIZE);
    memset(siderealObjectData.object_type, 0, MAX_GLOBAL_ELEMENT_SIZE);
    memset(siderealObjectData.object_con, 0, MAX_GLOBAL_ELEMENT_SIZE);
    memset(siderealObjectData.object_desc, 0, MAX_GLOBAL_ELEMENT_SIZE);
}

void setStars() {
    clearAllObjects();
    setObjectTableName();
    setID();
    setObjectStarName();
    setObjectStarType();
    // setObjectStarConstellation();
    setObjectStarDist();
    // distance from earth
    // distance from system
    // magnitude from earth
    // magnitude from system
}

void setNGC() {
    clearAllObjects();
    setObjectTableName();
    setID();
    setObjectNGCType();
    setObjectNGCConstellation(); 
    // distance
    // distance from system
    // magnitude from earth
    // magnitude from system
}

void setIC() {
    clearAllObjects();
    setObjectTableName();
    setID();
    setObjectICType();
    setObjectICConstellation();
    // distance from earth
    // distance from system
    // magnitude from earth
    // magnitude from system
}

void setOther() {
    clearAllObjects();
    setObjectTableName();
    setID();
    // name
    // type
    // constellation
    // distance from earth
    // distance from system
    // magnitude from earth
    // magnitude from system
}

void setMessier() {
    clearAllObjects();
    setObjectTableName();
    setAltID();
    setObjectMessierName();
    setObjectMessierType();
    setObjectMessierConstellation();
    setObjectMessierDist();
    // distance from system
    // magnitude from earth
    // magnitude from system
}

void setCaldwell() {
    clearAllObjects();
    setObjectTableName();
    setAltID();
    setObjectCaldwellName();
    setObjectCaldwelllType();
    setObjectCaldwellConstellation();
    setObjectCaldwellDist();
    // distance from system
    // magnitude from earth
    // magnitude from system
}

void setHerschel400() {
    clearAllObjects();
    setObjectTableName();
    setAltID();
    setObjectHerschelType();
    setObjectHerschelConstellation();
    // distance from earth (ngc)
    // distance from system
    // magnitude from earth
    // magnitude from system
}

// ----------------------------------------------------------------------------------------
// Identify Object.
// ----------------------------------------------------------------------------------------
// Useful for arbitray identification predicated upon manual input and or attitude input.
// ----------------------------------------------------------------------------------------
void IdentifyObject(int ra_hour, int ra_min, float ra_sec, int dec_d, int dec_m, float dec_s) {
    siderealObjectData.object_table_i= -1;
    siderealObjectData.object_number= -1;
    myAstroObj.setRAdec(myAstro.decimalDegrees(ra_hour, ra_min, ra_sec), myAstro.decimalDegrees(dec_d, dec_m, dec_s));
    myAstro.doRAdec2AltAz();
    myAstroObj.identifyObject();
    clearAllObjects();
    switch (myAstroObj.getIdentifiedObjectTable()) {
        case 1: siderealObjectData.object_table_i = INDEX_SIDEREAL_STAR_TABLE; setStars(); break; // Star
        case 2: siderealObjectData.object_table_i = INDEX_SIDEREAL_NGC_TABLE; setNGC(); break; // NGC
        case 3: siderealObjectData.object_table_i = INDEX_SIDEREAL_IC_TABLE; setIC(); break; // IC
        case 7: siderealObjectData.object_table_i = INDEX_SIDEREAL_OTHER_OBJECTS_TABLE; setOther(); break; // Other
        default: clearAllObjects();
    }
    if (myAstroObj.getAltIdentifiedObjectTable()) {
        switch (myAstroObj.getAltIdentifiedObjectTable()) {
            case 4: siderealObjectData.object_table_i = INDEX_SIDEREAL_MESSIER_TABLE; setMessier(); break; // Messier
            case 5: siderealObjectData.object_table_i = INDEX_SIDEREAL_CALDWELL_TABLE; setCaldwell(); break; // Caldwell
            case 6: siderealObjectData.object_table_i = INDEX_SIDEREAL_HERSHEL400_TABLE; setHerschel400(); break; // Herschel 400
            default: clearAllObjects();
        }
    }
}

// ----------------------------------------------------------------------------------------
// Track Planets.
// ----------------------------------------------------------------------------------------
void trackSun(void) {
    myAstro.doSun();
    siderealPlanetData.sun_ra = myAstro.getRAdec();
    siderealPlanetData.sun_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.sun_az = myAstro.getAzimuth();
    siderealPlanetData.sun_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    siderealPlanetData.sun_helio_ecliptic_lat = myAstro.getHelioLat();
    siderealPlanetData.sun_helio_ecliptic_long = myAstro.getHelioLong();
    siderealPlanetData.sun_radius_vector = myAstro.getRadiusVec();
    siderealPlanetData.sun_distance = myAstro.getDistance();
    siderealPlanetData.sun_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.sun_ecliptic_long = myAstro.getEclipticLongitude();
    siderealPlanetData.earth_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.earth_ecliptic_long = myAstro.getEclipticLongitude();
    myAstro.doSunRiseSetTimes();
    siderealPlanetData.sun_r = myAstro.getSunriseTime();
    siderealPlanetData.sun_s = myAstro.getSunsetTime();
}

void trackMoon(void) {
    myAstro.doMoon();
    siderealPlanetData.moon_ra = myAstro.getRAdec();
    siderealPlanetData.moon_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.moon_az = myAstro.getAzimuth();
    siderealPlanetData.moon_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    myAstro.doMoonRiseSetTimes();
    siderealPlanetData.moon_r = myAstro.getMoonriseTime();
    siderealPlanetData.moon_s = myAstro.getMoonsetTime();
    siderealPlanetData.moon_p = myAstro.getMoonPhase();
    siderealPlanetData.moon_lum = myAstro.getLunarLuminance();
}

void trackMercury(void) {
    myAstro.doMercury();
    siderealPlanetData.mercury_ra = myAstro.getRAdec();
    siderealPlanetData.mercury_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.mercury_az = myAstro.getAzimuth();
    siderealPlanetData.mercury_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    siderealPlanetData.mercury_helio_ecliptic_lat = myAstro.getHelioLat();
    siderealPlanetData.mercury_helio_ecliptic_long = myAstro.getHelioLong();
    siderealPlanetData.mercury_radius_vector = myAstro.getRadiusVec();
    siderealPlanetData.mercury_distance = myAstro.getDistance();
    siderealPlanetData.mercury_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.mercury_ecliptic_long = myAstro.getEclipticLongitude();
    myAstro.doXRiseSetTimes(1.454441e-2); // toDo: actual horizonatal displacement
    siderealPlanetData.mercury_r = myAstro.getRiseTime();
    siderealPlanetData.mercury_s = myAstro.getSetTime();
}

void trackVenus(void) {
    myAstro.doVenus();
    siderealPlanetData.venus_ra = myAstro.getRAdec();
    siderealPlanetData.venus_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.venus_az = myAstro.getAzimuth();
    siderealPlanetData.venus_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    siderealPlanetData.venus_helio_ecliptic_lat = myAstro.getHelioLat();
    siderealPlanetData.venus_helio_ecliptic_long = myAstro.getHelioLong();
    siderealPlanetData.venus_radius_vector = myAstro.getRadiusVec();
    siderealPlanetData.venus_distance = myAstro.getDistance();
    siderealPlanetData.venus_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.venus_ecliptic_long = myAstro.getEclipticLongitude();
    myAstro.doXRiseSetTimes(1.454441e-2); // toDo: actual horizonatal displacement
    siderealPlanetData.venus_r = myAstro.getRiseTime();
    siderealPlanetData.venus_s = myAstro.getSetTime();
}

void trackMars(void) {
    myAstro.doMars();
    siderealPlanetData.mars_ra = myAstro.getRAdec();
    siderealPlanetData.mars_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.mars_az = myAstro.getAzimuth();
    siderealPlanetData.mars_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    siderealPlanetData.mars_helio_ecliptic_lat = myAstro.getHelioLat();
    siderealPlanetData.mars_helio_ecliptic_long = myAstro.getHelioLong();
    siderealPlanetData.mars_radius_vector = myAstro.getRadiusVec();
    siderealPlanetData.mars_distance = myAstro.getDistance();
    siderealPlanetData.mars_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.mars_ecliptic_long = myAstro.getEclipticLongitude();
    myAstro.doXRiseSetTimes(1.454441e-2); // toDo: actual horizonatal displacement
    siderealPlanetData.mars_r = myAstro.getRiseTime();
    siderealPlanetData.mars_s = myAstro.getSetTime();
}

void trackJupiter(void) {
    myAstro.doJupiter();
    siderealPlanetData.jupiter_ra = myAstro.getRAdec();
    siderealPlanetData.jupiter_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.jupiter_az = myAstro.getAzimuth();
    siderealPlanetData.jupiter_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    siderealPlanetData.jupiter_helio_ecliptic_lat = myAstro.getHelioLat();
    siderealPlanetData.jupiter_helio_ecliptic_long = myAstro.getHelioLong();
    siderealPlanetData.jupiter_radius_vector = myAstro.getRadiusVec();
    siderealPlanetData.jupiter_distance = myAstro.getDistance();
    siderealPlanetData.jupiter_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.jupiter_ecliptic_long = myAstro.getEclipticLongitude();
    myAstro.doXRiseSetTimes(1.454441e-2); // toDo: actual horizonatal displacement
    siderealPlanetData.jupiter_r = myAstro.getRiseTime();
    siderealPlanetData.jupiter_s = myAstro.getSetTime();
}

void trackSaturn(void) {
    myAstro.doSaturn();
    siderealPlanetData.saturn_ra = myAstro.getRAdec();
    siderealPlanetData.saturn_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.saturn_az = myAstro.getAzimuth();
    siderealPlanetData.saturn_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    siderealPlanetData.saturn_helio_ecliptic_lat = myAstro.getHelioLat();
    siderealPlanetData.saturn_helio_ecliptic_long = myAstro.getHelioLong();
    siderealPlanetData.saturn_radius_vector = myAstro.getRadiusVec();
    siderealPlanetData.saturn_distance = myAstro.getDistance();
    siderealPlanetData.saturn_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.saturn_ecliptic_long = myAstro.getEclipticLongitude();
    myAstro.doXRiseSetTimes(1.454441e-2); // toDo: actual horizonatal displacement
    siderealPlanetData.saturn_r = myAstro.getRiseTime();
    siderealPlanetData.saturn_s = myAstro.getSetTime();
}

void trackUranus(void) {
    myAstro.doUranus();
    siderealPlanetData.uranus_ra = myAstro.getRAdec();
    siderealPlanetData.uranus_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.uranus_az = myAstro.getAzimuth();
    siderealPlanetData.uranus_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    siderealPlanetData.uranus_helio_ecliptic_lat = myAstro.getHelioLat();
    siderealPlanetData.uranus_helio_ecliptic_long = myAstro.getHelioLong();
    siderealPlanetData.uranus_radius_vector = myAstro.getRadiusVec();
    siderealPlanetData.uranus_distance = myAstro.getDistance();
    siderealPlanetData.uranus_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.uranus_ecliptic_long = myAstro.getEclipticLongitude();
    myAstro.doXRiseSetTimes(1.454441e-2); // toDo: actual horizonatal displacement
    siderealPlanetData.uranus_r = myAstro.getRiseTime();
    siderealPlanetData.uranus_s = myAstro.getSetTime();
}

void trackNeptune(void) {
    myAstro.doNeptune();
    siderealPlanetData.neptune_ra = myAstro.getRAdec();
    siderealPlanetData.neptune_dec = myAstro.getDeclinationDec();
    myAstro.doRAdec2AltAz();
    siderealPlanetData.neptune_az = myAstro.getAzimuth();
    siderealPlanetData.neptune_alt = myAstro.getAltitude() + myAstro.spData.DegreesAltitudeOffsetByElevationM;
    siderealPlanetData.neptune_helio_ecliptic_lat = myAstro.getHelioLat();
    siderealPlanetData.neptune_helio_ecliptic_long = myAstro.getHelioLong();
    siderealPlanetData.neptune_radius_vector = myAstro.getRadiusVec();
    siderealPlanetData.neptune_distance = myAstro.getDistance();
    siderealPlanetData.neptune_ecliptic_lat = myAstro.getEclipticLatitude();
    siderealPlanetData.neptune_ecliptic_long = myAstro.getEclipticLongitude();
    myAstro.doXRiseSetTimes(1.454441e-2); // toDo: actual horizonatal displacement
    siderealPlanetData.neptune_r = myAstro.getRiseTime();
    siderealPlanetData.neptune_s = myAstro.getSetTime();
}

// ----------------------------------------------------------------------------------------
// Clear Planet Data.
// ----------------------------------------------------------------------------------------
void clearSun(void) {
    siderealPlanetData.sun_ra = NAN;
    siderealPlanetData.sun_dec = NAN;
    siderealPlanetData.sun_az = NAN;
    siderealPlanetData.sun_alt = NAN;
    siderealPlanetData.sun_r = NAN;
    siderealPlanetData.sun_s = NAN;
}

void clearMoon(void) {
    siderealPlanetData.moon_ra = NAN;
    siderealPlanetData.moon_dec = NAN;
    siderealPlanetData.moon_az = NAN;
    siderealPlanetData.moon_alt = NAN;
    siderealPlanetData.moon_r = NAN;
    siderealPlanetData.moon_s = NAN;
    siderealPlanetData.moon_p = NAN;
    siderealPlanetData.moon_lum = NAN;
}

void clearMercury(void) {
    siderealPlanetData.mercury_ra = NAN;
    siderealPlanetData.mercury_dec = NAN;
    siderealPlanetData.mercury_az = NAN;
    siderealPlanetData.mercury_alt = NAN;
    siderealPlanetData.mercury_r = NAN;
    siderealPlanetData.mercury_s = NAN;
    siderealPlanetData.mercury_helio_ecliptic_lat = NAN;
    siderealPlanetData.mercury_helio_ecliptic_long = NAN;
    siderealPlanetData.mercury_radius_vector = NAN;
    siderealPlanetData.mercury_distance = NAN;
    siderealPlanetData.mercury_ecliptic_lat = NAN;
    siderealPlanetData.mercury_ecliptic_long = NAN;
}

void clearVenus(void) {
    siderealPlanetData.venus_ra = NAN;
    siderealPlanetData.venus_dec = NAN;
    siderealPlanetData.venus_az = NAN;
    siderealPlanetData.venus_alt = NAN;
    siderealPlanetData.venus_r = NAN;
    siderealPlanetData.venus_s = NAN;
    siderealPlanetData.venus_helio_ecliptic_lat = NAN;
    siderealPlanetData.venus_helio_ecliptic_long = NAN;
    siderealPlanetData.venus_radius_vector = NAN;
    siderealPlanetData.venus_distance = NAN;
    siderealPlanetData.venus_ecliptic_lat = NAN;
    siderealPlanetData.venus_ecliptic_long = NAN;
}

void clearMars(void) {
    siderealPlanetData.mars_ra = NAN;
    siderealPlanetData.mars_dec = NAN;
    siderealPlanetData.mars_az = NAN;
    siderealPlanetData.mars_alt = NAN;
    siderealPlanetData.mars_r = NAN;
    siderealPlanetData.mars_s = NAN;
    siderealPlanetData.mars_helio_ecliptic_lat = NAN;
    siderealPlanetData.mars_helio_ecliptic_long = NAN;
    siderealPlanetData.mars_radius_vector = NAN;
    siderealPlanetData.mars_distance = NAN;
    siderealPlanetData.mars_ecliptic_lat = NAN;
    siderealPlanetData.mars_ecliptic_long = NAN;
}

void clearJupiter(void) {
    siderealPlanetData.jupiter_ra = NAN;
    siderealPlanetData.jupiter_dec = NAN;
    siderealPlanetData.jupiter_az = NAN;
    siderealPlanetData.jupiter_alt = NAN;
    siderealPlanetData.jupiter_r = NAN;
    siderealPlanetData.jupiter_s = NAN;
    siderealPlanetData.jupiter_helio_ecliptic_lat = NAN;
    siderealPlanetData.jupiter_helio_ecliptic_long = NAN;
    siderealPlanetData.jupiter_radius_vector = NAN;
    siderealPlanetData.jupiter_distance = NAN;
    siderealPlanetData.jupiter_ecliptic_lat = NAN;
    siderealPlanetData.jupiter_ecliptic_long = NAN;
}

void clearSaturn(void) {
    siderealPlanetData.saturn_ra = NAN;
    siderealPlanetData.saturn_dec = NAN;
    siderealPlanetData.saturn_az = NAN;
    siderealPlanetData.saturn_alt = NAN;
    siderealPlanetData.saturn_r = NAN;
    siderealPlanetData.saturn_s = NAN;
    siderealPlanetData.saturn_helio_ecliptic_lat = NAN;
    siderealPlanetData.saturn_helio_ecliptic_long = NAN;
    siderealPlanetData.saturn_radius_vector = NAN;
    siderealPlanetData.saturn_distance = NAN;
    siderealPlanetData.saturn_ecliptic_lat = NAN;
    siderealPlanetData.saturn_ecliptic_long = NAN;
}

void clearUranus(void) {
    siderealPlanetData.uranus_ra = NAN;
    siderealPlanetData.uranus_dec = NAN;
    siderealPlanetData.uranus_az = NAN;
    siderealPlanetData.uranus_alt = NAN;
    siderealPlanetData.uranus_r = NAN;
    siderealPlanetData.uranus_s = NAN;
    siderealPlanetData.uranus_helio_ecliptic_lat = NAN;
    siderealPlanetData.uranus_helio_ecliptic_long = NAN;
    siderealPlanetData.uranus_radius_vector = NAN;
    siderealPlanetData.uranus_distance = NAN;
    siderealPlanetData.uranus_ecliptic_lat = NAN;
    siderealPlanetData.uranus_ecliptic_long = NAN;
}

void clearNeptune(void) {
    siderealPlanetData.neptune_ra = NAN;
    siderealPlanetData.neptune_dec = NAN;
    siderealPlanetData.neptune_az = NAN;
    siderealPlanetData.neptune_alt = NAN;
    siderealPlanetData.neptune_r = NAN;
    siderealPlanetData.neptune_s = NAN;
    siderealPlanetData.neptune_helio_ecliptic_lat = NAN;
    siderealPlanetData.neptune_helio_ecliptic_long = NAN;
    siderealPlanetData.neptune_radius_vector = NAN;
    siderealPlanetData.neptune_distance = NAN;
    siderealPlanetData.neptune_ecliptic_lat = NAN;
    siderealPlanetData.neptune_ecliptic_long = NAN;
}

void clearTrackPlanets(void) {
    clearSun();
    clearMoon();
    clearMercury();
    clearVenus();
    clearMars();
    clearJupiter();
    clearSaturn();
    clearUranus();
    clearNeptune();
}

// ----------------------------------------------------------------------------------------
// Track All Planets.
// ----------------------------------------------------------------------------------------
void trackPlanets(double latitude, double longitude,
    double utc_year, double utc_month, double utc_mday,
    double utc_hour, double utc_minute, double utc_second,
    double local_hour, double local_minute, double local_second,
    double altitude) {
    // ----------------------------------------------------------------------------------
    // Use degrees latitude & longitude converted from GNGGA/GNRMC data.
    // ----------------------------------------------------------------------------------
    myAstro.setLatLong(latitude, longitude);
    // ----------------------------------------------------------------------------------
    // RTC should be UTC (GMT).
    // ----------------------------------------------------------------------------------
    myAstro.setGMTdate((int)utc_year, (int)utc_month, (int)utc_mday);
    myAstro.setGMTtime((int)utc_hour, (int)utc_minute, (float)utc_second);
    // ----------------------------------------------------------------------------------
    // Set/reject DST.
    // ----------------------------------------------------------------------------------
    // myAstro.rejectDST();
    // myAstro.setDST();
    myAstro.useAutoDST(); // make optional and or use user defined UTC offset time.
    // ----------------------------------------------------------------------------------
    // Local time (RTC+-).
    // ----------------------------------------------------------------------------------
    myAstro.setLocalTime((int)local_hour, (int)local_minute, (float)local_second);
    // ----------------------------------------------------------------------------------
    // Elevation (experimental).
    // ----------------------------------------------------------------------------------
    myAstro.setElevationM(altitude);
    myAstro.spData.DegreesAltitudeOffsetByElevationM = myAstro.inRange90(myAstro.getDegreesAltitudeOffsetByElevationM(altitude));
    
    // -------------------------------------------------------
    // Do planet elements sun, once before doing other planets.
    // -------------------------------------------------------
    myAstro.doPlanetElements();
    myAstro.doSun();
    // -------------------------------------------------------
    // Now do other plans.
    // -------------------------------------------------------
    trackSun();
    trackMoon();
    trackMercury();
    trackVenus();
    trackMars();
    trackJupiter();
    trackSaturn();
    trackUranus();
    trackNeptune();
}

void myAstroBegin(void) {
    myAstro.begin();
}