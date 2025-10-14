// /*
//     Volcanos Library. Written by Benjamin Jack Cullen.

//     This library has been designed for low memory requirements.
// */

// #ifndef VOLCANOS_H
// #define VOLCANOS_H

// #ifdef __cplusplus
// extern "C" {
// #endif

// #include "config.h"

// // ----------------------------------------------------------------------------------------
// // Volcano data arrays
// // ----------------------------------------------------------------------------------------
// extern const char volcano_locations[MAX_LOCATIONS][MAX_GLOBAL_ELEMENT_SIZE];
// extern const char volcano_types[MAX_TYPES][MAX_GLOBAL_ELEMENT_SIZE];
// extern const int volcano_elevations[MAX_VOLCANOS];
// extern const float volcano_lat_long[MAX_VOLCANOS][2];
// extern const int volcano_index[MAX_VOLCANOS][2];

// // ----------------------------------------------------------------------------------------
// // Function prototypes
// // ----------------------------------------------------------------------------------------
// const char* getVolcanoName(int index);
// const char* getVolcanoLocation(int index);
// const char* getVolcanoType(int index);
// int getVolcanoElevation(int index);
// void getVolcanoLatLong(int index, float* lat, float* lon);

// #ifdef __cplusplus
// }
// #endif

// #endif