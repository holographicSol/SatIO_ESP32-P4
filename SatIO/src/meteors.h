/*
  Meteors Library. Written by Benjamin Jack Cullen.

*/

#ifndef METEORS_H
#define METEORS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

extern const char meteor_shower_names[MAX_METEOR_SHOWERS][MAX_GLOBAL_ELEMENT_SIZE];
// ----------------------------------------------------------------------------------------
/*
  Specify meteor datetime range (up to 2 concurrent calender months).
  {0: month_start, 1: day_start}, {0: month_end, 1: day_end}.
*/
// ----------------------------------------------------------------------------------------
extern const int meteor_shower_datetime[MAX_METEOR_SHOWERS][MAX_METEOR_SHOWER_DATETIME][MAX_METEOR_SHOWER_DATETIME_ELEMENTS];
// ----------------------------------------------------------------------------------------
/*
  Specify peak meteor datetime range (up to 2 concurrent calender months).
  {0: month_start, 1: day_start, 2: day_end}, {0: month_end, 1: day_start, 2: day_end}.
*/
// ----------------------------------------------------------------------------------------
extern const int meteor_shower_peaks[MAX_METEOR_SHOWERS][MAX_METEOR_SHOWER_PEAK_DATETIME][MAX_METEOR_SHOWER_PEAK_DATETIME_ELEMENTS];
// ----------------------------------------------------------------------------------------
/*
  Result values.
  0: In datetime range.
  1: In peak datetime range.
*/
// ----------------------------------------------------------------------------------------
extern bool meteor_shower_warning_system[MAX_METEOR_SHOWERS][MAX_METEOR_RESULT_ELEMENTS];

// ------------------------------------------------------------------------
// Function Prototypes.
// ------------------------------------------------------------------------
void setMeteorShowerWarning(int month, int mday);

#ifdef __cplusplus
}
#endif

#endif