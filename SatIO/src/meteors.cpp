/*
  Meteors Library. Written by Benjamin Jack Cullen.
*/

#include "meteors.h"

const char meteor_shower_names[MAX_METEOR_SHOWERS][MAX_GLOBAL_ELEMENT_SIZE] = {
  "Quadrantids",   // 0
  "Lyrids",        // 1
  "Eta Aquariids", // 2
  "Perseids",      // 3
  "Orionids",      // 4
  "Leonids",       // 5
  "Geminids",      // 6
  "Ursids"         // 7
};

const int meteor_shower_datetime[MAX_METEOR_SHOWERS]
                                [MAX_METEOR_SHOWER_DATETIME]
                                [MAX_METEOR_SHOWER_DATETIME_ELEMENTS] = {
  // ----------------------------
  // Quadrantids                0
  // ----------------------------
  { {12, 26}, {1, 15}},
  // ----------------------------
  // Lyrids                     1
  // ----------------------------
  { {4, 15},  {4, 25}},
  // ----------------------------
  // Eta Aquariids              2
  // ----------------------------
  { {4, 19},  {5, 28}},
  // ----------------------------
  // Perseids                   3
  // ----------------------------
  { {7, 17},  {8, 23}},
  // ----------------------------
  // Orionids                   4
  // ----------------------------
  { {10, 2},  {11, 7}},
  // ----------------------------
  // Leonids                    5
  // ----------------------------
  { {11, 6},  {11, 30}},
  // ----------------------------
  // Geminids                   6
  // ----------------------------
  { {11, 19},  {12, 24}},
  // ----------------------------
  // Ursids                     7
  // ----------------------------
  { {12, 17},  {12, 26}},
};

const int meteor_shower_peaks[MAX_METEOR_SHOWERS]
                             [MAX_METEOR_SHOWER_PEAK_DATETIME]
                             [MAX_METEOR_SHOWER_PEAK_DATETIME_ELEMENTS] = {
  // ----------------------------
  // Quadrantids                0
  // ----------------------------
  { {1, 3, 4}, {1, 4, 4} },
  // ----------------------------
  // Lyrids                     1
  // ----------------------------
  { {4, 22, 23}, {4, 23, 23} },
  // ----------------------------
  // Eta Aquariids              2
  // ----------------------------
  { {5, 5, 6}, {5, 6, 6} },
  // ----------------------------
  // Perseids                   3
  // ----------------------------
  { {8, 12, 13}, {8, 13, 13} },
  // ----------------------------
  // Orionids                   4
  // ----------------------------
  { {10, 21, 22}, {10, 22, 22} },
  // ----------------------------
  // Leonids                    5
  // ----------------------------
  { {11, 16, 17}, {11, 17, 17} },
  // ----------------------------
  // Geminids                   6
  // ----------------------------
  { {12, 13, 14}, {12, 14, 14} },
  // ----------------------------
  // Ursids                     7
  // ----------------------------
  { {12, 21, 22}, {12, 22, 22} },
};

bool meteor_shower_warning_system[MAX_METEOR_SHOWERS][MAX_METEOR_RESULT_ELEMENTS] = {
    {false, false}, {false, false}, {false, false}, {false, false}, // 0-3
    {false, false}, {false, false}, {false, false}, {false, false}  // 4-7
};

// ----------------------------------------------------------------------------------------
//                                                          METEOR SHOWER IN DATETIME RANGE
// ----------------------------------------------------------------------------------------
// Check and return, does not set.
// Allows spanning up to 2 consecutive calender months.
// ----------------------------------------------------------------------------------------
bool checkMeteorShowerWarning(int key, int month, int mday) {
    bool meteor_warning = false;
    // --------------------------------
    // Meteor shower start.
    // --------------------------------
    if (month == meteor_shower_datetime[key]
                                       [INDEDX_METEOR_DATETIME_START]
                                       [INDEDX_METEOR_DATETIME_MONTH_START]) {
        if (mday >= meteor_shower_datetime[key]
                                          [INDEDX_METEOR_DATETIME_START]
                                          [INDEDX_METEOR_DATETIME_DAY_START]) {
            meteor_warning = true;
        }
    }
    // --------------------------------
    // Meteor shower end.
    // --------------------------------
    if (month == meteor_shower_datetime[key]
                                       [INDEDX_METEOR_DATETIME_END]
                                       [INDEDX_METEOR_DATETIME_MONTH_END]) {
        if (mday <= meteor_shower_datetime[key]
                                          [INDEDX_METEOR_DATETIME_END]
                                          [INDEDX_METEOR_DATETIME_DAY_END]) {
            meteor_warning = true;
        }
    }
    return meteor_warning;
}

// ----------------------------------------------------------------------------------------
//                                                     METEOR SHOWER IN PEAK DATETIME RANGE
// ----------------------------------------------------------------------------------------
// Check and return, does not set.
// Allows spanning up to 2 consecutive calender months.
// ----------------------------------------------------------------------------------------
bool checkMeteorShowerPeakWarning(int key, int month, int mday) {
    bool meteor_peak_warning = false;
    // --------------------------------
    // Peak meteor shower start.
    // --------------------------------
    if (month == meteor_shower_peaks[key]
                                    [INDEDX_METEOR_PEAK_DATETIME_START]
                                    [INDEDX_METEOR_PEAK_DATETIME_MONTH_0_START]) {
        if (mday >= meteor_shower_peaks[key]
                                       [INDEDX_METEOR_PEAK_DATETIME_START]
                                       [INDEDX_METEOR_PEAK_DATETIME_MONTH_0_DAY_START] &&
            mday <= meteor_shower_peaks[key]
                                       [INDEDX_METEOR_PEAK_DATETIME_START]
                                       [INDEDX_METEOR_PEAK_DATETIME_MONTH_0_DAY_END]) {
            meteor_peak_warning = true;
        }
    }
    if (month == meteor_shower_peaks[key]
                                    [INDEDX_METEOR_PEAK_DATETIME_END]
                                    [INDEDX_METEOR_PEAK_DATETIME_MONTH_1_END]) {
        if (mday >= meteor_shower_peaks[key]
                                       [INDEDX_METEOR_PEAK_DATETIME_END]
                                       [INDEDX_METEOR_PEAK_DATETIME_MONTH_1_DAY_START] &&
            mday <= meteor_shower_peaks[key]
                                       [INDEDX_METEOR_PEAK_DATETIME_END]
                                       [INDEDX_METEOR_PEAK_DATETIME_MONTH_1_DAY_END]) {
            meteor_peak_warning = true;
        }
    }
    return meteor_peak_warning;
}

// ----------------------------------------------------------------------------------------
//                                                                SET METEOR SHOWER WARNING
// ----------------------------------------------------------------------------------------
// Set individual meteor warning system elements according to check results.
// ----------------------------------------------------------------------------------------
void setMeteorShowerWarning(int month, int mday) {
    for (int i=0; i < MAX_METEOR_SHOWERS; i++) {
        // ----------------------------
        // Set datetime range bool.
        // ----------------------------
        meteor_shower_warning_system[i][0] = checkMeteorShowerWarning(i, month, mday);
        // ----------------------------
        // Set peak datetime range bool.
        // ----------------------------
        if (meteor_shower_warning_system[i][INDEX_METEOR_RESULT_DATETIME] == true) {
            meteor_shower_warning_system[i][INDEX_METEOR_RESULT_PEAK_DATETIME] =
                checkMeteorShowerPeakWarning(i, month, mday);
        }
    }
}

// ----------------------------------------------------------------------------------------
//                                                                SUM METEOR SHOWER WARNING
// ----------------------------------------------------------------------------------------
// Return true if any meteors in range.
// ----------------------------------------------------------------------------------------
bool sumMeteorShowerWarning() {
    bool meteor_shower_sum = false;
    for (int i=0; i < MAX_METEOR_SHOWERS; i++) {
        if (meteor_shower_warning_system[i][INDEX_METEOR_RESULT_DATETIME] == true) {
            meteor_shower_sum = true;
        }
    }
    return meteor_shower_sum;
}

// ----------------------------------------------------------------------------------------
//                                                                SUM METEOR SHOWER WARNING
// ----------------------------------------------------------------------------------------
// Return true if any meteors in peak range.
// ----------------------------------------------------------------------------------------
bool sumMeteorShowerPeakWarning() {
    bool meteor_shower_peak_sum = false;
    for (int i=0; i < MAX_METEOR_SHOWERS; i++) {
        if (meteor_shower_warning_system[i][INDEX_METEOR_RESULT_PEAK_DATETIME] == true) {
            meteor_shower_peak_sum = true;
        }
    }
    return meteor_shower_peak_sum;
}