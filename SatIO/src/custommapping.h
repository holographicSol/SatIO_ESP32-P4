/*
    Custom Mapping Library. Written by Benjamin Jack Cullen.

    1 : Configure matrixData.mapping_config and select_map_algorithm as needed.
    2 : Call setMappedValues() to process mappable values.
    3 : Access mapped values in matrixData.mapped_value.

    Center Mapping example:
    example map analog stick axis x0 on admplex0 channel 0 into map slot 0: mapping -s 0 -m 1 -c0 16 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
    example map analog stick axis x1 on admplex0 channel 1 into map slot 1: mapping -s 1 -m 2 -c0 17 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
        
    - Utilize a map slot example (passthrough):
        (1) Link matrix switch output to map slots (from previous example).
        Matrix Switch, Map Slot:
        - set output map 0 0
        - set output map 1 1
        (2) Now the output is linked and if/when required can be set as the output for a switch.
        Matrix Switch, Output Mode:
        - set output mode 0 1
        - set output mode 1 1
        - This allows matrix switch 0 and 1 to output mapped results 0 & 1 values to port controller.
    
    - Utilize a map slot example (matrix):
        - Map slots can also be utilized by the matrix with mapped value function.
        - If matrix switch logic returns true then digital 1 can be output.
        - If matrix switch logic returns true then a different mapped value can be output.
        - If matrix switch logic returns true then the same mapped value can be output.
*/

#ifndef CUSTOMMAPPING_H
#define CUSTOMMAPPING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ----------------------------------------------------------------------------------------
// Custom Mapping Structure.
// ----------------------------------------------------------------------------------------
struct CustomMappingStruct {
    int32_t center_map_x0; // Negative axis mapped value.
    int32_t center_map_x1; // Positive axis mapped value.
    char element_name[MAX_MAPPABLE_VALUES][MAX_GLOBAL_ELEMENT_SIZE]; // Mapping names.

    // ------------------------------------------------------------------------------------
    /*
        Map Slot Mode.
            - map_mode[N] mode is allocated for the same mapping_config[N].

        map mode 0 : map min to max
        map mode 1 : center map x0
        map mode 2 : center map x1
    */
    // ------------------------------------------------------------------------------------
    int map_mode[1][MAX_MAP_SLOTS];

    // ------------------------------------------------------------------------------------
    /*
        Slot Index For Matrix Switches.
            - index_mapped_value[N] is allocated for the same matrix switch[N].
            - index_mapped_value[N] can point at at any mapped_value[N].
            - This way any matrix switch can be pointed at any mapped value and or share
              mapped values.
    */
    // ------------------------------------------------------------------------------------
    int index_mapped_value[1][MAX_MAP_SLOTS];

    // ------------------------------------------------------------------------------------
    /*
    Programmable mapping:

    Map Mode 0:
    0 : Value index        (default 0: digital low).
    1 : Expected value min (defalut 0: digital low).
    2 : Expected value max (defalut 1: digital high).
    3 : Output value min   (defalut 0: digital low).
    4 : Output value max   (defalut 1: digital high).
    5 : Null.

    Map Mode 1 & 2:
    0 : Value index  (default 0).
    1 : Center     : approximate center value.
    2 : Neg_range  : 0 to approximate center value.
    3 : Pos_range  : ADC max - neg range.
    4 : Output_max : maximum resulting value.
    5 : DEADZONE   : expected flutuation at center.

    */
    // ------------------------------------------------------------------------------------
    int32_t mapping_config[1][MAX_MAP_SLOTS][MAX_MAPPING_PARAMETERS];

    // ------------------------------------------------------------------------------------
    // Results from mapping.
    // ------------------------------------------------------------------------------------
    signed long mapped_value[1][MAX_MAP_SLOTS];
};
extern struct CustomMappingStruct mappingData;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
void centeredMap(int map_slot, float var);
void setMappedValues(void);
void zero_mapping(void);

#ifdef __cplusplus
}
#endif

#endif