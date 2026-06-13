/*
    Custom Mapping Library. Written by Benjamin Jack Cullen.
*/

#ifndef CUSTOMMAPPING_H
#define CUSTOMMAPPING_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/**
 * @struct CustomMappingStruct
 */
struct CustomMappingStruct {
    int32_t center_map_x0; // Negative axis mapped value.
    int32_t center_map_x1; // Positive axis mapped value.
    char char_map_value[MAX_MAPPABLE_VALUES][MAX_GLOBAL_ELEMENT_SIZE]; // Mapping names.

    /**
     * Map Mode.
     * Mode 0 maps min to max.
     * Mode 1 maps x0 (min to center).
     * Mode 2 maps x1 (center to max).
     * Aligns with map_mode, mapping_config, mapped_value.
     */
    int map_mode[1][MAX_MAP_SLOTS];

    /**
     * Map Mode Names.
     * 
     * Mode 0: [ 0: Title ] [ 1-5: C0-C5 ]
     * Mode 1: [ 0: Title ] [ 1-5: C0-C5 ]
     */
    char char_map_mode_names[3][MAX_GLOBAL_ELEMENT_SIZE];

    char char_map_mode_config_names[3][7][MAX_GLOBAL_ELEMENT_SIZE];

    /**
     * Map Configuration.
     *
     * Map Mode 0:
     * 0 : Value index         : (default 0: digital low).
     * 1 : Expected value min  : (defalut 0: digital low).
     * 2 : Expected value max  : (defalut 1: digital high).
     * 3 : Output value min    : (defalut 0: digital low).
     * 4 : Output value max    : (defalut 1: digital high).
     * 5 : Null.
     *
     * Map Mode 1 & 2:
     * 0 : Value index : (default 0).
     * 1 : Center      : approximate center value.
     * 2 : Neg_range   : 0 to approximate center value.
     * 3 : Pos_range   : ADC max - neg range.
     * 4 : Output_max  : maximum resulting value.
     * 5 : DEADZONE    : expected flutuation at center.
     *
     * Aligns with map_mode, mapping_config, mapped_value.
     */
    int32_t mapping_config[1][MAX_MAP_SLOTS][MAX_MAPPING_PARAMETERS];

    /**
     * Mapped results.
     *
     * Aligns with map_mode, mapping_config, mapped_value.
     */
    signed long mapped_value[1][MAX_MAP_SLOTS];
};
extern struct CustomMappingStruct mappingData;

/**
 * Centered Map.
 * @param map_slot Specify mapping_config slot
 * @param var Value to be mapped
 * @return No return
 */
void center_map(int map_slot, int32_t var);

/**
 * Map Values.
 * Maps values according to map_mode.
 * @return No return
 */
void map_values(void);

/**
 * @brief Get Target value to be mapped.
 * 
 * @param mapslot Specify target mapslot.
 * @return Char value is returned so that map value types are returned as a uniform type.
 */
double get_mapping_input_value(int map_slot);

/**
 * Reset a single map slot.
 * @param map_slot Specify map slot
 * @return No return
 */
void set_mapping_default(int map_slot);

/**
 * Reset all map slots.
 * Overrides all computer assists.
 * Clears all map configurations.
 * @return No return
 */
void set_all_mapping_default(void);

#endif