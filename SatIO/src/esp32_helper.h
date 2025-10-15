/*
    ESP32 Helper Library. Written by Benjamin Jack Cullen.

    Provides some ESP32 specific functionality.
*/

#ifndef ESP32_HELPER_H
#define ESP32_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

/**
 * Get name of partition type.
 * @param type
 * @return Returns name
 */
const char* get_partition_type_name(int type);

/**
 * Get name of partition sub type.
 * @param type
 * @return Returns name
 */
const char* get_partition_sub_type_name(int type, int subtype);

/**
 * Print partition table information.
 * @return Returns None
 */
void print_partition_table(void);

/**
 * Print RAM information.
 * @return Returns None
 */
void print_ram_info(void);

#ifdef __cplusplus
}
#endif

#endif