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

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
const char* getPartitionTypeName(int type);
const char* getPartitionSubTypeName(int type, int subtype);
void print_partition_table(void);
void print_ram_info(void);

#ifdef __cplusplus
}
#endif

#endif