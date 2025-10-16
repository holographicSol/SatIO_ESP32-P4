/*
  SatioFile - Written By Benjamin Jack Cullen.
*/

#ifndef SATIO_FILE_HELPER_H
#define SATIO_FILE_HELPER_H

#include <stdint.h>
#include <FS.h>
#include "SD_MMC.h"
#include "SPIFFS.h"
#include "config.h"

#define MAX_MATRIX_TAGS  12
#define MAX_MATRIX_SLOTS 10
#define MAX_MAPPING_TAGS 8
#define MAX_SYSTEM_TAGS  35

struct satioFileStruct {

    char matrix_tags[MAX_MATRIX_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    char matix_filepaths[MAX_MATRIX_SLOTS][MAX_GLOBAL_ELEMENT_SIZE];
    int matrix_file_slots[MAX_MATRIX_SLOTS];
    char current_matrix_filepath[MAX_GLOBAL_ELEMENT_SIZE];

    char mapping_tags[MAX_MAPPING_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    char mapping_filepath[MAX_GLOBAL_ELEMENT_SIZE];

    char system_tags[MAX_SYSTEM_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    char system_filepath[MAX_GLOBAL_ELEMENT_SIZE];
};
extern struct satioFileStruct satioFileData;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------

/**
 * Save current mapping data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool saveMappingFile(FS &fs, const char *filepath);

/**
 * Load mapping data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool loadMappingFile(FS &fs, const char *filepath);

/**
 * Delete mapping data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool deleteMappingFile(FS &fs, const char *filepath);

/**
 * Save current matrix data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool saveMatrixFile(FS &fs, const char *filepath);

/**
 * Load matrix data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool loadMatrixFile(FS &fs, const char *filepath);

/**
 * Delete matrix data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool deleteMatrixFile(FS &fs, const char *filepath);

/**
 * Save system data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool saveSystemFile(FS &fs, const char *filepath);

/**
 * Load system data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool loadSystemFile(FS &fs, const char *filepath);

/**
 * Delete matrix data.
 * @param fs Filesystem. Example SD_MMC
 * @param filepath Specify path to file
 */
bool deleteSystemFile(FS &fs, const char *filepath);

#endif