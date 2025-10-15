/*
    Custom Mapping Library. Written by Benjamin Jack Cullen.
*/

#include "custommapping.h"
#include <Arduino.h>
#include <stdlib.h>
#include "wtgps300p.h"
#include "wt901.h"
#include "multiplexers.h"
#include "matrix.h"

/**
 * @struct CustomMappingStruct
 */
struct CustomMappingStruct mappingData={
    .center_map_x0=0,
    .center_map_x1=0,
    .char_map_value={
    "Digital", // 0
    "YawGPATT", // 1
    "RollGPATT", // 2
    "PitchGPATT", // 3
    "Gyro0AccX", // 4
    "Gyro0AccY", // 5
    "Gyro0AccZ", // 6
    "Gyro0AngX", // 7
    "Gyro0AngY", // 8
    "Gyro0AngZ", // 9
    "Gyro0MagX", // 10
    "Gyro0MagY", // 11
    "Gyro0MagZ", // 12
    "Gyro0GyroX", // 13
    "Gyro0GyroY", // 14
    "Gyro0GyroZ", // 15
    "ADMPlex0_0", // 16
    "ADMPlex0_1", // 17
    "ADMPlex0_2", // 18
    "ADMPlex0_3", // 19
    "ADMPlex0_4", // 20
    "ADMPlex0_5", // 21
    "ADMPlex0_6", // 22
    "ADMPlex0_7", // 23
    "ADMPlex0_8", // 24
    "ADMPlex0_9", // 25
    "ADMPlex0_10", // 26
    "ADMPlex0_11", // 27
    "ADMPlex0_12", // 28
    "ADMPlex0_13", // 29
    "ADMPlex0_14", // 30
    "ADMPlex0_15", // 31
    },
    .map_mode={
      {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-39
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 70-79
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 80-89
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 90-99
      }
    },
  .mapping_config={
    {
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 0-4
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 5-9
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 10-14
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 15-19
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 20-24
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 25-29
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 30-34
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 35-39
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 40-44
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 45-49
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 50-54
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 55-59
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 60-64
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 65-69
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 70-74
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 75-79
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 80-84
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 85-89
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, // 90-04
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}  // 95-99
    }
  },
  .index_mapped_value={
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-39
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 70-79
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 80-89
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 90-99
  },
  .mapped_value={
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-39
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 70-79
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 80-89
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 90-99
    }
  },
};

void center_map(int map_slot, float var) {
    // Offset correction (center to 0).
    int32_t centered=var - mappingData.mapping_config[0][map_slot][INDEX_CMAP_CENTER];
    // Piecewise linear scaling.
    int32_t normalized;
    if (centered <= 0) {
         // Avoid division by zero and normalize.
        if (mappingData.mapping_config[0][map_slot][INDEX_CMAP_NEG_RANGE] != 0) {
            normalized=(centered * mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX]) /
            mappingData.mapping_config[0][map_slot][INDEX_CMAP_NEG_RANGE];}
        else {normalized=0;}
    }
    else {
         // Avoid division by zero and normalize.
        if (mappingData.mapping_config[0][map_slot][INDEX_CMAP_POS_RANGE] != 0) {
            normalized=(centered * mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX]) /
            mappingData.mapping_config[0][map_slot][INDEX_CMAP_POS_RANGE];}
        else {normalized=0;}
    }
    // Clamping to [-OUTPUT_MAX, OUTPUT_MAX].
    if (normalized > mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX])
        {normalized=mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX];}
    else if (normalized < -mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX])
        {normalized=-mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX];}
    // Apply deadzone.
    if (normalized > -mappingData.mapping_config[0][map_slot][INDEX_CMAP_DEADZONE] &&
        normalized < mappingData.mapping_config[0][map_slot][INDEX_CMAP_DEADZONE])
        {normalized=0;}
    // Apply opposite cutoff.
    if (normalized <= 0)
        {mappingData.center_map_x1=0; mappingData.center_map_x0=abs(normalized);}
    else if (normalized >= 0)
        {mappingData.center_map_x0=0; mappingData.center_map_x1=abs(normalized);}
    // Select either x0 or x1 according to configuration.
    if (mappingData.map_mode[0][map_slot]==MAP_CENTER_X0)
        {mappingData.mapped_value[0][map_slot]=mappingData.center_map_x0;}
    else if (mappingData.map_mode[0][map_slot]==MAP_CENTER_X1)
        {mappingData.mapped_value[0][map_slot]=mappingData.center_map_x1;}
}

void map_values_helper(int map_slot, double map_input_value) {
    if (mappingData.map_mode[0][map_slot]==MAP_MIN_TO_MAX) {
        if (!mappingData.mapping_config[0][map_slot][INDEX_MAP_EMAX] -
            mappingData.mapping_config[0][map_slot][INDEX_MAP_EMIN] <= 0)
            {
            mappingData.mapped_value[0][map_slot]=map(map_input_value,
            mappingData.mapping_config[0][map_slot][INDEX_MAP_EMIN],
            mappingData.mapping_config[0][map_slot][INDEX_MAP_EMAX],
            mappingData.mapping_config[0][map_slot][INDEX_MAP_OMIN],
            mappingData.mapping_config[0][map_slot][INDEX_MAP_EMAX]);
            }}
    else if (mappingData.map_mode[0][map_slot]==MAP_CENTER_X0 ||
             mappingData.map_mode[0][map_slot]==MAP_CENTER_X1)
             {center_map(map_slot, map_input_value);}
}

void map_values(void) {
    for (int map_slot=0; map_slot < MAX_MAPPABLE_VALUES; map_slot++) {
        switch ((int)mappingData.mapping_config[0][map_slot][INDEX_MAP_VALUE]) {
        case INDEX_MAPPABLE_VALUES_DIGITAL:
            mappingData.mapped_value[0][map_slot]=0; // todo: allow custom mapping for 0/1
            break;
        case INDEX_MAPPABLE_VALUES_YAWGPATT:
            map_values_helper(map_slot, atof(gpattData.yaw));
            break;
        case INDEX_MAPPABLE_VALUES_ROLLGPATT:
            map_values_helper(map_slot, atof(gpattData.roll));
            break;
        case INDEX_MAPPABLE_VALUES_PITCHGPATT:
            map_values_helper(map_slot, atof(gpattData.pitch));
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ACCX:
            map_values_helper(map_slot, gyroData.gyro_0_acc_x);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ACCY:
            map_values_helper(map_slot, gyroData.gyro_0_acc_y);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ACCZ:
            map_values_helper(map_slot, gyroData.gyro_0_acc_z);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGX:
            map_values_helper(map_slot, gyroData.gyro_0_ang_x);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGY:
            map_values_helper(map_slot, gyroData.gyro_0_ang_y);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0ANGZ:
            map_values_helper(map_slot, gyroData.gyro_0_ang_z);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGX:
            map_values_helper(map_slot, gyroData.gyro_0_mag_x);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGY:
            map_values_helper(map_slot, gyroData.gyro_0_mag_y);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0MAGZ:
            map_values_helper(map_slot, gyroData.gyro_0_mag_z);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROX:
            map_values_helper(map_slot, gyroData.gyro_0_gyr_x);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROY:
            map_values_helper(map_slot, gyroData.gyro_0_gyr_y);
            break;
        case INDEX_MAPPABLE_VALUES_GYRO0GYROZ:
            map_values_helper(map_slot, gyroData.gyro_0_gyr_z);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_0:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[0]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_1:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[1]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_2:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[2]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_3:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[3]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_4:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[4]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_5:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[5]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_6:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[6]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_7:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[7]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_8:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[8]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_9:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[9]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_10:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[10]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_11:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[11]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_12:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[12]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_13:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[13]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_14:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[14]);
            break;
        case INDEX_MAPPABLE_VALUES_ADMPLEX0_15:
            map_values_helper(map_slot, multiplexerData.ADMPLEX_0_DATA[15]);
            break;
        default:
            mappingData.mapped_value[0][map_slot]=0;
        }
    }
}

void set_mapping_default(int map_slot) {
    mappingData.map_mode[0][map_slot]=0;
    mappingData.index_mapped_value[0][map_slot]=0;
    mappingData.mapping_config[0][map_slot][0]=0;
    mappingData.mapping_config[0][map_slot][1]=0;
    mappingData.mapping_config[0][map_slot][2]=0;
    mappingData.mapping_config[0][map_slot][3]=0;
    mappingData.mapping_config[0][map_slot][4]=0;
    mappingData.mapping_config[0][map_slot][5]=0;
    mappingData.mapped_value[0][map_slot]=0;
}

void set_all_mapping_default(void) {
    for (int Mi=0; Mi < MAX_MAP_SLOTS; Mi++) {set_mapping_default(Mi);}
}