/*
    Custom Mapping Library. Written by Benjamin Jack Cullen.

    1 : Configure matrixData.mapping_config and select_map_algorithm as needed.
    2 : Call setMappedValues() to process mappable values.
    3 : Access mapped values in matrixData.mapped_value.

    Center Mapping example:
    example map analog stick axis x0 on admplex0 channel 0 into map slot 0: mapping -s 0 -m 1 -c0 16 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
    example map analog stick axis x1 on admplex0 channel 1 into map slot 1: mapping -s 1 -m 2 -c0 17 -c1 1974 -c2 1974 -c3 1894 -c4 255 -c5 50
    
    - Utilize a map slot example (matrix):
        - Map slots can also be utilized by the matrix with mapped value function.
        - If matrix switch logic returns true then digital 1 can be output.
        - If matrix switch logic returns true then a different mapped value can be output.
        - If matrix switch logic returns true then the same mapped value can be output.
*/

#include "custommapping.h"
#include <Arduino.h>
#include <stdlib.h>
#include "wtgps300p.h"
#include "wt901.h"
#include "multiplexers.h"
#include "matrix.h"

// ----------------------------------------------------------------------------------------
// Custom Mapping Structure.
// ----------------------------------------------------------------------------------------
struct CustomMappingStruct mappingData = {
    .center_map_x0 = 0,
    .center_map_x1 = 0,
    .element_name = {
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

    // -------------------------------------------------------------------------------------------------------
    /*
        Map Slot Mode.
            - map_mode[N] mode is allocated for the same mapping_config[N].

        map mode 0 : map min to max
        map mode 1 : center map x0
        map mode 2 : center map x1
    */
    // -------------------------------------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------------------------------------
    /*
        Slot Index For Matrix Switches.
            - index_mapped_value[N] is allocated for the same matrix switch[N].
            - index_mapped_value[N] can point at at any mapped_value[N].
    */  
    // -------------------------------------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------------------------------------
    /*
    Programmable mapping:

    Mode 0 (standard map):
    0 : Value Index      : matrix function number (default 0=None).
    1 : Input Value Min  : (defalut 0).
    2 : Input Value Max  : (defalut 0).
    3 : Output Value Min : (defalut 0).
    4 : Output Value Max : (defalut 0).
    5 : NULL             : (defalut 0).

    Mode 1 (center map):
    0 : Value Index : matrix function number (default 0=None).
    1 : Center      : approximate center value.
    2 : Neg Range   : (input) 0 to approximate center value.
    3 : Pos Range   : (input) value max - neg range.
    4 : Output Max  : (result) max desired value.
    5 : DEADZONE    : expected max flutuation at center.

    */ 
    // -------------------------------------------------------------------------------------------------------
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

  // -------------------------------------------------------------------------------------------------------
  // Results from mapping.
  // -------------------------------------------------------------------------------------------------------
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

// -------------------------------------------------------------------------------------------------------
// Centered Map.
// -------------------------------------------------------------------------------------------------------
void centeredMap(int map_slot, float var) {
    // ------------------------------------------------
    // Step 1: Offset correction (center to 0).
    // ------------------------------------------------
    int32_t centered = var - mappingData.mapping_config[0][map_slot][INDEX_CMAP_CENTER];
    // ------------------------------------------------
    // Step 2: Piecewise linear scaling.
    // ------------------------------------------------
    int32_t normalized;
    if (centered <= 0) {
        // --------------------------------------------
        // Negative direction.
        // --------------------------------------------
        if (mappingData.mapping_config[0][map_slot][INDEX_CMAP_NEG_RANGE] != 0) { // Avoid division by zero
            normalized = (centered * mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX]) / mappingData.mapping_config[0][map_slot][INDEX_CMAP_NEG_RANGE];
        } else {
            normalized = 0;
        }
    } else {
        // --------------------------------------------
        // Positive direction.
        // --------------------------------------------
        if (mappingData.mapping_config[0][map_slot][INDEX_CMAP_POS_RANGE] != 0) { // Avoid division by zero
            normalized = (centered * mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX]) / mappingData.mapping_config[0][map_slot][INDEX_CMAP_POS_RANGE];
        } else {
            normalized = 0;
        }
    }
    // ------------------------------------------------
    // Step 3: Clamping to [-OUTPUT_MAX, OUTPUT_MAX].
    // ------------------------------------------------
    if (normalized > mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX]) {
        normalized = mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX];
    } else if (normalized < -mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX]) {
        normalized = -mappingData.mapping_config[0][map_slot][INDEX_CMAP_OMAX];
    }
    // ------------------------------------------------
    // Step 4: Apply deadzone.
    // ------------------------------------------------
    if (normalized > -mappingData.mapping_config[0][map_slot][INDEX_CMAP_DEADZONE] && normalized < mappingData.mapping_config[0][map_slot][INDEX_CMAP_DEADZONE]) {
        normalized = 0;
    }
    // ------------------------------------------------
    // Apply opposite cutoff.
    // ------------------------------------------------
    if      (normalized<=0) {mappingData.center_map_x1=0; mappingData.center_map_x0=abs(normalized);} 
    else if (normalized>=0) {mappingData.center_map_x0=0; mappingData.center_map_x1=abs(normalized);}
    // Serial.println("X0 : " + String(center_map_x0));
    // Serial.println("X1 : " + String(center_map_x1));
    if      (mappingData.map_mode[0][map_slot]==MAP_CENTER_X0) {mappingData.mapped_value[0][map_slot]=mappingData.center_map_x0;}
    else if (mappingData.map_mode[0][map_slot]==MAP_CENTER_X1) {mappingData.mapped_value[0][map_slot]=mappingData.center_map_x1;}
    // Serial.println("mappingData.mapped_value[0][map_slot] : " + String(mappingData.mapped_value[0][map_slot]));
}

void setMappedValuesHelper(int map_slot, double map_input_value) {
  if (mappingData.map_mode[0][map_slot]==MAP_MIN_TO_MAX) {
    // test valid input range (may require more checks before handing values to map)
    if (mappingData.mapping_config[0][map_slot][INDEX_MAP_EMAX] - mappingData.mapping_config[0][map_slot][INDEX_MAP_EMIN] <= 0) {}
    else {
    mappingData.mapped_value[0][map_slot]=map(map_input_value,
                                              mappingData.mapping_config[0][map_slot][INDEX_MAP_EMIN],
                                              mappingData.mapping_config[0][map_slot][INDEX_MAP_EMAX],
                                              mappingData.mapping_config[0][map_slot][INDEX_MAP_OMIN],
                                              mappingData.mapping_config[0][map_slot][INDEX_MAP_EMAX]);}
  }
  else if (mappingData.map_mode[0][map_slot]==MAP_CENTER_X0 || mappingData.map_mode[0][map_slot]==MAP_CENTER_X1) {centeredMap(map_slot, map_input_value);}
}

void setMappedValues(void) {
  for (int map_slot=0; map_slot<MAX_MAPPABLE_VALUES; map_slot++) {
    // ------------------------------------------------
    // Create mapped values.
    // ------------------------------------------------
    switch ((int)mappingData.mapping_config[0][map_slot][INDEX_MAP_VALUE]) {
      case INDEX_MAPPABLE_VALUES_DIGITAL:
        mappingData.mapped_value[0][map_slot]=0; // todo: allow custom mapping for 0/1
        break;
      case INDEX_MAPPABLE_VALUES_YAWGPATT:
        setMappedValuesHelper(map_slot, atof(gpattData.yaw));
        break;
      case INDEX_MAPPABLE_VALUES_ROLLGPATT:
        setMappedValuesHelper(map_slot, atof(gpattData.roll));
        break;
      case INDEX_MAPPABLE_VALUES_PITCHGPATT:
        setMappedValuesHelper(map_slot, atof(gpattData.pitch));
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0ACCX:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_acc_x);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0ACCY:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_acc_y);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0ACCZ:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_acc_z);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0ANGX:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_ang_x);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0ANGY:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_ang_y);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0ANGZ:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_ang_z);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0MAGX:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_mag_x);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0MAGY:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_mag_y);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0MAGZ:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_mag_z);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0GYROX:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_gyr_x);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0GYROY:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_gyr_y);
        break;
      case INDEX_MAPPABLE_VALUES_GYRO0GYROZ:
        setMappedValuesHelper(map_slot, gyroData.gyro_0_gyr_z);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_0:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[0]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_1:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[1]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_2:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[2]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_3:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[3]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_4:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[4]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_5:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[5]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_6:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[6]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_7:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[7]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_8:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[8]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_9:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[9]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_10:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[10]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_11:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[11]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_12:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[12]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_13:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[13]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_14:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[14]);
        break;
      case INDEX_MAPPABLE_VALUES_ADMPLEX0_15:
        setMappedValuesHelper(map_slot, multiplexerData.ADMPLEX_0_DATA[15]);
        break;
      default:
        mappingData.mapped_value[0][map_slot]=0; // consideration ?
    }
  }
}

void zero_mapping(void) {
  Serial.println("[matrix] setting all matrix values to zero.");
  // -------------------------------
  // Iterate over each matrix matrix.
  // -------------------------------
  // ---------------------------
  // Override.
  // ---------------------------
  for (int Mi=0; Mi < MAX_MATRIX_SWITCHES; Mi++) {matrixData.computer_assist[0][Mi]=false; matrixData.override_output_value[0][Mi]=0; matrixData.matrix_switch_write_required[0][Mi]=true;}
  delay(100); // replace with something more specific
  // ---------------------------
  // Zero all.
  // ---------------------------
  for (int Mi=0; Mi < MAX_MAP_SLOTS; Mi++) {
    mappingData.map_mode[0][Mi]=0;
    mappingData.index_mapped_value[0][Mi]=0;
    mappingData.mapping_config[0][Mi][0]=0;
    mappingData.mapping_config[0][Mi][1]=0;
    mappingData.mapping_config[0][Mi][2]=0;
    mappingData.mapping_config[0][Mi][3]=0;
    mappingData.mapping_config[0][Mi][4]=0;
    mappingData.mapping_config[0][Mi][5]=0;
    mappingData.mapped_value[0][Mi]=0;
  }
}
