/*
    WT901 Library. Written by Benjamin Jack Cullen. Based on Witmotion example code.
*/

#ifndef WT901_H
#define WT901_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

// ------------------------------------------------------------------------
// Gyro data structure.
// ------------------------------------------------------------------------
struct GyroData {
  uint8_t gyro_0_s_cDataUpdate; // Update flags
  float gyro_0_fAcc[3];         // Acceleration (x, y, z)
  float gyro_0_fGyro[3];        // Gyroscope (x, y, z)
  float gyro_0_fAngle[3];       // Angles (roll, pitch, yaw)
  float gyro_0_ang_x;
  float gyro_0_ang_y;
  float gyro_0_ang_z;
  float gyro_0_acc_x;           // Processed acceleration x
  float gyro_0_acc_y;           // Processed acceleration y
  float gyro_0_acc_z;           // Processed acceleration z
  float gyro_0_gyr_x;           // Processed gyroscope x
  float gyro_0_gyr_y;           // Processed gyroscope y
  float gyro_0_gyr_z;           // Processed gyroscope z
  int16_t gyro_0_mag_x;         // Magnetic field x
  int16_t gyro_0_mag_y;         // Magnetic field y
  int16_t gyro_0_mag_z;         // Magnetic field z
  uint32_t gyro_0_c_uiBaud[9];  // Baud rates for scanning
  uint32_t gyro_0_current_uiBaud; // Current baud rate
  char gyro_sentence[MAX_GLOBAL_SERIAL_BUFFER_SIZE];
};
extern struct GyroData gyroData;

// ------------------------------------------------------------------------
// Function prototypes.
// ------------------------------------------------------------------------
void Gyro0UartSend(uint8_t *p_data, uint32_t uiSize); // Send data over UART
void Gyro0Delayms(uint16_t ucMs); // Delay in milliseconds
void Gyro0DataUpdata(uint32_t uiReg, uint32_t uiRegNum); // Update data flags
void Gyro0AutoScan(void); // Auto-scan baud rates
bool readGyro(void);
void initWT901(void);
void WT901CalAcc(void);
void WT901CalMagStart(void);
void WT901CalMagEnd(void);

#ifdef __cplusplus
}
#endif

#endif
