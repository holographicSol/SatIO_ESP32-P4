/*
    WT901 Library. Written by Benjamin Jack Cullen. Based on Witmotion example code.
*/

#include "wt901.h"
#include <Arduino.h>
#include "./wit_c_sdk.h"

/**
 * @struct GyroData
 * 
 * Data for gyroscope sensor data from WT901, including acceleration,
 * gyroscope readings, angles, magnetic fields, and configuration.
 */
struct GyroData gyroData = {
  .gyro_0_s_cDataUpdate = 0,
  .gyro_0_fAcc = {0.0f, 0.0f, 0.0f},
  .gyro_0_fGyro = {0.0f, 0.0f, 0.0f},
  .gyro_0_fAngle = {0.0f, 0.0f, 0.0f},
  .gyro_0_ang_x = 0.0f,
  .gyro_0_ang_y = 0.0f,
  .gyro_0_ang_z = 0.0f,
  .gyro_0_acc_x = 0.0f,
  .gyro_0_acc_y = 0.0f,
  .gyro_0_acc_z = 0.0f,
  .gyro_0_gyr_x = 0.0f,
  .gyro_0_gyr_y = 0.0f,
  .gyro_0_gyr_z = 0.0f,
  .gyro_0_mag_x = 0,
  .gyro_0_mag_y = 0,
  .gyro_0_mag_z = 0,
  .gyro_0_c_uiBaud={
    4800,   // 0
    9600,   // 1
    19200,  // 2
    38400,  // 3
    57600,  // 4
    115200, // 5
    230400, // 6
    460800, // 7
    921600  // 8
  },
  .gyro_0_current_uiBaud = 0
};

bool wt901_updated_data=false;

bool readGyro(void) {
    // Read serial data
    while (Serial2.available()) {
      WitSerialDataIn(Serial2.read());
    }
    wt901_updated_data=false;
    // Update data
    if (gyroData.gyro_0_s_cDataUpdate) {
      // Update values
      for (int i = 0; i < 3; i++) {
        gyroData.gyro_0_fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
        gyroData.gyro_0_fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
        gyroData.gyro_0_fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
      }
      // Process acceleration
      if (gyroData.gyro_0_s_cDataUpdate & GYRO_0_ACC_UPDATE) {
        gyroData.gyro_0_s_cDataUpdate &= ~GYRO_0_ACC_UPDATE;
        gyroData.gyro_0_acc_x = gyroData.gyro_0_fAcc[0];
        gyroData.gyro_0_acc_y = gyroData.gyro_0_fAcc[1];
        gyroData.gyro_0_acc_z = gyroData.gyro_0_fAcc[2];
        wt901_updated_data=true;
      }
      // Process angle
      if (gyroData.gyro_0_s_cDataUpdate & GYRO_0_ANGLE_UPDATE) {
        gyroData.gyro_0_s_cDataUpdate &= ~GYRO_0_ANGLE_UPDATE;
        gyroData.gyro_0_ang_x = gyroData.gyro_0_fAngle[0];
        gyroData.gyro_0_ang_y = gyroData.gyro_0_fAngle[1];
        gyroData.gyro_0_ang_z = gyroData.gyro_0_fAngle[2];
        wt901_updated_data=true;
      }
      // Process gyro
      if (gyroData.gyro_0_s_cDataUpdate & GYRO_0_UPDATE) {
        gyroData.gyro_0_s_cDataUpdate &= ~GYRO_0_UPDATE;
        gyroData.gyro_0_gyr_x = gyroData.gyro_0_fGyro[0];
        gyroData.gyro_0_gyr_y = gyroData.gyro_0_fGyro[1];
        gyroData.gyro_0_gyr_z = gyroData.gyro_0_fGyro[2];
        wt901_updated_data=true;
      }
      // Process magnetic field
      if (gyroData.gyro_0_s_cDataUpdate & GYRO_0_MAG_UPDATE) {
        gyroData.gyro_0_s_cDataUpdate &= ~GYRO_0_MAG_UPDATE;
        gyroData.gyro_0_mag_x = sReg[HX];
        gyroData.gyro_0_mag_y = sReg[HY];
        gyroData.gyro_0_mag_z = sReg[HZ];
        wt901_updated_data=true;
      }
      return wt901_updated_data;
    }
    return wt901_updated_data;
}

void Gyro0UartSend(uint8_t *p_data, uint32_t uiSize) {
  Serial2.write(p_data, uiSize);
  Serial2.flush();
}

void Gyro0Delayms(uint16_t ucMs) {
  delay(ucMs);
}

void Gyro0DataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  for (unsigned int i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case AZ:
        gyroData.gyro_0_s_cDataUpdate |= GYRO_0_ACC_UPDATE;
        break;
      case GZ:
        gyroData.gyro_0_s_cDataUpdate |= GYRO_0_UPDATE;
        break;
      case HZ:
        gyroData.gyro_0_s_cDataUpdate |= GYRO_0_MAG_UPDATE;
        break;
      case Yaw:
        gyroData.gyro_0_s_cDataUpdate |= GYRO_0_ANGLE_UPDATE;
        break;
      default:
        gyroData.gyro_0_s_cDataUpdate |= GYRO_0_READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

void Gyro0AutoScan(void) {
  unsigned int i, iRetry;
  for (i = 0; i < sizeof(gyroData.gyro_0_c_uiBaud) / sizeof(gyroData.gyro_0_c_uiBaud[0]); i++) {
    Serial.println("[Gyro0] Trying baud rate: " + String(gyroData.gyro_0_c_uiBaud[i]));
    Serial2.begin(gyroData.gyro_0_c_uiBaud[i]);
    Serial2.flush();
    iRetry = 2;
    gyroData.gyro_0_s_cDataUpdate = 0;
    do {
      WitReadReg(AX, 3);
      delay(200);
      while (Serial2.available()) {
        WitSerialDataIn(Serial2.read());
      }
      if (gyroData.gyro_0_s_cDataUpdate != 0) {
        Serial.println("[Gyro0] Found baud rate: " + String(gyroData.gyro_0_c_uiBaud[i]));
        gyroData.gyro_0_current_uiBaud = gyroData.gyro_0_c_uiBaud[i];
        return;
      }
      iRetry--;
    } while (iRetry);
  }
  Serial.println("[Gyro0] Sensor not found (check connection).");
}

void initWT901(void) {
  // --------------------------------------------------------------
  // Initialize.
  // --------------------------------------------------------------
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  Serial.println("[Gyro0] register serial write.");
	WitSerialWriteRegister(Gyro0UartSend);
  Serial.println("[Gyro0] register call back.");
	WitRegisterCallBack(Gyro0DataUpdata);
  Serial.println("[Gyro0] register delay");
  WitDelayMsRegister(Gyro0Delayms);
  // --------------------------------------------------------------
  // Set Baudrate.
  // --------------------------------------------------------------
  Serial.println("[Gyro0] performing baud rate autoscan...");
	Gyro0AutoScan(); // scan for current baud rate (required before trying to configure)
  Serial.println("[Gyro0] current baudrate: " + String(gyroData.gyro_0_current_uiBaud));
  if (gyroData.gyro_0_current_uiBaud!=230400) {
    Serial.println("[Gyro0] changing baud rate to: WIT_BAUD_230400");
    if (WitSetUartBaud(WIT_BAUD_230400) != WIT_HAL_OK) Serial.println("[Gyro0] Error setting baud rate (WIT_BAUD_230400).");
    else {Serial2.begin(gyroData.gyro_0_c_uiBaud[WIT_BAUD_230400]); Serial.println("[Gyro0] Baud rate modified successfully (WIT_BAUD_230400)");}
    Serial.println("[Gyro0] performing baud rate autoscan...");
    Gyro0AutoScan(); // confirm configured baud rate
  }
  else {Serial.println("[Gyro0] no need to change baudrate. ");}
  // --------------------------------------------------------------
  // Set Return rate.
  // --------------------------------------------------------------
  if (WitSetOutputRate(RRATE_200HZ) != WIT_HAL_OK) Serial.println("[Gyro0] Error setting return rate. (RRATE_200HZ)");
  else Serial.println("[Gyro0] Return rate modified successfully (RRATE_200HZ)");
  // --------------------------------------------------------------
  // Set Bandwidth.
  // --------------------------------------------------------------
  if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) {Serial.println("[Gyro0] Error setting bandwidth (BANDWIDTH_256HZ).");}
  else {Serial.println("[Gyro0] Bandwidth modified successfully (BANDWIDTH_256HZ)");}
}

void WT901CalAcc(void) {
  if (WitStartAccCali() != WIT_HAL_OK) Serial.println("error calibrating gyro0: acceleration");
}

void WT901CalMagStart(void) {
  if (WitStartMagCali() != WIT_HAL_OK) Serial.println("error calibrating gyro0: mag cal start");
}

void WT901CalMagEnd(void) {
  if (WitStopMagCali() != WIT_HAL_OK) Serial.println("error calibrating gyro0: mag cal end");
}
