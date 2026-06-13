/*
    WT901 Library. Written by Benjamin Jack Cullen. Based on Witmotion example code.
    Converted to ESP-IDF UART driver.
*/

#include "wt901.h"
#include <Arduino.h>
#include "./wit_c_sdk.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

// UART2 configuration for WT901
#define WT901_UART_NUM    UART_NUM_2
#define WT901_UART_TX_PIN 36
#define WT901_UART_RX_PIN 35
#define WT901_BUF_SIZE    1024

static const char *WT901_TAG = "WT901";
static bool wt901_uart_installed = false;

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
    0,      // 0 (unused)
    4800,   // 1 WIT_BAUD_4800
    9600,   // 2 WIT_BAUD_9600
    19200,  // 3 WIT_BAUD_19200
    38400,  // 4 WIT_BAUD_38400
    57600,  // 5 WIT_BAUD_57600
    115200, // 6 WIT_BAUD_115200
    230400, // 7 WIT_BAUD_230400
    460800, // 8 WIT_BAUD_460800
    921600  // 9 WIT_BAUD_921600
  },
  .gyro_0_current_uiBaud = 0
};

bool wt901_updated_data=false;

// Helper function to initialize UART2 with a specific baud rate
static void wt901_uart_init(int32_t baud_rate) {
    if (wt901_uart_installed) {
        // Just change baud rate if already installed
        uart_set_baudrate(WT901_UART_NUM, baud_rate);
        uart_flush_input(WT901_UART_NUM);
        ESP_LOGI(WT901_TAG, "UART2 baud rate changed to %ld", baud_rate);
        return;
    }
    
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(WT901_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(WT901_UART_NUM, WT901_UART_TX_PIN, WT901_UART_RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(WT901_UART_NUM, WT901_BUF_SIZE * 2, WT901_BUF_SIZE, 0, NULL, 0));
    wt901_uart_installed = true;
    ESP_LOGI(WT901_TAG, "UART2 initialized at %d baud", baud_rate);
}

// Helper function to read available bytes from UART2
static int wt901_uart_read(uint8_t *buf, size_t max_len) {
    size_t available = 0;
    uart_get_buffered_data_len(WT901_UART_NUM, &available);
    if (available > 0) {
        int len = uart_read_bytes(WT901_UART_NUM, buf, (available < max_len) ? available : max_len, 0);
        return len;
    }
    return 0;
}

bool readGyro(void) {
    // Read serial data using ESP-IDF UART driver
    uint8_t buf[128];
    int len = wt901_uart_read(buf, sizeof(buf));
    for (int i = 0; i < len; i++) {
        WitSerialDataIn(buf[i]);
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
  printf("[Gyro0] TX: ");
  for (uint32_t i = 0; i < uiSize; i++) {
    printf("%02X ", p_data[i]);
  }
  printf("\n");
  uart_write_bytes(WT901_UART_NUM, (const char *)p_data, uiSize);
  uart_wait_tx_done(WT901_UART_NUM, 1000 / portTICK_PERIOD_MS);
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

void printWitHalStatusDescription(int status) {
    switch (status) {
        case WIT_HAL_OK:
            printf("WIT_HAL_OK: There is no error.\n");
            break;
        case WIT_HAL_BUSY:
            printf("WIT_HAL_BUSY: Busy.\n");
            break;
        case WIT_HAL_TIMEOUT:
            printf("WIT_HAL_TIMEOUT: Timed out.\n");
            break;
        case WIT_HAL_ERROR:
            printf("WIT_HAL_ERROR: A generic error happened.\n");
            break;
        case WIT_HAL_NOMEM:
            printf("WIT_HAL_NOMEM: No memory.\n");
            break;
        case WIT_HAL_EMPTY:
            printf("WIT_HAL_EMPTY: The resource is empty.\n");
            break;
        case WIT_HAL_INVAL:
            printf("WIT_HAL_INVAL: Invalid argument.\n");
            break;
        default:
            printf("Unknown WIT_HAL status code.\n");
            break;
    }
}

int32_t res;

void testWT901(void) {
  delay(500);  // Let gyro stabilize
  printf("[Gyro0] Testing raw data...\n");
  int count = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) {  // 2s window
    uint8_t b;
    if (wt901_uart_read(&b, 1) > 0) {
      printf("[Gyro0] Raw: 0x%02X\n", b);
      if (b == 0x55) count++;
    }
  }
  printf("[Gyro0] 0x55 packets seen: %d\n", count);
}

void Gyro0AutoScan(void) {
  unsigned int i, iRetry;

  // iterate through baud rates and check for valid data (start at 1 to skip unused index 0)
  for (i = 1; i < MAX_GYRO_BAUDRATES; i++) {
    printf("[Gyro0] Trying baud rate: %ld\n", gyroData.gyro_0_c_uiBaud[i]);

    // Initialize UART with current baud rate
    wt901_uart_init(gyroData.gyro_0_c_uiBaud[i]);
    uart_flush_input(WT901_UART_NUM);

    // Reset counters and flags
    iRetry = 2;
    gyroData.gyro_0_s_cDataUpdate = 0;

    // Try reading data with current baud rate
    do {

      // Request data
      res = WitReadReg(AX, 3);

      // Delay between attempts
      delay(200);

      // Read data
      uint8_t buf[128];
      int len = wt901_uart_read(buf, sizeof(buf));

      // Process data
      for (int j = 0; j < len; j++) {WitSerialDataIn(buf[j]);}

      // Check if we got valid data
      if (gyroData.gyro_0_s_cDataUpdate != 0) {
        printf("[Gyro0] Found baud rate: %ld\n", gyroData.gyro_0_c_uiBaud[i]);
        gyroData.gyro_0_current_uiBaud = gyroData.gyro_0_c_uiBaud[i];
        return;
      }
      iRetry--;
    }
    // Retry current baud rate
    while (iRetry); 
  }
  printf("[Gyro0] Sensor not found (check connection).\n");
}

/** --------------------------------------------------------------
 * @brief Initializes the WT901 sensor.
 * 
 */
void initWT901(void) {
  // --------------------------------------------------------------
  // Initialize.
  // --------------------------------------------------------------
  printf("[Gyro0] initializing...\n");
	res = WitInit(WIT_PROTOCOL_NORMAL, 0x50);

  printf("[Gyro0] register serial write.\n");
	res = WitSerialWriteRegister(Gyro0UartSend);

  printf("[Gyro0] register call back.\n");
	res = WitRegisterCallBack(Gyro0DataUpdata);

  printf("[Gyro0] register delay\n");
  res = WitDelayMsRegister(Gyro0Delayms);

  // --------------------------------------------------------------
  // Scan Baudrate.
  // --------------------------------------------------------------
  printf("[Gyro0] performing baud rate autoscan...\n");
	Gyro0AutoScan();
  printf("[Gyro0] current baudrate: %ld\n", gyroData.gyro_0_current_uiBaud);

  // --------------------------------------------------------------
  // Set Baudrate.
  // --------------------------------------------------------------
  int desired_baud = 230400;
  if (gyroData.gyro_0_current_uiBaud!=desired_baud) {
    printf("[Gyro0] changing baud rate to: %d\n", desired_baud);

    // Unlock register
    printf("[Gyro0] Unlocking registers...\n");
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) {
      printf("[Gyro0] Error unlocking registers.\n");
    }
    delay(100);
    
    // Set baud rate
    printf("[Gyro0] Writing BAUD register (0x04) with value %d...\n", WIT_BAUD_230400);
    if (WitWriteReg(BAUD, WIT_BAUD_230400) != WIT_HAL_OK) {
      printf("[Gyro0] Error writing baud rate.\n");
    }
    delay(100);
    
    // Save to flash
    printf("[Gyro0] Saving to flash...\n");
    if (WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK) {
      printf("[Gyro0] Error saving settings.\n");
    } else {
      printf("[Gyro0] Settings saved to flash.\n");
    }
    
    // Wait for sensor to restart/apply new baud rate
    printf("[Gyro0] Waiting for sensor to apply new baud...\n");
    delay(1000);
    
    // Now switch ESP32 UART to new baud rate
    wt901_uart_init(gyroData.gyro_0_c_uiBaud[WIT_BAUD_230400]);
    delay(200);

    // Scan Baudrate
    printf("[Gyro0] performing baud rate autoscan...\n");
    Gyro0AutoScan();
    printf("[Gyro0] current baudrate: %ld\n", gyroData.gyro_0_current_uiBaud);
  }
  else {printf("[Gyro0] no need to change baudrate.\n");}

  // --------------------------------------------------------------
  // Set Return rate.
  // --------------------------------------------------------------
  if (WitSetOutputRate(RRATE_200HZ) != WIT_HAL_OK) printf("[Gyro0] Error setting return rate. (RRATE_200HZ)\n");
  else printf("[Gyro0] Return rate modified successfully (RRATE_200HZ)\n");
  
  // --------------------------------------------------------------
  // Set Bandwidth.
  // --------------------------------------------------------------
  if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) {printf("[Gyro0] Error setting bandwidth (BANDWIDTH_256HZ).\n");}
  else {printf("[Gyro0] Bandwidth modified successfully (BANDWIDTH_256HZ)\n");}
}

void WT901CalAcc(void) {
  if (WitStartAccCali() != WIT_HAL_OK) printf("error calibrating gyro0: acceleration\n");
}

void WT901CalMagStart(void) {
  if (WitStartMagCali() != WIT_HAL_OK) printf("error calibrating gyro0: mag cal start\n");
}

void WT901CalMagEnd(void) {
  if (WitStopMagCali() != WIT_HAL_OK) printf("error calibrating gyro0: mag cal end\n");
}
