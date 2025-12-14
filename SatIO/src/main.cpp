/*

  SatIO - Written by Benjamin Jack Cullen.

-----

  A general purpose programmable I/O platform for automation, throughput and LLM's.

  A huge matrix switch in a small package, supporting stacks of logic across
  70 output pins and 100 mapping slots.

-----

  Inference in Bayesian Reasoning? Moon tracking for example can be used to track the
  moon, it can also be used for one example; to track the tides, if the system is aware of
  moon/planetary positioning and datetime then marine life values may also be inferred
  relative to the inferred tide values and known datetime. There is a lot of data that
  can be used in many ways, with a kind of network effect.

-----

  Wiring For Keystudio ESP32 PLUS Development Board:

          ESP32: 1st ATMEGA2560 with shield as Output Port Controller (not on multiplexer):
          ESP32: IIC 1 SDA as io4 -> ATMEGA2560: I2C SDA
          ESP32: IIC 1 SCL as io5 -> ATMEGA2560: I2C SCL

          ESP32: 2nd ATMEGA2560 with shield as Input Port Controller (not on multiplexer):
          ESP32: IIC 2 SDA as io7 -> ATMEGA2560: I2C SDA
          ESP32: IIC 2 SCL as io8 -> ATMEGA2560: I2C SCL

          Other ESP32 I2C DS3231 (not on multiplexer) (5v):
          ESP32: IIC 0 SDA as io2 -> DS3231 (RTC): SDA
          ESP32: IIC 0 SCL as io3 -> DS3231 (RTC): SCL

          ESP32 i2C: I2C Multiplexing (3.3v):
          ESP32: IIC 0 SDA as io2 -> TCA9548A: SDA
          ESP32: IIC 0 SCL as io3 -> TCA9548A: SCL

          ESP32: Analog/Digital Multiplexing (3.3v):
          ESP32: io53   -> CD74HC4067: SIG
          ESP32: io23   -> CD74HC4067: S0
          ESP32: io22   -> CD74HC4067: S1
          ESP32: io21   -> CD74HC4067: S2
          ESP32: io20   -> CD74HC4067: S3

          ESP32: WTGPS300P (5v):
          ESP32: io27 RXD -> WTGPS300P: TXD
          ESP32: null TXD -> WTGPS300P: RXD

          ESP32: WT901 9-Axis Gyro:
          ESP32: Serial2 RXD -> WT901 TXD
          ESP32: Serial2 TXD -> WT901 RXD

-----

    To Do: AI I2C modules returning int's as classifiers.
    To Do: SRTM data. Use NASA shuttle radar topographical mission data.
    To Do: Ability to add custom IIC sensor modules after flashing.
    To Do: PCB fabrication.

-----

  There are some required custom libs included in complete project files:
  https://drive.google.com/drive/folders/13yynSxkKL-zxb7iLSkg0v0VXkSLgmtW-?usp=sharing

-----
*/

#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <iostream>
#include <assert.h>
#include <float.h>
#include <math.h>

#include <Arduino.h>
#include <Wire.h>
               
#include "./config.h"
#include "./REG.h"
#include "./strval.h"
#include "./meteors.h"
#include "./wtgps300p.h"
#include "./wt901.h"
#include "./multiplexers.h"
#include "./esp32_helper.h"
#include "./sidereal_helper.h"
#include "./hextodig.h"
#include "./ins.h"
#include "./satio.h"
#include "./custommapping.h"
#include "./matrix.h"
#include "./serial_infocmd.h"
#include "./system_data.h"
#include "./sdmmc_helper.h"
#include "./task_handler.h"

#include <sys/time.h>
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "esp_pm.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "SPIFFS.h"

/**
 * @brief Setup
 */
void setup() {
  // --------------------------------------------------------------
  // Required for operations in taks that may take longer than 5s..
  // --------------------------------------------------------------
  esp_task_wdt_config_t config = {
    .timeout_ms = 60* 1000, // 20 seconds
    .trigger_panic = true,  // Trigger panic if watchdog timer not reset
  };
  esp_task_wdt_reconfigure(&config);
  enableLoopWDT();
  // --------------------------------------------------------------
  // Warmup delay: some devices require at least one second start.
  // --------------------------------------------------------------
  delay(1000);
  // --------------------------------------------------------------
  // Initialize Pins.
  // --------------------------------------------------------------
  pinMode(ADMPLEX_0_S0, OUTPUT); 
  pinMode(ADMPLEX_0_S1, OUTPUT); 
  pinMode(ADMPLEX_0_S2, OUTPUT); 
  pinMode(ADMPLEX_0_S3, OUTPUT); 
  pinMode(ADMPLEX_0_SIG, INPUT); 
  analogReadResolution(16);
  analogSetAttenuation(ADC_11db);   // 0–3.3 V range
  digitalWrite(ADMPLEX_0_S0, LOW);
  digitalWrite(ADMPLEX_0_S1, LOW);
  digitalWrite(ADMPLEX_0_S2, LOW);
  digitalWrite(ADMPLEX_0_S3, LOW);
  // --------------------------------------------------------------
  // Initialize Serial 0.
  // --------------------------------------------------------------
  Serial.setRxBufferSize(2000); // ensure this is set before begin()
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(921600); while(!Serial);
  Serial.flush();
  Serial.println();
  Serial.println();
  Serial.println("[SERIAL] (Commands and general output)");
  Serial.println("[SERIAL] Baud rate: 921600");
  // --------------------------------------------------------------
  // System Information.
  // --------------------------------------------------------------
  Serial.println("[xPortGetCoreID] " +
    String(xPortGetCoreID()));
  Serial.println("[ESP_PM_CPU_FREQ_MAX] " +
    String(ESP_PM_CPU_FREQ_MAX));
  Serial.println("[ESP_PM_APB_FREQ_MAX] " +
    String(ESP_PM_APB_FREQ_MAX));
  Serial.println("[ESP_PM_NO_LIGHT_SLEEP] " +
    String(ESP_PM_NO_LIGHT_SLEEP));
  Serial.println("[CONFIG_ESPTOOLPY_FLASHFREQ] " +
    String(CONFIG_ESPTOOLPY_FLASHFREQ));
  Serial.println("[CONFIG_ESPTOOLPY_FLASHMODE] " +
    String(CONFIG_ESPTOOLPY_FLASHMODE));
  Serial.println("[CONFIG_LOG_DEFAULT_LEVEL] " +
    String(CONFIG_LOG_DEFAULT_LEVEL));
  Serial.println("[CONFIG_BOOTLOADER_LOG_LEVEL] " +
    String(CONFIG_BOOTLOADER_LOG_LEVEL));
  Serial.println("[CONFIG_ESP_CONSOLE_UART_BAUDRATE] " +
    String(CONFIG_ESP_CONSOLE_UART_BAUDRATE));
  Serial.println("[CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL] " +
    String(CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL));
  Serial.println("[getCpuFrequencyMhz] " +
    String(getCpuFrequencyMhz()));
  Serial.println("[APB_CLK_FREQ] " +
    String(getApbFrequency()));
  // --------------------------------------------------------------
  // Initialize NVS.
  // --------------------------------------------------------------
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ret = nvs_flash_init();}
  if (ret != ESP_OK) {printf("NVS init failed: %s\n", esp_err_to_name(ret));}
  // --------------------------------------------------------------
  // Initialize SPIFFS.
  // --------------------------------------------------------------
  Serial.println("[SPIFFS] attempting to mount");
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED, "/INTERNAL")){
      Serial.println("[SPIFFS] Mount Failed");
   }
  else {Serial.println("[SPIFFS] mounted successfully");}
  print_partition_table();
  print_ram_info();
  // --------------------------------------------------------------
  // Initialize SDCard and attempt to load files.
  // --------------------------------------------------------------
  sdcardBegin();
  sdmmcFlagData.load_system=true;
  sdcardFlagHandler();
  if (matrixData.load_matrix_on_startup) {
    sdmmcFlagData.load_mapping=true; sdcardFlagHandler();
    sdmmcFlagData.load_matrix=true; sdcardFlagHandler();}
  // --------------------------------------------------------------
  // Initialize I2C.
  // --------------------------------------------------------------
  Serial.println("[IIC] starting IIC as master");
  initInputPortController();
  initOutputPortController();
  // Wire.setClock(1000000);
  // Wire.setClock(800000L);
  // Wire.setClock(400000);
  // Wire.setClock(200000);
  // Wire.setClock(100000L);
  writeOutputPortControllerM0();
  // --------------------------------------------------------------
  // Initialize I2C Multiplexer(s).
  // --------------------------------------------------------------
  Serial.println("[IIC] setting IIC multiplexer channel: 0");
  // setMultiplexChannel_I2C(0, 0);
  // --------------------------------------------------------------
  // Initialize RTC (for UTC).
  // --------------------------------------------------------------
  Serial.println("[IIC] starting RTC");
  initRTC();
  // --------------------------------------------------------------
  // Initialize System Time (for local time).
  // --------------------------------------------------------------
  initSystemTime();
  // --------------------------------------------------------------
  // Initialize Serial 1 (for GPS).
  // --------------------------------------------------------------
  Serial.println("[SERIAL1] (GPS)");
  Serial1.setPins(36, -1, -1, -1); // ensure this is set before begin()
  Serial1.setRxBufferSize(2000); // ensure this is set before begin()
  Serial1.setTimeout(10); // ensure this is set before begin()
  Serial1.begin(115200); while(!Serial1);
  Serial1.flush();
  Serial.println("[SERIAL1] Baud rate: 115200");
  Serial.println("[SERIAL1] (hardware serial remap: Rx=36 Tx=-1)");
  // --------------------------------------------------------------
  // Initialize Serial 2 (for 9-Axis Gyro).
  // --------------------------------------------------------------
  Serial.println("[SERIAL2] (9-Axis Gyro)");
  Serial2.setPins(33, 32, -1, -1); // ensure this is set before begin()
  Serial2.setRxBufferSize(2000); // ensure this is set before begin()
  Serial2.setTimeout(1000); // ensure this is set before begin()
  Serial2.flush();
  Serial.println("[SERIAL2] (hardware serial remap: Rx=33 Tx=32)");
	Serial.println("[SERIAL2] initializing Gyro0");
  initWT901();
  delay(1000);
  // --------------------------------------------------------------
  // Create Tasks.
  // --------------------------------------------------------------
  createTaskLogging();
  createTaskSerialInfoCMD();
  createTaskStorage();
  createTaskMultiplexers();
  createTaskPortControllerInput();
  createTaskGyro();
  createTaskGPS();
  myAstroBegin();
  createTaskUniverse();
  createTaskSwitches();
  syncTasks();
}

/**
 * @brief Loop
 */
void loop() {
  getSystemTime();
  systemIntervalCheck();
  intervalBreach1Second();
  systemData.loops_a_second++;
}