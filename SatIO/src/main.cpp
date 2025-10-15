/*

  SatIO - Written by Benjamin Jack Cullen.

-----

  A general purpose programmable I/O platform for automation, throughput and LLM's.

  A huge matrix switch in a small package, supporting stacks of logic across
  70 output pins and 100 mapping slots.

-----

  Inference in Bayesian Reasoning? Moon tracking for example can be used to track the moon, it can also be used for one example; to 
  track the tides, if the system is aware of moon/planetary positioning and datetime then marine life values may also be inferred
  relative to the inferred tide values and known datetime. There is a lot of data that can be used in many ways, with a kind of network
  effect. Or more simply 'SatIO is one hell of a switch'.

-----

  Wiring For Keystudio ESP32 PLUS Development Board:

          ESP32: 1st ATMEGA2560 with shield as Port Controller (not on multiplexer):
          ESP32: I2C SDA -> ATMEGA2560: I2C SDA
          ESP32: I2C SCL -> ATMEGA2560: I2C SCL

          Other ESP32 I2C Devices (not on multiplexer):
          ESP32: SDA0 SCL0 -> DS3231 (RTC): SDA, SCL (5v)

          ESP32 i2C: I2C Multiplexing (3.3v) (for peripherals):
          ESP32: i2C -> TCA9548A: SDA, SCL

          ESP32: Analog/Digital Multiplexing (3.3v) (for peripherals):
          ESP32: io4    -> CD74HC4067: SIG
          ESP32: io32   -> CD74HC4067: S0
          ESP32: io33   -> CD74HC4067: S1
          ESP32: io16   -> CD74HC4067: S2
          ESP32: io17   -> CD74HC4067: S3
          CD74HC4067 C0 -> DHT11: SIG

          ESP32: WTGPS300P (5v) (for getting a downlink):
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

TaskHandle_t TaskGPS;
TaskHandle_t TaskUniverse;
TaskHandle_t TaskMultiplexers;
TaskHandle_t TaskGyro;
TaskHandle_t TaskSwitches;
TaskHandle_t TaskSerialInfoCMD;
TaskHandle_t TasKSystemTiming;
TaskHandle_t TaskStorage;

int DELAY_TASK_SYSTEM_TIMING=1;
int DELAY_TASK_SERIAL_INFOCMD=1;
int DELAY_TASK_MULTIPLEXERS=1;
int DELAY_TASK_GYRO0=1;
int DELAY_TASK_UNIVERSE=500;
int DELAY_TASK_GPS=1;
int DELAY_TASK_SWITCHES=1;
int DELAY_TASK_STORAGE=1000;
bool TICK_DELAY_TASK_SYSTEM_TIMING=false;
bool TICK_DELAY_TASK_SERIAL_INFOCMD=false;
bool TICK_DELAY_TASK_MULTIPLEXERS=false;
bool TICK_DELAY_TASK_GYRO0=false;
bool TICK_DELAY_TASK_UNIVERSE=false;
bool TICK_DELAY_TASK_GPS=false;
bool TICK_DELAY_TASK_SWITCHES=false;
bool TICK_DELAY_TASK_STORAGE=false;
bool global_task_sync=false;
long system_sync_retry_max=2000;

/**
 * Syncronize Taks.
 * 
 * @brief Time syncronize tasks.
 */
void syncTasks() {
  Serial.println("[syncronizing system] please wait");
  global_task_sync=false;
  while (!sync_rtc_bool==true) {
    getSystemTime();
    system_sync_retry_max--;
    if (system_sync_retry_max<=0)
      {Serial.println("[sync] took too long"); break;}
    delay(1);
  }
  global_task_sync=true;
  // Serial.println("unixtime sync: " + String(satioData.local_unixtime_uS));
}

/**
 * Storage Task.
 * 
 * @brief Performas many operations including:
 *  (1) Card insertion checks.
 *  (2) Mount automatically.
 *  (3) Unmount automatically.
 *  (4) Read/write operations.
 *  (5) Other storage operations.
 *  (6) Powers down the sdcard when not in use. 
 */
void taskStorage(void * pvParameters) {
  esp_task_wdt_add(nullptr);
  while (global_task_sync==false) {vTaskDelay(1);}
  while (1) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // SDCard Begin
    // ------------------------------------------------
    sdcardBegin();
    // statSDCard(); // uncomment to debug
    // ------------------------------------------------
    // Check Flags
    // ------------------------------------------------
    sdcardFlagHandler();
    // ------------------------------------------------
    // Power Down and persist sdcard data
    // ------------------------------------------------
    sdcardSleepMode0();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_STORAGE==false)
      {vTaskDelay(DELAY_TASK_STORAGE / portTICK_PERIOD_MS);}
    else {vTaskDelay(DELAY_TASK_STORAGE);}
  }
}

/**
 * GPS Task.
 * 
 * @brief Performas many operations including:
 *  (1) Collects, validates and stores GPS data.
 *  (2) Syncs INS data on successful validation.
 */
void taskGPS(void * pvParameters) {
  esp_task_wdt_add(nullptr);
  while (1) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Get, check and set gps data.
    // ------------------------------------------------
    readGPS();
    validateGPSData();
    // ------------------------------------------------
    // Set SatIO data.
    // ------------------------------------------------
    if ((gnggaData.valid_checksum=true) &&
        (gnrmcData.valid_checksum=true) &&
        (gpattData.valid_checksum=true)) {
        setSatIOData();
        // --------------------------------------------
        // Set INS data.
        // --------------------------------------------
        set_ins(satioData.degrees_latitude,
                satioData.degrees_longitude,
                atof(gnggaData.altitude),
                atof(gnrmcData.ground_heading),
                atof(gnrmcData.ground_speed),
                atof(gnggaData.gps_precision_factor),
                gyroData.gyro_0_ang_z);
        // --------------------------------------------
        // Count reads.
        // --------------------------------------------
        systemData.i_count_read_gps++;
        systemData.interval_breach_gps = 1;
        if (systemData.i_count_read_gps>=UINT32_MAX-2)
          {systemData.i_count_read_gps=0;}
    }
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_GPS==false)
      {vTaskDelay(DELAY_TASK_GPS / portTICK_PERIOD_MS);}
    else {vTaskDelay(DELAY_TASK_GPS);}
  }
}

/**
 * Universe Task.
 * 
 * @brief Stores various information about the universe!
 */
void taskUniverse(void * pvParameters) {
  esp_task_wdt_add(nullptr);
  while (global_task_sync==false) {vTaskDelay(1);}
  while (1) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Track Home Sun, Moon & Planets.
    // ------------------------------------------------
    trackPlanets(satioData.degrees_latitude,
                 satioData.degrees_longitude,
                 satioData.rtc_year,
                 satioData.rtc_month,
                 satioData.rtc_mday,
                 satioData.rtc_hour,
                 satioData.rtc_minute,
                 satioData.rtc_second,
                 satioData.local_hour,
                 satioData.local_minute,
                 satioData.local_second,
                 atol(gnggaData.altitude));
    systemData.i_count_track_planets++;
    systemData.interval_breach_track_planets = 1;
    if (systemData.i_count_track_planets>=UINT32_MAX-2)
      {systemData.i_count_track_planets=0;}
    // ------------------------------------------------
    // Track Meteors.
    // ------------------------------------------------
    setMeteorShowerWarning(satioData.local_month,
                           satioData.local_mday);
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_UNIVERSE==false)
      {vTaskDelay(DELAY_TASK_UNIVERSE / portTICK_PERIOD_MS);}
    else {vTaskDelay(DELAY_TASK_UNIVERSE);}
  }
}

/**
 * Switch Task.
 * 
 * @brief Performs various operations including:
 *  (1) Martix calculations.
 *  (2) Mapping values.
 *  (3) Sets output values.
 *  (4) Instructing the portcontroller accordingly.
 */
void taskSwitches(void * pvParameters) {
  esp_task_wdt_add(nullptr);
  while (global_task_sync==false) {vTaskDelay(1);}
  while ((1)) {
    esp_task_wdt_reset();
    if (matrixSwitch()) {
      systemData.i_count_matrix++;
      if (systemData.i_count_matrix>=UINT64_MAX-2)
        {systemData.i_count_matrix=0;}
    }
    map_values();
    setOutputValues();
    writePortControllerM1();
    SwitchStat();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_SWITCHES==false)
      {vTaskDelay(DELAY_TASK_SWITCHES / portTICK_PERIOD_MS);}
    else {vTaskDelay(DELAY_TASK_SWITCHES);}
  }
}

int i_chan=0;
/**
 * Multiplexer Task.
 * 
 * @brief Any reads/writes to multiplexers occur on this task.
 */
void taskMultiplexers(void * pvParameters) {
  esp_task_wdt_add(nullptr);
  while (global_task_sync==false) {vTaskDelay(1);}
  while (1) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Step over analog/digital multiplexer channela
    // ------------------------------------------------
    for (i_chan=0; i_chan < 16; i_chan++) {
      // ------------------------------------------------
      // Set multiplexer channel.
      // ------------------------------------------------
      setMultiplexChannel_AD(0, i_chan);
      multiplexerData.ADMPLEX_0_DATA[i_chan]=analogRead(ADMPLEX_0_SIG);
    }
    // ------------------------------------------------
    // Count reads.
    // ------------------------------------------------
    systemData.i_count_read_mplex++;
    systemData.interval_breach_mplex = 1;
    if (systemData.i_count_read_mplex>=UINT32_MAX-2)
      {systemData.i_count_read_mplex=0;}
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_MULTIPLEXERS==false)
      {vTaskDelay(DELAY_TASK_MULTIPLEXERS / portTICK_PERIOD_MS);}
    else {vTaskDelay(DELAY_TASK_MULTIPLEXERS);}
  }
}

/**
 * Gyro Task.
 * 
 * @brief Reads and stores gyroscopic data.
 */
void taskGyro(void * pvParameters) {
  esp_task_wdt_add(nullptr);
  while (global_task_sync==false) {vTaskDelay(1);}
  while (1) {
    esp_task_wdt_reset();
    if (readGyro()==true) {
      systemData.i_count_read_gyro_0++;
      systemData.interval_breach_gyro_0 = 1;
      if (systemData.i_count_read_gyro_0>=UINT32_MAX-2)
        {systemData.i_count_read_gyro_0=0;}
      // ----------------------------------------------
      // Estimate INS data.
      // INS data is fed bsck into INS.
      // ----------------------------------------------
      if (systemData.interval_breach_gyro_0==true) {
      if (ins_estimate_position(gyroData.gyro_0_ang_y,
                                gyroData.gyro_0_ang_z,
                                atof(gnrmcData.ground_heading),
                                atof(gnrmcData.ground_speed),
                                satioData.local_unixtime_uS)==true) {
                                systemData.i_count_read_ins++;
                                systemData.interval_breach_ins=1;
                                if (systemData.i_count_read_ins>=UINT32_MAX-2)
                                  {systemData.i_count_read_ins=0;}}
      }
    }
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_GYRO0==false)
      {vTaskDelay(DELAY_TASK_GYRO0 / portTICK_PERIOD_MS);}
    else {vTaskDelay(DELAY_TASK_GYRO0);}
  }
}

/**
 * Info Command Task.
 * 
 * @brief Processes a serial TXD and RXD operations:
 *  (1) Information out for other system and debug.
 *  (2) Commands in.
 */
void taskSerialInfoCMD(void * pvParameters) {
  esp_task_wdt_add(nullptr);
  while (global_task_sync==false) {vTaskDelay(1);}
  while (1) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Note that stat is ran in main loop, not here.
    // ------------------------------------------------
    CmdProcess();
    outputSentences();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_SERIAL_INFOCMD==false)
      {vTaskDelay(DELAY_TASK_SERIAL_INFOCMD / portTICK_PERIOD_MS);}
    else {vTaskDelay(DELAY_TASK_SERIAL_INFOCMD);}
  }
}

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
  while(!Wire.begin());
  Serial.println("[IIC] setting IIC clock: 400kHz (400000L)");
  Wire.setClock(400000L);
  // --------------------------------------------------------------
  // Initialize Port Controller.
  // --------------------------------------------------------------
  Serial.println("[IIC] instructing port controller set low...");
  writePortControllerM0();
  // --------------------------------------------------------------
  // Analog/Digital Multiplexers(s).
  // --------------------------------------------------------------
  Serial.println("[AD] setting analog/digital multiplexer channel: 0");
  setMultiplexChannel_AD(0, 0);
  // --------------------------------------------------------------
  // Initialize I2C Multiplexer(s).
  // --------------------------------------------------------------
  Serial.println("[IIC] setting IIC multiplexer channel: 0");
  setMultiplexChannel_I2C(0, 0);
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
  // xTask Storage.
  // --------------------------------------------------------------
  DELAY_TASK_STORAGE=500;       // delay time/ticks
  TICK_DELAY_TASK_STORAGE=false; // false for millisecond delay
  xTaskCreatePinnedToCore(
    taskStorage,   /* Function to implement the task */
    "TaskStorage", /* Name of the task */
    4096,          /* Stack size in words */
    NULL,          /* Task input parameter */
    2,             /* Priority of the task */
    &TaskStorage,  /* Task handle. */
    0);            /* Core where the task should run */
    delay(50);     /* delay between task creation */
  vTaskSuspend(TaskStorage);
  // --------------------------------------------------------------
  // xTask GPS.
  // --------------------------------------------------------------
  DELAY_TASK_GPS=1;         // delay time/ticks
  TICK_DELAY_TASK_GPS=true; // false for millisecond delay
  xTaskCreatePinnedToCore(
    taskGPS,   /* Function to implement the task */
    "TaskGPS", /* Name of the task */
    4096,      /* Stack size in words */
    NULL,      /* Task input parameter */
    3,         /* Priority of the task */
    &TaskGPS,  /* Task handle. */
    0);        /* Core where the task should run */
    delay(50); /* delay between task creation */
  // vTaskSuspend(TaskGPS);
  // --------------------------------------------------------------
  // xTask Serial Information Command.
  // --------------------------------------------------------------
  DELAY_TASK_SERIAL_INFOCMD=1;         // delay time/ticks
  TICK_DELAY_TASK_SERIAL_INFOCMD=true; // false for millisecond delay
  xTaskCreatePinnedToCore(
    taskSerialInfoCMD,   /* Function to implement the task */
    "TaskSerialInfoCMD", /* Name of the task */
    16384,                /* Stack size in words */
    NULL,                /* Task input parameter */
    3,                   /* Priority of the task */
    &TaskSerialInfoCMD,  /* Task handle. */
    0);                  /* Core where the task should run */
    delay(50);           /* delay between task creation */
  // vTaskSuspend(TaskSerialInfoCMD);
  // --------------------------------------------------------------
  // xTask Multiplexers.
  // --------------------------------------------------------------
  DELAY_TASK_MULTIPLEXERS=1;          // delay time/ticks
  TICK_DELAY_TASK_MULTIPLEXERS=true; // false for millisecond delay
  xTaskCreatePinnedToCore(
    taskMultiplexers,   /* Function to implement the task */
    "TaskMultiplexers", /* Name of the task */
    4096,               /* Stack size in words */
    NULL,               /* Task input parameter */
    3,                  /* Priority of the task */
    &TaskMultiplexers,  /* Task handle. */
    0);                 /* Core where the task should run */
    delay(50);          /* delay between task creation */
  vTaskSuspend(TaskMultiplexers);
  // --------------------------------------------------------------
  // xTask Gyro.
  // --------------------------------------------------------------
  DELAY_TASK_GYRO0=1;         // delay time/ticks
  TICK_DELAY_TASK_GYRO0=true; // false for millisecond delay
  xTaskCreatePinnedToCore(
    taskGyro,   /* Function to implement the task */
    "TaskGyro", /* Name of the task */
    4096,       /* Stack size in words */
    NULL,       /* Task input parameter */
    3,          /* Priority of the task */
    &TaskGyro,  /* Task handle. */
    0);         /* Core where the task should run */
    delay(50);  /* delay between task creation */
  vTaskSuspend(TaskGyro);
  // --------------------------------------------------------------
  // xTask Universe.
  // --------------------------------------------------------------
  myAstroBegin(); // call helper library begin function
  DELAY_TASK_UNIVERSE=1;       // delay time/ticks
  TICK_DELAY_TASK_UNIVERSE=false; // false for millisecond delay
  xTaskCreatePinnedToCore(
    taskUniverse,   /* Function to implement the task */
    "TaskUniverse", /* Name of the task */
    16384,          /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &TaskUniverse,  /* Task handle. */
    0);             /* Core where the task should run */
    delay(50);      /* delay between task creation */
  vTaskSuspend(TaskUniverse);
  // --------------------------------------------------------------
  // xTask TaskSwitches.
  // --------------------------------------------------------------
  DELAY_TASK_SWITCHES=1;         // delay time/ticks
  TICK_DELAY_TASK_SWITCHES=true; // false for millisecond delay
  xTaskCreatePinnedToCore(
    taskSwitches,   /* Function to implement the task */
    "TaskSwitches", /* Name of the task */
    4096,           /* Stack size in words */
    NULL,           /* Task input parameter */
    4,              /* Priority of the task */
    &TaskSwitches,  /* Task handle. */
    0);             /* Core where the task should run */
  delay(50);        /* delay between task creation */
  vTaskSuspend(TaskSwitches);

  // --------------------------------------------------------------
  // Wait for sync function to complete.
  // --------------------------------------------------------------
  syncTasks();
  // vTaskResume(TaskSerialInfoCMD);
  vTaskResume(TaskStorage);
  vTaskResume(TaskMultiplexers);
  vTaskResume(TaskGyro);
  vTaskResume(TaskUniverse);
  vTaskResume(TaskSwitches);
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