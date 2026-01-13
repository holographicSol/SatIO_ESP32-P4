/*
    TaskHandler - Written By Benjamin Jack Cullen.
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
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
#include "./satio_file.h"
#include "./custommapping.h"
#include "./matrix.h"
#include "./serial_infocmd.h"
#include "./system_data.h"
#include "./sdmmc_helper.h"
#include "./task_handler.h"
#include "./multi_display_controller.h"

TaskHandle_t TaskDisplay;
TickType_t   LastWakeTimeTaskDisplay;
TickType_t   DelayTicksTaskDisplay;

TaskHandle_t TaskSerialInfoCMD;
TickType_t   LastWakeTimeTaskSerialInfoCMD;
TickType_t   DelayTicksTaskSerialInfoCMD;

TaskHandle_t TaskStorage;
TickType_t   LastWakeTimeTaskStorage;
TickType_t   DelayTicksTaskStorage;

TaskHandle_t TaskMultiplexers;
TickType_t   LastWakeTimeTaskMultiplexers;
TickType_t   DelayTicksTaskMultiplexers;

TaskHandle_t TaskPortControllerInput;
TickType_t   LastWakeTimeTaskPortControllerInput;
TickType_t   DelayTicksTaskPortControllerInput;

TaskHandle_t TaskGyro;
TickType_t   LastWakeTimeTaskGyro;
TickType_t   DelayTicksTaskGyro;

TaskHandle_t TaskGPS;
TickType_t   LastWakeTimeTaskGPS;
TickType_t   DelayTicksTaskGPS;

TaskHandle_t TaskUniverse;
TickType_t   LastWakeTimeTaskUniverse;
TickType_t   DelayTicksTaskUniverse;

TaskHandle_t TaskSwitches;
TickType_t   LastWakeTimeTaskSwitches;
TickType_t   DelayTicksTaskSwitches;

TaskHandle_t TaskLogging;
TickType_t   LastWakeTimeTaskLogging;
TickType_t   DelayTicksTaskLogging;

/** ----------------------------------------------------------------------------
 * Syncronize Tasks.
 * 
 * @brief Time syncronize tasks.
 */
void syncTasks() {
  Serial.println("[syncronizing system] please wait");
  global_task_sync=false;
  while (satioData.sync_rtc_immediately_flag==true) {
    getSystemTime();
    system_sync_retry_max--;
    if (system_sync_retry_max<=0)
      {Serial.println("[sync] took too long"); break;}
    delay(1);
  }
  global_task_sync=true;
  // Serial.println("unixtime sync: " + String(satioData.local_unixtime_uS));
}

/** ----------------------------------------------------------------------------
 * Is Task Delayed.
 * 
 * @brief Returns bool for task delayed.
 */
bool isTaskDelayed(TaskHandle_t taskHandle) {
    if (taskHandle == NULL) return false;
    eTaskState state = eTaskGetState(taskHandle);
    return (state == eBlocked) || (state == eSuspended);
}

/*
  Todo:
      Task States: Running, Ready, Blocked, Suspended, Deleted, Invalid.
      Device States: Found, Not Found, Initializing.
*/

/** ----------------------------------------------------------------------------
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
  while (global_task_sync==false) {vTaskDelay(1);}
  for (;;) {
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
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_STORAGE / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_STORAGE);}
  }
}
void createTaskStorage() {
    xTaskCreatePinnedToCore(
    taskStorage,   /* Function to implement the task */
    "TaskStorage", /* Name of the task */
    4096,          /* Stack size in words */
    NULL,          /* Task input parameter */
    2,             /* Priority of the task */
    &TaskStorage,  /* Task handle. */
    0);            /* Core where the task should run */
    esp_task_wdt_add(TaskStorage);
}

/** ----------------------------------------------------------------------------
 * GPS Task.
 * 
 * @brief Performas many operations including:
 *  (1) Collects, validates and stores GPS data.
 *  (2) Syncs INS data on successful validation.
 * Consider renaming task to something like 'time and location'
 */
void taskGPS(void * pvParameters) {
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Get, check and set gps data.
    // ------------------------------------------------
    readGPS();
    validateGPSData();
    // ------------------------------------------------
    // Set SatIO data.
    // ------------------------------------------------
    if (((gnggaData.valid_checksum=true) &&
        (gnrmcData.valid_checksum=true) &&
        (gpattData.valid_checksum=true)) ||
        satioData.set_rtc_datetime_flag==true) {
        setSatIOData();
        // --------------------------------------------
        // Set INS data.
        // --------------------------------------------
        set_ins(satioData.degrees_latitude,
                satioData.degrees_longitude,
                satioData.altitude_converted,
                satioData.ground_heading,
                satioData.speed_converted,
                atof(gnggaData.gps_precision_factor),
                gyroData.gyro_0_ang_z);
        // --------------------------------------------
        // Counters.
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
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_GPS / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_GPS);}
  }
}
void createTaskGPS() {
    xTaskCreatePinnedToCore(
    taskGPS,   /* Function to implement the task */
    "TaskGPS", /* Name of the task */
    4096,      /* Stack size in words */
    NULL,      /* Task input parameter */
    3,         /* Priority of the task */
    &TaskGPS,  /* Task handle. */
    0);        /* Core where the task should run */
    esp_task_wdt_add(TaskGPS);
}

/** ----------------------------------------------------------------------------
 * Gyro Task.
 * 
 * @brief Reads and stores gyroscopic data.
 */
void taskGyro(void * pvParameters) {
  while (global_task_sync==false) {vTaskDelay(1);}
  for (;;) {
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
                          satioData.ground_heading,
                          satioData.speed_converted,
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
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_GYRO0 / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_GYRO0);}
  }
}
void createTaskGyro() {
    xTaskCreatePinnedToCore(
    taskGyro,   /* Function to implement the task */
    "TaskGyro", /* Name of the task */
    4096,       /* Stack size in words */
    NULL,       /* Task input parameter */
    3,          /* Priority of the task */
    &TaskGyro,  /* Task handle. */
    0);         /* Core where the task should run */
    esp_task_wdt_add(TaskGyro);
}


/** ----------------------------------------------------------------------------
 * Universe Task.
 * 
 * @brief Stores various information about the universe!
 */
void taskUniverse(void * pvParameters) {
  while (global_task_sync==false) {vTaskDelay(1);}
  for (;;) {
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
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_UNIVERSE / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_UNIVERSE);}
  }
}
void createTaskUniverse() {
    xTaskCreatePinnedToCore(
    taskUniverse,   /* Function to implement the task */
    "TaskUniverse", /* Name of the task */
    16384,          /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &TaskUniverse,  /* Task handle. */
    0);             /* Core where the task should run */
    esp_task_wdt_add(TaskUniverse);
}

/** ----------------------------------------------------------------------------
 * Switch Task.
 * 
 * @brief Performs various operations including:
 *  (1) Martix calculations.
 *  (2) Mapping values.
 *  (3) Sets output values.
 *  (4) Instructing the portcontroller accordingly.
 */
void taskSwitches(void * pvParameters) {
  while (global_task_sync==false) {vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Calculate.
    // ------------------------------------------------
    if (matrixSwitch()) {
      systemData.i_count_matrix++;
      if (systemData.i_count_matrix>=UINT64_MAX-2)
        {systemData.i_count_matrix=0;}
    }
    // ------------------------------------------------
    // Mapping.
    // ------------------------------------------------
    map_values();
    // ------------------------------------------------
    // Output.
    // ------------------------------------------------
    setOutputValues();
    writeOutputPortControllerM1();
    // SwitchStat();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_SWITCHES==false)
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_SWITCHES / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_SWITCHES);}
  }
}
void createTaskSwitches() {
    xTaskCreatePinnedToCore(
    taskSwitches,   /* Function to implement the task */
    "TaskSwitches", /* Name of the task */
    4096,           /* Stack size in words */
    NULL,           /* Task input parameter */
    4,              /* Priority of the task */
    &TaskSwitches,  /* Task handle. */
    0);             /* Core where the task should run */
    esp_task_wdt_add(TaskSwitches);
}


/** ----------------------------------------------------------------------------
 * Multi Display Controller.
 * @brief Sends instructions to multi display controller.
 */
long time_7seg = 0;
long prev_time_7seg = 0;
long date_7seg = 0;
long prev_date_7seg = 0;

void taskDisplay(void * pvParameters) {
  // while (global_task_sync==false) {vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();

    /* test multi display module and library  */

    // test request interrupt
    requestMultiDisplayControllerData(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
    ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0);

    if (ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0) {

      // test indiocators
      updateIndicator(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
                      0,
                      255, 0, 0);
      updateIndicator(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
                      9,
                      0, 0, 255);
      
      // test 7 segment 4 digit display 0
      update7Segment4Digit(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      0,
      gnggaData.satellite_count);
      
      // test 7 segment 4 digit display 1
      update7Segment4Digit(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      1,
      gnggaData.gps_precision_factor);

      // test 7 segment 6 digit display 0
      update7Segment6Digit(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      2,
      satioData.padded_local_time_HHMMSS
      );

      // test 7 segment 6 digit display 1
      update7Segment6Digit(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      3,
      satioData.padded_local_short_date_DDMMYY
      );

      // test SSD1306 0
      updateSSD1306(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      0,
      0,
      1, 1,
      gnggaData.satellite_count
      );
      updateSSD1306(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
        0,
        1,
        1, 12,
        satioData.formatted_local_time_HHMMSS
      );
      drawSSD1306Canvas(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0, 0);
      
      // test SSD1306 1
      updateSSD1306(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      1,
      0,
      1, 1,
      gnggaData.satellite_count
      );
      updateSSD1306(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      1,
      1,
      1, 16,
      satioData.formatted_local_time_HHMMSS
      );
      drawSSD1306Canvas(I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0, 1);
    }

    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_7SEG==false)
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_7SEG / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_7SEG);}
  }
}
void createTaskDisplay() {
    xTaskCreatePinnedToCore(
    taskDisplay,   /* Function to implement the task */
    "TaskDisplay", /* Name of the task */
    4096,           /* Stack size in words */
    NULL,           /* Task input parameter */
    4,              /* Priority of the task */
    &TaskDisplay,  /* Task handle. */
    1);             /* Core where the task should run */
    esp_task_wdt_add(TaskDisplay);
}


/** ----------------------------------------------------------------------------
 * Port Controller Input Task.
 * 
 * @brief Reads pins on portcontroller.
 */
void taskPortControllerInput(void * pvParameters) {
  while (global_task_sync==false) {vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Read Input Port Controller.
    // ------------------------------------------------
    if (readInputPortControllerM1()) {
      systemData.i_count_portcontroller_input++;
      if (systemData.i_count_portcontroller_input>=UINT64_MAX-2)
        {systemData.i_count_portcontroller_input=0;}
    }
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_PORTCONTROLLER_INPUT==false)
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_PORTCONTROLLER_INPUT / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_PORTCONTROLLER_INPUT);}
  }
}

void createTaskPortControllerInput() {
    xTaskCreatePinnedToCore(
    taskPortControllerInput,   /* Function to implement the task */
    "TaskPortControllerInput", /* Name of the task */
    4096,           /* Stack size in words */
    NULL,           /* Task input parameter */
    4,              /* Priority of the task */
    &TaskPortControllerInput,  /* Task handle. */
    0);             /* Core where the task should run */
    esp_task_wdt_add(TaskPortControllerInput);
}

/** ----------------------------------------------------------------------------
 * Info Command Task.
 * 
 * @brief Processes a serial TXD and RXD operations:
 *  (1) Information out for other system and debug.
 *  (2) Commands in.
 */
void taskSerialInfoCMD(void * pvParameters) {
  while (global_task_sync==false) {vTaskDelay(1);}
  for (;;) {
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
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_SERIAL_INFOCMD / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_SERIAL_INFOCMD);}
  }
}
void createTaskSerialInfoCMD() {
  xTaskCreatePinnedToCore(
    taskSerialInfoCMD,   /* Function to implement the task */
    "TaskSerialInfoCMD", /* Name of the task */
    16384,                /* Stack size in words */
    NULL,                /* Task input parameter */
    3,                   /* Priority of the task */
    &TaskSerialInfoCMD,  /* Task handle. */
    0);                  /* Core where the task should run */
    esp_task_wdt_add(TaskSerialInfoCMD);
}

/** ----------------------------------------------------------------------------
 * Info Command Task.
 * 
 * @brief Reads all analog/digital multiplexer channels.
 */
void taskMultiplexers(void * pvParameters) {
  while (!global_task_sync) {vTaskDelay(pdMS_TO_TICKS(10));}
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // read muiltiplexer channels (customize as required).
    // ------------------------------------------------
    readAllADMultiplexerAnalogChannels(ad_mux_0);
    // Serial.println("[ad] " + String(i_chan) + ": " + String(ad_mux_0.data[i_chan]));

    // ------------------------------------------------
    // Counters
    // ------------------------------------------------
    systemData.i_count_read_mplex++;
    systemData.interval_breach_mplex = 1;
    if (systemData.i_count_read_mplex >= UINT32_MAX - 2)
    systemData.i_count_read_mplex = 0;
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_MULTIPLEXERS==false)
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_MULTIPLEXERS / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_MULTIPLEXERS);}
  }
}
void createTaskMultiplexers() {
    xTaskCreatePinnedToCore(
    taskMultiplexers,   /* Function to implement the task */
    "TaskMultiplexers", /* Name of the task */
    4096,               /* Stack size in words */
    NULL,               /* Task input parameter */
    4,                  /* Priority of the task */
    &TaskMultiplexers,  /* Task handle. */
    1);                 /* Core where the task should run */
    esp_task_wdt_add(TaskMultiplexers);
}


/** ----------------------------------------------------------------------------
 * Logging Task.
 * 
 * @brief Writes logs to disk.
 */
void taskLogging(void * pvParameters) {
  for (;;) {
    esp_task_wdt_reset();
    // check disk space
    // delete old logs if required
    // write new log
    // dt,x,y,z
    // Serial.printf("[log] flag: %d\n", systemData.logging_enabled);
    if (systemData.logging_enabled) {
      Serial.printf("[log] setting write flag true\n");
      sdmmcFlagData.write_log=true;
    }
    // else {Serial.println("[log] not enabled");}
    // ------------------------------------------------
    // Counters
    // ------------------------------------------------.
    systemData.i_count_logging++;
    systemData.interval_breach_logging = 1;
    if (systemData.i_count_logging >= UINT32_MAX - 2)
    systemData.i_count_logging = 0;
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_LOGGING==false)
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_LOGGING / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_LOGGING);}
  }
}
void createTaskLogging() {
    xTaskCreatePinnedToCore(
    taskLogging,   /* Function to implement the task */
    "TaskLogging", /* Name of the task */
    4096,               /* Stack size in words */
    NULL,               /* Task input parameter */
    4,                  /* Priority of the task */
    &TaskLogging,  /* Task handle. */
    1);                 /* Core where the task should run */
    esp_task_wdt_add(TaskLogging);
}

/** ----------------------------------------------------------------------------
 * PowerCfg: Ultimate Performance.
 * 
 * @brief Sets all task delay timings to optimum performance.  
 */
void setTasksDelayUltimatePerformance() {
    // DELAY_TASK_SERIAL_INFOCMD=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_SERIAL_INFOCMD;
    // TICK_DELAY_TASK_SERIAL_INFOCMD=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_SERIAL_INFOCMD;
    // xTaskNotifyGive(TaskSerialInfoCMD);

    DELAY_TASK_MULTIPLEXERS=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_MULTIPLEXERS;
    TICK_DELAY_TASK_MULTIPLEXERS=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_MULTIPLEXERS;
    xTaskNotifyGive(TaskMultiplexers);

    DELAY_TASK_GYRO0=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_GYRO;
    TICK_DELAY_TASK_GYRO0=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_GYRO;
    xTaskNotifyGive(TaskGyro);

    DELAY_TASK_UNIVERSE=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_UNIVERSE;
    TICK_DELAY_TASK_UNIVERSE=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_UNIVERSE;
    xTaskNotifyGive(TaskUniverse);

    DELAY_TASK_GPS=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_GPS;
    TICK_DELAY_TASK_GPS=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_GPS;
    xTaskNotifyGive(TaskGPS);

    DELAY_TASK_SWITCHES=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_SWITCHES;
    TICK_DELAY_TASK_SWITCHES=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_SWITCHES;
    xTaskNotifyGive(TaskSwitches);

    DELAY_TASK_PORTCONTROLLER_INPUT=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_PORTCONTROLLER_INPUT;
    TICK_DELAY_TASK_PORTCONTROLLER_INPUT=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_PORTCONTROLLER_INPUT;
    xTaskNotifyGive(TaskPortControllerInput);

    DELAY_TASK_LOGGING=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_LOGGING;
    TICK_DELAY_TASK_LOGGING=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_LOGGING;
    xTaskNotifyGive(TaskLogging);

    // DELAY_TASK_STORAGE=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_STORAGE;
    // TICK_DELAY_TASK_STORAGE=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_STORAGE;
    // xTaskNotifyGive(TaskStorage);
    // xTaskNotifyGive(TaskSerialInfoCMD);
}

/** ----------------------------------------------------------------------------
 * PowerCfg: Power Saving.
 * 
 * @brief Sets all task delay timings to power saving.  
 */
void setTasksDelayPowerSaving() {
    // DELAY_TASK_SERIAL_INFOCMD=POWER_CONFIG_ULTIMATE_PERFORMANCE_DELAY_TASK_SERIAL_INFOCMD;
    // TICK_DELAY_TASK_SERIAL_INFOCMD=POWER_CONFIG_ULTIMATE_PERFORMANCE_TICK_DELAY_TASK_SERIAL_INFOCMD;

    DELAY_TASK_MULTIPLEXERS=POWER_CONFIG_1_SECOND_DELAY_TASK_MULTIPLEXERS;
    TICK_DELAY_TASK_MULTIPLEXERS=POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_MULTIPLEXERS;
    xTaskNotifyGive(TaskMultiplexers);

    DELAY_TASK_GYRO0=POWER_CONFIG_1_SECOND_DELAY_TASK_GYRO;
    TICK_DELAY_TASK_GYRO0=POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_GYRO;
    xTaskNotifyGive(TaskGyro);

    DELAY_TASK_UNIVERSE=POWER_CONFIG_1_SECOND_DELAY_TASK_UNIVERSE;
    TICK_DELAY_TASK_UNIVERSE=POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_UNIVERSE;
    xTaskNotifyGive(TaskUniverse);

    DELAY_TASK_GPS=POWER_CONFIG_1_SECOND_DELAY_TASK_GPS;
    TICK_DELAY_TASK_GPS=POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_GPS;
    xTaskNotifyGive(TaskGPS);

    DELAY_TASK_SWITCHES=POWER_CONFIG_1_SECOND_DELAY_TASK_SWITCHES;
    TICK_DELAY_TASK_SWITCHES=POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_SWITCHES;
    xTaskNotifyGive(TaskSwitches);

    DELAY_TASK_PORTCONTROLLER_INPUT=POWER_CONFIG_1_SECOND_DELAY_TASK_PORTCONTROLLER_INPUT;
    TICK_DELAY_TASK_PORTCONTROLLER_INPUT=POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_PORTCONTROLLER_INPUT;
    xTaskNotifyGive(TaskPortControllerInput);

    DELAY_TASK_LOGGING=POWER_CONFIG_1_SECOND_DELAY_TASK_LOGGING;
    TICK_DELAY_TASK_LOGGING=POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_LOGGING;
    xTaskNotifyGive(TaskLogging);

    // DELAY_TASK_STORAGE=POWER_CONFIG_1_SECOND_DELAY_TASK_STORAGE;
    // TICK_DELAY_TASK_STORAGE=POWER_CONFIG_1_SECOND_TICK_DELAY_TASK_STORAGE;
    // xTaskNotifyGive(TaskStorage);
}

/** ----------------------------------------------------------------------------
 * Set Tick.
 * 
 * @brief Manually override use of millisecond or ticks for delays.
 */
void setTick(TaskHandle_t task_handle, bool *tick_delay, bool use_tick) {
  *tick_delay=use_tick; xTaskNotifyGive(task_handle);
}

/** ----------------------------------------------------------------------------
 * Set Delay.
 * 
 * @brief Manually override delay timing.
 */
void setDelay(TaskHandle_t task_handle, long *task_delay, long time_delay) {
  *task_delay=time_delay; xTaskNotifyGive(task_handle);
}