/*
    TaskHandler - Written By Benjamin Jack Cullen.
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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
#include "freertos/semphr.h"
#include "i2c_helper.h"

SemaphoreHandle_t i2c_bus0_mutex;

TaskHandle_t TaskStorage;
TaskHandle_t TaskLogging;
TaskHandle_t TaskSerialInfoCMD;
TaskHandle_t TaskGPS;
TaskHandle_t TaskGyro;
TaskHandle_t TaskMultiplexers;
TaskHandle_t TaskPortControllerInput;
TaskHandle_t TaskUniverse;
TaskHandle_t TaskSwitches;
TaskHandle_t TasMultikDisplay;

// PRIORITIES (Lower = Higher Priority)
#define TASK_GPS_PRIORITY                   2    // High: Time/location sync critical
#define TASK_SERIALINFOCMD_PRIORITY         2    // High: User interaction & debugging
#define TASK_GYRO_PRIORITY                  3    // Medium: Sensor reading
#define TASK_MULTIPLEXERS_PRIORITY          3    // Medium: Analog multiplexing
#define TASK_SWITCHES_PRIORITY              3    // Medium: Logic processing
#define TASK_PORTCONTROLLERINPUT_PRIORITY   4    // Medium: I/O reading

#define TASK_STORAGE_PRIORITY               3    // Medium: I/O operations, can wait
#define TASK_LOGGING_PRIORITY               5    // Low: Asynchronous data recording
#define TASK_UNIVERSE_PRIORITY              4    // Low: Computational, non-critical delay
#define TASK_MULTI_DISPLAY_PRIORITY         5    // Low: UI updates can be deferred

// CORE ASSIGNMENT (Spread load across both cores)
#define TASK_GPS_CORE                       1    // Core 1: Critical for system sync
#define TASK_SERIALINFOCMD_CORE             0    // Core 0: Keep on main (timing-sensitive)
#define TASK_GYRO_CORE                      0    // Core 0: Sensor reading (less critical)
#define TASK_MULTIPLEXERS_CORE              0    // Core 0: ADC/multiplexing
#define TASK_PORTCONTROLLERINPUT_CORE       0    // Core 0: I2C/GPIO input
#define TASK_SWITCHES_CORE                  0    // Core 0: Outputs need responsiveness

#define TASK_UNIVERSE_CORE                  0    // Core 0: Heavy compute, can defer
#define TASK_STORAGE_CORE                   0    // Core 0: I/O-heavy, doesn't need Core 0
#define TASK_LOGGING_CORE                   0    // Core 0: I/O operations
#define TASK_MULTI_DISPLAY_CORE             0    // Core 0: UI responsiveness

// STACK SIZES (Adjusted for task complexity)
#define TASK_STORAGE_STACK_SIZE             6144    // +50%: SDMMC operations
#define TASK_LOGGING_STACK_SIZE             5120    // +25%: File I/O
#define TASK_SERIALINFOCMD_STACK_SIZE       16384   // Keep: Complex serial processing
#define TASK_GPS_STACK_SIZE                 5120    // +25%: NMEA parsing + INS updates
#define TASK_GYRO_STACK_SIZE                4608    // +12%: Math operations
#define TASK_MULTIPLEXERS_STACK_SIZE        4096    // Keep: Simple ADC reading
#define TASK_PORTCONTROLLERINPUT_STACK_SIZE 4608    // +12%: I2C buffer + processing
#define TASK_SWITCHES_STACK_SIZE            5120    // +25%: Matrix calculations, mappings
#define TASK_UNIVERSE_STACK_SIZE            20480   // +25%: Expensive float math (planets, etc.)
#define TASK_DISPLAY_STACK_SIZE             6144    // +50%: Multiple display updates + formatting

/** ----------------------------------------------------------------------------
 * Syncronize Tasks.
 * 
 * @brief Time syncronize tasks. Main loop and some tasks will not begin until
 *        this function has completed.
 * 
 *        Inital synchronization trigger is GPS milliseconds 00 (any second).
 * 
 *        After initial synchronization the system will attempt to synchronize
 *        every GPS seconds zero (minutely).
 */
void syncTasks() {
  Serial.println("[syncronizing system] please wait");
  global_task_sync=false;
  while (satioData.sync_rtc_immediately_flag==true) {
    getSystemTime();
    system_sync_retry_max--;
    if (system_sync_retry_max<=0) {Serial.println("[sync] took too long!"); break;}
    // Serial.println("[SYN] " + String(system_sync_retry_max));
    delayMicroseconds(1);
  }
  global_task_sync=true;
  // Serial.println("[SYN] syncronized." + String(system_sync_retry_max));
  // Serial.println("[SYN] local unixtime_uS " + String(satioData.local_unixtime_uS));
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
    TASK_STORAGE_STACK_SIZE, /* Stack size in words */
    NULL,          /* Task input parameter */
    TASK_STORAGE_PRIORITY, /* Priority of the task */
    &TaskStorage,          /* Task handle. */
    TASK_STORAGE_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TaskStorage);
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
    TASK_LOGGING_STACK_SIZE, /* Stack size in words */
    NULL,          /* Task input parameter */
    TASK_LOGGING_PRIORITY, /* Priority of the task */
    &TaskLogging,          /* Task handle. */
    TASK_LOGGING_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TaskLogging);
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
    TASK_SERIALINFOCMD_STACK_SIZE, /* Stack size in words */
    NULL,                /* Task input parameter */
    TASK_SERIALINFOCMD_PRIORITY, /* Priority of the task */
    &TaskSerialInfoCMD,          /* Task handle. */
    TASK_SERIALINFOCMD_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TaskSerialInfoCMD);
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
        (gpattData.valid_checksum=true))
          ||
          satioData.set_rtc_datetime_flag==true) {
        // -> syncRTC -- > setRTCDateTime --> setSystemTime, storeRTCSYNCTime
        satioData.set_rtc_datetime_flag=true;
        // xSemaphoreTake(i2c_bus0_mutex, 1000 / portTICK_PERIOD_MS);
        setSatIOData();
        // xSemaphoreGive(i2c_bus0_mutex);
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
    TASK_GPS_STACK_SIZE, /* Stack size in words */
    NULL,      /* Task input parameter */
    TASK_GPS_PRIORITY, /* Priority of the task */
    &TaskGPS,          /* Task handle. */
    TASK_GPS_CORE);    /* Core where the task should run */
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
    TASK_GYRO_STACK_SIZE, /* Stack size in words */
    NULL,       /* Task input parameter */
    TASK_GYRO_PRIORITY, /* Priority of the task */
    &TaskGyro,          /* Task handle. */
    TASK_GYRO_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TaskGyro);
}

/** ----------------------------------------------------------------------------
 * Multiplexer Task.
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
    for (uint8_t i_chan = 0; i_chan < 16; i_chan++) {
      readADMultiplexerAnalogChannel(ad_mux_0, i_chan);
      // Serial.println("[ad] " + String(i_chan) + ": " + String(ad_mux_0.data[i_chan]));
    }
    // ------------------------------------------------
    // Counters
    // ------------------------------------------------
    systemData.i_count_read_mplex_0++;
    systemData.interval_breach_mplex_0 = 1;
    if (systemData.i_count_read_mplex_0 >= UINT32_MAX - 2)
    systemData.i_count_read_mplex_0 = 0;
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
    TASK_MULTIPLEXERS_STACK_SIZE, /* Stack size in words */
    NULL,               /* Task input parameter */
    TASK_MULTIPLEXERS_PRIORITY, /* Priority of the task */
    &TaskMultiplexers,          /* Task handle. */
    TASK_MULTIPLEXERS_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TaskMultiplexers);
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
    if (readInputPortControllerReadPins(iic_2, I2C_ADDR_INPUT_PORTCONTROLLER)==true) {
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
    TASK_PORTCONTROLLERINPUT_STACK_SIZE, /* Stack size in words */
    NULL,                      /* Task input parameter */
    TASK_PORTCONTROLLERINPUT_PRIORITY, /* Priority of the task */
    &TaskPortControllerInput,          /* Task handle. */
    TASK_PORTCONTROLLERINPUT_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TaskPortControllerInput);
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
    writeOutputPortControllerSetPins(iic_1, I2C_ADDR_OUTPUT_PORTCONTROLLER);
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
    TASK_SWITCHES_STACK_SIZE, /* Stack size in words */
    NULL,           /* Task input parameter */
    TASK_SWITCHES_PRIORITY, /* Priority of the task */
    &TaskSwitches,          /* Task handle. */
    TASK_SWITCHES_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TaskSwitches);
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
    TASK_UNIVERSE_STACK_SIZE, /* Stack size in words */
    NULL,           /* Task input parameter */
    TASK_UNIVERSE_PRIORITY, /* Priority of the task */
    &TaskUniverse,          /* Task handle. */
    TASK_UNIVERSE_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TaskUniverse);
}

/** ----------------------------------------------------------------------------
 * Multi Display Controller.
 * @brief Sends instructions to multi display controller.
 */

void tasMultikDisplay(void * pvParameters) {
  // while (global_task_sync==false) {vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();

    /* test multi display module and library  */

    // test request interrupt
    requestMultiDisplayControllerData(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
    ISR_FLAG_MULTI_DISPLAY_CONTROLLER_0);

    if (ALLOW_UPDATE_MULTIDISPLAY_CONTROLLER_0) {

      // test indiocators
      updateIndicator(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
                      0,
                      255, 0, 0);
      updateIndicator(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
                      9,
                      0, 0, 255);
      
      // test 7 segment 4 digit display 0
      update7Segment4Digit(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      0,
      gnggaData.satellite_count);
      
      // test 7 segment 4 digit display 1
      update7Segment4Digit(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      1,
      gnggaData.gps_precision_factor);

      // test 7 segment 6 digit display 0
      update7Segment6Digit(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      2,
      satioData.padded_local_time_HHMMSS
      );

      // test 7 segment 6 digit display 1
      update7Segment6Digit(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      3,
      satioData.padded_local_short_date_DDMMYY
      );

      // test SSD1306 0
      updateSSD1306(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      0,
      0,
      1, 1,
      gnggaData.satellite_count
      );
      updateSSD1306(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
        0,
        1,
        1, 12,
        satioData.formatted_local_time_HHMMSS
      );
      drawSSD1306Canvas(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0, 0);
      
      // test SSD1306 1
      updateSSD1306(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      1,
      0,
      1, 1,
      gnggaData.satellite_count
      );
      updateSSD1306(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0,
      1,
      1,
      1, 16,
      satioData.formatted_local_time_HHMMSS
      );
      drawSSD1306Canvas(iic_0, I2C_MULTIDISPLAY_CONTROLLER_ADDRESS_0, 1);
    }

    systemData.i_count_display++;
    if (systemData.i_count_display>=UINT32_MAX-2)
      {systemData.i_count_display=0;}

    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (TICK_DELAY_TASK_DISPLAY==false)
      {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_DISPLAY / portTICK_PERIOD_MS);}
    else {xTaskNotifyWait(0x00, 0x00, NULL, DELAY_TASK_DISPLAY);}
  }
}
void createTaskMultiDisplay() {
    xTaskCreatePinnedToCore(
    tasMultikDisplay,   /* Function to implement the task */
    "TasMultikDisplay", /* Name of the task */
    TASK_DISPLAY_STACK_SIZE, /* Stack size in words */
    NULL,           /* Task input parameter */
    TASK_MULTI_DISPLAY_PRIORITY, /* Priority of the task */
    &TasMultikDisplay,          /* Task handle. */
    TASK_MULTI_DISPLAY_CORE);    /* Core where the task should run */
    esp_task_wdt_add(TasMultikDisplay);
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