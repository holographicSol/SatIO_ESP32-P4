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
// #include "./esp32_helper.h"
#include "./sidereal_helper.h"
#include "./hextodig.h"
#include "./ins.h"
#include "./satio.h"
#include "./satio_file.h"
#include "./custommapping.h"
#include "./matrix.h"
#include "./serial_infocmd.h"
#include "./system_data.h"
#include "./satio_file.h"
#include "./sdcard_helper.h"
#include "./task_handler.h"
// #include "./multi_display_controller.h"
#include "freertos/semphr.h"
#include "i2c_helper.h"
#include "./satio_lvgl.h"

SemaphoreHandle_t i2c_bus0_mutex;

TaskHandle_t TaskGPS;
TaskHandle_t TaskGyro;
TaskHandle_t TaskMultiplexers;
TaskHandle_t TaskSerialInfoCMD;
TaskHandle_t TaskSwitches;
TaskHandle_t TaskStorage;
TaskHandle_t TaskLogging;
TaskHandle_t TaskUniverse;

// PRIORITY
#define TASK_GPS_PRIORITY                   5    // High: Time/location sync critical
#define TASK_GYRO_PRIORITY                  4    // High: Sensor reading
#define TASK_MULTIPLEXERS_PRIORITY          4    // High: Analog multiplexing

#define TASK_SWITCHES_PRIORITY              4    // High: Logic processing
#define TASK_SERIALINFOCMD_PRIORITY         3    // High: User interaction & debugging
#define TASK_UNIVERSE_PRIORITY              2    // LOW: Computational, non-critical delay
#define TASK_STORAGE_PRIORITY               2    // LOW: I/O operations, can wait
#define TASK_LOGGING_PRIORITY               2    // LOW: Asynchronous data recording

// CORE 0 ASSIGNMENT
#define TASK_SERIALINFOCMD_CORE             0    // Core 0: Keep on main (timing-sensitive)
#define TASK_GYRO_CORE                      0    // Core 0: Sensor reading (less critical)
#define TASK_MULTIPLEXERS_CORE              0    // Core 0: ADC/multiplexing
#define TASK_PORTCONTROLLERINPUT_CORE       0    // Core 0: I2C/GPIO input
#define TASK_SWITCHES_CORE                  0    // Core 0: Outputs need responsiveness
#define TASK_UNIVERSE_CORE                  0    // Core 0: Heavy compute, can defer
#define TASK_STORAGE_CORE                   0    // Core 0: I/O-heavy, doesn't need Core 0
#define TASK_LOGGING_CORE                   0    // Core 0: I/O operations
#define TASK_DISPLAY_CORE                   0    // Core 0: UI responsiveness

// CORE 1 ASSIGNMENT
#define TASK_GPS_CORE                       0    // Critical for system timing regardless of gps

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
#define TASK_DISPLAY_STACK_SIZE             20480    // +50%: Multiple display updates + formatting

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
  esp_task_wdt_add(NULL);
  // esp_task_wdt_delete(NULL);
  // while (global_task_sync==false) {esp_task_wdt_reset(); vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // SDCard Begin
    // ------------------------------------------------
    sdcard_mount();
    esp_task_wdt_reset();
    // statSDCard(); // uncomment to debug
    // ------------------------------------------------
    // Check Flags
    // ------------------------------------------------
    sdcardFlagHandler();
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Power Down and persist sdcard data
    // ------------------------------------------------
    // sdcardSleepMode0();
    // vTaskDelay(500 / portTICK_PERIOD_MS); // give some time for sdcard to settle before unmount

    sdcard_unmount();
    esp_task_wdt_reset();
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
}

/** ----------------------------------------------------------------------------
 * Logging Task.
 * 
 * @brief Writes logs to disk.
 */
void taskLogging(void * pvParameters) {
  esp_task_wdt_add(NULL);
  while (global_task_sync==false) {esp_task_wdt_reset(); vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    // check disk space
    // delete old logs if required
    // write new log
    // dt,x,y,z
    // Serial.printf("[log] flag: %d\n", systemData.logging_enabled);
    if (systemData.logging_enabled) {
      Serial.printf("[log] setting write flag true\n");
      sdcardFlagData.write_log=true;
    }
    esp_task_wdt_reset();
    // else {Serial.println("[log] not enabled");}
    // ------------------------------------------------
    // Counters
    // ------------------------------------------------.
    systemData.i_count_logging++;
    systemData.interval_breach_logging = true;
    if (systemData.i_count_logging >= UINT32_MAX - 2)
    systemData.i_count_logging = 0;
    esp_task_wdt_reset();
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
}

/** ----------------------------------------------------------------------------
 * Info Command Task.
 * 
 * @brief Processes a serial TXD and RXD operations:
 *  (1) Information out for other system and debug.
 *  (2) Commands in.
 */
void taskSerialInfoCMD(void * pvParameters) {
  esp_task_wdt_add(NULL);
  while (global_task_sync==false) {esp_task_wdt_reset(); vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Note that stat is ran in main loop, not here.
    // ------------------------------------------------
    // CmdProcess(); // ported to espidf method
    outputSentences();
    esp_task_wdt_reset();
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
}

/** ----------------------------------------------------------------------------------------
 * @brief Interval breach (System counters).
 */
int64_t prev_tv_sec;

void intervalBreach1Second(void) {
  // if (systemData.interval_breach_1_second) {
    // store system time
    storeLocalTime();
    // store rtc time
    storeRTCTime();
    // set loop counter
    systemData.total_loops_a_second = systemData.loops_a_second;
    systemData.loops_a_second = 0;
    // set gps counters
    systemData.total_gps = systemData.i_count_read_gps;
    systemData.i_count_read_gps = 0;
    // set ins counters
    systemData.total_ins = systemData.i_count_read_ins;
    systemData.i_count_read_ins = 0;
    // set gyro counters
    systemData.total_gyro_0 = systemData.i_count_read_gyro_0;
    systemData.i_count_read_gyro_0 = 0;
    // set mplex counters
    systemData.total_mplex_0 = systemData.i_count_read_mplex_0;
    systemData.i_count_read_mplex_0 = 0;
    // set mplex counters
    systemData.total_matrix = systemData.i_count_matrix;
    systemData.i_count_matrix = 0;
    // set mplex counters
    systemData.total_portcontroller_output = systemData.i_count_port_controller_output;
    systemData.i_count_port_controller_output = 0;
    // set mplex counters
    systemData.total_universe = systemData.i_count_track_planets;
    systemData.i_count_track_planets = 0;
    // set mplex counters
    systemData.total_infocmd = systemData.i_count_read_serial_commands;
    systemData.i_count_read_serial_commands = 0;
    // set portcontroller input counters
    systemData.total_portcontroller_input = systemData.i_count_portcontroller_input;
    systemData.i_count_portcontroller_input = 0;
    // set display counters
    systemData.total_display = systemData.i_count_display;
    systemData.i_count_display = 0;
    // set second flags
    systemData.interval_breach_track_planets = 1;
    // set uptime
    systemData.uptime_seconds++;
    if (systemData.uptime_seconds >= LONG_MAX - 2)
      {systemData.uptime_seconds = 0;
        printf("[reset uptime_seconds] %ld\n", systemData.uptime_seconds);
      }
  // }
}

void system_timing() {
    // printf("[LOOP] %ld\n", systemData.loops_a_second);
    gettimeofday(&tv_now, NULL);
    timeinfo = localtime(&tv_now.tv_sec); // Assumes localtime works
    satioData.local_unixtime_uS = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    // printf("[LOOP] localtime: %lld\n", satioData.local_unixtime_uS);

    if (tv_now.tv_sec != prev_tv_sec) {
        prev_tv_sec = tv_now.tv_sec;
        systemData.interval_breach_1_second_output=true;
        intervalBreach1Second();
        printf(
            "[ %llu ] "
            "gps=%s "
            "rtc=%s "
            "lcl=%s "
            "syn=%s "
            "t_loop=%ld "
            "t_gps=%ld "
            "t_ins=%ld "
            "t_gyr=%ld "
            "t_mlx=%ld "
            "t_uni=%ld "
            "t_pci=%ld "
            "t_mtx=%ld "
            "t_pco=%ld "
            "t_dsp=%ld  "

            "sat=%s "
            "deg_lat=%.7f "
            "deg_lon=%.7f "
            "usr_lat=%.7f "
            "usr_lon=%.7f "
            "sys_lat=%.7f "
            "sys_lon=%.7f "
            "alt=%.2f "
            "ghd=%.2f "
            "spd=%.2f "
            
            "ang_x=%.2f "
            "ang_y=%.2f "
            "ang_z=%.2f "
            "gyr_x=%.2f "
            "gyr_y=%.2f "
            "gyr_z=%.2f "
            "acc_x=%.2f "
            "acc_y=%.2f "
            "acc_z=%.2f "
            "mag_x=%d "
            "mag_y=%d "
            "mag_z=%d\n",
            
            satioData.local_unixtime_uS,
            gnrmcData.utc_time,
            satioData.padded_rtc_time_HHMMSS,
            satioData.padded_local_time_HHMMSS,
            satioData.padded_rtc_sync_time_HHMMSS,
            systemData.total_loops_a_second,
            systemData.total_gps,
            systemData.total_ins,
            systemData.total_gyro_0,
            systemData.total_mplex_0,
            systemData.total_universe,
            systemData.total_portcontroller_input,
            systemData.total_matrix,
            systemData.total_portcontroller_output,
            systemData.total_display,

            gnggaData.satellite_count,
            satioData.degrees_latitude,
            satioData.degrees_longitude,
            satioData.user_degrees_latitude,
            satioData.user_degrees_longitude,
            satioData.system_degrees_latitude,
            satioData.system_degrees_longitude,
            satioData.altitude,
            satioData.ground_heading,
            satioData.speed,

            gyroData.gyro_0_ang_x,
            gyroData.gyro_0_ang_y,
            gyroData.gyro_0_ang_z,
            gyroData.gyro_0_gyr_x,
            gyroData.gyro_0_gyr_y,
            gyroData.gyro_0_gyr_z,
            gyroData.gyro_0_acc_x,
            gyroData.gyro_0_acc_y,
            gyroData.gyro_0_acc_z,
            gyroData.gyro_0_mag_x,
            gyroData.gyro_0_mag_y,
            gyroData.gyro_0_mag_z
        );
    }
}

/** ----------------------------------------------------------------------------
 * GPS Task.
 * 
 */
void taskGPS(void * pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();

    system_timing();
    // ------------------------------------------------
    // Get, check and set gps data.
    // ------------------------------------------------
    gnggaData.valid_checksum=false;
    gnrmcData.valid_checksum=false;
    gpattData.valid_checksum=false;
    readGPS();
    esp_task_wdt_reset();
    validateGPSData();
    esp_task_wdt_reset();
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
        setSatIOData();
        esp_task_wdt_reset();

        // --------------------------------------------
        // Set INS data. (Can be used without GPS)
        // --------------------------------------------
        set_ins(satioData.system_degrees_latitude,
                satioData.system_degrees_latitude,
                satioData.system_altitude,
                satioData.system_ground_heading,
                satioData.system_speed,
                atof(gnggaData.gps_precision_factor),
                gyroData.gyro_0_ang_z);
        esp_task_wdt_reset();

        // --------------------------------------------
        // Counters.
        // --------------------------------------------
        systemData.i_count_read_gps++;
        systemData.interval_breach_gps = true;
        if (systemData.i_count_read_gps>=UINT32_MAX-2)
          {systemData.i_count_read_gps=0;}
        esp_task_wdt_reset();
    }
    esp_task_wdt_reset();
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
}

/** ----------------------------------------------------------------------------
 * Gyro Task.
 * 
 * @brief Reads and stores gyroscopic data.
 */
void taskGyro(void * pvParameters) {
  esp_task_wdt_add(NULL);
  while (global_task_sync==false) {esp_task_wdt_reset(); vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    if (readGyro()==true) {
      esp_task_wdt_reset();
      systemData.i_count_read_gyro_0++;
      systemData.interval_breach_gyro_0 = true;
      if (systemData.i_count_read_gyro_0>=UINT32_MAX-2)
        {systemData.i_count_read_gyro_0=0;}
      esp_task_wdt_reset();
      // ----------------------------------------------
      // Estimate INS data. (Can be used without GPS)
      // INS data is fed bsck into INS.
      // ----------------------------------------------
      if (systemData.interval_breach_gyro_0==true) {
      if (ins_estimate_position(gyroData.gyro_0_ang_y,
                          gyroData.gyro_0_ang_z,
                          satioData.system_ground_heading,
                          satioData.system_speed,
                          satioData.local_unixtime_uS)==true) {
                          systemData.i_count_read_ins++;
                          systemData.interval_breach_ins=true;
                          if (systemData.i_count_read_ins>=UINT32_MAX-2)
                            {systemData.i_count_read_ins=0;}}
      esp_task_wdt_reset();
      }
    }
    esp_task_wdt_reset();
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
}

/** ----------------------------------------------------------------------------
 * Multiplexer Task.
 * 
 * @brief Reads all analog/digital multiplexer channels.
 */
void taskMultiplexers(void * pvParameters) {
  esp_task_wdt_add(NULL);
  while (global_task_sync==false) {esp_task_wdt_reset(); vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // read muiltiplexer channels (customize as required).
    // ------------------------------------------------
    for (uint8_t i_chan = 0; i_chan < 16; i_chan++) {
      readADMultiplexerAnalogChannel(ad_mux_0, i_chan);
      vTaskDelay(1);
      // esp_task_wdt_reset();
      // printf("[ad] %d: %f\n", i_chan, ad_mux_0.data[i_chan]);
    }
    // ------------------------------------------------
    // Counters
    // ------------------------------------------------
    systemData.i_count_read_mplex_0++;
    systemData.interval_breach_mplex_0 = true;
    if (systemData.i_count_read_mplex_0 >= UINT32_MAX - 2)
    systemData.i_count_read_mplex_0 = 0;
    esp_task_wdt_reset();
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
  esp_task_wdt_add(NULL);
  while (global_task_sync==false) {esp_task_wdt_reset(); vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Calculate.
    // ------------------------------------------------
    if (matrixSwitch()) {
      esp_task_wdt_reset();
      systemData.i_count_matrix++;
      systemData.interval_breach_matrix=true;
      if (systemData.i_count_matrix>=UINT64_MAX-2)
        {systemData.i_count_matrix=0;}
    }
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Mapping.
    // ------------------------------------------------
    map_values();
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Output.
    // ------------------------------------------------
    setOutputValues();
    esp_task_wdt_reset();
    writeOutputPortControllerSetPins(iic_2, I2C_ADDR_OUTPUT_PORTCONTROLLER);
    esp_task_wdt_reset();
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
}

/** ----------------------------------------------------------------------------
 * Universe Task.
 * 
 * @brief Stores various information about the universe!
 */
void taskUniverse(void * pvParameters) {
  esp_task_wdt_add(NULL);
  while (global_task_sync==false) {esp_task_wdt_reset(); vTaskDelay(1);}
  for (;;) {
    esp_task_wdt_reset();
    // ---------------------------------------------------------
    // Track Home Sun, Moon & Planets. (Can be used without GPS)
    // ---------------------------------------------------------
      trackPlanets(satioData.system_degrees_latitude,
                  satioData.system_degrees_longitude,
                  satioData.rtc_year,
                  satioData.rtc_month,
                  satioData.rtc_mday,
                  satioData.rtc_hour,
                  satioData.rtc_minute,
                  satioData.rtc_second,
                  satioData.local_hour,
                  satioData.local_minute,
                  satioData.local_second,
                  satioData.system_altitude
                );
    systemData.i_count_track_planets++;
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Track Meteors.
    // ------------------------------------------------
    setMeteorShowerWarning(satioData.local_month,
                           satioData.local_mday);
    esp_task_wdt_reset();
    systemData.interval_breach_track_planets = true;
    if (systemData.i_count_track_planets>=UINT32_MAX-2)
      {systemData.i_count_track_planets=0;}
    esp_task_wdt_reset();
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