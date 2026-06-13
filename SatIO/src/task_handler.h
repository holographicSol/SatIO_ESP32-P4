/*
    TaskHandler - Written By Benjamin Jack Cullen.
*/

#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <Arduino.h>
#include "i2c_helper.h"
// #include "lcdgfx.h"             // https://github.com/lexus2k/lcdgfx
// #include "lcdgfx_gui.h"         // https://github.com/lexus2k/lcdgfx

// extern DisplaySSD1306_128x64_I2C ssd1306_display_0;
// extern NanoCanvas<128,64,1> canvas_128x64_0;

extern SemaphoreHandle_t i2c_bus0_mutex;

extern TaskHandle_t TaskSerialInfoCMD;
extern TaskHandle_t TaskStorage;
extern TaskHandle_t TaskMultiplexers;
extern TaskHandle_t TaskPortControllerInput;
extern TaskHandle_t TaskGyro;
extern TaskHandle_t TaskGPS;
extern TaskHandle_t TaskUniverse;
extern TaskHandle_t TaskSwitches;
extern TaskHandle_t TaskLogging;

void createTaskDisplay();
void createTaskSerialInfoCMD();
void createTaskStorage();
void createTaskMultiplexers();
void createTaskPortControllerInput();
void createTaskGyro();
void createTaskGPS();
void createTaskUniverse();
void createTaskSwitches();
void createTaskLogging();

void createTaskTaskDisplay();

/** ----------------------------------------------------------------------------
 * Syncronize Tasks.
 * 
 * @brief Time syncronize tasks.
 */
void syncTasks();

/** ----------------------------------------------------------------------------
 * Is Task Delayed.
 * 
 * @brief Returns bool for task delayed.
 */
bool isTaskDelayed(TaskHandle_t taskHandle);

/** ----------------------------------------------------------------------------
 * PowerCfg: Ultimate Performance.
 * 
 * @brief Sets all task delay timings to optimum performance.  
 */
void setTasksDelayUltimatePerformance();

/** ----------------------------------------------------------------------------
 * PowerCfg: Power Saving.
 * 
 * @brief Sets all task delay timings to power saving.  
 */
void setTasksDelayPowerSaving();

/** ----------------------------------------------------------------------------
 * Set Tick.
 * 
 * @brief Manually override use of millisecond or ticks for delays.
 */
void setTick(TaskHandle_t task_handle, bool *tick_delay, bool use_tick);

/** ----------------------------------------------------------------------------
 * Set Delay.
 * 
 * @brief Manually override delay timing.
 */
void setDelay(TaskHandle_t task_handle, long *task_delay, long time_delay);

#endif // TASK_HANDLER_H