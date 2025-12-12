/*
    TaskHandler - Written By Benjamin Jack Cullen.
*/

#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"

extern TaskHandle_t TaskSerialInfoCMD;
extern TaskHandle_t TaskStorage;
extern TaskHandle_t TaskMultiplexers;
extern TaskHandle_t TaskPortControllerInput;
extern TaskHandle_t TaskGyro;
extern TaskHandle_t TaskGPS;
extern TaskHandle_t TaskUniverse;
extern TaskHandle_t TaskSwitches;

void createTaskSerialInfoCMD();
void createTaskStorage();
void createTaskMultiplexers();
void createTaskPortControllerInput();
void createTaskGyro();
void createTaskGPS();
void createTaskUniverse();
void createTaskSwitches();

void syncTasks();
bool isTaskDelayed(TaskHandle_t taskHandle);
void terminateTask(TaskHandle_t taskHandle, bool safe_abort);

void setTasksDelayUltimatePerformance();
void setTasksDelayPowerSaving();
void setTick(TaskHandle_t task_handle, bool *tick_delay, bool use_tick);
void setDelay(TaskHandle_t task_handle, long *task_delay, long time_delay);

#ifdef __cplusplus
}
#endif

#endif // TASK_HANDLER_H 