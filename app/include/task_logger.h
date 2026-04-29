#ifndef __TASK_LOGGER_H__
#define __TASK_LOGGER_H__

#include "tasks_config.h"
#include <stdio.h>
#include <string.h>
#include "uart.h" 
#include "queue.h"

#define RAD_TO_DEG (57.2957795f)

void Init_Log(void);
void Init_Log_Task(void);
void Log_Data(ModuleID_t module, float d0, float d1, float d2, float d3);
void Log_Msg(ModuleID_t module, const char* str);
void Log_Raw(ModuleID_t module, const uint8_t* data, uint8_t len);
void Log_UAVStatus(uint32_t Timestamp, uint8_t SystemId, uint8_t BaseMode, uint8_t SystemStatus, uint32_t CustomMode);
void Log_IMU(const float* acc, const float* gyro, uint32_t timestamp);
void Log_Height(const AltitudeData_t* alt_data);
#endif