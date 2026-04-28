#ifndef __TASK_LOGGER_H__
#define __TASK_LOGGER_H__

#include "tasks_config.h"
#include <stdio.h>
#include <string.h>
#include "uart.h" 
#include "queue.h"

void Init_Log(void);
void Logging_Task(void *pvParameters);
void Init_Log_Task(void);
void Log_Data(ModuleID_t module, float d0, float d1, float d2, float d3);
void Log_Msg(ModuleID_t module, const char* str);
void Log_Raw(ModuleID_t module, const uint8_t* data, uint8_t len);
void Log_Heartbeat(uint8_t type, uint8_t autopilot, uint8_t status, uint8_t mode, uint32_t timestamp);
#endif