#ifndef __TASK_SENSORS_H__
#define __TASK_SENSORS_H__

#include "tasks_config.h"
#include "tasks_interfaces.h"

#include <common/mavlink.h>
#include "logger.h"

void Set_Target_Task_Handle(TaskHandle_t taskHandle);
void Init_SensorData_Task(void *Parameters);

#endif /*#ifndef __TASK_SENSORS_H__*/