#ifndef __TASK_SENSORS_H__
#define __TASK_SENSORS_H__

#include "tasks_config.h"
#include "queue.h"
#include <common/mavlink.h>
#include "task_logger.h"
#include "stream_buffer.h"

void Set_Target_Task_Handle(TaskHandle_t taskHandle);
void Init_SensorData_Task(void *Parameters);

#endif /*#ifndef __TASK_SENSORS_H__*/