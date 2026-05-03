#ifndef __TASK_SENSORS_H__
#define __TASK_SENSORS_H__

#include "tasks_config.h"
#include "tasks_interfaces.h"
#include <common/mavlink.h>
#include "logger.h"

/**
 *  @brief  初始化Sensor 任务
 *  @param  Params 包含UART1 StreamBuffer和Attitude任务的句柄
 */
void Init_Sensor_Task(SensorTaskParams_t *Params);

#endif /*#ifndef __TASK_SENSORS_H__*/