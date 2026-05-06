#ifndef __TASK_ATTITUDE_H__
#define __TASK_ATTITUDE_H__

#include "tasks_config.h"
#include "task.h"
#include "tasks_interfaces.h"
#include "logger.h"
#include "MahonyAHRS.h" 
#include <math.h>
#include "pid.h"

/**
 *  @brief  初始化Attitude任务
 *  @return 此任务的句柄，Sensor任务初始化时会用到，用于实现两个任务间的任务通知功能。
 */
TaskHandle_t Init_Attitude_Task(void);

/* 定义电机限幅 */
#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)));

#endif /* #ifndef __TASK_ATTITUDE_H__ */