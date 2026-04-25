#ifndef __TASK_ATTITUDE_H__
#define __TASK_ATTITUDE_H__

#include "tasks_config.h"
#include "uart.h"

extern IMUData_t imu_data;

void Init_Attitude_Task(void);
TaskHandle_t Get_Attitude_Task_Handle(void);

#endif /* #ifndef __TASK_ATTITUDE_H__ */