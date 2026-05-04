#ifndef __TASK_ATTITUDE_H__
#define __TASK_ATTITUDE_H__

#include "tasks_config.h"
#include "task.h"
#include "tasks_interfaces.h"
#include "logger.h"
/**
 * @brief PID 控制器结构体
 */
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float Integral;         // 积分累加值
    float LastError;        // 上次误差（用于计算微分）
    float IntegralLimit;    // 积分限幅（防止积分饱和）
    float OutputLimit;      // 输出限幅
} PID_Controller_t;

/**
 *  @brief  初始化Attitude任务
 *  @return 此任务的句柄，Sensor任务初始化时会用到，用于实现两个任务间的任务通知功能。
 */
TaskHandle_t Init_Attitude_Task(void);

#endif /* #ifndef __TASK_ATTITUDE_H__ */