#ifndef __PID_H__
#define __PID_H__

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
 *  @brief PID 核心计算函数
 *  @param Pid Pid控制器结构体
 *  @param Target 目标值
 *  
 */
float Pid_Compute(PID_Controller_t *Pid, float Target, float Measure, float Dt);

#endif /* #ifndef __PID_H__ */
