#include "attitude.h"
#include <math.h>

// 必须引入 Mahony AHRS 算法头文件
#include "MahonyAHRS.h" 

#define RAD_TO_DEG 57.29577951f

/* 定义四个控制通道的 PID 实例 (纯大驼峰) */
static PID_Controller_t PidRoll  = {1.5f, 0.01f, 0.2f, 0.0f, 0.0f, 0.5f, 1.0f};
static PID_Controller_t PidPitch = {1.5f, 0.01f, 0.2f, 0.0f, 0.0f, 0.5f, 1.0f};
static PID_Controller_t PidYaw   = {2.0f, 0.05f, 0.1f, 0.0f, 0.0f, 0.8f, 1.0f};
static PID_Controller_t PidAlt   = {1.2f, 0.10f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

// ==========================================================
// 辅助函数：将 Mahony 输出的四元数转换为欧拉角 (函数名大驼峰+下划线)
// ==========================================================
static void Quaternion_To_Euler(float *Roll, float *Pitch, float *Yaw) {
    *Roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    *Pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    *Yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

/**
 * @brief PID 核心计算函数 (函数名大驼峰+下划线)
 */
static float Pid_Compute(PID_Controller_t *Pid, float Target, float Measure, float Dt) {
    float Error = Target - Measure;
    float POut = Pid->Kp * Error;

    Pid->Integral += Error * Dt;
    if (Pid->Integral > Pid->IntegralLimit) Pid->Integral = Pid->IntegralLimit;
    else if (Pid->Integral < -Pid->IntegralLimit) Pid->Integral = -Pid->IntegralLimit;
    float IOut = Pid->Ki * Pid->Integral;

    float DOut = Pid->Kd * (Error - Pid->LastError) / Dt;
    Pid->LastError = Error;

    float Output = POut + IOut + DOut;
    if (Output > Pid->OutputLimit) Output = Pid->OutputLimit;
    else if (Output < -Pid->OutputLimit) Output = -Pid->OutputLimit;

    return Output;
}

/**
 * @brief 姿态解算与控制任务 (函数名大驼峰+下划线)
 */
static void Attitude_Task(void *PvParameters) {
    MotorCommandMsg_t MotorMsg;

    uint64_t LastTimestamp = 0;
    float Dt = 0.005f; 
    const float HoverThrottle = 0.4f; 
    
    // 目标姿态与高度变量
    float TargetRoll = 0.0f, TargetPitch = 0.0f, TargetYaw = 0.0f, TargetAlt = 0.0f;
    
    // 当前解算出的欧拉角变量
    float CurrentRollRad = 0.0f, CurrentPitchRad = 0.0f, CurrentYawRad = 0.0f;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // --- 1. 获取时间片 Dt ---
        uint64_t CurrentTimestamp = CurrentImuData.Timestamp;
        if (LastTimestamp != 0) {
            Dt = (float)(CurrentTimestamp - LastTimestamp) / 1000000.0f;
            if (Dt <= 0.0f || Dt > 0.1f) Dt = 0.005f; 
        }
        LastTimestamp = CurrentTimestamp;

        // --- 2. 核心！执行姿态解算算法 ---
        MahonyAHRSupdate(
            CurrentImuData.GyroX,  CurrentImuData.GyroY,  CurrentImuData.GyroZ,
            CurrentImuData.AccelX, CurrentImuData.AccelY, CurrentImuData.AccelZ,
            CurrentImuData.MagX,   CurrentImuData.MagY,   CurrentImuData.MagZ
        );
        
        Quaternion_To_Euler(&CurrentRollRad, &CurrentPitchRad, &CurrentYawRad);

        // --- 3. 状态机与 PID 计算 ---
        static uint16_t BootCounter = 0;
        static float HomeAlt = 0.0f; // 💡 新增：用于锁死地面的初始高度
        
        // 倒计时器：等待约 500 个循环，让传感器稳定后再起飞
        if (BootCounter < 500) {
            BootCounter++;
            
            // 未解锁状态：重置积分器，更新目标值为当前实际姿态
            PidRoll.Integral = PidPitch.Integral = PidYaw.Integral = PidAlt.Integral = 0.0f;
            memset(MotorMsg.MotorOutputs, 0, sizeof(MotorMsg.MotorOutputs));
            
            TargetRoll  = CurrentRollRad;
            TargetPitch = CurrentPitchRad;
            TargetYaw   = CurrentYawRad;
            
            HomeAlt = CurrentBaroData.PressureAlt; // 💡 持续刷新并记录起飞前的地面高度
            TargetAlt = HomeAlt; 
        } else {
            // 已解锁状态：目标高度设为被锁死的“地面高度 + 1.0米”
            TargetAlt = HomeAlt + 1.0f; // 💡 目标高度现在固定了，不会再乱跑！
            
            float OutR = Pid_Compute(&PidRoll,  TargetRoll,  CurrentRollRad, Dt);
            float OutP = Pid_Compute(&PidPitch, TargetPitch, CurrentPitchRad, Dt);
            float OutY = Pid_Compute(&PidYaw,   TargetYaw,   CurrentYawRad, Dt);
            float OutT = Pid_Compute(&PidAlt,   TargetAlt,   CurrentBaroData.PressureAlt, Dt) + HoverThrottle;

            MotorMsg.MotorOutputs[0] = OutR; 
            MotorMsg.MotorOutputs[1] = OutP; 
            MotorMsg.MotorOutputs[2] = OutY; 
            MotorMsg.MotorOutputs[3] = OutT; 
        }

        // --- 4. 限幅与发送 ---
        for (int Index = 0; Index < 3; Index++) {
            if (MotorMsg.MotorOutputs[Index] > 1.0f) MotorMsg.MotorOutputs[Index] = 1.0f;
            else if (MotorMsg.MotorOutputs[Index] < -1.0f) MotorMsg.MotorOutputs[Index] = -1.0f;
        }
        
        if (MotorMsg.MotorOutputs[3] > 1.0f) MotorMsg.MotorOutputs[3] = 1.0f;
        else if (MotorMsg.MotorOutputs[3] < 0.0f) MotorMsg.MotorOutputs[3] = 0.0f;

        MotorMsg.TimestampUs = CurrentTimestamp;
        MotorMsg.SystemMode  = UAVStatus.BaseMode;
        
        xQueueSend(MotorControlQueue, &MotorMsg, 0);
    }
}

/**
 * @brief 姿态任务初始化 (函数名大驼峰+下划线)
 */
TaskHandle_t Init_Attitude_Task(void) {
    TaskHandle_t Handle = NULL;

    BaseType_t Result = xTaskCreate(Attitude_Task, 
                "AttitudeTask", 
                STACK_ATTITUDE_CTRL, 
                NULL, 
                PRIO_ATTITUDE_CTRL_TASK, 
                &Handle);
                
    if (Result != pdPASS) {
        Puts_UART0("\r\n[FATAL ERROR] Attitude Task Failed to Create!\r\n");
    }
    return Handle;
}