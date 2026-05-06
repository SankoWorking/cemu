#include "attitude.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.29577951f

// ==========================================================
// 1. 定义八个控制通道的 PID 实例 (串级架构)
// ==========================================================

/* 姿态外环 (角度环)：输入角度偏差 (rad)，输出目标角速度 (rad/s)。外环通常只需要 P */
static PID_Controller_t PidRollAngle  = {4.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f};
static PID_Controller_t PidPitchAngle = {4.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f};
static PID_Controller_t PidYawAngle   = {4.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f};

/* 姿态内环 (角速度环)：输入角速度偏差 (rad/s)，输出电机归一化控制量。内环需要 PID 全量 */
static PID_Controller_t PidRollRate  = {0.15f, 0.05f, 0.005f, 0.0f, 0.0f, 0.5f, 1.0f};
static PID_Controller_t PidPitchRate = {0.15f, 0.05f, 0.005f, 0.0f, 0.0f, 0.5f, 1.0f};
static PID_Controller_t PidYawRate   = {0.20f, 0.05f, 0.000f, 0.0f, 0.0f, 0.5f, 1.0f};

/* 高度外环 (位置环)：输入高度偏差 (m)，输出目标垂直爬升率 (m/s) */
static PID_Controller_t PidAltPos    = {1.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f};

/* 高度内环 (速度环)：输入爬升率偏差 (m/s)，输出油门补偿量 */
static PID_Controller_t PidAltRate   = {0.2f, 0.1f, 0.01f, 0.0f, 0.0f, 0.5f, 1.0f};

/**
 * @brief 辅助函数：将 Mahony 输出的四元数转换为欧拉角 
 */
static void Quaternion_To_Euler(float *Roll, float *Pitch, float *Yaw) {
    *Roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    *Pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    *Yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

// ==========================================================
// 4. 姿态解算与串级控制核心任务
// ==========================================================
static void Attitude_Task(void *PvParameters) {
    MotorCommandMsg_t MotorMsg;

    uint64_t LastTimestamp = 0;
    float Dt = 0.005f;
    const float HoverThrottle = 0.4f; 
    
    float TargetRoll = 0.0f, TargetPitch = 0.0f, TargetYaw = 0.0f, TargetAlt = 0.0f;
    float CurrentRollRad = 0.0f, CurrentPitchRad = 0.0f, CurrentYawRad = 0.0f;
    
    // 爬升率相关变量
    float CurrentClimbRate = 0.0f;
    float LastAlt = 0.0f;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        //计算时间片
        uint64_t CurrentTimestamp = CurrentImuData.Timestamp;
        if (LastTimestamp != 0) {
            Dt = (float)(CurrentTimestamp - LastTimestamp) / 1000000.0f;
            if (Dt <= 0.0f || Dt > 0.1f) Dt = 0.005f; 
        }
        LastTimestamp = CurrentTimestamp;

        //姿态解算
        MahonyAHRSupdate(
            CurrentImuData.GyroX,  CurrentImuData.GyroY,  CurrentImuData.GyroZ,
            CurrentImuData.AccelX, CurrentImuData.AccelY, CurrentImuData.AccelZ,
            CurrentImuData.MagX,   CurrentImuData.MagY,   CurrentImuData.MagZ
        );
        Quaternion_To_Euler(&CurrentRollRad, &CurrentPitchRad, &CurrentYawRad);

        //计算垂直爬升率
        float RawClimbRate = (CurrentBaroData.PressureAlt - LastAlt) / Dt;
        CurrentClimbRate = CurrentClimbRate * 0.8f + RawClimbRate * 0.2f; 
        LastAlt = CurrentBaroData.PressureAlt;

        static uint16_t BootCounter = 0;
        static float HomeAlt = 0.0f;
        
        if (BootCounter < 500) {
            // 初始化阶段：重置所有积分项
            BootCounter++;
            PidRollAngle.Integral = PidPitchAngle.Integral = PidYawAngle.Integral = PidAltPos.Integral = 0.0f;
            PidRollRate.Integral  = PidPitchRate.Integral  = PidYawRate.Integral  = PidAltRate.Integral  = 0.0f;
            memset(MotorMsg.MotorOutputs, 0, sizeof(MotorMsg.MotorOutputs));
            
            TargetRoll  = CurrentRollRad;
            TargetPitch = CurrentPitchRad;
            TargetYaw   = CurrentYawRad;
            
            HomeAlt = CurrentBaroData.PressureAlt;
            TargetAlt = HomeAlt; 
            CurrentClimbRate = 0.0f;
        } else {
            // 正常飞行控制阶段
            TargetAlt = HomeAlt + 2.0f; 
            
            //【外环】输入：位置/角度偏差 -> 输出：目标速度/角速度
            float TargetRollRate  = Pid_Compute(&PidRollAngle,  TargetRoll,  CurrentRollRad, Dt);
            float TargetPitchRate = Pid_Compute(&PidPitchAngle, TargetPitch, CurrentPitchRad, Dt);
            float TargetYawRate   = Pid_Compute(&PidYawAngle,   TargetYaw,   CurrentYawRad, Dt);
            float TargetClimbRate = Pid_Compute(&PidAltPos,     TargetAlt,   CurrentBaroData.PressureAlt, Dt);

            //【内环】输入：速度/角速度偏差 -> 输出：电机归一化控制增量
            float OutR = Pid_Compute(&PidRollRate,  TargetRollRate,  CurrentImuData.GyroX, Dt);
            float OutP = Pid_Compute(&PidPitchRate, TargetPitchRate, CurrentImuData.GyroY, Dt);
            float OutY = Pid_Compute(&PidYawRate,   TargetYawRate,   CurrentImuData.GyroZ, Dt);
            float OutT = Pid_Compute(&PidAltRate,   TargetClimbRate, CurrentClimbRate,     Dt) + HoverThrottle;

            // 假设 0:右前, 1:左后, 2:左前, 3:右后
            float M1 = OutT - OutR + OutP - OutY; // M1
            float M2 = OutT + OutR - OutP - OutY; // M2
            float M3 = OutT + OutR + OutP + OutY; // M3
            float M4 = OutT - OutR - OutP + OutY; // M4

            // 3. 限幅保护 (确保电机指令在 0.0 ~ 1.0 之间)
            M1 = CONSTRAIN(M1, 0.0f, 1.0f);
            M2 = CONSTRAIN(M2, 0.0f, 1.0f);
            M3 = CONSTRAIN(M3, 0.0f, 1.0f);
            M4 = CONSTRAIN(M4, 0.0f, 1.0f);

            MotorMsg.MotorOutputs[0] = M1;
            MotorMsg.MotorOutputs[1] = M2;
            MotorMsg.MotorOutputs[2] = M3;
            MotorMsg.MotorOutputs[3] = M4;

        }

        //输出限幅与打包发送
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

// ==========================================================
// 5. 姿态任务初始化
// ==========================================================
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