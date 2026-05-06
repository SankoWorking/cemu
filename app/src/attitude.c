#include "attitude.h"

// ==========================================================
// 1. 定义 PID 实例 (按照新逻辑：姿态串级 + 高度单环)
// ==========================================================

/* 姿态外环 (角度环)：输入角度偏差 (Rad)，输出目标角速度 (Rad/s)。外环通常只需要 P */
// 注意：新代码这里全设为了0.0f是飞不起来的，我保留了你原来的 P=4.5f
static PID_Controller_t PidRollAngle  = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f};
static PID_Controller_t PidPitchAngle = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f};
static PID_Controller_t PidYawAngle   = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f};

/* 姿态内环 (角速度环)：输入角速度偏差 (Rad/s)，输出电机归一化控制量 */
static PID_Controller_t PidRollRate  = {0.14f, 0.00f, 0.02f, 0.0f, 0.0f, 0.1f, 0.5f};
static PID_Controller_t PidPitchRate = {0.14f, 0.00f, 0.00f, 0.0f, 0.0f, 0.1f, 0.5f};
static PID_Controller_t PidYawRate   = {0.14f, 0.00f, 0.00f, 0.0f, 0.0f, 0.1f, 0.5f};

/* 高度单环：输入高度偏差 (m)，直接输出油门补偿量 (取消了内环) */
// 参数参考了新代码: kp=1.5, ki=0.1, kd=2.0
static PID_Controller_t PidAltPos    = {1.5f, 0.1f, 2.0f, 0.0f, 0.0f, 0.5f, 1.0f}; 

/**
 * @brief 辅助函数：将 Mahony 输出的四元数转换为欧拉角 (弧度 Radian)
 */
static void Quaternion_To_Euler_Rad(float *RollRad, float *PitchRad, float *YawRad) {
    *RollRad  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    
    // 防 NaN 保护
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) sinp = 1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    *PitchRad = asinf(sinp);
    
    *YawRad   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

// ==========================================================
// 4. 姿态控制核心任务
// ==========================================================
static void Attitude_Task(void *PvParameters) {
    MotorCommandMsg_t MotorMsg;

    uint64_t LastTimestamp = 0;
    float Dt = 0.01f; // 新代码默认 Dt 使用了 0.01f (100Hz)
    const float HoverThrottle = 0.3f; // 新代码悬停油门是 0.3f
    
    // 目标状态与当前状态均使用 弧度(Rad) 和 米(m)
    float TargetRollRad = 0.0f, TargetPitchRad = 0.0f, TargetYawRad = 0.0f, TargetAlt = 0.0f;
    float CurrentRollRad = 0.0f, CurrentPitchRad = 0.0f, CurrentYawRad = 0.0f;
    
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // 1. 计算时间片 Dt
        uint64_t CurrentTimestamp = CurrentImuData.Timestamp;
        if (LastTimestamp != 0) {
            Dt = (float)(CurrentTimestamp - LastTimestamp) / 1000000.0f;
            if (Dt <= 0.0f || Dt > 0.1f) Dt = 0.01f; 
        }
        LastTimestamp = CurrentTimestamp;

        // 【防 NaN 保护】如果无数据，注入 Z 轴重力
        if (CurrentImuData.AccelX == 0.0f && CurrentImuData.AccelY == 0.0f && CurrentImuData.AccelZ == 0.0f) {
            CurrentImuData.AccelZ = 1.0f; 
        }

        // 2. 姿态解算
        MahonyAHRSupdate(
            CurrentImuData.GyroX,  CurrentImuData.GyroY,  CurrentImuData.GyroZ,
            -CurrentImuData.AccelX, -CurrentImuData.AccelY, -CurrentImuData.AccelZ, // 新代码加速度计取了负号
            CurrentImuData.MagX,   CurrentImuData.MagY,   CurrentImuData.MagZ
        );
        Quaternion_To_Euler_Rad(&CurrentRollRad, &CurrentPitchRad, &CurrentYawRad);

        static uint16_t BootCounter = 0;
        static float HomeAlt = 0.0f;
        
        if (BootCounter < 300) {
            // 初始化阶段：重置所有积分项
            BootCounter++;
            PidRollAngle.Integral = PidPitchAngle.Integral = PidYawAngle.Integral = 0.0f;
            PidRollRate.Integral  = PidPitchRate.Integral  = PidYawRate.Integral  = 0.0f;
            PidAltPos.Integral = 0.0f; // 单环高度积分清零
            
            memset(MotorMsg.MotorOutputs, 0, sizeof(MotorMsg.MotorOutputs));
            
            TargetRollRad  = CurrentRollRad;
            TargetPitchRad = CurrentPitchRad;
            TargetYawRad   = CurrentYawRad;
            
            HomeAlt = CurrentBaroData.PressureAlt;
            TargetAlt = HomeAlt; 
        } else {
            // 正常控制阶段 (完全复刻新代码的 PID 逻辑)
            
            TargetAlt = HomeAlt + 5.0f; // 仿照新代码起飞 5 米
            
            // 【高度单环控制】输入：高度偏差 -> 输出：基础油门补偿
            float BaseThrust = Pid_Compute(&PidAltPos, TargetAlt, CurrentBaroData.PressureAlt, Dt);
            BaseThrust += HoverThrottle; 
            BaseThrust = CONSTRAIN(BaseThrust, 0.0f, 1.0f); // 新代码对 BaseThrust 进行了限幅

            // 【姿态外环】输入：弧度偏差 -> 输出：目标角速度 (Rad/s)
            float TargetRollRate  = Pid_Compute(&PidRollAngle,  TargetRollRad,  CurrentRollRad,  Dt);
            float TargetPitchRate = Pid_Compute(&PidPitchAngle, TargetPitchRad, CurrentPitchRad, Dt);
            float TargetYawRate   = Pid_Compute(&PidYawAngle,   TargetYawRad,   CurrentYawRad,   Dt);
            
            // 【姿态内环】输入：目标角速度 (Rad/s) 与 陀螺仪实际角速度 (Rad/s) -> 输出：增量
            // 注意：这里默认 CurrentImuData.Gyro 也是 Rad/s 单位
            float OutR = Pid_Compute(&PidRollRate,  TargetRollRate,  CurrentImuData.GyroX, Dt);
            float OutP = Pid_Compute(&PidPitchRate, TargetPitchRate, CurrentImuData.GyroY, Dt);
            float OutY = Pid_Compute(&PidYawRate,   TargetYawRate,   CurrentImuData.GyroZ, Dt);
            
            // 【电机混控】完全按照新代码的矩阵符号匹配
            float M1 = BaseThrust - OutR + OutP + OutY;
            float M2 = BaseThrust + OutR - OutP + OutY;
            float M3 = BaseThrust + OutR + OutP - OutY;
            float M4 = BaseThrust - OutR - OutP - OutY;

            // 基础限幅
            M1 = CONSTRAIN(M1, 0.0f, 1.0f);
            M2 = CONSTRAIN(M2, 0.0f, 1.0f);
            M3 = CONSTRAIN(M3, 0.0f, 1.0f);
            M4 = CONSTRAIN(M4, 0.0f, 1.0f);

            MotorMsg.MotorOutputs[0] = M1;
            MotorMsg.MotorOutputs[1] = M2;
            MotorMsg.MotorOutputs[2] = M3;
            MotorMsg.MotorOutputs[3] = M4;
        }

        // 输出二次限幅与打包发送
        for (int Index = 0; Index < 4; Index++) {
            if (MotorMsg.MotorOutputs[Index] > 1.0f) MotorMsg.MotorOutputs[Index] = 1.0f;
            else if (MotorMsg.MotorOutputs[Index] < 0.0f) MotorMsg.MotorOutputs[Index] = 0.0f;
        }
        
        MotorMsg.TimestampUs = CurrentTimestamp;
        MotorMsg.SystemMode  = UAVStatus.BaseMode;
        
        xQueueSend(MotorControlQueue, &MotorMsg, 0);
    }
}

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