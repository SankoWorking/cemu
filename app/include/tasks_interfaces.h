#ifndef __TASKS_INTERFACES_H__
#define __TASKS_INTERFACES_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"

#include <stddef.h>
#include <stdint.h>

extern volatile uint64_t CurrentSimTimeUs;
uint64_t Get_System_Time_Usec(void);
/*
 *  存放无人机状态的全局结构体，收到心跳包后会将无人机状态信息更新到此处。
 *
 *  Timestamp   ->  接收到消息包的系统时间
 *  SystemID    ->  发送方的系统ID
 *  BaseMode    ->  无人机基本模式
 *  SystemStatus->  无人机健康状况
 *  CustomMode  ->  飞行模式
 */
typedef struct {
    uint64_t Timestamp;
    uint8_t SystemId;
    uint8_t BaseMode;
    uint8_t SystemStatus;
    uint32_t CustomMode;
} UAVStatus_t;
extern UAVStatus_t UAVStatus;

/*
 *  气压计原始数据全局结构体 (替代原 AltitudeData_t)
 *  收到 HIL_SENSOR 消息包后更新。
 *   
 *  Timestamp    ->  接收到消息包的物理仿真时间 (微秒 us)
 *  AbsPressure  ->  绝对气压 (hPa 或 mbar)
 *  PressureAlt  ->  气压计推算出的海拔高度 (m)
 */
typedef struct {
    uint64_t Timestamp;
    float AbsPressure;
    float PressureAlt;
} BaroSensorData_t;

extern BaroSensorData_t CurrentBaroData;

/*
 *  IMU 与磁力计原始数据全局结构体 (替代原 AttitudeData_t)
 *  收到 HIL_SENSOR 消息包后更新。
 *
 *  Timestamp  ->  接收到消息包的物理仿真时间 (微秒 us)
 *  AccelX/Y/Z ->  三轴加速度 (m/s^2)
 *  GyroX/Y/Z  ->  三轴角速度 (rad/s)
 *  MagX/Y/Z   ->  三轴磁力计读数 (Gauss)
 */
typedef struct {
    uint64_t Timestamp;
    
    // 加速度计 (Accelerometer)
    float AccelX;
    float AccelY;
    float AccelZ;
    
    // 陀螺仪 (Gyroscope)
    float GyroX;
    float GyroY;
    float GyroZ;
    
    // 磁力计 (Magnetometer)
    float MagX;
    float MagY;
    float MagZ;
} ImuSensorData_t;

extern ImuSensorData_t CurrentImuData;

/*
 *  配置UART1对应的stream buffer
 */
#define UART1_RX_STREAM_BUFFER_SIZE  512
#define UART1_RX_TRIGGER_LEVEL 1

/*
 *  声明UART1收到数据后用于存放数据的Buffer，传感器数据处理任务会在此Buffer中读取数据。
 */
extern StreamBufferHandle_t Uart1RxStreamBuffer;

/**
 *  @brief 初始化任务间用于通信的所有接口
 */
void Init_Interfaces(void);

/* Sensor任务参数结构体 */
typedef struct {
    StreamBufferHandle_t UARTBuffer;
    TaskHandle_t AttitudeTaskHandle;
} SensorTaskParams_t;

/**
 *  @brief Attitude 向 Command 发送混控后的电机输出
 * 
 *  MotorOutputs[4]  ->  四个电机的输出值 (对应四旋翼 M1-M4),取值范围: 0.0f 
 *                      (停止) 到 1.0f (全速)对应 MAVLink 消息中的 controls[0] 
 *                      到 controls[3]
 *  TimestampUs      ->  微秒级系统时间戳,用于同步仿真环境 (Gazebo) 的物理引擎时间
 *  SystemMode       ->  无人机运行模式,128 代表 MAV_MODE_FLAG_SAFETY_ARMED (已
 *                       解锁)
 */
typedef struct {
    uint64_t TimestampUs;
    float MotorOutputs[4];
    uint8_t SystemMode;
} MotorCommandMsg_t;
extern QueueHandle_t MotorControlQueue;

/**
 *  为了获取微妙级的系统时间而定义的SysTick 硬件寄存器基地址 (Cortex-M 标准) 
 */
#define SCS_BASE            (0xE000E000UL)
#define SysTick_BASE        (SCS_BASE +  0x0010UL)
/* 定义 SysTick 结构体 */
typedef struct {
  volatile uint32_t CTRL;    /* 控制及状态寄存器 (0xe000e010) */
  volatile uint32_t LOAD;    /* 重装载数值寄存器 (0xe000e014) */
  volatile uint32_t VAL;     /* 当前数值寄存器 (0xe000e018) */
  volatile uint32_t CALIB;   /* 校准数值寄存器 (0xe000e01c) */
} SysTick_t;
/* 将 SysTick 指针映射到该地址 */
#define SysTick             ((SysTick_t *) SysTick_BASE)
/* 必要的位定义 */
#define SysTick_CTRL_COUNTFLAG_Msk (1UL << 16)

#endif /* #ifndef __TASKS_INTERFACES_H__ */
