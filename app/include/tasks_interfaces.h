#ifndef __TASKS_INTERFACES_H__
#define __TASKS_INTERFACES_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"

#include <stddef.h>
#include <stdint.h>

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
    uint32_t Timestamp;
    uint8_t SystemId;
    uint8_t BaseMode;
    uint8_t SystemStatus;
    uint32_t CustomMode;
} UAVStatus_t;
extern UAVStatus_t UAVStatus;

/*
 *  无人机海拔结全局构体，收到海拔消息包会将最新的海拔信息更新到此处。
 *   
 *  Timestamp   ->  接收到消息包的系统时间
 *  Alttitude   ->  无人机当前海拔
 *  ClimbRate   ->  无人机爬升速率
 */
typedef struct {
    float Alttitude;
    float ClimbRate;
    uint32_t Timestamp;
} AltitudeData_t;
extern AltitudeData_t CurrentAltitude;

/*
 *  无人机姿态全局结构体，收到姿态数据包后会更新此全局结构体。
 *
 *  Timestamp   ->  接收到消息包的系统时间
 *  Roll        ->  横滚角 (rad, 范围: -pi..+pi)
 *  Pitch       ->  俯仰角 (rad, 范围: -pi/2..+pi/2)
 *  Yaw         ->  航向角 (rad, 范围: -pi..+pi)
 *  RollSpeed   ->  横滚角速度 (rad/s)
 *  PitchSpeed  ->  俯仰角速度 (rad/s)
 *  YawSpeed    ->  航向角速度 (rad/s)
 */
typedef struct {
    float Roll;
    float Pitch;
    float Yaw;
    float RollSpeed;
    float PitchSpeed;
    float YawSpeed;
    uint32_t Timestamp;
} AttitudeData_t;
extern AttitudeData_t CurrentAttitude;

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
extern QueueHandle_t xMotorControlQueue;

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
