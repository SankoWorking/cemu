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

/*
 *  初始化任务间用于通信的所有接口
 */
void Init_Interfaces(void);
#endif /* #ifndef __TASKS_INTERFACES_H__ */