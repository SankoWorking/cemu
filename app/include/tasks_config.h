#ifndef __TASKS_CONFIG_H__
#define __TASKS_CONFIG_H__

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/*
================================================================================
                            任务优先级定义 (Numerical Higher = More Urgent)
================================================================================
*/

#define PRIO_SAFETY_MONITOR_TASK    ( configMAX_PRIORITIES - 1 )  // 最高：安全监控
#define PRIO_ATTITUDE_CTRL_TASK     ( configMAX_PRIORITIES - 2 )  // 极高：姿态控制/PID
#define PRIO_SENSOR_DATA_TASK       ( configMAX_PRIORITIES - 3 )  // 高：传感器采集与融合
#define PRIO_COMMAND_PARSE_TASK     ( tskIDLE_PRIORITY + 3 )      // 中：指令解析 (遥控/地面站)
#define PRIO_LOGGING_TASK           ( tskIDLE_PRIORITY + 2 )      // 低：日志记录
#define PRIO_STATUS_LED_TASK        ( tskIDLE_PRIORITY + 1 )      // 最低：状态指示灯/心跳

/*
================================================================================
                            任务堆栈大小定义 (单位: Word)
================================================================================
*/
// 注意：在 FreeRTOS 中，堆栈单位通常是 Word (4 Bytes)，需根据算法复杂度调整
#define STACK_SAFETY_MONITOR        ( 128 )
#define STACK_ATTITUDE_CTRL         ( 512 )  // 浮点运算和矩阵运算较多，建议给大一点
#define STACK_SENSOR_DATA           ( 256 )
#define STACK_COMMAND_PARSE         ( 256 )
#define STACK_LOGGING               ( 512 )  // 文件系统底层调用可能占用较多堆栈
#define STACK_STATUS_LED            ( 64  )

/*
================================================================================
                            任务运行频率/周期定义 (单位: ms)
================================================================================
*/
#define PERIOD_SAFETY_MONITOR_MS    pdMS_TO_TICKS(10)   // 100Hz
#define PERIOD_ATTITUDE_CTRL_MS     pdMS_TO_TICKS(2)    // 500Hz (飞控核心频率)
#define PERIOD_SENSOR_DATA_MS       pdMS_TO_TICKS(1)    // 1000Hz (高频采集)
#define PERIOD_COMMAND_PARSE_MS     pdMS_TO_TICKS(20)   // 50Hz
#define PERIOD_LOGGING_MS           pdMS_TO_TICKS(100)  // 10Hz
#define PERIOD_STATUS_LED_MS        pdMS_TO_TICKS(500)  // 2Hz

/*
================================================================================
                            任务全局数据结构
================================================================================
*/

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
extern AttitudeData_t CurrtentAttitude;

#endif /* #ifndef __TASKS_CONFIG_H__ */