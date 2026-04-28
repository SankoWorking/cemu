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
                            任务间共享数据结构
================================================================================
*/

//TODO 注释 IMU数据
typedef struct {
    float acc[3];
    float gyro[3];
    uint32_t Timestamp;
} IMUData_t;
extern IMUData_t imu_data;

//TODO 注释
// 消息类型枚举
typedef enum {
    LOG_TYPE_DATA = 0,
    LOG_TYPE_MSG  = 1,
    LOG_TYPE_RAW_HEX = 2
} LogType_t;

//TODO 模块 ID 枚举
typedef enum {
    MOD_SYS = 0,
    MOD_IMU,
    MOD_PID,
    MOD_NAV
} ModuleID_t;

//TODO 注释
typedef struct {
    LogType_t LogType;
    ModuleID_t ModuleID;
    
    union {
        float Data[4];
        char  Msg[16];
        uint8_t Raw[4];
    } payload;
    
    uint32_t Timestamp;
} LogMessage_t;

#endif /* #ifndef __TASKS_CONFIG_H__ */