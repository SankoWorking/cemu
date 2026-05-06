#ifndef __TASK_LOGGER_H__
#define __TASK_LOGGER_H__

#include "tasks_config.h"
#include "uart.h" 
#include "queue.h"
#include <stdarg.h>


#include <stdio.h>
#include <string.h>

/**
 *  定义用于弧度转换为角度的常量
 */
#define RAD_TO_DEG (57.2957795f)

#define MAX_TEXT_LENGTH 32

/**
 *  日志类型枚举。
 *  LOG_TYPE_DATA       ->  浮点数类型的日志，主要用于在调试过程中打印信息。
 *  LOG_TYPE_MSG        ->  文本类型的日志，打印纯文本类型的日志。
 *  LOG_TYPE_RAW_HEX    ->  字节流类型的日志，可用于调试串口时打印原始数据。
 *  LOG_TYPE_UAV_STATUS ->  无人机系统状态日志。
 *  LOG_TYPE_ATT        ->  无人机姿态日志。
 *  LOG_TYPE_ALT        ->  无人机海拔日志。
 */
typedef enum {
    LOG_TYPE_GENERIC = 0,
    LOG_TYPE_UAV_STATUS = 1,
    LOG_TYPE_ATT = 2,
    LOG_TYPE_ALT = 3,
    LOG_TYPE_MOTOR = 4
} LogType_t;

/**
 *  无人机状态日志结构体，会被用于无人机日志结构体的载荷联合体部分，是无人机状态结构体去掉时间戳的版本。
 *
 *  SystemID    ->  发送方的系统ID
 *  BaseMode    ->  无人机基本模式
 *  SystemStatus->  无人机健康状况
 *  CustomMode  ->  飞行模式
 */
typedef struct {
    uint8_t SystemId;
    uint8_t BaseMode;
    uint8_t SystemStatus;
    uint32_t CustomMode;
} LogUAVStatus_t;

/*
 *  无人机海拔日志结构体，会被用于无人机日志结构体的载荷联合体部分，是无人机状态结构体去掉时间戳的版本。
 *
 *  Alttitude   ->  无人机当前海拔
 *  ClimbRate   ->  无人机爬升速率
 */
typedef struct {
    float Alttitude;
    float ClimbRate;
} LogAltitude_t;

/*
 *  无人机姿态日志结构体，会被用于无人机日志结构体的载荷联合体部分，是无人机姿态结构体去掉时间戳、角速度的版本。
 *
 *  Roll        ->  横滚角 (rad, 范围: -pi..+pi)
 *  Pitch       ->  俯仰角 (rad, 范围: -pi/2..+pi/2)
 *  Yaw         ->  航向角 (rad, 范围: -pi..+pi)
 */
typedef struct {
    float Roll;
    float Pitch;
    float Yaw;
} LogAttitude_t;

typedef struct {
    float M[4];
    uint8_t Mode;
} LogMotor_t;

/*
 *  无人机日志消息结构体，内部包含一个联合体，可根据需要填充不同的日志内容。
 *
 *  LogType     ->  日志枚举类型
 *  Timestamp   ->  日志时间戳，真实传感器数据的日志会保持与获取数据的时间同步
 *  Payload     ->  日志载荷联合体。
 */
typedef struct {
    uint64_t Timestamp;
    LogType_t LogType;
    union {
        LogUAVStatus_t UAVStatus;
        LogAttitude_t Att;
        LogAltitude_t Alt;
        LogMotor_t Motor;
        char Text[MAX_TEXT_LENGTH];
    } Payload;
} LogMessage_t;

/**
 * @brief 初始化日志任务。
 */
void Init_Log_Task(void);


/**
 * @brief   格式化打印，功能完全模仿printf。会将传入的参数解析后打包为LogMessage_t类型
 *          然后将其放入LogQueue队列。最大文本长度为MAX_TEXT_LENGTH。
 * @param Format 格式化字符串
 * @param ... 变长参数列表
 */
void Log_Generic(const char* Format, ...);

/*
 *  用于打印无人机状态日志的函数，会将无人机的状态信息塞入日志任务队列。
 *
 *  @param Timestamp 无人机状态更新的时间戳
 *  @param Roll 滚转角
 *  @param Pitch 俯仰角
 *  @param Yaw 航向角
 */
void Log_Attitude(uint64_t Timestamp, float Roll, float Pitch, float Yaw); 
/*
 *  用于打印无人机状态日志的函数，会将无人机的状态信息塞入日志任务队列。
 *
 *  @param Timestamp 无人机状态更新的时间戳
 *  @param SystemId 当前无人机的系统ID
 *  @param BaseMode 无人机基本模式
 *  @param SystemStatus 无人机健康情况
 *  @param CustomMode 无人机飞行模式
 */
void Log_UAVStatus(uint64_t Timestamp, uint8_t SystemId, uint8_t BaseMode, uint8_t SystemStatus, uint32_t CustomMode);

/*
 *  用于打印无人机海拔的函数，会将无人机的海拔信息塞入日志任务队列。
 *
 *  @param Timestamp 无人机海拔更新的时间戳
 *  @param Alttitude 无人机的海拔
 *  @param ClimbRate 无人机的爬升速度
 */
void Log_Altitude(uint64_t Timestamp, float Alttitude, float ClimbRate);

/**
 *  用于打印无人机海拔的函数，会将无人机的海拔信息塞入日志任务队列。
 *
 *  @param Timestamp 无人机海拔更新的时间戳
 *  @param M1 电机的参数
 *  @param M2 电机的参数
 *  @param M3 电机的参数
 *  @param M4 电机的参数
 *  @param  Mode 无人机是否已经解锁
 */
void Log_Motor(uint64_t Timestamp, float M1, float M2, float M3, float M4, uint8_t Mode);


#endif  /* #ifndef __TASK_LOGGER_H__ */