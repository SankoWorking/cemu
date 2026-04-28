#include "task_logger.h"

static QueueHandle_t LogQueue = NULL;

//TODO 注释 这里先初始化日志队列，确保其他模块初始化时日志队列已经存在
void Init_Log(void) {
    if (LogQueue == NULL) {
        LogQueue = xQueueCreate(50, sizeof(LogMessage_t));
    }
}

void Logging_Task(void *pvParameters) {
    LogMessage_t Log;
    char Buffer[128]; // 扩容：防止 IMU 6浮点数+时间戳 溢出
    const char* ModuleNames[] = {"SYS", "IMU", "PID", "NAV"};

    for (;;) {
        if (xQueueReceive(LogQueue, &Log, portMAX_DELAY) == pdPASS) {
            int len = 0;
            
            switch (Log.LogType) {
                case LOG_TYPE_DATA:
                    len = snprintf(Buffer, sizeof(Buffer), 
                                   "[%lu] [%s] DATA: %.3f, %.3f, %.3f, %.3f\r\n",
                                   Log.Timestamp, ModuleNames[Log.ModuleID],
                                   Log.payload.Data[0], Log.payload.Data[1],
                                   Log.payload.Data[2], Log.payload.Data[3]);
                    break;

                case LOG_TYPE_IMU:
                    len = snprintf(Buffer, sizeof(Buffer), 
                                   "[%lu] [IMU] A:%.2f,%.2f,%.2f G:%.3f,%.3f,%.3f\r\n",
                                   Log.Timestamp,
                                   Log.payload.IMU.acc[0], Log.payload.IMU.acc[1], Log.payload.IMU.acc[2],
                                   Log.payload.IMU.gyro[0], Log.payload.IMU.gyro[1], Log.payload.IMU.gyro[2]);
                    break;

                case LOG_TYPE_MSG:
                    len = snprintf(Buffer, sizeof(Buffer), 
                                   "[%lu] [%s] MSG: %s\r\n",
                                   Log.Timestamp, ModuleNames[Log.ModuleID], Log.payload.Msg);
                    break;

                case LOG_TYPE_HEARTBEAT:
                    len = snprintf(Buffer, sizeof(Buffer), 
                                   "[%lu] [MAV] HB: T:%d A:%d S:%d M:%d\r\n",
                                   Log.Timestamp,
                                   Log.payload.Heartbeat.type, Log.payload.Heartbeat.autopilot,
                                   Log.payload.Heartbeat.status, Log.payload.Heartbeat.mode);
                    break;

                case LOG_TYPE_RAW_HEX:
                    len = snprintf(Buffer, sizeof(Buffer), 
                                   "[%lu] [%s] RAW: %02X %02X %02X %02X...\r\n",
                                   Log.Timestamp, ModuleNames[Log.ModuleID],
                                   Log.payload.Raw[0], Log.payload.Raw[1],
                                   Log.payload.Raw[2], Log.payload.Raw[3]);
                    break;

                case LOG_TYPE_ATT:
                    // 打印格式：[时间戳] [ATT] R:横滚 P:俯仰 Y:航向 (单位: 度)
                    len = snprintf(Buffer, sizeof(Buffer), 
                                "[%lu] [ATT] R:%.1f, P:%.1f, Y:%.1f | vR:%.2f, vP:%.2f\r\n",
                                Log.Timestamp,
                                Log.payload.Att.roll,
                                Log.payload.Att.pitch,
                                Log.payload.Att.yaw,
                                Log.payload.Att.rollspeed,
                                Log.payload.Att.pitchspeed);
                    break;
                
                default: break;
            }

            if (len > 0) {
                Puts_UART(Buffer);
            }
        }
    }
}

//TODO 注释 这里启动日志任务
void Init_Log_Task(void) {
    xTaskCreate(Logging_Task, 
                "LogTask", 
                STACK_LOGGING, 
                NULL, 
                PRIO_LOGGING_TASK, 
                NULL);
}

void Log_IMU(const float* acc, const float* gyro, uint32_t timestamp) {
    LogMessage_t msg;
    msg.LogType = LOG_TYPE_IMU;
    msg.ModuleID = MOD_IMU;
    msg.Timestamp = timestamp;
    
    // 拷贝 3轴加速度和 3轴陀螺仪
    memcpy(msg.payload.IMU.acc, acc, sizeof(float) * 3);
    memcpy(msg.payload.IMU.gyro, gyro, sizeof(float) * 3);
    
    // 必须 0 等待，防止卡住算法任务
    xQueueSend(LogQueue, &msg, 0);
}

//TODO 注释 发送数值日志
void Log_Data(ModuleID_t module, float d0, float d1, float d2, float d3) {
    LogMessage_t msg;
    msg.LogType = LOG_TYPE_DATA;
    msg.ModuleID = module;
    msg.Timestamp = xTaskGetTickCount();
    msg.payload.Data[0] = d0;
    msg.payload.Data[1] = d1;
    msg.payload.Data[2] = d2;
    msg.payload.Data[3] = d3;
    
    xQueueSend(LogQueue, &msg, 0);
}

//TODO 注释 发送文本日志
void Log_Msg(ModuleID_t module, const char* str) {
    LogMessage_t msg;
    msg.LogType = LOG_TYPE_MSG;
    msg.ModuleID = module;
    msg.Timestamp = xTaskGetTickCount();
    
    strncpy(msg.payload.Msg, str, 15);
    msg.payload.Msg[15] = '\0';
    
    xQueueSend(LogQueue, &msg, 0);
}

void Log_Raw(ModuleID_t module, const uint8_t* data, uint8_t len) {
    LogMessage_t msg;
    msg.LogType = LOG_TYPE_RAW_HEX;
    msg.ModuleID = module;
    msg.Timestamp = xTaskGetTickCount();
   
    if (len > 16) len = 16;
    
    memcpy(msg.payload.Raw, data, len);
    
    xQueueSend(LogQueue, &msg, 0);
}

void Log_Heartbeat(uint8_t type, uint8_t autopilot, uint8_t status, uint8_t mode, uint32_t timestamp) {
    LogMessage_t msg;
    msg.LogType = LOG_TYPE_HEARTBEAT;
    msg.ModuleID = MOD_SYS;
    msg.Timestamp = timestamp;
    msg.payload.Heartbeat.type = type;
    msg.payload.Heartbeat.autopilot = autopilot;
    msg.payload.Heartbeat.status = status;
    msg.payload.Heartbeat.mode = mode;
    
    xQueueSend(LogQueue, &msg, 0);
}

void Log_Attitude(const AttitudeData_t* att) {
    LogMessage_t msg;
    msg.LogType = LOG_TYPE_ATT; // 需在枚举中定义 LOG_TYPE_ATT
    msg.ModuleID = MOD_SYS;     // 或者你定义的 MOD_NAV
    msg.Timestamp = att->Timestamp;

    // 转换为角度存储，方便串口直接查看数值
    msg.payload.Att.roll  = att->roll  * RAD_TO_DEG;
    msg.payload.Att.pitch = att->pitch * RAD_TO_DEG;
    msg.payload.Att.yaw   = att->yaw   * RAD_TO_DEG;
    
    // 角速度通常保留弧度/秒即可
    msg.payload.Att.rollspeed  = att->rollspeed;
    msg.payload.Att.pitchspeed = att->pitchspeed;
    msg.payload.Att.yawspeed   = att->yawspeed;

    xQueueSend(LogQueue, &msg, 0);
}