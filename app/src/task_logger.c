#include "task_logger.h"

static QueueHandle_t LogQueue = NULL;

//TODO 注释 这里先初始化日志队列，确保其他模块初始化时日志队列已经存在
void Init_Log(void) {
    if (LogQueue == NULL) {
        LogQueue = xQueueCreate(20, sizeof(LogMessage_t));
    }
}

//TODO 注释 日志任务
void Logging_Task(void *pvParameters) {
    LogMessage_t Log;
    char Buffer[80]; // 用于存放最终格式化好的完整字符串
    const char* ModuleNames[] = {"SYS", "IMU", "PID", "NAV"};

    for (;;) {
        if (xQueueReceive(LogQueue, &Log, portMAX_DELAY) == pdPASS) {
            
            int len = 0;
            if (Log.LogType == LOG_TYPE_DATA) {
                len = snprintf(Buffer, sizeof(Buffer), 
                               "[%lu] [%s] DATA: %.3f, %.3f, %.3f, %.3f\r\n",
                               Log.Timestamp,
                               ModuleNames[Log.ModuleID],
                               Log.payload.Data[0], Log.payload.Data[1],
                               Log.payload.Data[2], Log.payload.Data[3]);
            } 
            else if (Log.LogType == LOG_TYPE_MSG) {
                len = snprintf(Buffer, sizeof(Buffer), 
                               "[%lu] [%s] MSG: %s\r\n",
                               Log.Timestamp,
                               ModuleNames[Log.ModuleID],
                               Log.payload.Msg);
            }else if (Log.LogType == LOG_TYPE_RAW_HEX) {
                len = snprintf(Buffer, sizeof(Buffer), 
                                "[%lu] [%s] RAW: 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                                Log.Timestamp,
                                ModuleNames[Log.ModuleID],
                                Log.payload.Raw[0], Log.payload.Raw[1],
                                Log.payload.Raw[2], Log.payload.Raw[3]);
            }else if (Log.LogType == LOG_TYPE_HEARTBEAT) {
                    len = snprintf(Buffer, sizeof(Buffer), 
                   "[%lu] [MAV] HB: T:%d A:%d S:%d M:%d\r\n",
                   Log.Timestamp,
                   Log.payload.Heartbeat.type,
                   Log.payload.Heartbeat.autopilot,
                   Log.payload.Heartbeat.status,
                   Log.payload.Heartbeat.mode);
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