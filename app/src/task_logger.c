#include "task_logger.h"

static QueueHandle_t LogQueue = NULL;

void Init_Log_Queue(void) {
    if (LogQueue == NULL) {
        LogQueue = xQueueCreate(50, sizeof(LogMessage_t));
    }
}

/*
 *  格式化数据日志的内联函数，会被用于日志任务中，目的是增强代码易读性。
 *  
 *  @param Buffer 存放格式化后日志的缓冲区
 *  @param Size 缓冲区的大小
 *  @param Log  待格式化的日志消息
 *  @return 格式化后的字符串长度
 */
static inline int Format_Data_Log(char *Buffer, size_t Size, const LogMessage_t *Log) {
    return snprintf(Buffer, Size, "%.3f", Log->Payload.Data);
}

/*
 *  格式化纯文本日志的内联函数，会被用于日志任务中，目的是增强代码易读性。
 *  
 *  @param Buffer 存放格式化后日志的缓冲区
 *  @param Size 缓冲区的大小
 *  @param Log  待格式化的日志消息
 *  @return 格式化后的字符串长度
 */
static inline int Format_MSG_Log(char *Buffer, size_t Size, const LogMessage_t *Log) {
    return snprintf(Buffer, Size, "[%lu] [MSG]: %s\r\n",
                    Log->Timestamp, 
                    Log->Payload.Msg);
}

/*
 *  格式化无人机状态日志的内联函数，会被用于日志任务中，目的是增强代码易读性。
 *  
 *  @param Buffer 存放格式化后日志的缓冲区
 *  @param Size 缓冲区的大小
 *  @param Log  待格式化的日志消息
 *  @return 格式化后的字符串长度
 */
static inline int Format_UAV_Status_Log(char *Buffer, size_t Size, const LogMessage_t *Log) {
    return snprintf(Buffer, Size, "[%u] [UAV] ST: ID:%d B:%d S:%d C:%u\r\n",
                    (unsigned int)Log->Timestamp,
                    Log->Payload.UAVStatus.SystemId, 
                    Log->Payload.UAVStatus.BaseMode,
                    Log->Payload.UAVStatus.SystemStatus, 
                    Log->Payload.UAVStatus.CustomMode);
}

/*
 *  格式化字节流日志的内联函数，会被用于日志任务中，目的是增强代码易读性。
 *  
 *  @param Buffer 存放格式化后日志的缓冲区
 *  @param Size 缓冲区的大小
 *  @param Log  待格式化的日志消息
 *  @return 格式化后的字符串长度
 */
static inline int Format_Raw_Hex_Log(char *Buffer, size_t Size, const LogMessage_t *Log) {
    return snprintf(Buffer, Size, "[%lu] [RAW]: %02X %02X %02X %02X...\r\n",
                    Log->Timestamp,
                    Log->Payload.Raw[0], 
                    Log->Payload.Raw[1],
                    Log->Payload.Raw[2], 
                    Log->Payload.Raw[3]);
}

/*
 *  格式化无人机姿态日志的内联函数，会被用于日志任务中，目的是增强代码易读性。
 *  
 *  @param Buffer 存放格式化后日志的缓冲区
 *  @param Size 缓冲区的大小
 *  @param Log  待格式化的日志消息
 *  @return 格式化后的字符串长度
 */
static inline int Format_ATT_Log(char *Buffer, size_t Size, const LogMessage_t *Log) {
    return snprintf(Buffer, Size, "[%lu] [ATT] R:%.1f, P:%.1f, Y:%.1f\r\n",
                    Log->Timestamp,
                    Log->Payload.Att.Roll,
                    Log->Payload.Att.Pitch,
                    Log->Payload.Att.Yaw);
}

/*
 *  格式化无人机海拔日志的内联函数，会被用于日志任务中，目的是增强代码易读性。
 *  
 *  @param Buffer 存放格式化后日志的缓冲区
 *  @param Size 缓冲区的大小
 *  @param Log  待格式化的日志消息
 *  @return 格式化后的字符串长度
 */
static inline int Format_ALT_Log(char *Buffer, size_t Size, const LogMessage_t *Log) {
    return snprintf(Buffer, Size, "[%lu] [ALT] H:%.2f m | Vz:%.2f m/s\r\n",
                    Log->Timestamp, 
                    Log->Payload.Alt.Alttitude, 
                    Log->Payload.Alt.ClimbRate);
}

/*
 *  日志任务的任务函数，也是最终调用UART0串口发送日志到终端的函数，参数未使用。
 */
static void Logging_Task(void *pvParameters) {
    LogMessage_t Log;
    char Buffer[128];
    const char* ModuleNames[] = {"SYS", "IMU", "PID", "NAV"};

    for (;;) {
        if (xQueueReceive(LogQueue, &Log, portMAX_DELAY) == pdPASS) {
            int Len = 0;
            
            switch (Log.LogType) {
                case LOG_TYPE_DATA:
                    Len = Format_Data_Log(Buffer, sizeof(Buffer), &Log);
                    break;
                case LOG_TYPE_MSG:
                    Len = Format_MSG_Log(Buffer, sizeof(Buffer), &Log);
                    break;
                case LOG_TYPE_RAW_HEX:
                    Len = Format_Raw_Hex_Log(Buffer, sizeof(Buffer), &Log);
                    break;
                case LOG_TYPE_UAV_STATUS:
                    Len = Format_UAV_Status_Log(Buffer, sizeof(Buffer), &Log);
                    break;
                case LOG_TYPE_ATT:
                    Len = Format_ATT_Log(Buffer, sizeof(Buffer), &Log);
                    break;
                case LOG_TYPE_ALT:
                    Len = Format_ALT_Log(Buffer, sizeof(Buffer), &Log);
                    break;
                default: 
                    break;
            }

            if (Len > 0) {
                Puts_UART(Buffer);
            }
        }
    }
}

void Init_Log_Task(void) {
    xTaskCreate(Logging_Task, 
                "LogTask", 
                STACK_LOGGING, 
                NULL, 
                PRIO_LOGGING_TASK, 
                NULL);
}

void Log_Data(float Float) {
    LogMessage_t Msg;
    Msg.LogType = LOG_TYPE_DATA;
    Msg.Timestamp = xTaskGetTickCount();
    Msg.Payload.Data = Float;
    xQueueSend(LogQueue, &Msg, 0);
}

void Log_Msg(const char* Str) {
    LogMessage_t Msg;
    Msg.LogType = LOG_TYPE_MSG;
    Msg.Timestamp = xTaskGetTickCount();
    strncpy(Msg.Payload.Msg, Str, MAX_MSG_LENGTH-1);
    Msg.Payload.Msg[MAX_MSG_LENGTH-1] = '\0';
    xQueueSend(LogQueue, &Msg, 0);
}

void Log_Raw(const uint8_t* Byte, uint8_t Len) {
    LogMessage_t Msg;
    Msg.LogType = LOG_TYPE_RAW_HEX;
    Msg.Timestamp = xTaskGetTickCount();
    if (Len > MAX_RAW_LENGTH) Len = MAX_RAW_LENGTH;
    memcpy(Msg.Payload.Raw, Byte, Len);
    xQueueSend(LogQueue, &Msg, 0);
}

void Log_Attitude(uint32_t Timestamp, float Roll, float Pitch, float Yaw) {
    LogMessage_t Msg;
    Msg.LogType = LOG_TYPE_ATT;
    Msg.Timestamp = Timestamp;
    Msg.Payload.Att.Roll  = Roll  * RAD_TO_DEG;
    Msg.Payload.Att.Pitch = Pitch * RAD_TO_DEG;
    Msg.Payload.Att.Yaw   = Yaw   * RAD_TO_DEG;
    xQueueSend(LogQueue, &Msg, 0);
}

void Log_Altitude(uint32_t Timestamp, float Alttitude, float ClimbRate) {
    LogMessage_t Msg;
    Msg.LogType = LOG_TYPE_ALT;
    Msg.Timestamp = Timestamp;
    Msg.Payload.Alt.Alttitude = Alttitude;
    Msg.Payload.Alt.ClimbRate = ClimbRate;
    xQueueSend(LogQueue, &Msg, 0);
}

void Log_UAVStatus(uint32_t Timestamp, uint8_t SystemId, uint8_t BaseMode, uint8_t SystemStatus, uint32_t CustomMode){
    LogMessage_t Msg;
    Msg.LogType = LOG_TYPE_UAV_STATUS;
    Msg.Timestamp = Timestamp;
    Msg.Payload.UAVStatus.SystemId = SystemId;
    Msg.Payload.UAVStatus.BaseMode = BaseMode;
    Msg.Payload.UAVStatus.SystemStatus = SystemStatus;
    Msg.Payload.UAVStatus.CustomMode = CustomMode;
    xQueueSend(LogQueue, &Msg, 0);
}