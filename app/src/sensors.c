#include "sensors.h"
#include "tasks_interfaces.h"

/**
 * @brief 处理心跳包
 */
static inline void Process_Heartbeat_Message(const mavlink_message_t* Msg, UAVStatus_t* StatusPtr) {
    mavlink_heartbeat_t Heartbeat;
    mavlink_msg_heartbeat_decode(Msg, &Heartbeat);
    
    // 注意：在直接 HIL 模式下，这里的 Mode 可能由网桥或其它 GCS 模拟
    StatusPtr->Timestamp = CurrentSimTimeUs; 
    StatusPtr->SystemId = Msg->sysid;
    StatusPtr->BaseMode = Heartbeat.base_mode;
    StatusPtr->SystemStatus = Heartbeat.system_status;
    StatusPtr->CustomMode = __builtin_bswap32(Heartbeat.custom_mode);
}

/**
 * @brief 处理 HIL_SENSOR 消息 (ID: 107)
 * 这是直接从 Gazebo 物理引擎获取的原始数据
 */
static inline void Process_HIL_Sensor_Message(const mavlink_message_t* Msg, TaskHandle_t TargetTask) {
    mavlink_hil_sensor_t Hil;
    mavlink_msg_hil_sensor_decode(Msg, &Hil);

    // 更新全局仿真时间戳 (微秒)
    CurrentSimTimeUs = Hil.time_usec;

    // 更新 IMU 数据 (用于 Mahony 姿态解算)
    CurrentImuData.Timestamp = Hil.time_usec;
    CurrentImuData.AccelX    = Hil.xacc;
    CurrentImuData.AccelY    = Hil.yacc;
    CurrentImuData.AccelZ    = Hil.zacc;
    CurrentImuData.GyroX     = Hil.xgyro;
    CurrentImuData.GyroY     = Hil.ygyro;
    CurrentImuData.GyroZ     = Hil.zgyro;
    CurrentImuData.MagX      = Hil.xmag;
    CurrentImuData.MagY      = Hil.ymag;
    CurrentImuData.MagZ      = Hil.zmag;

    // 更新气压计高度数据 (用于高度控制)
    CurrentBaroData.Timestamp   = Hil.time_usec;
    CurrentBaroData.AbsPressure = Hil.abs_pressure;
    CurrentBaroData.PressureAlt = Hil.pressure_alt;

    // 💡 关键：释放二值信号量，通知 Attitude_Task 开始执行 PID 计算
    if (TargetTask != NULL) {
        xTaskNotifyGive(TargetTask);
    }
}

/**
 * @brief MAVLink 消息分发器
 */
static void Process_Sensor_data(const mavlink_message_t *Msg, TaskHandle_t Handle){
    switch (Msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            Process_Heartbeat_Message(Msg, &UAVStatus);
            break;
        }
        case MAVLINK_MSG_ID_HIL_SENSOR: { // <--- 修改：监听 107 号 HIL 传感器消息
            Process_HIL_Sensor_Message(Msg, Handle);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief 传感器解析任务
 */
static void Sensor_Task(void * pvParameters) {
    // 静态存储参数，防止栈空间释放导致野指针
    static SensorTaskParams_t Params; 
    Params = *(SensorTaskParams_t *)pvParameters;

    StreamBufferHandle_t UART1Buffer = Params.UARTBuffer;
    TaskHandle_t TargetTaskHandle = Params.AttitudeTaskHandle;

    uint8_t RXTempBuffer[64]; // 稍微增大缓冲区以提高吞吐量
    mavlink_message_t Msg;
    mavlink_status_t Status;

    for(;;){
        // 阻塞等待 UART 接收流缓冲区数据
        size_t Received = xStreamBufferReceive(UART1Buffer, RXTempBuffer, sizeof(RXTempBuffer), portMAX_DELAY);
        if(Received > 0) {
            for(size_t i = 0; i < Received; i++) {
                // MAVLink 字节流解析
                if (mavlink_parse_char(MAVLINK_COMM_0, RXTempBuffer[i], &Msg, &Status)) {
                    Process_Sensor_data(&Msg, TargetTaskHandle);
                }
            }
        } 
    }
}

/**
 * @brief 初始化传感器任务接口
 */
void Init_Sensor_Task(SensorTaskParams_t *Params){
    xTaskCreate(Sensor_Task, 
                "SensorTask", 
                STACK_SENSOR_DATA, 
                (void *)Params, 
                PRIO_SENSOR_DATA_TASK, 
                NULL);
}