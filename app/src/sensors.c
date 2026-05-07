#include "sensors.h"
#include "tasks_interfaces.h"

/**
 * @brief 处理 HIL_SENSOR 消息 (ID: 107)。从Gazebo获取仿真时间，并更新全局仿真时间戳CurrentSimTimeUs
 * 
 */
static inline void Process_HIL_Sensor_Message(const mavlink_message_t* Msg, TaskHandle_t TargetTask) {
    mavlink_hil_sensor_t Hil;
    mavlink_msg_hil_sensor_decode(Msg, &Hil);

    CurrentSimTimeUs = Hil.time_usec;

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

    if (Hil.abs_pressure != 0.0f) {
        CurrentBaroData.Timestamp   = Hil.time_usec;
        CurrentBaroData.AbsPressure = Hil.abs_pressure;
        CurrentBaroData.PressureAlt = Hil.pressure_alt;
    }
    if (TargetTask != NULL) {
        xTaskNotifyGive(TargetTask);
    }
}

/**
 * @brief MAVLink 消息分发器
 */
static void Process_Sensor_data(const mavlink_message_t *Msg, TaskHandle_t Handle) {
    if (Msg->msgid == MAVLINK_MSG_ID_HIL_SENSOR) {
        Process_HIL_Sensor_Message(Msg, Handle);
    }
}
/**
 * @brief 传感器解析任务
 */
static void Sensor_Task(void * pvParameters) {
    static SensorTaskParams_t Params; 
    Params = *(SensorTaskParams_t *)pvParameters;

    StreamBufferHandle_t UART1Buffer = Params.UARTBuffer;
    TaskHandle_t TargetTaskHandle = Params.AttitudeTaskHandle;

    uint8_t RXTempBuffer[64];
    mavlink_message_t Msg;
    mavlink_status_t Status;

    for(;;){
        size_t Received = xStreamBufferReceive(UART1Buffer, RXTempBuffer, sizeof(RXTempBuffer), portMAX_DELAY);
        if(Received > 0) {
            for(size_t i = 0; i < Received; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, RXTempBuffer[i], &Msg, &Status)) {
                    Process_Sensor_data(&Msg, TargetTaskHandle);
                }
            }
        } 
    }
}

void Init_Sensor_Task(SensorTaskParams_t *Params){
    xTaskCreate(Sensor_Task, 
                "SensorTask", 
                STACK_SENSOR_DATA, 
                (void *)Params, 
                PRIO_SENSOR_DATA_TASK, 
                NULL);
}