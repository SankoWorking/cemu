#include "sensors.h"

/**
 * @brief 处理并更新心跳包状态的内联函数
 * 
 * @param Msg 指向接收到的 MAVLink 消息原始数据的指针
 * @param StatusPtr 指向全局或任务内 UAV 状态结构体的指针
 */
static inline void Process_Heartbeat_Message(const mavlink_message_t* Msg, UAVStatus_t* StatusPtr) {
    mavlink_heartbeat_t Heartbeat;
    mavlink_msg_heartbeat_decode(Msg, &Heartbeat);
    StatusPtr->Timestamp = xTaskGetTickCount(); 
    StatusPtr->SystemId = Msg->sysid;
    StatusPtr->BaseMode = Heartbeat.base_mode;
    StatusPtr->SystemStatus = Heartbeat.system_status;
    StatusPtr->CustomMode = Heartbeat.custom_mode;
    Log_UAVStatus(StatusPtr->Timestamp, 
                  StatusPtr->SystemId, 
                  StatusPtr->BaseMode, 
                  StatusPtr->SystemStatus, 
                  StatusPtr->CustomMode);
}

/**
 * @brief 处理姿态数据包并触发控制任务
 * 
 * @param Msg 指向接收到的 MAVLink 消息
 * @param AttPtr 指向全局姿态状态结构体 (CurrentAttitude)
 * @param TargetTask 姿态控制任务的任务句柄 (task_attitude)
 */
static inline void Process_Attitude_Message(const mavlink_message_t* Msg, 
                                          AttitudeData_t* AttPtr, 
                                          TaskHandle_t TargetTask) {
    mavlink_attitude_t AttRaw;
    mavlink_msg_attitude_decode(Msg, &AttRaw);
    AttPtr->Roll       = AttRaw.roll;
    AttPtr->Pitch      = AttRaw.pitch;
    AttPtr->Yaw        = AttRaw.yaw;
    AttPtr->RollSpeed  = AttRaw.rollspeed;
    AttPtr->PitchSpeed = AttRaw.pitchspeed;
    AttPtr->YawSpeed   = AttRaw.yawspeed;
    AttPtr->Timestamp = xTaskGetTickCount(); 
    if (TargetTask != NULL) {
        xTaskNotifyGive(TargetTask);
    }
}

/**
 * @brief 处理 VFR_HUD 消息并更新高度与爬升率状态
 * 
 * @param Msg 指向接收到的 MAVLink 消息
 * @param AltPtr 指向全局高度状态结构体 (CurrentAltitude)
 */
static inline void Process_VFR_HUD_Message(const mavlink_message_t* Msg, AltitudeData_t* AltPtr) {
    mavlink_vfr_hud_t hud;
    mavlink_msg_vfr_hud_decode(Msg, &hud);
    AltPtr->Alttitude = hud.alt;
    AltPtr->ClimbRate = hud.climb;
    AltPtr->Timestamp = xTaskGetTickCount(); 
    Log_Altitude(AltPtr->Timestamp, AltPtr->Alttitude, AltPtr->ClimbRate);
}

/**
 * @brief  传感器数据分发处理函数
 * @note   该函数根据 MAVLink 消息 ID 将接收到的原始消息分发给对应的解析内联函数。
 *         这是 UART 接收任务与系统全局状态量（UAVStatus, CurrentAttitude 等）之间的核心接口。
 * 
 * @param  Msg: 指向已完成 MAVLink 帧解析的消息结构体指针
 */
static void Process_Sensor_data(const mavlink_message_t *Msg, TaskHandle_t Handle){
    switch (Msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            Process_Heartbeat_Message(Msg, &UAVStatus);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            Process_Attitude_Message(Msg, &CurrentAttitude, Handle);
            break;
        }
        case MAVLINK_MSG_ID_VFR_HUD: {
            Process_VFR_HUD_Message(Msg, &CurrentAltitude);
            break;
        }
        default:
            break;
    }
}

static void Sensor_Task(void * pvParameters) {
    SensorTaskParams_t *Params = (SensorTaskParams_t *)pvParameters;

    StreamBufferHandle_t UART1Buffer = Params->UARTBuffer;
    TaskHandle_t TargetTaskHandle = Params->AttitudeTaskHandle;

    uint8_t RXTempBuffer[32];
    mavlink_message_t Msg;
    static mavlink_status_t Status;
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
