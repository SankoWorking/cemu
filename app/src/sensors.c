#include "sensors.h"

static TaskHandle_t targetTaskHandle = NULL;

static void Process_Sensor_data(const mavlink_message_t *Msg){
    switch (Msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t Heartbeat;

            mavlink_msg_heartbeat_decode(Msg, &Heartbeat);
            
            UAVStatus.Timestamp = xTaskGetTickCount();
            UAVStatus.SystemId = Msg->sysid;
            UAVStatus.BaseMode = Heartbeat.base_mode;
            UAVStatus.SystemStatus = Heartbeat.system_status;
            UAVStatus.CustomMode = Heartbeat.custom_mode;
            
            Log_UAVStatus(UAVStatus.Timestamp, UAVStatus.SystemId, UAVStatus.BaseMode, UAVStatus.SystemStatus, UAVStatus.CustomMode);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t AttRaw;
            mavlink_msg_attitude_decode(Msg, &AttRaw);
            
            CurrentAttitude.Roll       = AttRaw.roll;
            CurrentAttitude.Pitch      = AttRaw.pitch;
            CurrentAttitude.Yaw        = AttRaw.yaw;
            CurrentAttitude.RollSpeed  = AttRaw.rollspeed;
            CurrentAttitude.PitchSpeed = AttRaw.pitchspeed;
            CurrentAttitude.YawSpeed   = AttRaw.yawspeed;
            CurrentAttitude.Timestamp  = xTaskGetTickCount();
            if (targetTaskHandle != NULL) {
                xTaskNotifyGive(targetTaskHandle);
            }
            
            break;
        }
        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t hud;
            mavlink_msg_vfr_hud_decode(Msg, &hud);
            CurrentAltitude.Alttitude = hud.alt;
            CurrentAltitude.ClimbRate = hud.climb;
            CurrentAltitude.Timestamp = xTaskGetTickCount();
            Log_Altitude(CurrentAltitude.Timestamp, CurrentAltitude.Alttitude, CurrentAltitude.ClimbRate);
            break;
        }
        default:
            break;
    }
}

static void Sensor_Task(void * pvParameters) {
    StreamBufferHandle_t UART1Buffer = (StreamBufferHandle_t)pvParameters;
    uint8_t RXTempBuffer[32];
    mavlink_message_t Msg;
    static mavlink_status_t Status;
    for(;;){
        size_t Received = xStreamBufferReceive(UART1Buffer, RXTempBuffer, sizeof(RXTempBuffer), portMAX_DELAY);
        if(Received > 0) {
            for(size_t i = 0; i < Received; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, RXTempBuffer[i], &Msg, &Status)) {
                    Process_Sensor_data(&Msg);
                }
            }
        } 
    }
}

void Init_SensorData_Task(void *Parameters){
    xTaskCreate(Sensor_Task, "SensorTask", STACK_SENSOR_DATA, (void *)Parameters, PRIO_SENSOR_DATA_TASK, NULL);
}

void Set_Target_Task_Handle(TaskHandle_t taskHandle) {
    targetTaskHandle = taskHandle;
}
