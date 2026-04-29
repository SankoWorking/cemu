#include "task_sensors.h"

IMUData_t imu_data;
AttitudeData_t current_attitude;
AltitudeData_t current_altitude;
UAVStatus_t UAVStatus;
static TaskHandle_t targetTaskHandle = NULL;
QueueHandle_t SensorQueue = NULL;

static void Process_Sensor_data(const mavlink_message_t *msg){
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;

            mavlink_msg_heartbeat_decode(msg, &hb);
            
            UAVStatus.Timestamp = xTaskGetTickCount();
            UAVStatus.SystemId = msg->sysid;
            UAVStatus.BaseMode = hb.base_mode;
            UAVStatus.SystemStatus = hb.system_status;
            UAVStatus.CustomMode = hb.custom_mode;
            
            Log_UAVStatus(UAVStatus.Timestamp, UAVStatus.SystemId, UAVStatus.BaseMode, UAVStatus.SystemStatus, UAVStatus.CustomMode);
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att_raw;
            mavlink_msg_attitude_decode(msg, &att_raw);
            
            current_attitude.roll       = att_raw.roll;
            current_attitude.pitch      = att_raw.pitch;
            current_attitude.yaw        = att_raw.yaw;
            current_attitude.rollspeed  = att_raw.rollspeed;
            current_attitude.pitchspeed = att_raw.pitchspeed;
            current_attitude.yawspeed   = att_raw.yawspeed;
            current_attitude.Timestamp  = xTaskGetTickCount();
            if (targetTaskHandle != NULL) {
                xTaskNotifyGive(targetTaskHandle);
            }
            
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t hud;
            mavlink_msg_vfr_hud_decode(msg, &hud);
            
            current_altitude.alt        = hud.alt;        // 对应 VFR_HUD 中的 alt
            current_altitude.climb_rate = hud.climb;      // 对应 VFR_HUD 中的 climb
            current_altitude.throttle   = (float)hud.throttle; 
            current_altitude.Timestamp  = xTaskGetTickCount();
            
            //Log_Height(&current_altitude);

            break;
        }
        /* GAZEBO 没发送IMU原始数据
        case MAVLINK_MSG_ID_HIGHRES_IMU: {
            Log_Msg(MOD_SYS, "IMU MSG got");
            mavlink_highres_imu_t imu_raw;
            mavlink_msg_highres_imu_decode(msg, &imu_raw);

            // 映射到你的 IMUData_t 结构体
            // 加速度计数据 (m/s^2)
            imu_data.acc[0] = imu_raw.xacc;
            imu_data.acc[1] = imu_raw.yacc;
            imu_data.acc[2] = imu_raw.zacc;

            // 陀螺仪数据 (rad/s)
            imu_data.gyro[0] = imu_raw.xgyro;
            imu_data.gyro[1] = imu_raw.ygyro;
            imu_data.gyro[2] = imu_raw.zgyro;

            // 更新时间戳 (转换为系统 Tick)
            imu_data.Timestamp = xTaskGetTickCount();
            //Log_IMU(imu_data.acc, imu_data.gyro, imu_data.Timestamp); 
            
            if (targetTaskHandle != NULL) {
                xTaskNotifyGive(targetTaskHandle);
            }
            
            break;
        }
        */

        default:
            //Log_Data(MOD_SYS, msg->msgid, 0, 0, 0);
            break;
    }
}

static void Sensor_Task(void * pvParameters) {
    uint8_t byte;
    mavlink_message_t msg;
    static mavlink_status_t status;
    for(;;){
        if(xQueueReceive(SensorQueue, &byte, portMAX_DELAY)) {
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)){
                    Process_Sensor_data(&msg);
            }

            while (xQueueReceive(SensorQueue, &byte, 0) == pdPASS) {
                if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                    Process_Sensor_data(&msg);
                }
            }
        }   
    }
}


//TODO 注释
void Init_SensorData_Task(void){
    SensorQueue = xQueueCreate(512, sizeof(uint8_t));
    xTaskCreate(Sensor_Task, "SensorTask", STACK_SENSOR_DATA, NULL, PRIO_SENSOR_DATA_TASK, NULL);
}

void Set_Target_Task_Handle(TaskHandle_t taskHandle) {
    targetTaskHandle = taskHandle;
}
