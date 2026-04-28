#include "task_sensors.h"

IMUData_t imu_data;
AttitudeData_t current_attitude;
SystemStatus_t SytemStatus;
static TaskHandle_t targetTaskHandle = NULL;
QueueHandle_t SensorQueue = NULL;

static void Process_Sensor_data(const mavlink_message_t *msg){
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            // 1. 定义解析用的结构体
            mavlink_heartbeat_t hb;
            
            // 2. 解码：将原始消息解析到结构体中
            mavlink_msg_heartbeat_decode(msg, &hb);
            
            // 3. 执行业务逻辑
            SytemStatus.is_connected = 1;
            SytemStatus.remote_system_id = msg->sysid; // 记录对方的 ID
            SytemStatus.base_mode = hb.base_mode;      // 记录飞行模式（是否解锁等）
            SytemStatus.system_status = hb.system_status; // 记录系统健康度
            SytemStatus.Timestamp = xTaskGetTickCount();

            Log_Heartbeat(hb.type, hb.autopilot, hb.system_status, hb.base_mode, SytemStatus.Timestamp);
            
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
