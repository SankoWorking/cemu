#include "task_sensors.h"

IMUData_t imu_data;
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

        default:
            
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
            /*
            if (targetTaskHandle != NULL) {
                xTaskNotifyGive(targetTaskHandle);
            }
            */
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
