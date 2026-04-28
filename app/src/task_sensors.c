#include "task_sensors.h"

IMUData_t imu_data;
static TaskHandle_t targetTaskHandle = NULL;
QueueHandle_t SensorQueue = NULL;

static void Process_Sensor_data(void){

}

static void Sensor_Task(void * pvParameters) {
    uint8_t data;
    for(;;){
        if(xQueueReceive(SensorQueue, &data, portMAX_DELAY)) {
            do {

                Log_Raw(MOD_SYS, &data, 1); 
                
            } while (xQueueReceive(SensorQueue, &data, 0));
            if (targetTaskHandle != NULL) {
                xTaskNotifyGive(targetTaskHandle);
            }
        }   
    }
}

void Init_SensorData_Task(void){
    //TODO 修改队列的定义   初始化消息队列
    SensorQueue = xQueueCreate(64, sizeof(uint8_t));
    xTaskCreate(Sensor_Task, "SensorTask", STACK_SENSOR_DATA, NULL, PRIO_SENSOR_DATA_TASK, NULL);
}

void Set_Target_Task_Handle(TaskHandle_t taskHandle) {
    targetTaskHandle = taskHandle;
}
