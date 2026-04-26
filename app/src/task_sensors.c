#include "task_sensors.h"

IMUData_t imu_data;
static TaskHandle_t targetTaskHandle = NULL;

static void Process_Sensor_data(void){
    imu_data.Timestamp = xTaskGetTickCount();
}

static void Sensor_Task(void * pvParameters) {

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskFrequency = PERIOD_SENSOR_DATA_MS;
    for(;;){
        vTaskDelayUntil(&lastWakeTime, taskFrequency);
        Process_Sensor_data();

        if (targetTaskHandle != NULL) {
            xTaskNotifyGive(targetTaskHandle);
        }
    }
}

void Init_SensorData_Task(void){
    xTaskCreate(Sensor_Task, "SensorTask", STACK_SENSOR_DATA, NULL, PRIO_SENSOR_DATA_TASK, NULL);
}

void Set_Target_Task_Handle(TaskHandle_t taskHandle) {
    targetTaskHandle = taskHandle;
}