#include "task_attitude.h"

static TaskHandle_t attitudeTaskHandle = NULL;

static void Attitude_Task(void * pvParameters) {
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Puts_UART("Get Data\n");
        Putu_UART(imu_data.timestamp);
        Putc_UART('\n');
    }
}

void Init_Attitude_Task(void) {
    xTaskCreate(Attitude_Task, 
                "AttitudeTask", 
                STACK_ATTITUDE_CTRL, 
                NULL, 
                PRIO_ATTITUDE_CTRL_TASK, 
                &attitudeTaskHandle);
}

TaskHandle_t Get_Attitude_Task_Handle(void) {
    return attitudeTaskHandle;
}