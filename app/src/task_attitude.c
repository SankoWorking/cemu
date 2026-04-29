#include "task_attitude.h"

static TaskHandle_t attitudeTaskHandle = NULL;

static void Attitude_Task(void * pvParameters) {
    uint32_t counter = 0;
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (++counter >= 10) { 
            
            //Log_Attitude(&current_attitude);
            counter = 0;
        }
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