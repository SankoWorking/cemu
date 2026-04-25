#include "tasks_init.h"

static TaskHandle_t taskHandle = NULL;

void Init_System(void) {
    Init_Attitude_Task();
    taskHandle = Get_Attitude_Task_Handle();
    Set_Target_Task_Handle(taskHandle);
    Init_SensorData_Task();

    vTaskStartScheduler();
}