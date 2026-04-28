#include "tasks_init.h"

static TaskHandle_t taskHandle = NULL;

//TODO 注释
void Init_System(void) {
    Init_Log();
    Init_Log_Task();
    
    Init_UART1_Interrupt();

    Init_Attitude_Task();
    taskHandle = Get_Attitude_Task_Handle();
    Set_Target_Task_Handle(taskHandle);
    Init_SensorData_Task();

    Log_Msg(MOD_SYS, "System, Started");
    vTaskStartScheduler();
    
}