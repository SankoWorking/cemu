#include "tasks_init.h"

static TaskHandle_t taskHandle = NULL;

//TODO 注释
void Init_System(void) {
    Init_Log_Queue();
    Init_Log_Task();

    Init_Attitude_Task();
    taskHandle = Get_Attitude_Task_Handle();
    Set_Target_Task_Handle(taskHandle);
    
    Init_UART0_Interrupt();
    Init_UART1_Interrupt();

    
    Init_SensorData_Task((void *)Uart1RxStreamBuffer);

    Init_Heartbeat_Tasks();
    Log_Msg("System, Started");
    vTaskStartScheduler();
    
}