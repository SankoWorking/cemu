#include "tasks_init.h"

static SensorTaskParams_t SensorParams;

void Init_System(void) {
    Init_Interfaces();

    Init_Log_Task();

    TaskHandle_t AttitudeTaskHandle = Init_Attitude_Task();
    
    Init_UART0_Interrupt();
    Init_UART1_Interrupt();

    SensorParams.UARTBuffer = Uart1RxStreamBuffer;
    SensorParams.AttitudeTaskHandle = AttitudeTaskHandle;
    Init_Sensor_Task(&SensorParams);

    Init_Command_Tasks();

    Log_Generic("System Started");
    vTaskStartScheduler();
}