#include "tasks_interfaces.h"

AttitudeData_t CurrentAttitude;
AltitudeData_t CurrentAltitude;
UAVStatus_t UAVStatus;

StreamBufferHandle_t Uart1RxStreamBuffer = NULL;

/*
 *  初始化UART1转发数据所使用的Stream Buffer
 */
static void Init_UART1_Stream_Buffer(void){
    Uart1RxStreamBuffer = xStreamBufferCreate(UART1_RX_STREAM_BUFFER_SIZE, UART1_RX_TRIGGER_LEVEL);
}

void Init_Interfaces(void){
    Init_UART1_Stream_Buffer();
}
