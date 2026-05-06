#include "tasks_interfaces.h"

QueueHandle_t MotorControlQueue = NULL;
StreamBufferHandle_t Uart1RxStreamBuffer = NULL;

volatile uint64_t CurrentSimTimeUs = 0;
UAVStatus_t UAVStatus = {0};
BaroSensorData_t CurrentBaroData = {0};
ImuSensorData_t CurrentImuData = {0};

/** 
 *  @brief 初始化UART1转发数据所使用的Stream Buffer
 */
static void Init_UART1_Stream_Buffer(void){
    Uart1RxStreamBuffer = xStreamBufferCreate(UART1_RX_STREAM_BUFFER_SIZE, UART1_RX_TRIGGER_LEVEL);
}

/** 
 *  @brief 初始化电机控制指令队列
 */
static void Init_Motor_Control_Queue(void){
    MotorControlQueue = xQueueCreate(10, sizeof(MotorCommandMsg_t));
}

void Init_Interfaces(void){
    Init_UART1_Stream_Buffer();
    Init_Motor_Control_Queue();
}
