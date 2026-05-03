#include "tasks_interfaces.h"

AttitudeData_t CurrentAttitude;
AltitudeData_t CurrentAltitude;
UAVStatus_t UAVStatus;

StreamBufferHandle_t Uart1RxStreamBuffer = NULL;

/** 
 *  @brief 初始化UART1转发数据所使用的Stream Buffer
 */
static void Init_UART1_Stream_Buffer(void){
    Uart1RxStreamBuffer = xStreamBufferCreate(UART1_RX_STREAM_BUFFER_SIZE, UART1_RX_TRIGGER_LEVEL);
}

void Init_Interfaces(void){
    Init_UART1_Stream_Buffer();
}


/**
 *  @brief 获取系统从启动至今的 64 位微秒时间戳
 *  @return uint64_t 微秒级时间戳 (us)
 */
uint64_t Get_System_Time_Usec(void) {
    uint32_t Ticks;
    uint32_t SystickVal;
    uint32_t TickPeriod = configCPU_CLOCK_HZ / configTICK_RATE_HZ;

    //进入临界区
    vPortEnterCritical();
    {
        Ticks = xTaskGetTickCount();
        SystickVal = SysTick->VAL;
        if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) && (SystickVal < (TickPeriod / 2))) {
            Ticks++;
        }
    }
    vPortExitCritical();
    uint64_t usec = (uint64_t)Ticks * (1000000 / configTICK_RATE_HZ);
    uint32_t cycles_elapsed = TickPeriod - SystickVal;
    usec += (uint64_t)cycles_elapsed * 1000000 / configCPU_CLOCK_HZ;

    return usec;
}