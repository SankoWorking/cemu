#include "uart.h"

//TODO* 需要调整为信号量加中断的设计，这里只是配置了一下休眠。
void Putc_UART(char c) {
	while ( UART0_FR_R & (1 << 5) ){
		vTaskDelay(1);
	};
	UART0_DR_R = c;
}

//TODO 注释
void Puts_UART(const char *s) {
	while (*s) {
		Putc_UART(*s++);
	}
}

static int Getc_UART(char *c){
	if (!(UART1_FR_R & (1 << 4))) {
		*c = (char)(UART1_DR_R & 0xFF);
		return 1;
	}
	return 0;
}

//TODO 注释 初始化了串口的中断。
void Init_UART1_Interrupt(void) {
    *(volatile uint32_t *)0x400FE104 |= (1 << 1); // RCGC1 UART
    *(volatile uint32_t *)0x400FE108 |= (1 << 3); // RCGC2 GPIO D (UART1引脚)

    *(volatile uint32_t *)0x4000751C |= (1 << 2) | (1 << 3); // GPIOD_DEN_R

    // 2. 开启 GPIO D 的复用功能 (Alternate Function Select)
    *(volatile uint32_t *)0x40007420 |= (1 << 2) | (1 << 3); // GPIOD_AFSEL_R

    // 2. 配置 UART1 参数 (必须配置，否则 QEMU 可能不转发数据)
    UART1_CTL_R &= ~0x01; 
    UART1_IBRD_R = 10; 
    UART1_FBRD_R = 54;
    UART1_LCRH_R = (0x3 << 5) | (1 << 4); // 8-N-1, Enable FIFO
	//将IFLS_R清零，并设置[5:3]区间和[2:0]区间，将FIFO阈值设置为1/2（RX）和1/8（TX）
    UART1_IFLS_R &= ~0x3FU;
    UART1_IFLS_R |= (0x2U << 3) | (0x0U << 0);

	//开启接收FIFO达阈值中断和接收超时中断，未开启发送FIFO低于阈值中断
    UART1_IM_R |= (1U << 4) | (1U << 6); 

	//设置串口中断的优先级为5,处于freeRTOS的可控范围内，可以调用fromISR结尾的freeRTOS API
    NVIC_PRI1_R = (NVIC_PRI1_R & ~(7U << 21)) | (5U << 21);
    
	//使能串口中断
    NVIC_EN0_R = (1U << 6); 
    UART1_CTL_R |= (1 << 0) | (1 << 8) | (1 << 9);
}

void UART1_Handler(void) {
    //读取串口MIS
    uint32_t status = *(volatile uint32_t *)(UART1_BASE + 0x040);
    
	//定义是否切换任务的标记
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // --- 处理【接收】部分 (RX 或 RT 超时) ---
    if (status & ((1 << 4) | (1 << 6))) {
		uint8_t data;
		while (Getc_UART(&data)){
        	xQueueSendFromISR(SensorQueue, &data, &xHigherPriorityTaskWoken);
		}
    }

	//清除中断标志位
    UART1_IC_R = status; // UARTICR

    // 3. 统一进行上下文切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}