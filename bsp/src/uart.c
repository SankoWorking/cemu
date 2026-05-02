#include "uart.h"

StreamBufferHandle_t Uart1RxStreamBuffer = NULL;

/*
 *  声明UART0发送FIFO满足发送需求（TX FIFO 低于阈值）的二值信号量
 */
static SemaphoreHandle_t Uart0TxSem = NULL;

/*
 *  声明UART1发送FIFO满足发送需求（TX FIFO 低于阈值）的二值信号量
 */
static SemaphoreHandle_t Uart1TxSem = NULL;

/*
 *  UART0的串口打印函数。会将c写入UART0的数据寄存器，当FIFO满时，会阻塞等待标志TX FIFO空阈值的二值信号量。
 *  @param c 待打印的字节
 */
static void Putc_UART0(char c) {
	while (UART0_FR_R & (1 << 5)) {
        xSemaphoreTake(Uart0TxSem, portMAX_DELAY);
    }
    UART0_DR_R = c;
}

void Puts_UART0(const char *s) {
	while (*s) {
		Putc_UART0(*s++);
	}
}

/*
 * UART1的字节发送函数。
 * 逻辑与UART0相同：当TX FIFO满时，阻塞等待TX FIFO低于阈值的中断信号量。
 */
static void Putc_UART1(uint8_t c) {
    // 判断TX FIFO是否已满 (TXFF, bit 5)
    while (UART1_FR_R & (1 << 5)) {
        xSemaphoreTake(Uart1TxSem, portMAX_DELAY);
    }
    UART1_DR_R = c;
}

/*
 * UART1的指令流发送函数。
 * 供飞控控制任务调用，向14540端口发送PX4/Gazebo控制指令。
 */
void Send_UART1(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        Putc_UART1(data[i]);
    }
}

void Init_UART1_Interrupt(void) {
    Uart1RxStreamBuffer = xStreamBufferCreate(UART1_RX_STREAM_BUFFER_SIZE, UART1_RX_TRIGGER_LEVEL);
    Uart1TxSem = xSemaphoreCreateBinary();
    //开启UART1和GPIOD的外设时钟
    UART1_RCC_R |= (1 << 1);
    UART1_RCC_GPIOD_R |= (1 << 3);

    //启用GPIOD的数字功能
    UART1_GPIOD_DEN_R |= (1 << 2) | (1 << 3);

    //配置GPIOD的引脚复用，使其受UART1控制
    UART1_GPIOD_AFSEL_R |= (1 << 2) | (1 << 3);

    //配置UART1
    UART1_CTL_R &= ~0x01U; 
    UART1_IBRD_R = 10; 
    UART1_FBRD_R = 54;
    UART1_LCRH_R = (0x3 << 5) | (1 << 4);
	
    //设置RXFIFO和TXFIFO的阈值，将FIFO阈值设置为1/2（RX）和1/8（TX）
    UART1_IFLS_R &= ~0x3FU;
    UART1_IFLS_R |= (0x2U << 3) | (0x0U << 0);

	//开启接收FIFO达阈值中断和接收超时中断，未开启发送FIFO低于阈值中断
    UART1_IM_R |= (1U << 4) | (1U << 5) | (1U << 6);

	//设置串口中断的优先级为5,处于freeRTOS的可控范围内，可以调用fromISR结尾的freeRTOS API
    NVIC_PRI1_R = (NVIC_PRI1_R & ~(7U << 21)) | (5U << 21);
	//使能串口中断
    NVIC_EN0_R  |= (1U << 6); 

    //使能串口
    UART1_CTL_R |= (1 << 0) | (1 << 8) | (1 << 9);
}

void Init_UART0_Interrupt(void) {
    Uart0TxSem = xSemaphoreCreateBinary();
    xSemaphoreGive(Uart0TxSem);

    //开启UART0和GPIOA的外设时钟
    UART0_RCC_R |= (1 << 0);
    UART0_RCC_GPIOA_R |= (1 << 0);

    //启用GPIOA的数字功能
    UART0_GPIOA_DEN_R |= (1 << 0) | (1 << 1);

    //配置GPIOD的引脚复用，使其受UART1控制
    UART0_GPIOA_AFSEL_R |= (1 << 0) | (1 << 1);

    //禁用UART0后配置波特率及传输格式
    UART0_CTL_R &= ~0x01U; 
    UART0_IBRD_R = 10; 
    UART0_FBRD_R = 54;
    UART0_LCRH_R = (0x3 << 5) | (1 << 4);
    
    // 将 TX FIFO 阈值设置为 1/8 (即剩余空间较多时触发中断，方便连续填充)
    UART0_IFLS_R &= ~0x3FU;
    UART0_IFLS_R |= (0x0U << 0); 

    // 仅开启发送 FIFO 低于阈值中断 (TXIM)
    UART0_IM_R = (1U << 5); 

    //配置UART0串口中断优先级，将其配置为6,低于UART1的优先级，因为此串口只负责打印数据到终端。
    NVIC_PRI1_R = (NVIC_PRI1_R & ~(7U << 13)) | (6U << 13);
    
    //使能串口中断
    NVIC_EN0_R |= (1U << 5); 

    //使能串口
    UART0_CTL_R |= (1 << 0) | (1 << 8);
}

/*
 * UART0的中断服务函数。当UART0的TX FIFO低于阈值时会触发中断，在此服务函数中会释放二值信号量。
 */
void UART0_Handler(void) {
    //从UART1对应的MIS中读取屏蔽后的中断状态
    uint32_t status = UART0_MIS_R;
    
    //清除中断标志位
    UART0_IC_R = status;
    //判断是否为发送中断
    if (status & (1 << 5)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(Uart0TxSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * UART1的中断服务函数。
 * 当RX FIFO达阈值时会触发中断，在中断中将收到的字节流放入StreamBuffer
 */
void UART1_Handler(void) {
    //从UART1对应的MIS中读取屏蔽后的中断状态
    uint32_t status = UART1_MIS_R;
    uint8_t TempDataArr[16];
    uint8_t Count = 0;
    
    UART1_IC_R = status;
    BaseType_t HigherPriorityTaskWoken = pdFALSE;

    if (status & ((1 << 4) | (1 << 6))) {
		while (!(UART1_FR_R & (1 << 4))) { 
            TempDataArr[Count++] = (uint8_t)(UART1_DR_R & 0xFF);
            if (Count >= 16) {
                xStreamBufferSendFromISR(Uart1RxStreamBuffer, TempDataArr, Count, &HigherPriorityTaskWoken);
                Count = 0;
            }            
        }
        if (Count > 0) {
            xStreamBufferSendFromISR(Uart1RxStreamBuffer, TempDataArr, Count, &HigherPriorityTaskWoken);
        }
    }
    
    if (status & (1 << 5)) {
        xSemaphoreGiveFromISR(Uart1TxSem, &HigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}