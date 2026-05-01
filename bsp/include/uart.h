#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include <FreeRTOS.h>
#include "stream_buffer.h"
#include "semphr.h"

/*
 *  UART1配置，此串口用于接收Gazebo从14540端口发来的传感器数据以及发送控制指令到14540端口。
 *  
 *  串口工作原理：物理层包括TX、RX、GND。
 *              数据帧结构一般包含，起始位、数据位、校验位、停止位。常态下信号线为高电平。通
 *              信开始时，发送方拉低电平，通知接收方准备接收。数据位包含实际传输的内容，通常
 *              为8位，也可以是5、6或7位。校验位是可选位，用于奇校验或偶校验。停止位，发送方
 *              将电平拉高，保持1、1.5或2个周期，表示当前帧结束。
 *              波特率，约定的采样频率，双方需保持一致。
 *  
 *  UART1_BASE      ->  UART1基地址
 *  UART1_DR_R      ->  Data Register（数据寄存器），也就是串口的基地址。向此寄存器写入一个
 *                      字节时，数据会被放入发送FIFO。当从这个寄存器读取数据时，会获得接收FIFO
 *                      的第一个第一个数据。
 *                      [7:0]实际传输或接收的8位数据，因为最低八位就是传输或接收数据的有效位，所
 *                      以可以直接向此寄存器写入或读取字节。
 *                      [8] [9] [10] [11]为会有一些状态信息可供读取。
 *  UART1_FR_R      ->  Flag Register(标志寄存器)。
 *                      [3] 1表示串口忙，0表示传输完成。
 *                      [4] RXFE
 *                      [5] TXFF
 *                      [6] RXFF
 *                      [7] TXFE
 *  UART1_LCRH_R    ->  Line Control Register High（线路控制寄存器-高位字节）负责定义数据传输
 *                      的格式。如数据帧长度、停止位长度、校验位配置等。
 *  UART1_CTL_R     ->  Control Register (控制寄存器) 负责整个UART模块的启用和模式选择。
 *                      [0] 总开关，配置波特率通常需要将其置0。
 *                      [8] TXE 发送使能。
 *                      [9] RXE 接收使能。
 *  UART1_IBRD_R
 *  UART1_FBRD_R    ->  配置波特率的寄存器
 *  UART1_IFLS_R    ->  Interrupt FIFO Level Select(中断FIFO级别选择寄存器)，负责定
 *                      FIFO剩下（或堆积）多少数据时，才触发中断。
 *  UART1_IM_R      ->  Interrupt Mask(中断屏蔽寄存器)，在嵌入式系统中，虽然硬件可以检测到各种
 *                      事件，但并不是每个事件都必须打断CPU的工作。IM寄存器就是用来控制哪些硬件事
 *                      允许向CPU发送中断请求。
 *  UART1_IC_R      ->  Interrupt Clear（中断清除寄存器），负责手动确认关闭已经处理完的中断信号。
 *  UART1_MIS_R     ->  Masked Interrupt Status(屏蔽后中断状态寄存器)，是RIS（原始中断状态）和
 *                      IM与运算后的结果。
 *  UART1_RCC_R     ->  用于开启外设时钟
 *  UART1_RCC_GPIOD_R ->  用于开启UART1对应的GPIO口的外设时钟
 *  UART1_GPIOD_DEN_R ->  GPIO Port D Digital Enable 用于将UART1对应的GPIO口配置为数字模式
 *  UART1_GPIOD_AFSEL_R -> 实现GPIO引脚复用，将其配置为硬件联动模式，使其由UART1控制。
 */
#define UART1_BASE          0x4000D000
#define UART1_DR_R          (*(volatile unsigned int *)(UART1_BASE + 0x00))
#define UART1_FR_R          (*(volatile unsigned int *)(UART1_BASE + 0x18))
#define UART1_LCRH_R        (*(volatile uint32_t *)(UART1_BASE + 0x02C))
#define UART1_CTL_R         (*(volatile uint32_t *)(UART1_BASE + 0x030))
#define UART1_IBRD_R        (*(volatile uint32_t *)(UART1_BASE + 0x024))
#define UART1_FBRD_R        (*(volatile uint32_t *)(UART1_BASE + 0x028))
#define UART1_IFLS_R        (*(volatile uint32_t *)(UART1_BASE + 0x034))
#define UART1_IM_R          (*(volatile uint32_t *)(UART1_BASE + 0x038))
#define UART1_IC_R          (*(volatile uint32_t *)(UART1_BASE + 0x044))
#define UART1_MIS_R         (*(volatile uint32_t *)(UART1_BASE + 0x040))
#define UART1_RCC_R         (*(volatile uint32_t *)0x400FE104)
#define UART1_RCC_GPIOD_R   (*(volatile uint32_t *)0x400FE108)
#define UART1_GPIOD_DEN_R   (*(volatile uint32_t *)0x4000751C)
#define UART1_GPIOD_AFSEL_R (*(volatile uint32_t *)0x40007420)

/*
 *  NVIC_EN0_R    ->  在NVIC中配置中断通道，在LM3S6965evb中UART1中断号为6，UART0为5
 *  NVIC_PRI1_R   -> 在NVIC中配置中断优先级，NVIC_PRI1_R此寄存器负责中断4、5、6、7
 */
#define NVIC_EN0_R    (*(volatile uint32_t *)0xE000E100)
#define NVIC_PRI1_R   (*(volatile uint32_t *)0xE000E404)

/*
 *  配置UART1对应的stream buffer
 */
#define UART1_RX_STREAM_BUFFER_SIZE  512
#define UART1_RX_TRIGGER_LEVEL 1

/*
 *  UART0配置
 */
#define UART0_BASE          0x4000C000
#define UART0_DR_R          (*(volatile unsigned int *)(UART0_BASE + 0x00))
#define UART0_FR_R          (*(volatile unsigned int *)(UART0_BASE + 0x18))
#define UART0_LCRH_R        (*(volatile uint32_t *)(UART0_BASE + 0x02C))
#define UART0_CTL_R         (*(volatile uint32_t *)(UART0_BASE + 0x030))
#define UART0_IBRD_R        (*(volatile uint32_t *)(UART0_BASE + 0x024))
#define UART0_FBRD_R        (*(volatile uint32_t *)(UART0_BASE + 0x028))
#define UART0_IFLS_R        (*(volatile uint32_t *)(UART0_BASE + 0x034))
#define UART0_IM_R          (*(volatile uint32_t *)(UART0_BASE + 0x038))
#define UART0_IC_R          (*(volatile uint32_t *)(UART0_BASE + 0x044))
#define UART0_MIS_R         (*(volatile uint32_t *)(UART0_BASE + 0x040))
#define UART0_RCC_R         (*(volatile uint32_t *)0x400FE104)
#define UART0_RCC_GPIOA_R   (*(volatile uint32_t *)0x400FE108)
#define UART0_GPIOA_DEN_R   (*(volatile uint32_t *)0x4000451C)
#define UART0_GPIOA_AFSEL_R (*(volatile uint32_t *)0x40004420)

/*
 *  通过UART0打印字符串的函数。
 */
void Puts_UART0(const char *s);

/*
 *  初始化UART1。负责开启UART1的外设时钟，配置UART1中断，配置NVIC中断。开启GPIOD的外设时钟并配置引脚复用。
 *  UART1负责读取传感器数据和发送控制指令，中断优先级高于UART0，但仍处于freeRTOS可控的范围内。
 */
void Init_UART1_Interrupt(void);

/*
 *  初始化UART0。负责开启UART0的外设时钟，配置UART0中断，配置NVIC中断。开启GPIOA的外设时钟并配置引脚复用。
 *  此串口仅负责打印消息至终端，中断优先级低于UART1
 */
void Init_UART0_Interrupt(void);

/*
 *  声明UART1收到数据后用于存放数据的Buffer，传感器数据处理任务会在此Buffer中读取数据。
 */
extern StreamBufferHandle_t Uart1RxStreamBuffer;

/*
 *  声明UART0发送FIFO满足发送需求（低于发送阈值）的二值信号量
 */
extern SemaphoreHandle_t Uart0TxSem;

#endif /* #ifndef __UART_H__ */