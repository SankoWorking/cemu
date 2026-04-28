#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "task_sensors.h"
#include "queue.h"

#define UART1_BASE 0x4000D000

#define UART1_DR_R (*(volatile unsigned int *)(UART1_BASE + 0x00))
#define UART1_FR_R (*(volatile unsigned int *)(UART1_BASE + 0x18))
#define UART1_LCRH_R        (*(volatile uint32_t *)(UART1_BASE + 0x02C)) // 线控制（数据格式/FIFO）
#define UART1_CTL_R         (*(volatile uint32_t *)(UART1_BASE + 0x030)) // 控制寄存器
#define UART1_IBRD_R        (*(volatile uint32_t *)(UART1_BASE + 0x024))
#define UART1_FBRD_R        (*(volatile uint32_t *)(UART1_BASE + 0x028))
#define UART1_IFLS_R        (*(volatile uint32_t *)(UART1_BASE + 0x034)) // 中断级别选择 (阈值)
#define UART1_IM_R          (*(volatile uint32_t *)(UART1_BASE + 0x038)) // 中断屏蔽 (使能中断)
#define UART1_IC_R         (*(volatile uint32_t *)(UART1_BASE + 0x044)) // 中断清除

#define UART0_BASE 0x4000C000
#define UART0_DR_R (*(volatile unsigned int *)(UART0_BASE + 0x00))
#define UART0_FR_R (*(volatile unsigned int *)(UART0_BASE + 0x18))
#define UART0_IFLS_R        (*(volatile uint32_t *)(UART0_BASE + 0x034)) // 中断级别选择 (阈值)
#define UART0_IM_R          (*(volatile uint32_t *)(UART0_BASE + 0x038)) // 中断屏蔽 (使能中断)
#define UART0_IC_R         (*(volatile uint32_t *)(UART0_BASE + 0x044)) // 中断清除

#define NVIC_EN0_R         (*(volatile uint32_t *)0xE000E100) // 中断使能 (UART0 是 5 号中断)
#define NVIC_PRI1_R        (*(volatile uint32_t *)0xE000E404) // 优先级配置



void Putc_UART(char c);
void Puts_UART(const char *s);
void Init_UART1_Interrupt(void);

#endif