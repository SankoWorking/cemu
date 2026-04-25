#ifndef UART_H
#define UART_H

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>

#define UART0_BASE 0x4000C000
#define UART_DR (*(volatile unsigned int *)(UART0_BASE + 0x00))
#define UART_FR (*(volatile unsigned int *)(UART0_BASE + 0x18))

void Putc_UART(char c);
void Puts_UART(const char *s);
void Putu_UART(uint32_t uint32);

#endif