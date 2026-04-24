#ifndef UART_H
#define UART_H

#define UART0_BASE 0x4000C000
#define UART_DR (*(volatile unsigned int *)(UART0_BASE + 0x00))
#define UART_FR (*(volatile unsigned int *)(UART0_BASE + 0x18))

void uart_putc(char c);
void uart_puts(const char *s);

#endif