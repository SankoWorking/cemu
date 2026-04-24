#include "uart.h"

void uart_putc(char c) {
	//while (!(UART_FR & (1 << 5)));
	UART_DR = c;
}

void uart_puts(const char *s) {
	while (*s) {
		uart_putc(*s++);
	}
}