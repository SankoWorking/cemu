#include "uart.h"

void Putc_UART(char c) {
	// uint32_t timeout = 10000;
	while ( UART_FR & (1 << 5) );
	UART_DR = c;
}

void Puts_UART(const char *s) {
	while (*s) {
		Putc_UART(*s++);
	}
}

    