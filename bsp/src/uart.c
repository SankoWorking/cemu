#include "uart.h"

void Putc_UART(char c) {
	// uint32_t timeout = 10000;
	// while ((UART_FR & (1 << 5)) && timeout--);
	UART_DR = c;
}

void Puts_UART(const char *s) {
	while (*s) {
		Putc_UART(*s++);
	}
}

void Putu_UART(uint32_t uint32){
	char buffer[12];

	sprintf(buffer, "%" PRIu32, uint32);
	Puts_UART(buffer);
}

    