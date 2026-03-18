#define UART0_BASE 0x4000C000
#define UART_DR (*(volatile unsigned int *)(UART0_BASE + 0x00))
#define UART_FR (*(volatile unsigned int *)(UART0_BASE + 0x18))

void uart_putc(char c) {
//	while (!(UART_FR & (1 << 5)));
	UART_DR = c;
}

void uart_puts(const char *s) {
	while (*s) {
		uart_putc(*s++);
	}
}

int main(void) {
	uart_puts("Success! Hello from LM3S6965 on QEMU\n");
	while (1){
	}
}
