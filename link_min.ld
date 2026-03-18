ENTRY(_start)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 1024K
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
	.text : {
		. = ALIGN(4);
		KEEP(*(.isr_vector))
		. = ALIGN(4);
		*(.text*)
		*(.rodata*)
	} > FLASH
}
