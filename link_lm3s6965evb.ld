ENTRY(_start)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 1024K
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
	.text : {
		. = ALIGN(4);
		KEEP(*(.isr_vector))
		. = ALIGN(4);
		*(.text*)
		*(.rodata*)
		_etext = .;
	} > FLASH

	_sidata = LOADADDR(.data);

	.data : {
		. = ALIGN(4);
		_sdata = .;
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} > RAM AT > FLASH

	.bss : {
		. = ALIGN(4);
		_sbss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} > RAM

	/DISCARD/ : {
		*(.ARM.attributes)
	}
}
