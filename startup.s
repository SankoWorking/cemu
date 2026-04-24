.syntax unified
.cpu cortex-m4
.thumb

.global _start
.global Reset_Handler
.global vPortSVCHandler
.global xPortPendSVHandler
.global xPortSysTickHandler

.extern _estack
.extern _slidata
.extern _slidata
.extern _edata
.extern _sbss
.extern _ebss

.section .isr_vector, "a"
.align 2
_start:
	.word 0x20008000
	.word Reset_Handler + 1
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word vPortSVCHandler + 1
	.word 0
	.word 0
	.word xPortPendSVHandler + 1
	.word xPortSysTickHandler + 1

.section .text
Reset_Handler:
	ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    mov r3, #0
    b   copy_data_check

copy_data_loop:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4

copy_data_check:
    adds r4, r0, r3
    cmp r4, r1 
    bcc copy_data_loop

    ldr r0, =_sbss
    ldr r1, =_ebss
    mov r2, #0
    b   zero_bss_check

zero_bss_loop:
    str r2, [r0], #4

zero_bss_check:
    cmp r0, r1
    bcc zero_bss_loop

	bl main
	b .
