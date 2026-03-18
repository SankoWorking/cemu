.syntax unified
.cpu cortex-m4
.thumb

.global _start
.global Reset_Handler

.section .isr_vector, "a"
.align 2
_start:
	.word 0x20008000
	.word Reset_Handler + 1

.section .text
Reset_Handler:
	bl main
	b .
