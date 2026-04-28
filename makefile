TARGET = firmware.elf

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
QEMU = qemu-system-arm

FREERTOS_DIR = FreeRTOS
PORTABLE_DIR = $(FREERTOS_DIR)/portable/GCC/ARM_CM3
MEMMANG_DIR = $(FREERTOS_DIR)/portable/MemMang

C_INCLUDES += \
	-I. \
	-I$(FREERTOS_DIR)/include \
	-I$(PORTABLE_DIR) \
	-Iapp/include \
	-Ibsp/include

C_SOURCES += \
	$(FREERTOS_DIR)/tasks.c \
	$(FREERTOS_DIR)/list.c \
	$(FREERTOS_DIR)/queue.c \
	$(FREERTOS_DIR)/timers.c \
	$(MEMMANG_DIR)/heap_4.c \
	$(PORTABLE_DIR)/port.c \
	$(wildcard app/src/*.c) \
	$(wildcard bsp/src/*.c)

ASM_SOURCES = startup.s

CFLAGS = -mcpu=cortex-m3 $(C_INCLUDES) -mthumb -nostartfiles -ffreestanding -g -O0
LDFLAGS = -T link_lm3s6965evb.ld


.PHONY: all clean run debug

all: $(TARGET)

$(TARGET): $(ASM_SOURCES) $(C_SOURCES)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@
	@echo "Complied $(TARGET) successfully!"

run: $(TARGET)
	$(QEMU) -M lm3s6965evb -kernel $(TARGET) -nographic \
	-chardev udp,id=con0,port=0,localaddr=127.0.0.1,localport=4560 \
	-serial mon:stdio \
	-serial chardev:con0
clean:
	rm -f $(TARGET)
