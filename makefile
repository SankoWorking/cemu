CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
QEMU = qemu-system-arm

CFLAGS = -mcpu=cortex-m4 -mthumb -nostartfiles -ffreestanding -g -O0
LDFLAGS = -T link_min.ld 

SRCS = startup.s main.c
TARGET = firmware.elf

.PHONY: all clean run debug

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -o $(TARGET)
	@echo "complied $(TARGET)"

run: $(TARGET)
	$(QEMU) -M lm3s6965evb -kernel $(TARGET) -nographic

debug: $(TARGET)
	$(QEMU) -M lm3s6965evb -kernel $(TARGET) -nographic -s -S

clean:
	rm -f $(TARGET)
