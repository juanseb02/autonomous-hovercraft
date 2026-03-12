MCU     = atmega328p
F_CPU   = 16000000UL
CC      = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude
PORT    = /dev/ttyUSB0

TARGET  = hovercraft
SRC_DIR = src
INC_DIR = include

SRCS    = $(SRC_DIR)/main.c \
          $(SRC_DIR)/imu.c \
          $(SRC_DIR)/sensors.c \
          $(SRC_DIR)/pid.c \
          $(SRC_DIR)/motion.c

OBJS    = $(SRCS:.c=.o)

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -I$(INC_DIR) -Os -Wall -Wextra -std=c11

all: $(TARGET).hex

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(TARGET).hex
	$(AVRDUDE) -c arduino -p $(MCU) -P $(PORT) -b 115200 -U flash:w:$<

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).hex

.PHONY: all flash clean

