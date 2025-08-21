################################################################################
# Automatically-generated file.
# Edit it at your own risk.
################################################################################

# Folders
SRC_DIR := ../Drivers/src
INC_DIR := ../Drivers/inc
OBJ_DIR := ../bin/__objects
BIN_DIR := ../bin

# MCU & toolchain
MCU := #TBD
CC := arm-none-eabi-gcc
LD := arm-none-eabi-ld
CFLAGS := -mcpu=$(MCU) -mthumb -Wall -I$(INC_DIR)
LDFLAGS := -T#LinkerPath

RM := rm -rf

# Sources & objects
SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))

#Target
TARGET := $(BIN_DIR)/Transmitter.elf

LINKER_SCRIPT := #LinkerPath

ifeq ($(wildcard $(LINKER_SCRIPT)),)
all:
	@$(MAKE) fail-specified-linker-script-missing
else
all: $(OBJ_DIR) $(TARGET)
endif

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@

clean:
	rm -rf $(OBJ_DIR)/*.o $(TARGET)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean fail-specified-linker-script-missing warn-no-linker-script-specified
