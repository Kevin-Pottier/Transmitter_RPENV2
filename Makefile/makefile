################################################################################
# Automatically-generated file.
# Edit it at your own risk.
################################################################################

# Folders
SRC_DIRS := ../Drivers/src ../Core
INC_DIRS := ../Drivers/inc ../Core
OBJ_DIR := ../bin/__objects
BIN_DIR := ../bin

# MCU & toolchain
MCU := #TBD
CC := arm-none-eabi-gcc
LD := arm-none-eabi-ld
CFLAGS := -mcpu=$(MCU) -mthumb -Wall $(addprefix -I,$(INC_DIRS))
LDFLAGS := -Tlinker.ld

RM := rm -rf

# Sources & objects
SRCS := $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c))
OBJS := $(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SRCS)))

#Target
TARGET := $(BIN_DIR)/Transmitter.elf

LINKER_SCRIPT := linker.ld

ifeq ($(wildcard $(LINKER_SCRIPT)),)
all:
	@$(MAKE) fail-specified-linker-script-missing
else
all: $(OBJ_DIR) $(TARGET)
endif

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Compilation rules for each source folder
$(OBJ_DIR)/%.o: ../Drivers/src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: ../Core/%.c
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
