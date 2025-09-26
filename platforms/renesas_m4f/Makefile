# Makefile for Renesas Cortex-M4F Dead Reckoning System

# Target device configuration
DEVICE ?= RX65N
BOARD ?= RX65N_RSK

# Toolchain configuration
CROSS_COMPILE ?= rx-elf-
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size

# Directories
SRC_DIR = src
INC_DIR = inc
BUILD_DIR = build
CONFIG_DIR = config
SCRIPTS_DIR = scripts

# Core source files
CORE_SRCS = $(wildcard $(SRC_DIR)/*.c)
SENSOR_SRCS = $(wildcard $(SRC_DIR)/sensors/*.c)
HAL_SRCS = $(wildcard $(SRC_DIR)/hal/*.c)

SRCS = $(CORE_SRCS) $(SENSOR_SRCS) $(HAL_SRCS)
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)

# Include paths
INCLUDES = -I$(INC_DIR) \
           -I$(CONFIG_DIR) \
           -I$(SRC_DIR) \
           -I$(SRC_DIR)/sensors \
           -I$(SRC_DIR)/hal

# Compiler flags
CFLAGS = -mcpu=rx600 \
         -mlittle-endian-data \
         -mno-allow-string-insns \
         -Wall -Wextra \
         -ffunction-sections \
         -fdata-sections \
         -std=c99 \
         $(INCLUDES)

# Debug/Release configuration
ifdef DEBUG
    CFLAGS += -g -O0 -DDEBUG
    BUILD_TYPE = debug
else
    CFLAGS += -O2 -DNDEBUG
    BUILD_TYPE = release
endif

# Linker flags
LDFLAGS = -mcpu=rx600 \
          -mlittle-endian-data \
          -Wl,--gc-sections \
          -T $(SCRIPTS_DIR)/linker_$(DEVICE).ld

# Libraries
LIBS = -lm -lc

# Target names
TARGET = dr_vehicle_$(DEVICE)
ELF = $(BUILD_DIR)/$(TARGET).elf
HEX = $(BUILD_DIR)/$(TARGET).hex
BIN = $(BUILD_DIR)/$(TARGET).bin

# Default target
all: $(ELF) $(HEX) $(BIN) size

# Create build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/$(SRC_DIR)
	@mkdir -p $(BUILD_DIR)/$(SRC_DIR)/sensors
	@mkdir -p $(BUILD_DIR)/$(SRC_DIR)/hal

# Compile source files
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# Link executable
$(ELF): $(OBJS)
	@echo "LD $@"
	@$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

# Generate Intel HEX file
$(HEX): $(ELF)
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O ihex $< $@

# Generate binary file
$(BIN): $(ELF)
	@echo "OBJCOPY $@"
	@$(OBJCOPY) -O binary $< $@

# Show size information
size: $(ELF)
	@echo "Size information:"
	@$(SIZE) $<

# Generate disassembly
disasm: $(ELF)
	@$(OBJDUMP) -d $< > $(BUILD_DIR)/$(TARGET).dis

# Clean build files
clean:
	@echo "Cleaning build files..."
	@rm -rf $(BUILD_DIR)

# Flash firmware to device
flash: $(HEX)
	@echo "Flashing $(HEX) to $(DEVICE)..."
	@$(SCRIPTS_DIR)/flash.sh $(HEX) $(DEVICE)

# Debug with GDB
debug: $(ELF)
	@echo "Starting GDB debug session..."
	@$(CROSS_COMPILE)gdb $<

# Configuration targets
config:
	@echo "Configuring for device: $(DEVICE)"
	@echo "Board: $(BOARD)"
	@cp $(CONFIG_DIR)/$(DEVICE)_config.h $(INC_DIR)/device_config.h

# Help
help:
	@echo "Dead Reckoning System - Renesas Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all      - Build firmware (default)"
	@echo "  clean    - Clean build files"
	@echo "  flash    - Flash firmware to device"
	@echo "  debug    - Start GDB debug session"
	@echo "  config   - Configure for target device"
	@echo "  disasm   - Generate disassembly listing"
	@echo "  size     - Show memory usage"
	@echo "  help     - Show this help"
	@echo ""
	@echo "Variables:"
	@echo "  DEVICE   - Target device (default: RX65N)"
	@echo "  BOARD    - Target board (default: RX65N_RSK)"
	@echo "  DEBUG    - Enable debug build (DEBUG=1)"
	@echo ""
	@echo "Examples:"
	@echo "  make DEVICE=RX72N"
	@echo "  make DEBUG=1"
	@echo "  make flash DEVICE=RX65N"

# Include dependency files
-include $(OBJS:.o=.d)

.PHONY: all clean flash debug config disasm size help