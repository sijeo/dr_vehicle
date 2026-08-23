# Build standalone IMU calibration / streaming / test binaries
# Target: 32-bit Raspberry Pi OS / armhf / armv7l

# ----------------------------------------------------------------------
# Toolchain
# ----------------------------------------------------------------------
# Default cross compiler for 32-bit Raspberry Pi OS.
CROSS_COMPILE ?= arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc

# Optional sysroot.
# If you have a valid Raspberry Pi root filesystem/sysroot, build like:
#
#   make SYSROOT=/path/to/raspi/sysroot
#
# If SYSROOT is empty, no --sysroot flag is used.
SYSROOT ?= $(HOME)/rpi_work/rpi_tools/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/arm-linux-gnueabihf/sysroot

ifeq ($(strip $(SYSROOT)),)
SYSROOT_FLAGS :=
else
SYSROOT_FLAGS := --sysroot=$(SYSROOT)
endif

# ----------------------------------------------------------------------
# Flags
# ----------------------------------------------------------------------
CFLAGS ?= -Wall -Wextra -O2 -g
CFLAGS += $(SYSROOT_FLAGS)

# Raspberry Pi 32-bit safe defaults.
# arm-linux-gnueabihf-gcc already implies hard-float ABI.
CFLAGS += -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=hard

LDFLAGS ?=
LDFLAGS += $(SYSROOT_FLAGS)

LDLIBS := -lm -lpthread

RM ?= rm -f

# ----------------------------------------------------------------------
# Sources / objects
# ----------------------------------------------------------------------
LIB_SRCS := imu_core.c imu_wire.c
LIB_OBJS := $(LIB_SRCS:.c=.o)

BINS := imu_streamer imu_simulator test_imu_core

all: $(BINS)

# ----------------------------------------------------------------------
# Compile object files
# ----------------------------------------------------------------------
%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c -o $@ $<

# ----------------------------------------------------------------------
# Link programs
# ----------------------------------------------------------------------
imu_streamer: imu_streamer.o $(LIB_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

imu_simulator: imu_simulator.o $(LIB_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

test_imu_core: test_imu_core.o $(LIB_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

# ----------------------------------------------------------------------
# Convenience targets
# ----------------------------------------------------------------------
armhf: clean
	$(MAKE) CROSS_COMPILE=arm-linux-gnueabihf- all

native: clean
	$(MAKE) CROSS_COMPILE= all

test: test_imu_core
	./test_imu_core

clean:
	$(RM) $(BINS) *.o *.d

# ----------------------------------------------------------------------
# Dependency files
# ----------------------------------------------------------------------
-include $(LIB_OBJS:.o=.d) imu_streamer.d imu_simulator.d test_imu_core.d

.PHONY: all clean test armhf native
