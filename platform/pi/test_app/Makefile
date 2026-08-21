# Build the standalone IMU calibration / streaming / test binaries.
#
# The root cause of "undefined reference to imu_config_set_dr_defaults /
# imu_pipeline_init / imu_cal_state_name / imu_pipeline_process /
# imu_wire_encode" when compiling imu_streamer.c alone is that those
# symbols are DEFINED in imu_core.c and imu_wire.c. Headers only carry
# declarations; the linker needs the .c files (or object files) too.
#
# Usage:
#   make              # build all three binaries: imu_streamer, imu_simulator, test_imu_core
#   make imu_streamer # build a single target
#   make test         # build and run test_imu_core
#   make clean        # remove all binaries and .o files
#
# Cross-compile note (host x86 -> Pi ARM): override CC on the command line
#   make CC=arm-linux-gnueabihf-gcc

CC       ?= gcc
CFLAGS   ?= -O2 -Wall -Wextra -Wpedantic -std=c11 -D_GNU_SOURCE
LDFLAGS  ?=
LDLIBS   ?= -lm -pthread

# Shared library sources — every program in this directory links against
# both. Keeping them in one variable means adding a new program only needs
# a new target-source line at the bottom.
LIB_SRCS := imu_core.c imu_wire.c
LIB_OBJS := $(LIB_SRCS:.c=.o)

BINS := imu_streamer imu_simulator test_imu_core

all: $(BINS)

# Pattern rule for object files (picks up header changes via -MMD dep files).
%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c -o $@ $<

# Per-target program rules. Each program is its own main.c plus the shared
# LIB_OBJS. Adding a new program: add a new "foo: foo.o $(LIB_OBJS)" line.
imu_streamer: imu_streamer.o $(LIB_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

imu_simulator: imu_simulator.o $(LIB_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

test_imu_core: test_imu_core.o $(LIB_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

# Convenience: build-and-run the unit tests.
test: test_imu_core
	./test_imu_core

clean:
	rm -f $(BINS) *.o *.d

# Auto-generated header-dependency includes (from -MMD -MP above).
-include $(LIB_OBJS:.o=.d) imu_streamer.d imu_simulator.d test_imu_core.d

.PHONY: all clean test
