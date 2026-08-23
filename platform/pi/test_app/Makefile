# Build the standalone IMU calibration / streaming / test binaries.
#
# The root cause of "undefined reference to imu_config_set_dr_defaults /
# imu_pipeline_init / imu_cal_state_name / imu_pipeline_process /
# imu_wire_encode" when compiling imu_streamer.c alone is that those
# symbols are DEFINED in imu_core.c and imu_wire.c. Headers only carry
# declarations; the linker needs the .c files (or object files) too.
#
# ============================ ARCHITECTURE ==============================
# The "Syntax error: (" unexpected" shell error at runtime means the ELF
# arch doesn't match the kernel — the shell falls back to reading the
# binary as a script and chokes on the first byte after the magic. The
# usual causes on a Pi are:
#   - Built 64-bit (aarch64) but the Pi runs 32-bit Raspberry Pi OS (armv7l)
#   - Built 32-bit (armhf)   but the Pi runs 64-bit Raspberry Pi OS (aarch64)
# Check with `uname -m` on the Pi and `file <binary>` on the build host.
#
# Convenience targets pick the right toolchain up-front:
#   make armhf        # 32-bit ARM hard-float — Raspberry Pi OS 32-bit (armv7l)
#   make aarch64      # 64-bit ARM — Raspberry Pi OS 64-bit
#   make native       # whatever gcc is on this box (default alias below)
#
# Or set the toolchain yourself:
#   make                                              # native gcc
#   make CROSS_COMPILE=arm-linux-gnueabihf-           # 32-bit armhf
#   make CROSS_COMPILE=aarch64-linux-gnu-             # 64-bit aarch64
#
# Other targets:
#   make imu_streamer # build a single program
#   make test         # build and run test_imu_core
#   make clean        # remove all binaries, objects, and dep files
#
# Install the cross toolchains once (on Debian/Ubuntu/Raspberry Pi OS):
#   sudo apt install gcc-arm-linux-gnueabihf   # for armhf target
#   sudo apt install gcc-aarch64-linux-gnu     # for aarch64 target
# ========================================================================

CROSS_COMPILE ?=
CC       ?= $(CROSS_COMPILE)gcc
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

# ---- Per-target-architecture convenience targets ----
# These force the cross-compiler AND wipe stale object files from a previous
# arch build, so the resulting binaries are guaranteed to match the label.
# (Without the clean, %.o from an aarch64 build would silently link into an
# armhf binary and produce an ELF the linker refuses.)
armhf:
	$(MAKE) clean
	$(MAKE) CROSS_COMPILE=arm-linux-gnueabihf- all

aarch64:
	$(MAKE) clean
	$(MAKE) CROSS_COMPILE=aarch64-linux-gnu- all

native:
	$(MAKE) clean
	$(MAKE) CROSS_COMPILE= all

clean:
	rm -f $(BINS) *.o *.d

# Auto-generated header-dependency includes (from -MMD -MP above).
-include $(LIB_OBJS:.o=.d) imu_streamer.d imu_simulator.d test_imu_core.d

.PHONY: all clean test armhf aarch64 native
