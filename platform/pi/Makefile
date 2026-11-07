# Both drivers must be rebuilt whenever their *_ioctl.h header changes: the
# ioctl numbers encode the struct sizes, so a stale .ko answers ENOTTY.
obj-m := mpu6050_char.o neo6m_gnss_serdev.o

KDIR := $(HOME)/rpi_work/kernel_rpi/linux
CROSS_COMPILE ?= arm-linux-gnueabihf-
SYSROOT ?= $(HOME)/rpi_work/rpi_tools/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/arm-linux-gnueabihf/sysroot
CC := $(CROSS_COMPILE)gcc
RM ?= rm -f

CORE_DIR := $(abspath ../../core)
CORE_SRCS := $(wildcard $(CORE_DIR)/*.c)

all:
	$(MAKE) -C $(KDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean
	$(RM) test_app/imu_streamer
	$(RM) dr_app_1
	$(RM) dr_dead_reckoning_app
	$(RM) dr_dead_reckoning_app_ins15

# Build user-space app without kernel include paths
app: dr_dead_reckoning_app.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o dr_dead_reckoning_app $< $(CORE_SRCS) -lm -lpthread

app_v2: dr_dead_reckoning_app_ins15.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o dr_dead_reckoning_app_ins15 $< $(CORE_SRCS) -lm -lpthread

app_v3: dr_dead_reckoning_app_ins15_v2.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o dr_dead_reckoning_app_ins15 $< $(CORE_SRCS) -lm -lpthread

# Final DR application: dr_dead_reckoning_app_ins15_v3.c + dr_nav.c + the
# IMU chain proven in test_app/ (imu_core, imu_fusion, imu_mount, imu_wire).
DR_APP_SRCS := dr_dead_reckoning_app_ins15_v3.c dr_nav.c \
               test_app/imu_core.c test_app/imu_fusion.c test_app/imu_mount.c test_app/imu_wire.c
app_v4: $(DR_APP_SRCS)
	$(CC) --sysroot=$(SYSROOT) -std=gnu99 -O2 -Wall -Wextra -I. -Itest_app -pthread \
		-o dr_dead_reckoning_app $(DR_APP_SRCS) -lm

test_neo6m: test_app/test_neo6m_gnss_serdev.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o test_neo6m_gnss_serdev $< $(CORE_SRCS) -lm -lpthread

deploy:
	scp *.ko sijeo@10.81.69.7:/home/sijeo/dr_vehicle/

deploy_app:
	scp dr_dead_reckoning_app sijeo@10.81.69.7:/home/sijeo/dr_vehicle/

deploy_app_2:
	scp dr_dead_reckoning_app_ins15 sijeo@10.81.69.7:/home/sijeo/dr_vehicle/
	
deploy_test:
	scp test_app/imu_streamer sijeo@10.81.69.7:/home/sijeo/dr_vehicle/

# DR app + loader script + systemd unit (PI_HOST=user@ip to override)
PI_HOST ?= sijeo@10.81.69.7
deploy_dr:
	scp dr_dead_reckoning_app load_dr_stack.sh dr_app.service $(PI_HOST):/home/sijeo/dr_vehicle/

.PHONY: all clean app app_v4 driver_test deploy deploy_app deploy_dr