obj-m := mpu6050_char.o

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
	$(RM) test_app/mpu6050_lateral_streamer
	$(RM) dr_app_1

# Build user-space app without kernel include paths
app: dr_app_1.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o dr_app_1 $< $(CORE_SRCS) -lm

driver_test: test_app/mpu6050_lateral_streamer.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o test_app/mpu6050_lateral_streamer $< $(CORE_SRCS) -lm

deploy:
	scp *.ko sijeo@10.225.104.7:/home/sijeo/dr_vehicle/

deploy_app:
	scp dr_app_1 sijeo@10.225.104.7:/home/sijeo/dr_vehicle/

deploy_test:
	scp test_app/mpu6050_lateral_streamer sijeo@10.225.104.7:/home/sijeo/dr_vehicle/
.PHONY: all clean app driver_test deploy deploy_app