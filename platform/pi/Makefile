obj-m := ism330dlctr_char.o

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
	$(RM) test_app/mpu6050_lateral_streamer_v2
	$(RM) dr_app_1
	$(RM) dr_dead_reckoning_app
	$(RM) dr_dead_reckoning_app_ins15

# Build user-space app without kernel include paths
app: dr_dead_reckoning_app.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o dr_dead_reckoning_app $< $(CORE_SRCS) -lm -lpthread

app_v2: dr_dead_reckoning_app_ins15.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o dr_dead_reckoning_app_ins15 $< $(CORE_SRCS) -lm -lpthread


driver_test: test_app/mpu6050_lateral_streamer_v2.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -I$(CORE_DIR) -o test_app/mpu6050_lateral_streamer_v2 $< $(CORE_SRCS) -lm

deploy:
	scp *.ko sijeo@10.72.102.7:/home/sijeo/dr_vehicle/

deploy_app:
	scp dr_dead_reckoning_app sijeo@10.72.102.7:/home/sijeo/dr_vehicle/

deploy_app_2:
	scp dr_dead_reckoning_app_ins15 sijeo@10.72.102.7:/home/sijeo/dr_vehicle/
	
deploy_test:
	scp test_app/mpu6050_lateral_streamer_v2 sijeo@10.72.102.7:/home/sijeo/dr_vehicle/
	
.PHONY: all clean app driver_test deploy deploy_app