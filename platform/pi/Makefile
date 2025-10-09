obj-m := mpu6050_iio_driver.o

KDIR := $(HOME)/rpi_work/kernel_rpi/linux
CROSS_COMPILE ?= arm-linux-gnueabihf-
SYSROOT ?= $(HOME)/rpi_work/rpi_tools/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/arm-linux-gnueabihf/sysroot
CC := $(CROSS_COMPILE)gcc

all:
	$(MAKE) -C $(KDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean
	$(RM) test_app/test_mpu6050_iio

# Build user-space app without kernel include paths
app: test_app/test_mpu6050_iio.c
	$(CC) --sysroot=$(SYSROOT) -O2 -Wall -Wextra -o test_app/test_mpu6050_iio $<

deploy:
	scp *.ko sijeo@192.168.1.2:/home/sijeo/dr_vehicle/

deploy_app:
	scp test_app/test_mpu6050_iio sijeo@192.168.1.2:/home/sijeo/dr_vehicle/

.PHONY: all clean app