// SPDX-License-Identifier: GPL-2.0

/**
 * @file mpu6050_ioctl.h
 * @brief Userspace ABI for MPU6050 character driver.
 */
#ifndef _MPU6050_IOCTL_H_
#define _MPU6050_IOCTL_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define MPU6050_CHARDEV_NAME "mpu6050"

/** accel/gyro full-scale enums (match driver) */
typedef enum { ACCEL_2G = 0, ACCEL_4G = 1, ACCEL_8G = 2, ACCEL_16G = 3 } mpu6050_accel_fs;
typedef enum { GYRO_250DPS = 0, GYRO_500DPS = 1, GYRO_1000DPS = 2, GYRO_2000DPS = 3 } mpu6050_gyro_fs;

/** One sample frame (raw + corrected) */
struct mpu6050_sample {
    __s64 t_ns;  /**< monotonic boot time when captured */
    __s16 ax, ay, az; /**< raw accel LSB*/
    __s16 gx, gy, gz; /**< raw gyro LSB */
    __s16 temp; /**< raw temp LSB */
    float ax_corr, ay_corr, az_corr; /**< calibrated accel */
    float gx_corr, gy_corr, gz_corr; /**< calibrated gyro */
}__attribute__((packed));

/** full-scale configuration */
struct mpu6050_fs {
    __u32 accel; /**< enum mpu6050_accel_fs */
    __u32 gyro;  /**< enum mpu6050_gyro_fs */
};

/** affine calibration structure  */
struct mpu6050_cal {
    float bias[3];  /**< bias for x,y,z */
    float scale[3]; /**< scale for x,y,z */
};
/** calibration pair for accel and gyro */
struct mpu6050_cal_pair {
    struct mpu6050_cal accel; /**< accel calibration */
    struct mpu6050_cal gyro;  /**< gyro calibration */
};

/** Stationary calibration request  */
struct mpu6050_cal_req {
    __u32 duration_ms; /**< averaging window in ms */
    float expect_g_raw[3]; /**< expected accel vector in raw LSB (e.g. [0,0,+1g]) */
};

/* IOCTL commands */
#define MPU6050_IOC_MAGIC 'M'
#define MPU6050_IOC_GET_WHOAMI    _IOR(MPU6050_IOC_MAGIC, 0x01, __u32)
#define MPU6050_IOC_GET_ODR      _IOR(MPU6050_IOC_MAGIC, 0x02, __u32)
#define MPU6050_IOC_SET_ODR      _IOW(MPU6050_IOC_MAGIC, 0x03, __u32)
#define MPU6050_IOC_GET_FS       _IOR(MPU6050_IOC_MAGIC, 0x04, struct mpu6050_fs)
#define MPU6050_IOC_SET_FS       _IOW(MPU6050_IOC_MAGIC, 0x05, struct mpu6050_fs)
#define MPU6050_IOC_GET_CAL      _IOR(MPU6050_IOC_MAGIC, 0x06, struct mpu6050_cal_pair)
#define MPU6050_IOC_SET_CAL      _IOW(MPU6050_IOC_MAGIC, 0x07, struct mpu6050_cal_pair)
#define MPU6050_IOC_RUN_CAL_STATIONARY _IOWR(MPU6050_IOC_MAGIC, 0x08, struct mpu6050_cal_req)

#endif // _MPU6050_IOCTL_H_