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

/* ---------------------------------------------------------------------
 * Extended sample (ISM330BX / LSM6DSV family only).
 *
 * Selected per open file with MPU6050_IOC_SET_READ_FMT(MPU6050_FMT_EXT).
 * The default format stays struct mpu6050_sample so existing readers
 * (DR app, tools) are unaffected.
 *
 * SFLP fields hold the most recent output of the on-chip sensor-fusion
 * low-power block at the time the accel/gyro pair was produced:
 *   quat_h   game rotation vector x,y,z as IEEE-754 half floats;
 *            w = sqrt(1 - x^2 - y^2 - z^2) (ST reference driver sflp2q()).
 *   gravity  gravity vector, sensor frame, 0.061 mg/LSB.
 *   gbias    SFLP gyro bias estimate, sensor frame, 4.375 mdps/LSB.
 * ------------------------------------------------------------------- */
#define MPU6050_FMT_LEGACY          0u
#define MPU6050_FMT_EXT             1u

#define MPU6050_EXT_QUAT_VALID      (1u << 0)
#define MPU6050_EXT_GRAVITY_VALID   (1u << 1)
#define MPU6050_EXT_GBIAS_VALID     (1u << 2)
#define MPU6050_EXT_FIFO_STREAM     (1u << 3)  /**< sample came from the HW FIFO */
#define MPU6050_EXT_FIFO_OVERRUN    (1u << 4)  /**< HW FIFO overran before this sample */
#define MPU6050_EXT_SAMPLE_DROPPED  (1u << 5)  /**< samples lost before this one */

struct mpu6050_ext_sample {
    __s64 t_ns;            /**< CLOCK_BOOTTIME of the sample (FIFO: reconstructed, regular spacing) */
    __u32 seq;             /**< driver sample counter */
    __u32 flags;           /**< MPU6050_EXT_* */
    __s16 ax, ay, az;      /**< raw accel LSB, sensor frame */
    __s16 gx, gy, gz;      /**< raw gyro LSB, sensor frame */
    __s16 temp;            /**< raw temp, 256 LSB/degC, 0 = 25 degC */
    __u16 quat_h[3];       /**< SFLP game rotation x,y,z (half float) */
    __s16 gravity[3];      /**< SFLP gravity, 0.061 mg/LSB */
    __s16 gbias[3];        /**< SFLP gyro bias, 4.375 mdps/LSB */
    __u16 reserved;
    __u32 sflp_seq;        /**< increments on every new quaternion */
    __u32 dropped;         /**< cumulative lost samples */
} __attribute__((packed));

/** On-chip filter / SFLP / FIFO configuration (ISM330BX). */
struct mpu6050_filter_cfg {
    __s32 gyro_lpf1_bw;    /**< -1 = LPF1 off, 0..7 = CTRL6.LPF1_G_BW (7 narrowest) */
    __s32 accel_lpf2_bw;   /**< -1 = LPF2 off, 0..7 = CTRL8.HP_LPF2_XL_BW (ODR/4 .. ODR/800) */
    __u32 sflp_enable;     /**< 1 = run SFLP game rotation vector */
    __u32 sflp_odr_hz;     /**< 15/30/60/120/240/480, clamped to <= sensor ODR (GET returns actual) */
    __u32 sflp_fifo_mask;  /**< bit0 quaternion, bit1 gravity, bit2 gyro bias */
    __u32 fifo_stream;     /**< 1 = accel/gyro read from the HW FIFO (no lost samples) */
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
#define MPU6050_IOC_SET_READ_FMT _IOW(MPU6050_IOC_MAGIC, 0x09, __u32)
#define MPU6050_IOC_GET_FILTER   _IOR(MPU6050_IOC_MAGIC, 0x0A, struct mpu6050_filter_cfg)
#define MPU6050_IOC_SET_FILTER   _IOW(MPU6050_IOC_MAGIC, 0x0B, struct mpu6050_filter_cfg)
#define MPU6050_IOC_SFLP_RESET   _IO(MPU6050_IOC_MAGIC, 0x0C)

#endif // _MPU6050_IOCTL_H_