#ifndef IMU_FUSION_H
#define IMU_FUSION_H

/*
 * Attitude + gravity-compensation stage that sits after imu_core.
 *
 *   - Decodes the ISM330BX SFLP outputs delivered by the driver
 *     (game rotation vector, gravity vector, gyro bias).
 *   - Runs a host-side Mahony AHRS on the imu_core output as an
 *     independent reference (and fallback when SFLP is unavailable).
 *   - Removes gravity from the accelerometer so dead reckoning gets
 *     specific force in the vehicle frame and in the heading-free
 *     level frame.
 *
 * Frames: vehicle = FLU (x forward, y left, z up). World/level = z up,
 * yaw origin arbitrary (no magnetometer). Quaternions are (w, x, y, z)
 * and rotate vehicle vectors into the world frame: v_w = q v_b q*.
 * Euler angles follow REP-103 (ZYX): roll + = right side down,
 * pitch + = nose down, yaw + = counter-clockwise (left turn).
 */

#include "imu_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Low bits mirror MPU6050_EXT_* from mpu6050_ioctl.h. */
enum {
    IMU_FF_SFLP_VALID      = 1U << 0,
    IMU_FF_GRAVITY_VALID   = 1U << 1,
    IMU_FF_GBIAS_VALID     = 1U << 2,
    IMU_FF_FIFO_STREAM     = 1U << 3,
    IMU_FF_FIFO_OVERRUN    = 1U << 4,
    IMU_FF_SAMPLE_DROPPED  = 1U << 5,
    IMU_FF_AHRS_VALID      = 1U << 8,
    IMU_FF_CONV_DECIDED    = 1U << 9,   /* SFLP quaternion convention confirmed */
    IMU_FF_CONV_CONJUGATE  = 1U << 10,  /* ... and it needed conjugating */
    IMU_FF_ACCEL_TRUSTED   = 1U << 11,  /* AHRS applied accel correction */
    IMU_FF_LEGACY_DRIVER   = 1U << 12,  /* driver has no extended format */
    IMU_FF_LIN_FROM_SFLP   = 1U << 13,  /* linear accel used the SFLP attitude */
    IMU_FF_SFLP_SUSPECT    = 1U << 14,  /* SFLP attitude disagrees with gravity; AHRS used */
    IMU_FF_AHRS_SFLP_BIAS  = 1U << 15   /* AHRS gyro used the SFLP bias (tracks temperature) */
};

/* Driver-side extras for one sample (copied from mpu6050_ext_sample). */
typedef struct {
    bool     have_ext;
    uint32_t flags;          /* MPU6050_EXT_* */
    uint32_t seq;
    uint32_t dropped;
    uint32_t sflp_seq;
    uint16_t quat_h[3];      /* half floats x, y, z */
    int16_t  gravity[3];     /* 0.061 mg/LSB */
    int16_t  gbias[3];       /* 4.375 mdps/LSB */
    int16_t  temp;           /* 256 LSB/degC, 0 = 25 degC */
} imu_sflp_input_t;

typedef struct {
    float q[4];
    float integral[3];
    float kp;
    float ki;
    bool  initialized;
} imu_mahony_t;

typedef struct {
    float mount_R_bv[3][3];
    imu_mahony_t ahrs;
    uint64_t last_ns;

    /* SFLP quaternion convention check (sensor->world vs world->sensor). */
    uint32_t conv_n;
    double   conv_err_direct;
    double   conv_err_conj;
    int      conv;           /* 0 = undecided (direct used), 1 = direct, 2 = conjugate */

    /* SFLP health: running angle between the quaternion's gravity and the
     * SFLP gravity output (or the accelerometer at rest). */
    uint32_t sflp_err_n;
    float    sflp_err_ema_deg;
} imu_fusion_t;

typedef struct {
    uint32_t flags;          /* IMU_FF_* */
    uint32_t sflp_seq;
    uint32_t driver_seq;
    uint32_t driver_dropped;

    float q_sflp[4];
    float euler_sflp_deg[3];         /* roll, pitch, yaw */
    float q_ahrs[4];
    float euler_ahrs_deg[3];
    float gravity_sflp_vehicle_mps2[3];
    float gbias_sflp_dps[3];         /* vehicle frame */
    float lin_acc_vehicle_mps2[3];   /* filtered accel minus gravity, FLU */
    float lin_acc_level_mps2[3];     /* forward-horizontal, left-horizontal, up */
    float temp_c;
    float g_ref_mps2;
    float tilt_consistency_deg;      /* angle(accel, attitude-predicted gravity) */
    float sflp_consistency_deg;      /* SFLP quaternion vs accelerometer at rest, smoothed; > 10 = suspect */
} imu_fusion_output_t;

#define IMU_FUSION_FLOAT_COUNT 30u

float imu_half_to_float(uint16_t h);
void  imu_quat_to_euler_deg(const float q[4], float euler_deg[3]);

/* mount_R_bv is only copied. It is not declared const because C will not
 * convert float (*)[3] to const float (*)[3] implicitly (GCC warns). */
void imu_fusion_init(imu_fusion_t *f, float mount_R_bv[3][3], float kp, float ki);

/* Replace the sensor->vehicle rotation at runtime (mount calibration).
 * Restarts the AHRS and the SFLP checks, which depend on the frame. */
void imu_fusion_set_mount(imu_fusion_t *f, float mount_R_bv[3][3]);

void imu_fusion_update(imu_fusion_t *f,
                       const imu_sflp_input_t *in,
                       const imu_output_t *core,
                       float g_ref_mps2,
                       imu_fusion_output_t *out);

#ifdef __cplusplus
}
#endif
#endif
