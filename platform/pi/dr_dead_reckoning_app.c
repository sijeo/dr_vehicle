// dr_dead_reckoning_app.c
//
// RPi3B + MPU6050 + Neo6M dead-reckoning navigation app
// - Waits for first GNSS fix -> sets ENU origin
// - Attitude EKF (quaternion) from IMU
// - Position/velocity EKF (6-State) in ENU
// - GNSS position + velocity + yaw (course over ground )
// - HDOP-adaptive GNSS covariance
// - NHC, ZUPT, velocity decay
// - Q inflation during GNSS outrage
// - Fade-in of GNSS after outage
//
// NOTE: This file expects these headers to exist and be implemented:
// - mpu6050_ioctl.h    (with struct mpu6050_sample, device path, IOCTLs)
// - neo6m_gnss_ioctl.h (with struct neo6m_gnss_fix, IOCTL NEO6M_GNSS_IOC_GET_FIX)
// - dr_math.h / dr_types.h (quaternions, matrices, vector helpers )
//
// 

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>

// ================Project Headers (adjust path as needed )============== //
#include "mpu6050_ioctl.h"
#include "neo6m_gnss_ioctl.h"
#include "../core/dr_math.h"
#include "../core/dr_types.h"

//==========Device Paths (adjust if needed )=======//
#define IMU_DEVICE_PATH     "/dev/mpu6050-0"
#define NEO6M_DEVICE_PATH   "/dev/neo6m-0"

//====== Constant & tuning =======//

#define IMU_HZ          100.0f
#define DT_IMU          (1.0/IMU_HZ)
#define GNSS_TIMEOUT_S  2.0f

#define GRAVITY         9.80665f
#define DEG2RAD        (3.14159265358979323846/180.0) 

// Yaw dead-band (when nearly still )
#define YAW_DEADBAND_RAD    (0.03f)          // ~1.7 deg/s
#define ACC_STILL_TOL       (0.1f*GRAVITY)
#define GYRO_STILL_TOL_RAD  (3.0f * DEG2RAD)

// ZUPT (Zero-Velocity update) detection
#define ZUPT_ACC_THR        (0.4f)
#define ZUPT_GYRO_THR       (5.0f*DEG2RAD)
#define ZUPT_COUNT_REQUIRED  5
#define VEL_DECAY            0.98f
#define VEL_EPS              1e-3f

// Adaptive Q Scaling (tiers by outage length)
#define TIER_A      2.0f
#define TIER_B      10.0f
#define TIER_C      60.0f
#define QSCL_A      2.0f
#define QSCL_B      5.0f
#define QSCL_C      10.0f
#define QSCL_D      20.0f

/* GNSS gating and Fade in */
#define CHI2_3DOF_GATE      16.2f       // ~99.5% for 3 DOF
#define R_GNSS_POS_VAR      4.0f        // base position var (m^2) before HDOP scaling
#define FADE_IN_FACT_INIT   4.0f        // R = FADE_IN_FACT_INIT * R_base when GNSS just returns
#define FADE_IN_STEPS       3           // Number of GNSS updates to Fade in

// NHC Measurement Noise (m/s^2)
#define R_NHC_VY    0.01f
#define R_NHC_VZ    0.04f

// ZUPT measurement noise (m/s)^2
#define R_ZUPT_V    (0.0004f)

// GNSS Velocity measurement Noise (m/s)^2
#define RV_VEL_E    (0.09f)     // (0.3 m/s)^2
#define RV_VEL_N    (0.09f)     //
#define RV_VEL_U    (0.36f)     // (0.6 m/s)^2

/* Yaw fusion from GNSS Heading */
#define YAW_FUSION_SPEED_MIN    (1.0f)  // m/s; only fuse heading above this 
#define YAW_FUSION_HDOP_MAX     (4.0f)  // only fuse heading when HDOP <= this
#define YAW_FUSION_ALPHA        (0.05f) // 0..1 fraction of yaw error corrected per update

/** Time Utilities  */
static inline uint64_t monotonic_ns( void ) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + tv.tv_nsec;   
}

static inline double now_sec( void ){
    return monotonic_ns() * 1e-9;
}

/* Attitude EKF (4-state quaternion )*/
typedef struct {
    dr_quatf_t q; /*< body->world*/
    float P[16];  /*< 4x4 Covariance */
} ekf4_t;

static void ekf4_init(ekf4_t *e) {
    e->q = dr_quat_identity();
    memset(e->P, 0, sizeof(e->P));
    /* Small initial uncertainity */
    e->P[0] = e->P[5] = e->P[10] = e->P[15] = 1e-3f;
}

static void quat_deriv( const dr_quatf_t *q, const float w[3], dr_quatf_t *dqdt ){
    float wx = w[0], wy = w[1], wz = w[2];
    dqdt->w = -0.5f * (wx*q->x + wy*q->y + wz*q->z);
    dqdt->x = 0.5f * (wx*q->w + wz*q->y - wy*q->z);
    dqdt->y = 0.5f * (wy*q->w - wz*q->x + wx*q->z);
    dqdt->z = 0.5f * (wz*q->w + wy*q->x - wx*q->y);
}

static void gravity_body_from_q( const dr_quatf_t *q, float gb[3]) {
    dr_quatf_t qc = (dr_quatf_t){q->w, q->x, q->y, q->z};
    dr_vec3f_t gw = dr_v3(0, 0, 1);
    dr_vec3f_t gbv = dr_q_rotate(qc, gw);
    float n = sqrtf(gbv.x*gbv.x + gbv.y*gbv.y + gbv.z*gbv.z);
    gb[0] = gbv.x / n;
    gb[1] = gbv.y / n;
    gb[2] = gbv.z / n;
}

static ekf4_predict( ekf4_t *e, const float gyro[3], float dt ){
    
}