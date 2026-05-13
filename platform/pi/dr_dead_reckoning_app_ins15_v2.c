// dr_dead_reckoning_app_ins15.c
//
// 15-state INS (Error-State EKF) + GNSS (TAU1204) dead-reckoning app for RPi3B + ISM330
//
// Output: ENU only (stdout) AFTER first GNSS fix initializes ENU origin and nav state.
//
// State (nominal):
//   p^e (3)  position in ENU (m)
//   v^e (3)  velocity in ENU (m/s)
//   q^e_b(4) attitude quaternion body->ENU (unit)
//
// Error-state EKF x (15):
//   δp (3), δv (3), δθ (3), b_a (3), b_g (3)
//
// Measurements:
//   - GNSS position: z_p = p^e + noise
//   - GNSS velocity (optional): z_v = v^e + noise
//   - NHC (non-holonomic constraints): v_y^b ≈ 0, v_z^b ≈ 0
//   - ZUPT: v^e ≈ 0 when vehicle is stationary
//
// GNSS outage behavior:
//   - No "reuse last fix": do NOT inject old GNSS.
//   - Keep predicting with IMU mechanization.
//   - Inflate process noise during outage (q_scale tiers).
//   - Keep applying NHC + ZUPT + velocity decay to bound drift.
//   - When GNSS returns: gate by NIS; fade-in measurement covariance for a few fixes.
//
// Logging:
//   - Creates a new CSV file each run under /home/sijeo/nav_logs/ (change LOG_DIR).
//   - Logs (1 Hz): timestamp, ENU p/v/a, raw IMU, GNSS fields, outage_s, NIS, q_scale.
//
// Dependencies:
//   - mpu6050_ioctl.h (struct mpu6050_sample with ax,ay,az,gx,gy,gz raw counts)
//   - neo6m_gnss_ioctl.h (struct neo6m_gnss_fix; reused ABI for TAU1204 dual-band GNSS)
//     If you haven't extended the driver yet, set HAVE_GNSS_HDOP=0 and HAVE_GNSS_HEADING=0 below.
//
// Build (example):
//   gcc -O2 -pthread -lm -o dr_ins15 dr_dead_reckoning_app_ins15.c
//
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/stat.h>

#include "mpu6050_ioctl.h"
#include "neo6m_gnss_ioctl.h"

// ------------------------- Configuration -------------------------

#define IMU_DEVICE_PATH     "/dev/mpu6050-0"
#define NEO6M_DEVICE_PATH   "/dev/neo6m0"

#define IMU_HZ              100.0f
#define DT_IMU_DEFAULT      (1.0f/IMU_HZ)

#define GNSS_TIMEOUT_S      2.0f

// GNSS fusion period: only inject a GNSS update at most this often.
// Intermediate samples are ignored for fusion (they are still logged).
// Why: at 1 Hz, every noisy fix tugs the EKF; at 10 s, predict has time to
// settle between corrections and only the longer-term GNSS trend pulls the
// filter — much less scatter from 1-Hz multipath/timing jitter.
#define GNSS_FUSION_PERIOD_S  10.0

// Reacquisition / snap parameters (GNSS returns after outage)
#define REACQ_MIN_OUTAGE_S     5.0f     // start reacq if outage longer than this
#define REACQ_STEPS           8        // number of GNSS attempts in reacq mode
#define REACQ_R_MULT          100.0f   // inflate Rpos during reacq
#define REACQ_CHI2_GATE       200.0f   // relaxed gate during reacq (3 dof)
#define SNAP_MIN_OUTAGE_S     15.0f    // allow snap after long outage
#define SNAP_INNOV_M          120.0f   // if horizontal innovation exceeds, snap to GNSS
#define SNAP_HDOP_MAX         2.5f     // require decent HDOP for snap (TAU1204 multi-constellation)
#define NAV_INIT_HDOP_MAX     10.0f    // relaxed for first fix — get initialized fast
#define NAV_INIT_MIN_FIXES    2        // skip first N fixes (TAU1204 converges faster than single-band)
#define GNSS_FUSION_HDOP_MAX  4.0f     // strict HDOP gate after initial convergence
                                       // (TAU1204 typically reports 2.0-3.5, must not reject these)
#define GNSS_HDOP_CONVERGE_FIXES 5     // number of accepted fixes before tightening HDOP

#define GRAVITY             9.80665f
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD ( (float)M_PI / 180.0f )

// GNSS driver ABI uses neo6m_gnss_fix struct (TAU1204 reuses same driver interface).
#ifndef HAVE_GNSS_HDOP
#define HAVE_GNSS_HDOP      1
#endif
#ifndef HAVE_GNSS_HEADING
#define HAVE_GNSS_HEADING   1
#endif

// Yaw dead-band for near-stationary conditions (limits gyro z noise from integrating into yaw)
#define YAW_DEADBAND_RAD    (0.01f)           // ~1.7 deg/s
#define ACC_STILL_TOL       (0.04f*GRAVITY)
#define GYRO_STILL_TOL_RAD  (1.0f * DEG2RAD)

// ZUPT detection thresholds (must accommodate vehicle vibration + IMU noise)
// Log analysis showed old thresholds (0.175, 2°/s) were never met: ZUPT fired 0/1925 rows.
#define ZUPT_ACC_THR        (0.6f)             // | |a|-g | < thr  (was 0.175 — too tight for mounted IMU)
#define ZUPT_GYRO_THR       (5.0f * DEG2RAD)   // per-axis gyro threshold (was 2°/s — below IMU noise floor)
#define ZUPT_COUNT_REQUIRED 10                  // require 10 consecutive still samples (100ms) for confidence

// Mild velocity decay (helps bound drift when filter is "almost stopped")
#define VEL_DECAY           0.98f
#define VEL_EPS             1e-3f

// Outage-tiered Q inflation
// Profile: gentle in the first ~10 s (IMU integration still reliable), then
// faster growth so EKF position covariance honestly reflects DR drift bound.
// Capped at 6× to avoid covariance blow-up that historically caused position
// snaps when GNSS came back.
#define TIER_A  2.0f
#define TIER_B  10.0f
#define TIER_C  60.0f
#define QSCL_A  1.3f       // first 2 s: minimal inflate
#define QSCL_B  2.5f       // 2-10 s: moderate
#define QSCL_C  4.0f       // 10-60 s: meaningful drift growth
#define QSCL_D  6.0f       // >60 s: ceiling — honest about long-DR uncertainty

// GNSS gating and fade-in
// Gate relaxed to chi²(3) 99.99% so transient prediction errors (e.g. brief
// IMU spike, attitude correction) don't reject otherwise-good GNSS fixes.
// Outliers are still caught — chi²=24 means innovation σ-distance > 4.9.
#define CHI2_3DOF_GATE      24.0f            // chi²(3) 99.99% = 23.93 (was 16.0)
#define R_GNSS_POS_VAR      0.6f             // base (m^2), TAU1204 CEP<1m → tighten so GNSS pulls EKF harder
#define FADE_IN_FACT_INIT   4.0f
#define FADE_IN_STEPS       3

// Measurement noise — TAU1204 dual-band: position CEP <1 m, velocity ~0.1 m/s
#define RV_VEL_E            (0.01f)          // (0.10 m/s)^2 — was 0.02
#define RV_VEL_N            (0.01f)
#define RV_VEL_U            (0.25f)          // (0.50 m/s)^2 — was 0.36
#define R_NHC_VY            (0.01f)          // (0.10 m/s)^2 — stricter no-slip for ground vehicle
#define R_NHC_VZ            (0.025f)         // (0.16 m/s)^2 — stricter no-jump
#define R_ZUPT_V            (0.0004f)        // (m/s)^2

// GNSS yaw fusion (optional complementary yaw correction)
#define YAW_FUSION_SPEED_MIN    (1.0f)       // m/s
#define YAW_FUSION_HDOP_MAX     (3.0f)       // tightened for TAU1204 multi-constellation
#define YAW_FUSION_ALPHA        (0.05f)      // fraction per GNSS update

// Vehicle dynamics sanity limits (reject physically impossible values).
// Tighter clamps bound how fast IMU bias can integrate into velocity error
// during the 10-second predict gap between GNSS fusions.
//   At 4 m/s² clamp, even a worst-case 100 %-of-clamp bias accumulates
//   only 40 m/s of velocity error over 10 s — bounded enough that the
//   EKF can recover at the next fusion.
#define MAX_ACCEL_ENU_MPS2  4.0f     // ~0.4 g — covers normal driving (was 6.0)
#define MAX_GNSS_SPEED_MPS  40.0f    // ~145 km/h — realistic road cap (was 55)
#define MAX_GNSS_ACCEL_MPS2  6.0f    // ~0.6 g between fixes

// IMU signal processing pipeline
#define MEDFILT_LEN        5        // median filter window: 5 samples = 50 ms @ 100 Hz
#define IMU_LPF_ALPHA      0.3f     // IIR low-pass weight on new sample (~20 Hz cutoff @ 100 Hz)
#define BUMP_ACC_THR_MPS2  5.0f     // |acc_vert - g| to declare bump (m/s²; ~0.5g, vertical axis only)
#define BUMP_GYRO_THR_RS   (8.0f * DEG2RAD)  // per-axis gyro vibration threshold (rad/s)

// GNSS sawtooth correction: alpha-beta position-velocity tracker
// 2nd-order filter that tracks both position and velocity, removing
// the 1 Hz PVT sawtooth without creating N-sample oscillations.
//   predicted_pos = prev_pos + prev_vel * dt
//   residual      = raw_fix  - predicted_pos
//   smoothed_pos  = predicted_pos + alpha * residual
//   smoothed_vel  = prev_vel      + (beta/dt) * residual
#define AB_ALPHA            0.4f   // position correction gain (0=pure predict, 1=raw)
#define AB_BETA             0.1f   // velocity correction gain (keeps velocity smooth)
#define AB_MAX_DT           3.0f   // reset tracker if gap exceeds this (seconds)
#define AB_MAX_RESIDUAL     8.0f   // reset tracker if residual exceeds this (metres)

// Logging
#define LOG_DIR             "/home/sijeo/nav_logs"

// ISM330 gyro scale: ±500 dps full-scale → 32768/500 = 65.536 LSB/dps
// (was 57.143 which corresponds to ±573 dps — 14.7% too fast, causing yaw drift)
#define GYRO_LSB_PER_DPS    65.536f

#define aWGS        6378137.0
#define fWGS        (1.0/298.257223563)
#define bWGS        (6378137.0 * (1 - fWGS))
#define  e2         ((aWGS*aWGS - bWGS*bWGS) / (aWGS*aWGS))


#define CAL_FILE_PATH       "/home/sijeo/dr_vehicle/imu_cal.bin"
#define CAL_MAGIC           0x43414C31u /* 'CAL1' — legacy dr_cal_t blob */
#define CAL_VERSION         1u

// Production calibration framework (imu_calib_t)
#define IMU_CALIB_MAGIC     0x494D4332u /* 'IMC2' */
#define IMU_CALIB_VERSION   2u
#define IMU_CALIB_PATH      "/home/sijeo/dr_vehicle/imu_calib.bin"

// ---------- Compile-time switches for production calibration ----------
// All default ON except verbose IMU-rate debug logging.
#ifndef ENABLE_BOOT_CALIBRATION
#define ENABLE_BOOT_CALIBRATION      1   // stationary-verified boot-time bias estimation
#endif
#ifndef ENABLE_RUNTIME_GYRO_CAL
#define ENABLE_RUNTIME_GYRO_CAL      1   // slow gyro-bias refinement during stops
#endif
#ifndef ENABLE_CAL_FILE_SAVE
#define ENABLE_CAL_FILE_SAVE         1   // persist imu_calib_t to flash
#endif
#ifndef ENABLE_IMU_DEBUG_LOGS
#define ENABLE_IMU_DEBUG_LOGS        0   // 1 = per-step IMU_PREP / BUMP_STATS spam
#endif
#ifndef ENABLE_MOUNT_MATRIX
#define ENABLE_MOUNT_MATRIX          1   // apply mount_R_bv inside preprocess
#endif

// Sanity limits used by imu_calib_validate(). A loaded calibration is rejected
// (and defaults are used) if any of these are exceeded.
#define CAL_SANITY_GYRO_BIAS_MAX    (5.0f * DEG2RAD)  // 5 °/s
#define CAL_SANITY_ACCEL_BIAS_MAX   (1.0f)             // 1 m/s²  (~0.1 g)
#define CAL_SANITY_ACCEL_NORM_MIN   (9.0f)             // m/s²
#define CAL_SANITY_ACCEL_NORM_MAX   (10.6f)            // m/s²
#define CAL_SANITY_ACC_VAR_MAX      (0.25f)            // (0.5 m/s²)² — stationary variance
#define CAL_SANITY_GYRO_VAR_MAX     ((0.5f*DEG2RAD)*(0.5f*DEG2RAD))   // (0.5 °/s)²

// Runtime gyro-bias-learner gating
#define RT_CAL_WINDOW_S              2.0f                  // continuous-still window required
#define RT_CAL_ALPHA                 0.005f                // slow LPF on bias
#define RT_CAL_GYRO_STILL_RAD        (1.0f * DEG2RAD)     // gyro magnitude threshold
#define RT_CAL_ACC_DEV_MPS2          (0.30f)               // |a|-g threshold
#define RT_CAL_EKF_SPEED_MAX         (0.20f)               // m/s
#define RT_CAL_GNSS_SPEED_MAX        (0.30f)               // m/s
#define RT_CAL_MIN_SAVE_INTERVAL_S   60.0                  // persist at most every 60 s

// =====================================================================
//   AHRS / complementary attitude filter (runs at IMU rate before EKF)
// =====================================================================
#ifndef ENABLE_AHRS_FILTER
#define ENABLE_AHRS_FILTER           1
#endif
#ifndef ENABLE_AHRS_GNSS_YAW
#define ENABLE_AHRS_GNSS_YAW         1
#endif
#ifndef ENABLE_AHRS_ACCEL_CORR
#define ENABLE_AHRS_ACCEL_CORR       1
#endif
#ifndef ENABLE_AHRS_DEBUG_LOGS
#define ENABLE_AHRS_DEBUG_LOGS       0
#endif

// Complementary gains (per IMU update at ~100 Hz)
#define AHRS_ROLLPITCH_ALPHA         0.02f      // 2 %/sample — slow accel correction
#define AHRS_YAW_GNSS_ALPHA          0.05f      // 5 %/GNSS-update yaw correction

// Accelerometer-trust gates
#define AHRS_ACCEL_NORM_TOL          1.5f                  // m/s² — | |a|-g | bound
#define AHRS_ACCEL_TRUST_GYRO_MAX    (10.0f * DEG2RAD)     // per-axis gyro cap during accel correction
#define AHRS_ACCEL_DOT_SPEED_MIN_S   0.6f                  // (m/s) per second longitudinal — reject during hard accel/brake

// GNSS-yaw-trust gates
#define AHRS_GNSS_YAW_SPEED_MIN      2.0f                  // m/s — below this COG is too noisy
#define AHRS_GNSS_YAW_HDOP_MAX       3.0f
#define AHRS_YAW_INNOV_MAX           (45.0f * DEG2RAD)     // reject wild jumps

// EKF yaw-measurement-update noise sigma when AHRS yaw is taken as truth
#define AHRS_TO_EKF_R_YAW            ((3.0f*DEG2RAD)*(3.0f*DEG2RAD))

/**--------------------------GNSS yaw-rate learner tuning --------------------
 * Learn gyro Z scale using GNSS course/heading during turns.
 * Units rad/s, m/s, seconds
 */
#define YAW_LEARN_MIN_SPEED_MPS     3.0f    // raised: COG too noisy below 3 m/s
#define YAW_LEARN_MAX_HDOP          2.0f    // tightened: TAU1204 achieves lower HDOP routinely
#define YAW_LEARN_TURN_RATE_THR     0.03f   // ~ 1.7deg/s
#define YAW_LEARN_MIN_TURN_DEG      20.0f
#define YAW_LEARN_MIN_SEG_S         3.0f
#define YAW_LEARN_MAX_SEG_S         25.0f   // reduced: limits gyro bias accumulation error
#define YAW_LEARN_BETA_SCALE        0.05f   // LPF update for scale
#define YAW_LEARN_SCALE_MIN         0.90f   // tightened: ISM330 shouldn't need >10% correction
#define YAW_LEARN_SCALE_MAX         1.10f
#define YAW_LEARN_PERSIST_PERIOD_S  60.0


// -------- Boot-time stationary calibration thresholds --------
// Used by apply_poweron_calibration to verify the vehicle is actually still.
// If either threshold is exceeded the accumulator resets and calibration
// re-starts — this prevents engine vibration or vehicle motion from being
// baked into the gyro/accel bias estimates.
#define BOOT_CAL_DURATION_S      20.0f                  // accumulate this many seconds of still data
#define BOOT_CAL_GYRO_STILL_RAD  (3.0f * DEG2RAD)        // per-axis gyro must stay below ~3 °/s
#define BOOT_CAL_ACC_STILL_MPS2  (0.50f)                 // | |a|-g | must stay below ~5 % of g
#define BOOT_CAL_RESET_GRACE_S   1.0f                    // print "moved, restarting" at most once/sec

#define CAL_LED_GPIO        13     //BCM GPIO 13
#define CAL_LED_BLINK_HZ    1.0     // 2.0Hz blink while calibration pending

#define BOOT_CAL_N_SAMPLES  500     // e.g. 500 samples (tune based on IMU rate )

/* Noise Figures of IMU — ISM330DLC datasheet + practical margin
 * Datasheet typical: accel 80 µg/√Hz = 7.85e-4 m/s²/√Hz
 *                    gyro  3.8 mdps/√Hz = 6.63e-5 rad/s/√Hz
 * Tuning rationale:
 *   - Process noise (SIGMA_ACCEL/GYRO) sized at ~10× datasheet — real on-vehicle
 *     noise is dominated by suspension/engine vibration, not sensor floor.
 *     Higher Q lets GNSS pull tighter in steady-state (less EKF overconfidence).
 *   - Bias random walks (BIAS) tightened — once biases converge from GNSS/ZUPT,
 *     they should stay stable through outages so DR drifts only from velocity noise.
 */
#define IMU_SIGMA_ACCEL         (0.008f)    // m/s²/√Hz — 10× datasheet, vibration-aware
#define IMU_SIGMA_GYRO          (0.0007f)   // rad/s/√Hz — 10× datasheet, road-coupling aware
#define IMU_SIGMA_ACCEL_BIAS    (0.0008f)   // m/s²/√Hz — slow walk so bias survives DR outages
#define IMU_SIGMA_GYRO_BIAS     (0.0002f)   // rad/s/√Hz — slow walk for stable heading in DR
#define IMU_COVAR_POS   2.0f
#define IMU_COVAR_VEL   1.0f           // m/s
#define IMU_COVAR_ATT   (2.0f*DEG2RAD) // rad
#define IMU_COVAR_BA    0.5f           // m/s^2 (~0.05g 1σ: covers typical ISM330 accel bias residual)
#define IMU_COVAR_BG    0.15f          // rad/s  (~8.6°/s 1σ: covers boot-cal residual up to ~6°/s)


static int cal_led_exported = 0;
static int cal_led_value_fd = -1;

// ------------------------- Small math utils -------------------------

static inline uint64_t monotonic_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static inline double now_sec(void) { return (double)monotonic_ns() * 1e-9; }

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static float medf5(const float buf[5]) {
    float s[5]; int i, j; float t;
    s[0]=buf[0]; s[1]=buf[1]; s[2]=buf[2]; s[3]=buf[3]; s[4]=buf[4];
    for (i = 1; i < 5; i++) {
        t = s[i]; j = i-1;
        while (j >= 0 && s[j] > t) { s[j+1] = s[j]; j--; }
        s[j+1] = t;
    }
    return s[2];
}

static inline float wrap_pi(float a) {
    while (a > (float)M_PI) a -= 2.0f*(float)M_PI;
    while (a <= -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}

typedef struct { float w, x, y, z; } quatf;
typedef struct { float x, y, z; } vec3f;

static inline vec3f v3(float x, float y, float z) { return (vec3f){x,y,z}; }

static inline float v3_norm(vec3f a) { return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z); }

static inline vec3f v3_add(vec3f a, vec3f b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline vec3f v3_sub(vec3f a, vec3f b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline vec3f v3_scale(vec3f a, float s) { return v3(a.x*s, a.y*s, a.z*s); }

static inline vec3f v3_cross(vec3f a, vec3f b) {
    return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

static inline float v3_dot(vec3f a, vec3f b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

static inline quatf q_identity(void) { return (quatf){1,0,0,0}; }

static inline quatf q_normalize(quatf q) {
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n <= 0) return q_identity();
    float inv = 1.0f / n;
    return (quatf){q.w*inv, q.x*inv, q.y*inv, q.z*inv};
}

static inline quatf q_mul(quatf a, quatf b) {
    return (quatf){
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

static inline quatf q_from_small_angle(vec3f dtheta) {
    // small-angle quaternion: [1, 0.5*dtheta]
    return q_normalize((quatf){1.0f, 0.5f*dtheta.x, 0.5f*dtheta.y, 0.5f*dtheta.z});
}

static inline void R_from_q(quatf q, float R[9]) {
    // ENU: columns are body axes in world? Here: R maps body->world: v_w = R * v_b
    q = q_normalize(q);
    float w=q.w, x=q.x, y=q.y, z=q.z;
    R[0] = 1 - 2*(y*y + z*z);  R[1] = 2*(x*y - w*z);      R[2] = 2*(x*z + w*y);
    R[3] = 2*(x*y + w*z);      R[4] = 1 - 2*(x*x + z*z);  R[5] = 2*(y*z - w*x);
    R[6] = 2*(x*z - w*y);      R[7] = 2*(y*z + w*x);      R[8] = 1 - 2*(x*x + y*y);
}

static inline float yaw_from_q(quatf q) {
    // yaw about +Z in ENU
    float siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    return atan2f(siny_cosp, cosy_cosp);
}

static inline quatf q_from_yaw_delta(float dpsi) {
    float half = 0.5f*dpsi;
    return (quatf){cosf(half), 0.0f, 0.0f, sinf(half)};
}

// Quaternion from Euler angles (ZYX convention: yaw, pitch, roll)
// Produces body→ENU quaternion
static inline quatf q_from_euler(float roll, float pitch, float yaw) {
    float cr = cosf(0.5f*roll),  sr = sinf(0.5f*roll);
    float cp = cosf(0.5f*pitch), sp = sinf(0.5f*pitch);
    float cy = cosf(0.5f*yaw),   sy = sinf(0.5f*yaw);
    return q_normalize((quatf){
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    });
}

static bool mat3_inv(const float A[9], float invA[9]) {
    float a=A[0], b=A[1], c=A[2];
    float d=A[3], e=A[4], f=A[5];
    float g=A[6], h=A[7], i=A[8];
    float A11 = (e*i - f*h);
    float A12 = -(d*i - f*g);
    float A13 = (d*h - e*g);
    float det = a*A11 + b*A12 + c*A13;
    if (fabsf(det) < 1e-12f) return false;
    float invdet = 1.0f/det;
    invA[0] = A11*invdet;
    invA[1] = (c*h - b*i)*invdet;
    invA[2] = (b*f - c*e)*invdet;
    invA[3] = A12*invdet;
    invA[4] = (a*i - c*g)*invdet;
    invA[5] = (c*d - a*f)*invdet;
    invA[6] = A13*invdet;
    invA[7] = (b*g - a*h)*invdet;
    invA[8] = (a*e - b*d)*invdet;
    return true;
}

// ========================== EKF MODE SELECTION ==========================
// set to 1 for 2D EKF (recommeded for ground vehicles), 0 for 3D EKF
#ifndef USE_2D_EKF
#define USE_2D_EKF 1
#endif 
#if USE_2D_EKF
// ===================== 2D EKF IMPLEMENTATION =====================
/* 7-State Error State EKF for ground vehicle dead reckoning
* State Vector (Nominal):
* p_E, p_N      : Position in ENU (m)
* v_E, v_N      : Velocity in ENU (m/s)
* psi           : Yaw angle (rad, From East, CCW positive)
* b_a           : Accelerometer bias (m/s^2)
* b_g           : Gyro bias (rad/s)
* Error State Vector:
* δp_E, δp_N    : Position error in ENU (m)
* δv_E, δv_N    : Velocity error in ENU (m/s)
* δψ            : Yaw angle error (rad)
* δb_a          : Accelerometer bias error (m/s^2)
* δb_g          : Gyro bias error (rad/s)
* This is vastly simpler than the 15 state 3D filter and much more robust
* for a ground vehicles where vertical dynamics are irrelevant.
*/
// ===================================================================

typedef struct {
    /* Nominal State */
    float p_E, p_N;      // Position in ENU (m)
    float v_E, v_N;      // Velocity in ENU (m/s)
    float psi;           // Yaw angle (rad, From East, CCW positive)
    float b_a;           // Accelerometer bias (m/s^2)
    float b_g;           // Gyro bias (rad/s)
    float P[7*7];  // Error covariance matrix (7x7)
} ins2d_t;

// 2D process noise — tuned for ISM330 on a vehicle
//   Datasheet: accel 7.85e-4 m/s²/√Hz, gyro 6.63e-5 rad/s/√Hz
//   On-vehicle real noise is dominated by suspension/engine vibration.
//   SIGMA_ACCEL=0.05 (≈64× datasheet) — bounds DR velocity drift
//     while keeping per-step process noise tight enough for GNSS to pull tightly
//     (per-step velocity σ = 0.05·√0.01 = 0.005 m/s ≪ R_VEL).
//   SIGMA_GYRO=0.003 (≈45× datasheet, ~0.17°/s/√Hz) — heading uncertainty
//     grows ~0.94° after a 30-s outage — bounded enough to keep DR position usable.
//   Bias random walks tightened — once biases converge under GNSS coverage,
//   they stay near-locked through outages, so DR drift comes from σ_a/σ_g only.
#define INS2D_SIGMA_ACCEL         (0.05f)    // m/s²/√Hz — vehicle vibration realistic
#define INS2D_SIGMA_GYRO          (0.003f)   // rad/s/√Hz — vehicle road-coupling
#define INS2D_SIGMA_BA            (0.0005f)  // m/s²/√Hz — slow accel-bias walk
#define INS2D_SIGMA_BG            (0.00005f) // rad/s/√Hz — very slow gyro-bias walk

// 2D measurement noise — TAU1204 dual-band: position CEP<1m, velocity ~0.1 m/s
#define INS2D_R_POS              (0.4f)     // m² — base; multiplied by HDOP² in fusion
#define INS2D_R_VEL              (0.01f)    // (0.10 m/s)² — TAU1204 dual-band spec
#define INS2D_R_HDG              (0.01f)    // unused — heading R is computed dynamically (5°/speed)
#define INS2D_R_NHC              (0.01f)    // (0.10 m/s)² — strict no-slip for ground vehicle

// 2D Gating
#define INS2D_CHI2_2DOF_GATE      13.8f      // chi²(2) 99.9% — rejects multipath outliers
#define INS2D_CHI2_1DOF_GATE      10.8f      // chi²(1) 99.9%

static void ins2d_init(ins2d_t *S) {
    int i;
    memset(S, 0, sizeof(*S));
    S->psi = 0.0f;
    // Initial Covariance
    const float p0 = 5.0f; // m
    const float v0 = 1.0f; // m/s
    const float psi0 = 30.0f * DEG2RAD; // rad
    const float ba0 = 0.2f; // m/s^2
    const float bg0 = 0.05f; // rad/s

    for(i = 0; i < 7*7; i++) S->P[i] = 0.0f;
    S->P[0*7 + 0] = p0*p0; // var(p_E)
    S->P[1*7 + 1] = p0*p0; // var(p_N)
    S->P[2*7 + 2] = v0*v0; // var(v_E)
    S->P[3*7 + 3] = v0*v0; // var(v_N)
    S->P[4*7 + 4] = psi0*psi0; // var(psi)
    S->P[5*7 + 5] = ba0*ba0; // var(b_a)
    S->P[6*7 + 6] = bg0*bg0; // var(b_g)
}

// 7x7 Matrix operations 
static void mat7_mul( const float *A, const float *B, float *C )
{
    int r, c, k;
    for(r = 0; r < 7; r++){
        for(c = 0; c < 7; c++){
            float sum = 0.0f;
            for(k = 0; k < 7; k++){
                sum += A[r*7 + k] * B[k*7 + c];
            }
            C[r*7 + c] = sum;
        }
    }
}

static void mat7_T(const float *A, float *AT)
{
    int r, c;
    for(r = 0; r < 7; r++){
        for(c = 0; c < 7; c++){
            AT[c*7 + r] = A[r*7 + c];
        }
    }
}

static void mat7_add(float *A, const float *B)
{
    int i;
    for(i = 0; i < 7*7; i++){
        A[i] += B[i];
    }
}

static bool mat2_inv(const float A[4], float invA[4])
{
    float det = A[0]*A[3] - A[1]*A[2];
    if (fabsf(det) < 1e-6f) return false;
    float inv = 1.0f / det;
    invA[0] =  A[3] * inv;
    invA[1] = -A[1] * inv;
    invA[2] = -A[2] * inv;
    invA[3] =  A[0] * inv;
    return true;
}

/**
 * 2D Mechanization + Covariance propogation 
 * Uses body frame forward acceleration and yaw rate from IMU 
 */
 static void ins2d_predict(ins2d_t *S, float a_forward, float omeag_z, float dt, float q_scale ) 
 {
    // Remove Biases
    float af = a_forward - S->b_a;
    float wz = omeag_z - S->b_g;

    // Current Heading 
    float cpsi = cosf(S->psi);
    float spsi = sinf(S->psi);

    // Velocity Prediction (body frame accel rotated to ENU)
    // For ground vehicle, a_forward acts along the heading direction.
    float a_E = af * cpsi;
    float a_N = af * spsi;

    // Kinematics
    S->p_E += S->v_E * dt + 0.5f * a_E * dt * dt;
    S->p_N += S->v_N * dt + 0.5f * a_N * dt * dt;
    S->v_E += a_E * dt;
    S->v_N += a_N * dt;
    S->psi += wz * dt;
    S->psi = wrap_pi(S->psi);

    // Linearized state Transition matrix F (7x7)
    // x = [p_E, p_N, v_E, v_N, psi, b_a, b_g]
    float F[7*7] = {0};
    memset(F, 0, sizeof(F));

    // Position derivatives
    F[0*7 + 2] = 1.0f; // dp_E/dv_E
    F[1*7 + 3] = 1.0f; // dp_N/dv_N
    F[2*7 + 4] = -af * spsi; // dv_E/dpsi
    F[3*7 + 4] = af * cpsi;  // dv_N/dpsi
    F[2*7 + 5] = -cpsi; // dv_E/dba
    F[3*7 + 5] = -spsi; // dv_N/dba
    F[4*7 + 6] = -1.0f; // dpsi/dbg

    // Discretize F to get state transition matrix Phi = I + F*dt (Euler method)
    float Phi[7*7];
    int i, r, c;
    memset(Phi, 0, sizeof(Phi));
    for (i = 0; i < 7; i++) Phi[i*7 + i] = 1.0f;   // identity
    for (r = 0; r < 7; r++)
    {
        for (c = 0; c < 7; c++)
        {
            Phi[r*7 + c] += F[r*7 + c] * dt;
        }
    }
    //Process Noise Matrix Q (7x7).
    // q_scale (outage tier) inflates STATE noise (acc/gyro) only — NOT bias walks.
    // Inflating bias walk during outage lets b_a, b_g wander with no measurement
    // to constrain them; when GNSS comes back the bias estimate is junk and the
    // EKF takes a long time (or never) to recover. Hold biases at their last
    // good value through the outage.
    float sa2  = q_scale * INS2D_SIGMA_ACCEL * INS2D_SIGMA_ACCEL;
    float sg2  = q_scale * INS2D_SIGMA_GYRO  * INS2D_SIGMA_GYRO;
    float sba2 = INS2D_SIGMA_BA * INS2D_SIGMA_BA;   // no q_scale
    float sbg2 = INS2D_SIGMA_BG * INS2D_SIGMA_BG;   // no q_scale
    float Q[7*7] = {0};
    memset(Q, 0, sizeof(Q));
    Q[0*7 + 0] = sa2 * dt * dt * dt / 3.0f; // var(p_E)
    Q[1*7 + 1] = sa2 * dt * dt * dt / 3.0f; // var(p_N)

    Q[2*7 + 2] = sa2 * dt ; // var(v_E)
    Q[3*7 + 3] = sa2 * dt ; // var(v_N)
    // Heading Noise
    Q[4*7 + 4] = sg2 * dt ; // var(psi)
    Q[5*7 + 5] = sba2 * dt; // var(b_a)
    Q[6*7 + 6] = sbg2 * dt; // var(b_g)

    // Prediction Covariance Propagation: P = Phi * P * Phi^T + Q
    float PhiP[7*7], PhiT[7*7], PhiP_PhiT[7*7];
    mat7_mul(Phi, S->P, PhiP);
    mat7_T(Phi, PhiT);
    mat7_mul(PhiP, PhiT, PhiP_PhiT);
    memcpy(S->P, PhiP_PhiT, sizeof(PhiP_PhiT));
    mat7_add(S->P, Q);

 }

 // Inject error-state correction into nominal state.
 static void ins2d_inject( ins2d_t *S, const float dx[7] ) {
    // Update nominal state with error correction
    S->p_E += dx[0];
    S->p_N += dx[1];
    S->v_E += dx[2];
    S->v_N += dx[3];
    S->psi += dx[4];
    S->psi = wrap_pi(S->psi);
    S->b_a += dx[5];
    S->b_g += dx[6];

 }

 // Generic Scalar measurement update 
 static bool ins2d_update_scalar( ins2d_t *S, const float H[7], float z, float h, float R, float *out_NIS ) {
    // S_scalar = H * P * H^T + R
    float HP[7];
    int i;
    for (i = 0; i < 7; i++) {
        HP[i] = 0.0f;
        int j;
        for (j = 0; j < 7; j++) {
            HP[i] += H[j] * S->P[j*7 + i];
        }
    }
    float S_scalar = R;
    for (i = 0; i < 7; i++) {
        S_scalar += HP[i] * H[i];
    }

    if(fabsf(S_scalar) < 1e-12f) return false; // avoid division by zero
    float Sinv = 1.0f / S_scalar;

    float nu = z - h; // innovation
    if (out_NIS) {
        *out_NIS = nu * nu * Sinv; // NIS = nu^2 / S
    }
    // Kalman Gain K = P * H^T * Sinv
    float K[7];
    for (i = 0; i < 7; i++) {
        K[i] = 0.0f;
        int j;
        for (j = 0; j < 7; j++) {
            K[i] += S->P[i*7 + j] * H[j];
        }
        K[i] *= Sinv;
    }

    //dx = K * nu
    float dx[7];
    for (i = 0; i < 7; i++) {
        dx[i] = K[i] * nu;
    }
    // Inject correction
    ins2d_inject(S, dx);

    // Joseph Covariance Update: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    float I_KH[7*7];
    for (i = 0; i < 7; i++) {
        int j;
        for (j = 0; j < 7; j++) {
            I_KH[i*7 + j] = ((i == j) ? 1.0f : 0.0f) - K[i] * H[j];
        }    
    }
    float temp[7*7], I_KH_T[7*7], P1[7*7];
    mat7_mul(I_KH, S->P, temp);
    mat7_T(I_KH, I_KH_T);
    mat7_mul(temp, I_KH_T, P1);
    // Add K*R*K^T (rank-1 outer product: K is 7x1, so KRKt[i][j] = K[i]*R*K[j])
    for (i = 0; i < 7; i++) {
        int j;
        for (j = 0; j < 7; j++) {
            S->P[i*7 + j] = P1[i*7 + j] + K[i] * R * K[j];
        }
    }
    return true;
}

// 2D vector measurement update (position)
static bool ins2d_update_2d( ins2d_t *S, const float H[2*7], const float z[2], const float h[2], 
                        const float Rdiag[2], float *out_NIS ) {
    // S_2d = H * P * H^T + R
    float HP[2*7];
    int i;
    for (i = 0; i < 2; i++) {
        int j;
        for (j = 0; j < 7; j++) {
            HP[i*7 + j] = 0.0f;
            int k;
            for (k = 0; k < 7; k++) {
                HP[i*7 + j] += H[i*7 + k] * S->P[k*7 + j];
            }
        }
    }
    float S22[4]; // 2x2 innovation covariance
    for (i = 0; i < 2; i++) {
        int j;
        for (j = 0; j < 2; j++) {
            S22[i*2 + j] = 0.0f;
            int k;
            for (k = 0; k < 7; k++) {
                S22[i*2 + j] += HP[i*7 + k] * H[j*7 + k];
            }
            if (i == j) {
                S22[i*2 + j] += Rdiag[i];
            }
        }
    }
    float Sinv[4];
    if (!mat2_inv(S22, Sinv)) return false; // singular innovation covariance
    // Innovation nu = z - h
    float nu[2] = { z[0] - h[0], z[1] - h[1] };
    if (out_NIS) {
        // NIS = nu^T * S^-1 * nu
        float tmp[2] = {0};
        int i, j;
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 2; j++) {
                tmp[i] += Sinv[i*2 + j] * nu[j];
            }
        }
        *out_NIS = tmp[0] * nu[0] + tmp[1] * nu[1];
    }
    // Kalman Gain K = P * H^T * S^-1
    float PHt[7*2];
    for (i = 0; i < 7; i++) {
        int j;
        for (j = 0; j < 2; j++) {
            PHt[i*2 + j] = 0.0f;
            int k;
            for (k = 0; k < 7; k++) {
                PHt[i*2 + j] += S->P[i*7 + k] * H[j*7 + k];
            }
        }
    }
    float K[7*2];
    for (i = 0; i < 7; i++) {
        int j;
        for (j = 0; j < 2; j++) {
            K[i*2 + j] = 0.0f;
            int k;
            for (k = 0; k < 2; k++) {
                K[i*2 + j] += PHt[i*2 + k] * Sinv[k*2 + j];
            }
        }
    }
    // dx = K * nu
    float dx[7] = {0};
    {
        int j;
        for (i = 0; i < 7; i++) {
            for (j = 0; j < 2; j++) {
                dx[i] += K[i*2 + j] * nu[j];
            }
        }
    }
    // Inject correction
    ins2d_inject(S, dx);
    // Joseph Covariance Update: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    float KH[7*7];
    memset(KH, 0, sizeof(KH));
    for( i=0; i < 7; i++) {
        int j, k;
        for (j = 0; j < 7; j++) {
            for (k = 0; k < 2; k++) {
                KH[i*7 + j] += K[i*2 + k] * H[k*7 + j];
            }
        }
    }

    float I_KH[7*7];
    for( i=0; i < 7; i++) {
        int j;
        for (j = 0; j < 7; j++) {
            I_KH[i*7 + j] = ((i == j) ? 1.0f : 0.0f) - KH[i*7 + j];
        }    
    }

    float temp[7*7], I_KH_T[7*7], P1[7*7];
    mat7_mul(I_KH, S->P, temp);
    mat7_T(I_KH, I_KH_T);
    mat7_mul(temp, I_KH_T, P1);
    // Add K*R*K^T  (K is 7x2, R is diagonal 2x2 → KRKt[i][j] = sum_k K[i][k]*R[k]*K[j][k])
    float KRKt[7*7];
    memset(KRKt, 0, sizeof(KRKt));
    for (i = 0; i < 7; i++) {
        int j, k;
        for (j = 0; j < 7; j++) {
            for (k = 0; k < 2; k++) {
                KRKt[i*7 + j] += K[i*2 + k] * Rdiag[k] * K[j*2 + k];
            }
        }
    }

    for (i = 0; i < 7*7; i++) {
        S->P[i] = P1[i] + KRKt[i];
    }

    return true;
}

// ---- NIS-only computation for the 2D update (state and P unchanged) ----
// Returns true and writes NIS = nu^T * (H P H^T + R)^-1 * nu.
// Use this to gate-check before calling the actual update functions, so an
// outlier never gets injected into the state.
static bool ins2d_nis_2d(const ins2d_t *S, const float H[2*7], const float z[2],
                         const float h[2], const float Rdiag[2], float *out_NIS)
{
    float HP[2*7];
    int i;
    for (i = 0; i < 2; i++) {
        int j;
        for (j = 0; j < 7; j++) {
            HP[i*7 + j] = 0.0f;
            int k;
            for (k = 0; k < 7; k++) HP[i*7 + j] += H[i*7 + k] * S->P[k*7 + j];
        }
    }
    float S22[4];
    for (i = 0; i < 2; i++) {
        int j;
        for (j = 0; j < 2; j++) {
            S22[i*2 + j] = 0.0f;
            int k;
            for (k = 0; k < 7; k++) S22[i*2 + j] += HP[i*7 + k] * H[j*7 + k];
            if (i == j) S22[i*2 + j] += Rdiag[i];
        }
    }
    float Sinv[4];
    if (!mat2_inv(S22, Sinv)) return false;
    float nu[2] = { z[0] - h[0], z[1] - h[1] };
    float tmp[2] = {0};
    for (i = 0; i < 2; i++) {
        int j;
        for (j = 0; j < 2; j++) tmp[i] += Sinv[i*2 + j] * nu[j];
    }
    if (out_NIS) *out_NIS = tmp[0]*nu[0] + tmp[1]*nu[1];
    return true;
}

static bool ins2d_nis_gnss_pos(const ins2d_t *S, float z_E, float z_N, float R_pos, float *out_NIS)
{
    float H[2*7] = {0};
    H[0*7 + 0] = 1.0f;
    H[1*7 + 1] = 1.0f;
    float h[2] = { S->p_E, S->p_N };
    float Rd[2] = { R_pos, R_pos };
    float z[2]  = { z_E, z_N };
    return ins2d_nis_2d(S, H, z, h, Rd, out_NIS);
}

static bool ins2d_nis_gnss_vel(const ins2d_t *S, float z_vE, float z_vN, float R_vel, float *out_NIS)
{
    float H[2*7] = {0};
    H[0*7 + 2] = 1.0f;
    H[1*7 + 3] = 1.0f;
    float h[2] = { S->v_E, S->v_N };
    float Rd[2] = { R_vel, R_vel };
    float z[2]  = { z_vE, z_vN };
    return ins2d_nis_2d(S, H, z, h, Rd, out_NIS);
}

// GNSS position update (E, N only )
static bool ins2d_update_gnss_pos( ins2d_t *S, float z_E, float z_N, float R_pos, float *out_NIS )
{
    float H[2*7] = {0};
    H[0*7 + 0] = 1.0f; // p_E
    H[1*7 + 1] = 1.0f; // p_N
    float h[2] = { S->p_E, S->p_N };
    float Rdiag[2] = { R_pos, R_pos };
    float z[2] = { z_E, z_N };
    return ins2d_update_2d(S, H, z, h, Rdiag, out_NIS);
}

// GNSS velocity update (E, N only )
static bool ins2d_update_gnss_vel( ins2d_t *S, float z_vE, float z_vN, float R_vel, float *out_NIS ) 
{
    float H[2*7] = {0};
    H[0*7 + 2] = 1.0f; // v_E
    H[1*7 + 3] = 1.0f; // v_N
    float h[2] = { S->v_E, S->v_N };
    float Rdiag[2] = { R_vel, R_vel };
    float z[2] = { z_vE, z_vN };
    return ins2d_update_2d(S, H, z, h, Rdiag, out_NIS);
}

// GNSS heading update (yaw only)
static bool ins2d_update_heading( ins2d_t *S, float z_psi, float R_hdg, float *out_NIS )
{
    float H[7] = {0};
    H[4] = 1.0f; // psi
    float nu = wrap_pi(z_psi - S->psi); // innovation with angle wrapping
    return ins2d_update_scalar(S, H, S->psi + nu, S->psi, R_hdg, out_NIS);
}

// Non Holomonic constraint : lateral velocity should be zero for a ground vehicle
// v_lateral = -v_E * sin(psi) + v_N * cos(psi) ≈ 0
static void ins2d_update_nhc( ins2d_t *S ) {
    float cpsi = cosf(S->psi);
    float spsi = sinf(S->psi);

    // v_lat = -v_E * spsi + v_N * cpsi
    float v_lat = -S->v_E * spsi + S->v_N * cpsi;
    float H[7] = {0};
    H[2] = -spsi; // dv_lat/dv_E
    H[3] = cpsi;  // dv_lat/dv_N
    H[4] = -S->v_E * cpsi - S->v_N * spsi; // dv_lat/dpsi

    ins2d_update_scalar(S, H, 0.0f, v_lat, INS2D_R_NHC, NULL);

}
// ZUPT update: zero velocity constraint when stationary (can be used to constrain accel bias)
static void ins2d_update_zupt( ins2d_t *S ) {
    // Update v_E = 0
    float H_vE[7] = {0};
    H_vE[2] = 1.0f; // v_E
    ins2d_update_scalar(S, H_vE, 0.0f, S->v_E, 0.001f, NULL);

    // Update v_N = 0
    float H_vN[7] = {0};
    H_vN[3] = 1.0f; // v_N
    ins2d_update_scalar(S, H_vN, 0.0f, S->v_N, 0.001f, NULL);
}

/**
 * Extract forward acceleration from calibrated body-frame accelerometer
 * for a properly mounted IMU, where X axis points forward along the vehicle's longitudinal axis.
 */
static float ins2d_get_forward_accel(vec3f acc_b, quatf q) {
    // Rotate body-frame accel to ENU
    float R[9];
    R_from_q(q, R);
    float a_E = R[0]*acc_b.x + R[1]*acc_b.y + R[2]*acc_b.z;
    float a_N = R[3]*acc_b.x + R[4]*acc_b.y + R[5]*acc_b.z;

    /**
     * Subtract gravity (already done if acc_b is specific force )
     * for ground vehicle, forward accel is the horizontal component
     * projected onto heading direction(will be computed in predict using heading)
     * Return magnitude of horizontal acceleration
     * The direction is handled in ins2d_predict using the heading 
     */
    float a_horiz = sqrtf(a_E*a_E + a_N*a_N);
    /**
     * Sign : positive if accelerating in heading direction
     * This is approximate for better results, use body freame X-axis directly
     */
    return acc_b.x; // use body-frame forward accel directly (assuming proper mounting)
}
#endif // USE_2D_EKF



static void cal_led_init( void )
{
    char path[128];
    int i;

    /* Export GPIO */
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if( fd >= 0) {
        dprintf(fd, "%d", CAL_LED_GPIO);
        close(fd);
    }

    /*Waiti for sysfs node to appear */
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", CAL_LED_GPIO);
    for(i = 0; i < 50; i++){
        if(access(path, F_OK) == 0) break;
        usleep(2000);
    }

    // Direction = Out
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", CAL_LED_GPIO);
    fd = open(path, O_WRONLY);
    if( fd >= 0 ){
        write(fd, "out", 3);
        close(fd);
    }

    //Open Value fd once(faster)
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", CAL_LED_GPIO);
    cal_led_value_fd = open(path, O_WRONLY);
    if( cal_led_value_fd >= 0 ){
        cal_led_exported = 1;
        write(cal_led_value_fd, "0", 1); //LED off initially 
    }
}

static void cal_led_set(int on ){
    if( !cal_led_exported ) return;
    lseek(cal_led_value_fd, 0, SEEK_SET );
    write(cal_led_value_fd, on ? "1" : "0", 1);
}

static void cal_led_deinit( void ){
    if(cal_led_value_fd >= 0){
        cal_led_set(0);
        close(cal_led_value_fd);
        cal_led_value_fd = -1;
    }

    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if( fd >= 0) {
        dprintf(fd, "%d", CAL_LED_GPIO);
        close(fd);
    }
}


//---------------------------------NON blocking blink helper ----------------------
static void cal_led_update(bool cal_in_progress, bool cal_done, double now_s){
    static double last_toggle_s = 0.0;
    static int led_state = 0;

    if( cal_done )
    {
        cal_led_set(1);
        led_state = 1;
        return;
    }

    if(!cal_in_progress ){
        cal_led_set(0);
        led_state = 0;
        return;
    }

    if( (now_s - last_toggle_s) >= 0.5 ){
        led_state = !led_state;
        cal_led_set(led_state);
        last_toggle_s = now_s;
    }
}

// ------------------------- WGS84 helpers (LLA/ECEF/ENU) -------------------------

typedef struct { double lat, lon, h; } lla_t;
typedef struct { double x, y, z; } ecef_t;



static ecef_t lla2ecef(lla_t L) {
    double s = sin(L.lat), c = cos(L.lat);
    double N = aWGS / sqrt(1.0 - e2*s*s);
    ecef_t e;
    e.x = (N + L.h)*c*cos(L.lon);
    e.y = (N + L.h)*c*sin(L.lon);
    e.z = (N*(1-e2) + L.h)*s;
    return e;
}

static void ecef2enu(ecef_t e, lla_t ref, ecef_t e0, double out[3]) {
    double s1 = sin(ref.lat), c1 = cos(ref.lat);
    double s0 = sin(ref.lon), c0 = cos(ref.lon);
    double dx = e.x - e0.x;
    double dy = e.y - e0.y;
    double dz = e.z - e0.z;

    out[0] = -s0*dx + c0*dy;                          // East
    out[1] = -s1*c0*dx - s1*s0*dy + c1*dz;             // North
    out[2] =  c1*c0*dx + c1*s0*dy + s1*dz;             // Up
}

static ecef_t enu2ecef(double e_m, double n_m, double u_m, lla_t ref, ecef_t e0) {
    double s1 = sin(ref.lat), c1 = cos(ref.lat);
    double s0 = sin(ref.lon), c0 = cos(ref.lon);
    ecef_t r;
    r.x = e0.x + (-s0)*e_m + (-s1*c0)*n_m + (c1*c0)*u_m;
    r.y = e0.y + ( c0)*e_m + (-s1*s0)*n_m + (c1*s0)*u_m;
    r.z = e0.z +             (    c1)*n_m  + (   s1)*u_m;
    return r;
}

static lla_t ecef2lla(ecef_t e) {
    lla_t L;
    int i;
    L.lon = atan2(e.y, e.x);
    double p = sqrt(e.x*e.x + e.y*e.y);
    double lat = atan2(e.z, p*(1.0 - e2));
    for (i = 0; i < 10; i++) {
        double s = sin(lat);
        double N = aWGS / sqrt(1.0 - e2*s*s);
        double h = p / cos(lat) - N;
        double lat_new = atan2(e.z, p*(1.0 - e2*(N/(N+h))));
        if (fabs(lat_new - lat) < 1e-12) { lat = lat_new; break; }
        lat = lat_new;
    }
    {
        double s = sin(lat);
        double N_val = aWGS / sqrt(1.0 - e2*s*s);
        L.lat = lat;
        L.h = p / cos(lat) - N_val;
    }
    return L;
}

// ------------------------- Calibration (persistent + online trims ) -------------------------
// (Keep your LSQ accel matrix + offset; and gyro bias in raw counts.)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t crc32;

    /* Baseline accel: SI = C* raw + O */
    float accel_C[3][3];
    float accel_O[3];

    /* Baseline gyro: (raw - bias_counts) -> dps -> rad/s */
    float gyro_bias_counts[3];

    /* Optional single-axis trims learned online (start at 1.0 ) */
    float gyro_z_scale;     /* yaw-rate scale correction (k) */
    float accel_x_scale;    /* optional (leave 1.0 unless you enable it )*/

    uint8_t calibrated_once; /* 0 -until first install stationary calibration successfully saved */
    uint8_t _pad[3];
} dr_cal_t;

/* static const float ACCEL_C[3][3] = {
    {0.0005964462462844185f, -9.211739001666678e-07f, 1.5763305507490624e-05f},
    {-1.1573398643476584e-06f, 0.0006037351040586055f, 3.881537441769146e-07f},
    {-3.851697134466662e-05f, -3.2356391081574615e-05f, 0.0005895175306304627f}
};

static const float ACCEL_O[3] = { -0.1329121010477303f, -0.047222673271787766f, 1.257425727446983f };

// Gyro bias in raw counts (constant here; can be learned slowly in future)
static const float GYRO_B[3] = { -153.461f, 69.446f, 992.782f };*/

static dr_cal_t g_cal;

/* =====================================================================
 * Production IMU calibration framework (imu_calib_t).
 *
 * Frame conventions:
 *   IMU/body frame:  X = sensor X (forward when mounted with X→front)
 *                    Y = sensor Y (left when mounted with Y→left)
 *                    Z = sensor Z (up when mounted Z→up)
 *   Vehicle frame:   X_v = vehicle forward (along chassis)
 *                    Y_v = vehicle left
 *                    Z_v = vehicle up
 *
 * mount_R_bv is the rotation that converts IMU-frame vectors to vehicle
 * frame:  v_vehicle = mount_R_bv * v_imu_body.
 *
 * Default mount_R_bv = identity (assumes IMU is physically aligned with
 * the vehicle chassis). After an installation calibration is performed
 * (drive a straight line at constant speed, compare GNSS course with
 * gyro-integrated heading) update mount_R_bv and persist it.
 * ===================================================================== */

typedef struct {
    uint32_t magic;
    uint32_t version;

    float    accel_bias[3];        // m/s²    (subtract from calibrated accel)
    float    gyro_bias[3];         // rad/s   (subtract from calibrated gyro)

    float    accel_scale[3];       // unitless scale correction (default 1)
    float    gyro_scale[3];        // unitless scale correction (default 1)

    float    accel_misalign[3][3]; // optional cross-axis correction (default I)
    float    mount_R_bv[3][3];     // IMU → vehicle frame (default I)

    float    temperature_c;        // sensor temperature at calibration time
    uint64_t created_unix_s;       // wall-clock at creation

    // Quality metrics captured during the boot-stationary window
    float    boot_accel_var[3];          // per-axis accel sample variance (m²/s⁴)
    float    boot_gyro_var[3];           // per-axis gyro  sample variance (rad²/s²)
    float    boot_accel_norm_mean;       // mean(|acc|) — should be ≈ 9.80665
    float    boot_accel_norm_std;
    float    boot_gyro_norm_mean;        // mean(|gyro|) — should be ≈ 0
    float    boot_gyro_norm_std;

    uint32_t boot_sample_count;
    uint32_t valid_flags;          // bit0 boot-cal ok, bit1 runtime-cal active,
                                   // bit2 mount matrix verified
    uint32_t checksum;             // CRC32 over the rest (this field zeroed)
} imu_calib_t;

#define CAL_FLAG_BOOT_OK         (1u << 0)
#define CAL_FLAG_RUNTIME_OK      (1u << 1)
#define CAL_FLAG_MOUNT_OK        (1u << 2)
#define CAL_FLAG_DEGRADED        (1u << 3)

static imu_calib_t g_imu_cal;

/* IMU quality state — set by boot/runtime cal and bump detector.
 * Used by the EKF to inflate process noise when the sensor is unreliable. */
static volatile int g_imu_quality_degraded = 0;
static double       g_last_calib_save_s    = 0.0;

/* ---- CRC32 (IEEE 802.3 polynomial, reflected — good enough for a small blob) ---- */
static uint32_t crc32_simple(const void *data, size_t nbytes) {
    size_t i;
    int b;
    const uint8_t *p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (i = 0; i < nbytes; i++) {
        crc ^= p[i];
        for (b = 0; b < 8; b++) {
            uint32_t m = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & m);
        }
    }
    return ~crc;
}

static uint32_t imu_calib_checksum(const imu_calib_t *cal) {
    imu_calib_t tmp = *cal;
    tmp.checksum = 0;
    return crc32_simple(&tmp, sizeof(tmp));
}

/* 3x3 matrix helpers (used by mount-rotation and misalignment) */
static void mat3_identity(float R[3][3]) {
    int i, j;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            R[i][j] = (i == j) ? 1.0f : 0.0f;
}

static void mat3_mul_vec(const float R[3][3], const float v[3], float out[3]) {
    out[0] = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
    out[1] = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
    out[2] = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
}

static void apply_mount_rotation(const imu_calib_t *cal,
                                 const float imu_vec[3],
                                 float vehicle_vec[3])
{
#if ENABLE_MOUNT_MATRIX
    mat3_mul_vec(cal->mount_R_bv, imu_vec, vehicle_vec);
#else
    vehicle_vec[0] = imu_vec[0];
    vehicle_vec[1] = imu_vec[1];
    vehicle_vec[2] = imu_vec[2];
#endif
}

static void imu_calib_set_defaults(imu_calib_t *cal) {
    memset(cal, 0, sizeof(*cal));
    cal->magic         = IMU_CALIB_MAGIC;
    cal->version       = IMU_CALIB_VERSION;
    cal->accel_scale[0] = cal->accel_scale[1] = cal->accel_scale[2] = 1.0f;
    cal->gyro_scale [0] = cal->gyro_scale [1] = cal->gyro_scale [2] = 1.0f;
    mat3_identity(cal->accel_misalign);
    mat3_identity(cal->mount_R_bv);
    cal->temperature_c = 25.0f;
    cal->valid_flags   = 0;
}

/* Sanity validation — returns 0 if ok, negative reason code otherwise.
 *   -1 = magic/version mismatch
 *   -2 = checksum mismatch (caller should compute before)
 *   -3 = gyro bias out of range
 *   -4 = accel bias out of range
 *   -5 = accel norm mean out of plausible gravity range
 *   -6 = variance excessive (vibration / motion during cal)
 *   -7 = NaN/Inf in any field
 */
static int imu_calib_validate(const imu_calib_t *cal) {
    int i, j;
    if (cal->magic != IMU_CALIB_MAGIC)   return -1;
    if (cal->version != IMU_CALIB_VERSION) return -1;
    // NaN/Inf scan over float members
    const float *f = (const float *)&cal->accel_bias[0];
    size_t nf = (offsetof(imu_calib_t, temperature_c) +
                 sizeof(cal->temperature_c) -
                 offsetof(imu_calib_t, accel_bias)) / sizeof(float);
    for (i = 0; i < (int)nf; i++) {
        float v = f[i];
        if (!(v == v) || (v > 1e30f) || (v < -1e30f)) return -7;
    }
    for (i = 0; i < 3; i++) {
        if (fabsf(cal->gyro_bias[i])  > CAL_SANITY_GYRO_BIAS_MAX)  return -3;
        if (fabsf(cal->accel_bias[i]) > CAL_SANITY_ACCEL_BIAS_MAX) return -4;
        // Scales must be near 1
        if (cal->accel_scale[i] < 0.5f || cal->accel_scale[i] > 1.5f) return -4;
        if (cal->gyro_scale [i] < 0.5f || cal->gyro_scale [i] > 1.5f) return -3;
    }
    if (cal->boot_sample_count > 0) {
        if (cal->boot_accel_norm_mean < CAL_SANITY_ACCEL_NORM_MIN ||
            cal->boot_accel_norm_mean > CAL_SANITY_ACCEL_NORM_MAX) return -5;
        for (i = 0; i < 3; i++) {
            if (cal->boot_accel_var[i] > CAL_SANITY_ACC_VAR_MAX)  return -6;
            if (cal->boot_gyro_var [i] > CAL_SANITY_GYRO_VAR_MAX) return -6;
        }
    }
    // Mount/misalign rows finite and not absurd
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
        float m = cal->mount_R_bv[i][j];
        if (!(m == m) || fabsf(m) > 2.0f) return -7;
        float a = cal->accel_misalign[i][j];
        if (!(a == a) || fabsf(a) > 2.0f) return -7;
    }
    return 0;
}

static int imu_calib_load(const char *path, imu_calib_t *cal) {
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    imu_calib_t tmp;
    ssize_t rd = read(fd, &tmp, sizeof(tmp));
    close(fd);
    if (rd != (ssize_t)sizeof(tmp)) return -2;
    uint32_t saved = tmp.checksum;
    uint32_t calc  = imu_calib_checksum(&tmp);
    if (saved != calc) return -2;
    int reason = imu_calib_validate(&tmp);
    if (reason != 0) return reason;
    *cal = tmp;
    return 0;
}

static int imu_calib_save_atomic(const char *path, const imu_calib_t *cal) {
#if !ENABLE_CAL_FILE_SAVE
    (void)path; (void)cal;
    return 0;
#else
    imu_calib_t tmp = *cal;
    tmp.checksum = imu_calib_checksum(&tmp);
    char tmp_path[256];
    int n = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    if (n <= 0 || n >= (int)sizeof(tmp_path)) return -1;
    int fd = open(tmp_path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return -2;
    ssize_t wr = write(fd, &tmp, sizeof(tmp));
    if (wr != (ssize_t)sizeof(tmp)) {
        close(fd); unlink(tmp_path); return -3;
    }
    if (fsync(fd) != 0) { close(fd); unlink(tmp_path); return -4; }
    close(fd);
    if (rename(tmp_path, path) != 0) { unlink(tmp_path); return -5; }
    return 0;
#endif
}

/* Bridge: convert legacy g_cal (LSQ + raw gyro counts) into the production
 * imu_calib_t bias fields. The 6-position LSQ accelerometer matrix in g_cal
 * already corrects for scale and cross-axis errors, so accel_misalign/scale
 * here stay at identity and the residual offset is folded into accel_bias.
 * Called whenever g_cal changes (boot cal completes or a fresh default load). */
static void imu_calib_sync_from_g_cal(imu_calib_t *cal) {
    int i;
    const float L = DEG2RAD / GYRO_LSB_PER_DPS;
    for (i = 0; i < 3; i++) {
        cal->gyro_bias[i]  = g_cal.gyro_bias_counts[i] * L;
        cal->accel_bias[i] = 0.0f;   // accel offset is baked into g_cal.accel_O
    }
    cal->gyro_scale[2] *= g_cal.gyro_z_scale;
}

static void cal_set_defaults_from_lsq( void ) {
    memset(&g_cal, 0, sizeof(g_cal));
    g_cal.magic = CAL_MAGIC;
    g_cal.version = CAL_VERSION;

    /* Seed with your current LSQ Values (so the behavior deosn't regress )*/
    g_cal.accel_C[0][0] =  -2.2750481313183215e-05f; g_cal.accel_C[0][1] =  5.765265695963156e-06f; g_cal.accel_C[0][2] = 0.0005988349971499568f;
    g_cal.accel_C[1][0] = -2.3509856530055813e-05f; g_cal.accel_C[1][1] =  0.0006000767045318638f; g_cal.accel_C[1][2] = -3.3823576680918605e-06f;
    g_cal.accel_C[2][0] = 0.0005917895662743669f; g_cal.accel_C[2][1] = 5.579396736232053e-05f; g_cal.accel_C[2][2] =   5.296175035094069e-06f;

    g_cal.accel_O[0] = -0.04324381676101664f;
    g_cal.accel_O[1] = 0.16600572009786152f;
    g_cal.accel_O[2] = 0.5714660018044386f;

    g_cal.gyro_bias_counts[0] = -38.741f;
    g_cal.gyro_bias_counts[1] = 57.638f;
    g_cal.gyro_bias_counts[2] = -16.246f;

    g_cal.gyro_z_scale = 1.0f;
    g_cal.accel_x_scale = 1.0f;
    g_cal.calibrated_once = 0;
}

#if 0
static void cal_ensure_dir(void) {
    // Ensure /var/lib/dr exists
    struct stat st;
    if(stat("/var/lib", &st) !=0 ) {
        /* /var/lib should exist on linux; ignore if not */
        return ;

    }
    if(stat("dr", &st) == 0 ) {
        if(S_ISDIR(st.st_mode)) return;
        /* if its a file don't overwrite */
        return;
    }
    (void)mkdir("dr", 0755);
}


static bool cal_load_from_file( void ) {
    int fd = open( CAL_FILE_PATH, O_RDONLY | O_CLOEXEC );
    if ( fd < 0 ) return false;

    dr_cal_t tmp;
    ssize_t rd = read( fd, &tmp, sizeof(tmp) );
    close(fd);
    if( rd != (ssize_t)sizeof(tmp)) return false;
    if( tmp.magic != CAL_MAGIC || tmp.version != CAL_VERSION ) return false;

    uint32_t saved_crc = tmp.crc32;
    tmp.crc32 = 0;
    uint32_t calc_crc = crc32_simple(&tmp, sizeof(tmp));
    if( calc_crc != saved_crc ) return false;

    tmp.crc32 = saved_crc;
    g_cal = tmp;
    return true;
}

static bool cal_save_to_file( void ){

    /*cal_ensure_dir();*/

    /* Update CRC */
    g_cal.crc32 = 0;
    g_cal.crc32 = crc32_simple(&g_cal, sizeof(g_cal));

    /*Atomic Write: write temp then rename */
    char tmp_path[256];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", CAL_FILE_PATH);

    int fd = open(tmp_path, O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if ( fd < 0 ) return false;

    ssize_t wr = write(fd, &g_cal, sizeof(g_cal));
    if( wr != (ssize_t)sizeof(g_cal)) { close(fd); unlink(tmp_path); return false; }

    fsync(fd);
    close(fd);
    if( rename(tmp_path, CAL_FILE_PATH ) != 0) { unlink(tmp_path); return false; }
    return true;
}
#endif 
static void calib_accel(const struct mpu6050_sample *raw, vec3f *acc_mps2) {
    float a0 = (float)raw->ax, a1 = (float)raw->ay, a2 = (float)raw->az;

    float x = g_cal.accel_C[0][0]*a0 + g_cal.accel_C[0][1]*a1 + g_cal.accel_C[0][2]*a2 + g_cal.accel_O[0];
    float y = g_cal.accel_C[1][0]*a0 + g_cal.accel_C[1][1]*a1 + g_cal.accel_C[1][2]*a2 + g_cal.accel_O[1];
    float z = g_cal.accel_C[2][0]*a0 + g_cal.accel_C[2][1]*a1 + g_cal.accel_C[2][2]*a2 + g_cal.accel_O[2];

    x *= g_cal.accel_x_scale; /**<< Optional tiny trim (leave at 1.0 unless enabled ) */

    acc_mps2->x = x;
    acc_mps2->y = y;
    acc_mps2->z = z;
}


static void calib_gyro(const struct mpu6050_sample *raw, vec3f *gyro_radps) {
    float g0 = (float)raw->gx - g_cal.gyro_bias_counts[0];
    float g1 = (float)raw->gy - g_cal.gyro_bias_counts[1];
    float g2 = (float)raw->gz - g_cal.gyro_bias_counts[2];
    
    float dps0 = g0 / GYRO_LSB_PER_DPS;
    float dps1 = g1 / GYRO_LSB_PER_DPS;
    float dps2 = (g2 / GYRO_LSB_PER_DPS) * g_cal.gyro_z_scale;

    gyro_radps->x = dps0 * DEG2RAD;
    gyro_radps->y = dps1 * DEG2RAD;
    gyro_radps->z = dps2 * DEG2RAD;
}

/* =====================================================================
 * Centralized IMU preprocessing
 *
 * Source-of-truth convention for biases (avoid double-subtract):
 *   g_cal              — owns the APPLIED bias / LSQ matrix / scale used
 *                        by calib_accel() and calib_gyro().
 *   g_imu_cal.*_bias   — mirrors the same SI bias values for persistence
 *                        and diagnostics ONLY.
 * imu_preprocess_sample() therefore does NOT re-subtract g_imu_cal.*_bias
 * (it would be applied twice). It only adds the corrections that g_cal
 * cannot represent: per-axis residual scale, cross-axis misalignment and
 * the mount→vehicle rotation.
 *
 * Order of operations:
 *   1. raw counts → physical units            (calib_accel / calib_gyro)
 *   2. accel_misalign (3×3, default identity)
 *   3. per-axis residual scale (gyro_scale, accel_scale)
 *   4. mount_R_bv: rotate IMU/body frame into vehicle frame
 *
 * The output vectors are in vehicle frame and SI units:
 *   accel_vehicle[] = m/s² with gravity still present
 *   gyro_vehicle[]  = rad/s
 *
 * Median/low-pass filtering is applied separately in the fusion thread
 * after this function returns, because the filter state is shared with
 * the bump detector. ===================================================*/
static void imu_preprocess_sample(const imu_calib_t *cal,
                                  const struct mpu6050_sample *raw,
                                  float accel_vehicle[3],
                                  float gyro_vehicle[3])
{
    /* 1) Raw → SI using legacy g_cal (bias already removed inside) */
    vec3f acc_b, gyro_b;
    calib_accel(raw,  &acc_b);
    calib_gyro (raw,  &gyro_b);

    /* 2) Optional accel cross-axis (misalignment) correction */
    float a_in[3]  = { acc_b.x,  acc_b.y,  acc_b.z };
    float a_aligned[3];
    mat3_mul_vec(cal->accel_misalign, a_in, a_aligned);

    /* 3) Per-axis residual scale (default 1.0 — no-op until set by runtime) */
    a_aligned[0] *= cal->accel_scale[0];
    a_aligned[1] *= cal->accel_scale[1];
    a_aligned[2] *= cal->accel_scale[2];

    float g_in[3] = { gyro_b.x, gyro_b.y, gyro_b.z };
    g_in[0] *= cal->gyro_scale[0];
    g_in[1] *= cal->gyro_scale[1];
    g_in[2] *= cal->gyro_scale[2];

    /* 4) IMU/body → vehicle frame (default mount = identity) */
    apply_mount_rotation(cal, a_aligned, accel_vehicle);
    apply_mount_rotation(cal, g_in,      gyro_vehicle);
}

/* =====================================================================
 * Conservative runtime gyro-bias learner.
 *
 * Updates g_imu_cal.gyro_bias (and the legacy g_cal counts) with a slow
 * LPF only when the vehicle has been confirmed stationary for at least
 * RT_CAL_WINDOW_S seconds. Accelerometer bias is intentionally NOT
 * updated at runtime — that requires confident attitude.
 *
 * Save is throttled to RT_CAL_MIN_SAVE_INTERVAL_S between persistence
 * writes; the file write is also performed on clean program exit.
 *
 * Returns true if a bias update was applied this call. =================*/
typedef struct {
    bool   armed;
    double t_start;
    double sum[3];        // rad/s
    double sum_sq[3];     // rad²/s²
    int    N;
} rt_cal_t;

static rt_cal_t g_rt_cal;

#if ENABLE_RUNTIME_GYRO_CAL
static bool runtime_gyro_cal_update(const float gyro_vehicle_rps[3],
                                    const float accel_vehicle_mps2[3],
                                    bool ekf_speed_low,
                                    bool gnss_speed_low,
                                    double now_s)
{
    float gnorm = sqrtf(gyro_vehicle_rps[0]*gyro_vehicle_rps[0] +
                        gyro_vehicle_rps[1]*gyro_vehicle_rps[1] +
                        gyro_vehicle_rps[2]*gyro_vehicle_rps[2]);
    float anorm = sqrtf(accel_vehicle_mps2[0]*accel_vehicle_mps2[0] +
                        accel_vehicle_mps2[1]*accel_vehicle_mps2[1] +
                        accel_vehicle_mps2[2]*accel_vehicle_mps2[2]);
    float adev  = fabsf(anorm - GRAVITY);
    bool still = (gnorm < (RT_CAL_GYRO_STILL_RAD * 1.7320508f)) &&  // ~root3 of per-axis
                 (adev  < RT_CAL_ACC_DEV_MPS2) &&
                 ekf_speed_low && gnss_speed_low &&
                 !g_imu_quality_degraded;

    if (!still) {
        g_rt_cal.armed = false;
        return false;
    }

    if (!g_rt_cal.armed) {
        g_rt_cal.armed   = true;
        g_rt_cal.t_start = now_s;
        g_rt_cal.N       = 0;
        g_rt_cal.sum[0] = g_rt_cal.sum[1] = g_rt_cal.sum[2] = 0.0;
        g_rt_cal.sum_sq[0] = g_rt_cal.sum_sq[1] = g_rt_cal.sum_sq[2] = 0.0;
    }
    g_rt_cal.sum[0]    += gyro_vehicle_rps[0];
    g_rt_cal.sum[1]    += gyro_vehicle_rps[1];
    g_rt_cal.sum[2]    += gyro_vehicle_rps[2];
    g_rt_cal.sum_sq[0] += (double)gyro_vehicle_rps[0] * gyro_vehicle_rps[0];
    g_rt_cal.sum_sq[1] += (double)gyro_vehicle_rps[1] * gyro_vehicle_rps[1];
    g_rt_cal.sum_sq[2] += (double)gyro_vehicle_rps[2] * gyro_vehicle_rps[2];
    g_rt_cal.N++;

    if ((now_s - g_rt_cal.t_start) < RT_CAL_WINDOW_S || g_rt_cal.N < 50)
        return false;

    /* Variance gate — verifies the window really was still */
    double invN = 1.0 / (double)g_rt_cal.N;
    double v0 = g_rt_cal.sum_sq[0]*invN - (g_rt_cal.sum[0]*invN)*(g_rt_cal.sum[0]*invN);
    double v1 = g_rt_cal.sum_sq[1]*invN - (g_rt_cal.sum[1]*invN)*(g_rt_cal.sum[1]*invN);
    double v2 = g_rt_cal.sum_sq[2]*invN - (g_rt_cal.sum[2]*invN)*(g_rt_cal.sum[2]*invN);
    if (v0 > CAL_SANITY_GYRO_VAR_MAX || v1 > CAL_SANITY_GYRO_VAR_MAX || v2 > CAL_SANITY_GYRO_VAR_MAX) {
        g_rt_cal.armed = false;
        return false;
    }

    float mean[3] = { (float)(g_rt_cal.sum[0]*invN),
                      (float)(g_rt_cal.sum[1]*invN),
                      (float)(g_rt_cal.sum[2]*invN) };

    /* The gyro samples passed into this function were produced by
     * calib_gyro(), which already subtracted g_cal.gyro_bias_counts.
     * The residual 'mean' is therefore the small drift not yet accounted
     * for. We push that residual directly into g_cal.gyro_bias_counts
     * (the single source of truth used by every preprocessing path),
     * scaled by RT_CAL_ALPHA so adaptation is slow. */
    float old_bias_rps[3] = {
        g_cal.gyro_bias_counts[0] * (DEG2RAD / GYRO_LSB_PER_DPS),
        g_cal.gyro_bias_counts[1] * (DEG2RAD / GYRO_LSB_PER_DPS),
        g_cal.gyro_bias_counts[2] * (DEG2RAD / GYRO_LSB_PER_DPS)
    };
    const float counts_per_rps = GYRO_LSB_PER_DPS / DEG2RAD;
    g_cal.gyro_bias_counts[0] += RT_CAL_ALPHA * mean[0] * counts_per_rps;
    g_cal.gyro_bias_counts[1] += RT_CAL_ALPHA * mean[1] * counts_per_rps;
    g_cal.gyro_bias_counts[2] += RT_CAL_ALPHA * mean[2] * counts_per_rps;
    float new_bias_rps[3] = {
        g_cal.gyro_bias_counts[0] * (DEG2RAD / GYRO_LSB_PER_DPS),
        g_cal.gyro_bias_counts[1] * (DEG2RAD / GYRO_LSB_PER_DPS),
        g_cal.gyro_bias_counts[2] * (DEG2RAD / GYRO_LSB_PER_DPS)
    };
    /* Keep g_imu_cal in sync with g_cal so a save captures the latest. */
    g_imu_cal.gyro_bias[0] = new_bias_rps[0];
    g_imu_cal.gyro_bias[1] = new_bias_rps[1];
    g_imu_cal.gyro_bias[2] = new_bias_rps[2];

    g_imu_cal.valid_flags |= CAL_FLAG_RUNTIME_OK;

    fprintf(stdout, "[CAL_RUNTIME gyro_bias_update] old=(%.4f,%.4f,%.4f)°/s new=(%.4f,%.4f,%.4f)°/s "
                    "win=%.1fs N=%d var=(%.2g,%.2g,%.2g)\n",
            old_bias_rps[0]/DEG2RAD, old_bias_rps[1]/DEG2RAD, old_bias_rps[2]/DEG2RAD,
            new_bias_rps[0]/DEG2RAD, new_bias_rps[1]/DEG2RAD, new_bias_rps[2]/DEG2RAD,
            (float)(now_s - g_rt_cal.t_start), g_rt_cal.N, v0, v1, v2);

    /* Throttled persistence */
#if ENABLE_CAL_FILE_SAVE
    if ((now_s - g_last_calib_save_s) >= RT_CAL_MIN_SAVE_INTERVAL_S) {
        if (imu_calib_save_atomic(IMU_CALIB_PATH, &g_imu_cal) == 0) {
            g_last_calib_save_s = now_s;
        }
    }
#endif

    /* Reset accumulator so we sample a fresh window next stop */
    g_rt_cal.armed = false;
    return true;
}
#endif /* ENABLE_RUNTIME_GYRO_CAL */

/* =====================================================================
 *   AHRS / complementary attitude filter
 *
 * Vehicle (= body) frame conventions in this file:
 *     X = vehicle forward
 *     Y = vehicle left
 *     Z = vehicle up
 *     +ωz (gyro Z) = counter-clockwise from above = left turn = +yaw rate
 *
 * Yaw convention (ENU, matching the rest of this codebase):
 *     yaw = 0     → vehicle X axis points East
 *     yaw = +π/2  → vehicle X axis points North
 *
 * State:
 *   roll, pitch, yaw — Euler angles, derived from quaternion q after each
 *                      update. q is the authoritative attitude (body→ENU).
 *
 * Update sequence on each IMU step (called from fusion_thread):
 *   1. gyro integration             → predict q
 *   2. accelerometer correction     → blend roll/pitch toward acc-derived
 *                                      angles, gated by ahrs_accel_is_trusted
 *   3. GNSS course correction       → blend yaw toward GNSS course, gated by
 *                                      ahrs_gnss_yaw_is_trusted
 *   4. extract Euler outputs        → roll/pitch/yaw used by ahrs_remove_gravity
 *                                      and by the 2D EKF as the yaw reference
 * ===================================================================== */

typedef enum {
    AHRS_QUALITY_GOOD            = 0,
    AHRS_QUALITY_ACCEL_REJECTED  = 1,   // accel correction skipped (this step)
    AHRS_QUALITY_BUMP            = 2,   // bump detector flagged degraded IMU
    AHRS_QUALITY_YAW_UNOBSERVED  = 3,   // no recent GNSS yaw correction
    AHRS_QUALITY_BAD             = 4    // both accel and yaw observations unavailable
} ahrs_quality_t;

typedef struct {
    /* Authoritative attitude */
    quatf q;        // body → ENU

    /* Euler outputs (derived from q after each step) */
    float roll;     // rad — right wing down positive
    float pitch;    // rad — nose up positive
    float yaw;      // rad — ENU heading, wrapped to (-π, π]

    /* (optional) Per-AHRS gyro bias estimate. Currently mirrors g_imu_cal so
     * the AHRS doesn't run a second bias estimator that diverges from the EKF. */
    float gyro_bias[3];

    /* Smoothed scalar diagnostics */
    float accel_norm_lpf;   // m/s²
    float gyro_norm_lpf;    // rad/s

    double last_update_s;
    int    initialized;

    /* Latched gate results from the most recent ahrs_update() call */
    int    accel_trusted;
    int    gnss_yaw_trusted;

    /* Configurable gains (set at init, may be tuned per quality) */
    float  roll_pitch_alpha;
    float  yaw_alpha_gnss;

    /* Uncertainty proxies (variance, rad²). Grow when corrections are
     * rejected; shrink when corrections are accepted. Read by EKF for
     * adaptive process-noise scaling. */
    float  roll_var;
    float  pitch_var;
    float  yaw_var;

    /* Heading observation latch */
    double last_yaw_obs_s;       // time of last accepted GNSS yaw correction

    ahrs_quality_t quality;

    /* Diagnostic counters */
    uint32_t reject_accel_count;
    uint32_t accept_accel_count;
    uint32_t gnss_yaw_update_count;
} ahrs_state_t;

/* (Reserved) Tracking for hard-accel detection via GNSS Δv / dt.
 * Not used yet — ahrs_accel_is_trusted currently gates on accel-norm/gyro/bump
 * which is sufficient for ground vehicles at IMU rate. */

/* Quaternion → Euler (ZYX) extraction used by the AHRS.
 *   yaw   = atan2(2(wz + xy), 1 - 2(y² + z²))
 *   pitch = asin (2(wy - zx))            — clamped before asin
 *   roll  = atan2(2(wx + yz), 1 - 2(x² + y²))
 * Same convention as q_from_euler() defined earlier in this file. */
static void q_to_euler_zyx(quatf q, float *roll, float *pitch, float *yaw) {
    q = q_normalize(q);
    float w = q.w, x = q.x, y = q.y, z = q.z;
    float sinp = 2.0f * (w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp);
    *roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    *yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}

/* Initialize AHRS from a stationary accelerometer sample and (optional) GNSS yaw.
 *   accel_vehicle[]   — vehicle-frame specific force, gravity present (m/s²).
 *   initial_yaw_rad   — ENU course (used only if yaw_valid != 0).
 *   yaw_valid         — 1 if GNSS heading was reliable, 0 otherwise. */
static void ahrs_init_from_accel(ahrs_state_t *A,
                                 const float accel_vehicle[3],
                                 float initial_yaw_rad,
                                 int yaw_valid)
{
    memset(A, 0, sizeof(*A));

    float amag = sqrtf(accel_vehicle[0]*accel_vehicle[0] +
                       accel_vehicle[1]*accel_vehicle[1] +
                       accel_vehicle[2]*accel_vehicle[2]);
    if (amag > 1.0f) {
        // Standard stationary accel→tilt: gravity vector in body frame
        // is [Rx, Ry, Rz]*g  ≈  accel_vehicle when at rest.
        A->pitch = atan2f(-accel_vehicle[0],
                          sqrtf(accel_vehicle[1]*accel_vehicle[1] +
                                accel_vehicle[2]*accel_vehicle[2]));
        A->roll  = atan2f(accel_vehicle[1], accel_vehicle[2]);
    } else {
        A->roll  = 0.0f;
        A->pitch = 0.0f;
    }

    if (yaw_valid) {
        A->yaw     = wrap_pi(initial_yaw_rad);
        A->yaw_var = (5.0f * DEG2RAD) * (5.0f * DEG2RAD);   // GNSS course σ≈3°
        A->quality = AHRS_QUALITY_GOOD;
    } else {
        A->yaw     = 0.0f;
        A->yaw_var = (60.0f * DEG2RAD) * (60.0f * DEG2RAD); // unknown
        A->quality = AHRS_QUALITY_YAW_UNOBSERVED;
    }

    A->q = q_from_euler(A->roll, A->pitch, A->yaw);

    A->roll_var          = (5.0f * DEG2RAD) * (5.0f * DEG2RAD);
    A->pitch_var         = (5.0f * DEG2RAD) * (5.0f * DEG2RAD);
    A->accel_norm_lpf    = amag;
    A->gyro_norm_lpf     = 0.0f;
    A->roll_pitch_alpha  = AHRS_ROLLPITCH_ALPHA;
    A->yaw_alpha_gnss    = AHRS_YAW_GNSS_ALPHA;

    A->initialized       = 1;
    A->accel_trusted     = 1;
    A->gnss_yaw_trusted  = yaw_valid;
    A->last_update_s     = now_sec();
    A->last_yaw_obs_s    = yaw_valid ? A->last_update_s : 0.0;
}

/* Accelerometer trust check.
 * Trusted iff the body is in quasi-static motion:
 *     accel norm close to gravity AND gyro low AND no bump active AND
 *     no large longitudinal acceleration inferred from GNSS speed change. */
static int ahrs_accel_is_trusted(const float accel_vehicle[3],
                                 const float gyro_vehicle[3],
                                 float vehicle_speed_mps)
{
    (void)vehicle_speed_mps;
    float amag = sqrtf(accel_vehicle[0]*accel_vehicle[0] +
                       accel_vehicle[1]*accel_vehicle[1] +
                       accel_vehicle[2]*accel_vehicle[2]);
    if (fabsf(amag - GRAVITY) > AHRS_ACCEL_NORM_TOL) return 0;
    if (fabsf(gyro_vehicle[0]) > AHRS_ACCEL_TRUST_GYRO_MAX) return 0;
    if (fabsf(gyro_vehicle[1]) > AHRS_ACCEL_TRUST_GYRO_MAX) return 0;
    if (fabsf(gyro_vehicle[2]) > AHRS_ACCEL_TRUST_GYRO_MAX) return 0;
    if (g_imu_quality_degraded) return 0;   // bump detector vetoes accel correction
    return 1;
}

/* GNSS-yaw trust check. */
static int ahrs_gnss_yaw_is_trusted(int gnss_heading_valid,
                                    float gnss_heading_rad,
                                    float gnss_speed_mps,
                                    float gnss_hdop,
                                    float current_yaw_rad)
{
    if (!gnss_heading_valid) return 0;
    if (gnss_speed_mps < AHRS_GNSS_YAW_SPEED_MIN) return 0;
    if (gnss_hdop > AHRS_GNSS_YAW_HDOP_MAX) return 0;
    float innov = wrap_pi(gnss_heading_rad - current_yaw_rad);
    if (fabsf(innov) > AHRS_YAW_INNOV_MAX) return 0;
    return 1;
}

/* Remove the gravity contribution from accel_vehicle using AHRS attitude.
 * Output is the linear (vehicle-frame) acceleration excluding gravity.
 *
 *   gravity_body = R^T * [0, 0, g]  with  R = R_from_q(A->q)  (body→ENU)
 *   lin_accel    = accel_vehicle − gravity_body
 *
 * For a level vehicle, gravity_body = [0, 0, g] and lin_accel[2] removes g,
 * lin_accel[0] is forward acceleration, lin_accel[1] is lateral. */
static void ahrs_remove_gravity(const ahrs_state_t *A,
                                const float accel_vehicle[3],
                                float lin_accel_vehicle[3])
{
    float R[9]; R_from_q(A->q, R);
    /* R^T column 2 = R row 2 = [R[6], R[7], R[8]] — gravity expressed in body. */
    float gx_b = R[6] * GRAVITY;
    float gy_b = R[7] * GRAVITY;
    float gz_b = R[8] * GRAVITY;
    lin_accel_vehicle[0] = accel_vehicle[0] - gx_b;
    lin_accel_vehicle[1] = accel_vehicle[1] - gy_b;
    lin_accel_vehicle[2] = accel_vehicle[2] - gz_b;
}

/* Central AHRS update.
 * Must be called every IMU step AFTER imu_preprocess_sample() and BEFORE
 * ins2d_predict(). Updates A->q and the derived euler outputs.            */
static void ahrs_update(ahrs_state_t *A,
                        const float accel_vehicle[3],
                        const float gyro_vehicle[3],
                        float dt,
                        int   gnss_heading_valid,
                        float gnss_heading_rad,
                        float gnss_speed_mps,
                        float gnss_hdop)
{
#if !ENABLE_AHRS_FILTER
    (void)A; (void)accel_vehicle; (void)gyro_vehicle; (void)dt;
    (void)gnss_heading_valid; (void)gnss_heading_rad; (void)gnss_speed_mps; (void)gnss_hdop;
    return;
#else
    if (!A->initialized) return;
    double now_s = now_sec();
    A->last_update_s = now_s;

    /* Diagnostic norms (LPF over a few seconds for stability) */
    const float dn_alpha = 0.05f;
    float amag = sqrtf(accel_vehicle[0]*accel_vehicle[0] +
                       accel_vehicle[1]*accel_vehicle[1] +
                       accel_vehicle[2]*accel_vehicle[2]);
    float gmag = sqrtf(gyro_vehicle [0]*gyro_vehicle [0] +
                       gyro_vehicle [1]*gyro_vehicle [1] +
                       gyro_vehicle [2]*gyro_vehicle [2]);
    A->accel_norm_lpf = (1.0f - dn_alpha)*A->accel_norm_lpf + dn_alpha*amag;
    A->gyro_norm_lpf  = (1.0f - dn_alpha)*A->gyro_norm_lpf  + dn_alpha*gmag;

    /* 1) Predict: integrate gyro into the quaternion.
     *    The gyro vector here is already vehicle-frame, bias-removed by the
     *    preprocessor + runtime cal — we don't subtract another bias here. */
    vec3f w = v3(gyro_vehicle[0], gyro_vehicle[1], gyro_vehicle[2]);
    quatf dq = q_from_small_angle(v3_scale(w, dt));
    A->q = q_normalize(q_mul(A->q, dq));

    /* Process-noise-like growth of uncertainty per step */
    float sigma_w_step2 = (0.5f * DEG2RAD)*(0.5f * DEG2RAD) * dt;   // (0.5°/s/√Hz)²·dt
    A->roll_var  += sigma_w_step2;
    A->pitch_var += sigma_w_step2;
    A->yaw_var   += sigma_w_step2;

    /* 2) Accelerometer roll/pitch correction (gated) */
#if ENABLE_AHRS_ACCEL_CORR
    int acc_ok = ahrs_accel_is_trusted(accel_vehicle, gyro_vehicle, gnss_speed_mps);
    A->accel_trusted = acc_ok;
    if (acc_ok) {
        /* Tilt from gravity vector in vehicle frame */
        float roll_acc  = atan2f(accel_vehicle[1], accel_vehicle[2]);
        float pitch_acc = atan2f(-accel_vehicle[0],
                                 sqrtf(accel_vehicle[1]*accel_vehicle[1] +
                                       accel_vehicle[2]*accel_vehicle[2]));
        /* Read current euler from q, then blend roll/pitch only, keep yaw */
        float r_cur, p_cur, y_cur;
        q_to_euler_zyx(A->q, &r_cur, &p_cur, &y_cur);
        float r_new = (1.0f - A->roll_pitch_alpha)*r_cur + A->roll_pitch_alpha*roll_acc;
        float p_new = (1.0f - A->roll_pitch_alpha)*p_cur + A->roll_pitch_alpha*pitch_acc;
        A->q   = q_from_euler(r_new, p_new, y_cur);
        A->roll  = r_new;
        A->pitch = p_new;
        A->yaw   = y_cur;
        A->roll_var  *= (1.0f - A->roll_pitch_alpha);
        A->pitch_var *= (1.0f - A->roll_pitch_alpha);
        A->accept_accel_count++;
#if ENABLE_AHRS_DEBUG_LOGS
        printf("[AHRS_ACCEL_ACCEPT] roll_acc=%.2f pitch_acc=%.2f\n",
               roll_acc/DEG2RAD, pitch_acc/DEG2RAD);
#endif
    } else {
        A->reject_accel_count++;
#if ENABLE_AHRS_DEBUG_LOGS
        printf("[AHRS_ACCEL_REJECT] reason=%s accel_norm=%.2f gyro_norm=%.2f\n",
               g_imu_quality_degraded ? "bump" : "dynamic",
               amag, gmag/DEG2RAD);
#endif
    }
#endif /* ENABLE_AHRS_ACCEL_CORR */

    /* 3) GNSS yaw / course correction (gated) */
#if ENABLE_AHRS_GNSS_YAW
    /* Need a current yaw estimate that reflects step (1) and (2) */
    float r_cur2, p_cur2, y_cur2;
    q_to_euler_zyx(A->q, &r_cur2, &p_cur2, &y_cur2);
    int yaw_ok = ahrs_gnss_yaw_is_trusted(gnss_heading_valid, gnss_heading_rad,
                                          gnss_speed_mps, gnss_hdop, y_cur2);
    A->gnss_yaw_trusted = yaw_ok;
    if (yaw_ok) {
        float innov = wrap_pi(gnss_heading_rad - y_cur2);
        float y_new = wrap_pi(y_cur2 + A->yaw_alpha_gnss * innov);
        A->q = q_from_euler(r_cur2, p_cur2, y_new);
        A->roll  = r_cur2;
        A->pitch = p_cur2;
        A->yaw   = y_new;
        A->yaw_var *= (1.0f - A->yaw_alpha_gnss);
        A->gnss_yaw_update_count++;
        A->last_yaw_obs_s = now_s;
#if ENABLE_AHRS_DEBUG_LOGS
        printf("[AHRS_GNSS_YAW_UPDATE] old=%.2f gnss=%.2f new=%.2f innov=%.2f\n",
               y_cur2/DEG2RAD, gnss_heading_rad/DEG2RAD, y_new/DEG2RAD, innov/DEG2RAD);
#endif
    }
#endif /* ENABLE_AHRS_GNSS_YAW */

    /* 4) Refresh Euler outputs from authoritative q */
    q_to_euler_zyx(A->q, &A->roll, &A->pitch, &A->yaw);

    /* 5) Quality classification (used by EKF for process-noise adaptation) */
    if (g_imu_quality_degraded) {
        A->quality = AHRS_QUALITY_BUMP;
    } else if (!A->accel_trusted && (now_s - A->last_yaw_obs_s) > 30.0) {
        A->quality = AHRS_QUALITY_BAD;
    } else if (!A->accel_trusted) {
        A->quality = AHRS_QUALITY_ACCEL_REJECTED;
    } else if ((now_s - A->last_yaw_obs_s) > 60.0) {
        A->quality = AHRS_QUALITY_YAW_UNOBSERVED;
    } else {
        A->quality = AHRS_QUALITY_GOOD;
    }
#endif /* ENABLE_AHRS_FILTER */
}

/* EKF yaw-measurement update: feed AHRS yaw into the 2D EKF as a
 * scalar heading observation. Keeps the two yaw integrators coupled
 * so they cannot drift apart. */
#if USE_2D_EKF
static void ins2d_update_yaw_from_ahrs(ins2d_t *S, float ahrs_yaw, float R_yaw)
{
    /* Reuse the existing scalar heading update path. ins2d_update_heading
     * already wraps the innovation correctly. */
    ins2d_update_heading(S, ahrs_yaw, R_yaw, NULL);
}
#endif
typedef struct {
    bool   active;
    int    N;
    double t0;
    double t_last_progress;     /* for 1-Hz progress logs */
    double gyro_sum[3];         /* raw counts */
    double acc_sum[3];          /* m/s^2 (after LSQ) */
    double gyro_sum_sq[3];      /* raw counts²  — for variance */
    double acc_sum_sq[3];       /* m²/s⁴        — for variance */
    double acc_norm_sum;        /* |a|  — for norm mean/std */
    double acc_norm_sum_sq;
    double gyro_norm_sum;       /* |gyro| in rad/s */
    double gyro_norm_sum_sq;
} still_accum_t;

static void still_reset(still_accum_t *A ){
    memset(A, 0, sizeof(*A));
}

/* Boot calibration result code:
 *    1 = success (g_cal & g_imu_cal updated, calibration persisted to flash)
 *    0 = in progress
 *   -1 = motion detected; accumulator reset
 *   -2 = completed but rejected by sanity check (calibration NOT applied) */
static int apply_poweron_calibration(still_accum_t *A,
    const struct mpu6050_sample *raw,
    vec3f acc_b_lsq,
    float still_required_s,
    double now_s )
{
    /* ---- Stillness verification ----
     * Industry-standard practice: only accumulate samples while the vehicle
     * is genuinely stationary. If motion is detected (engine vibration above
     * threshold, vehicle moving, hand-held shake, etc.) restart accumulation.
     * Otherwise the vibration is averaged into the gyro/accel bias estimate
     * and the EKF starts with a biased IMU, causing the position scatter
     * we observed in earlier logs.
     */
    const float Lgz_inv = DEG2RAD / GYRO_LSB_PER_DPS;  // counts -> rad/s
    float gx_rps = (float)raw->gx * Lgz_inv;
    float gy_rps = (float)raw->gy * Lgz_inv;
    float gz_rps = (float)raw->gz * Lgz_inv;
    float acc_norm = v3_norm(acc_b_lsq);
    float acc_dev = fabsf(acc_norm - GRAVITY);
    float gyro_norm = sqrtf(gx_rps*gx_rps + gy_rps*gy_rps + gz_rps*gz_rps);

    bool moving = (fabsf(gx_rps) > BOOT_CAL_GYRO_STILL_RAD ||
                   fabsf(gy_rps) > BOOT_CAL_GYRO_STILL_RAD ||
                   fabsf(gz_rps) > BOOT_CAL_GYRO_STILL_RAD ||
                   acc_dev      > BOOT_CAL_ACC_STILL_MPS2);

    if (moving) {
        static double t_last_warn = 0.0;
        if (now_s - t_last_warn > BOOT_CAL_RESET_GRACE_S) {
            printf("[CAL_BOOT moved] restarting reason=|g_dev|=%.2f gyro=(%.2f,%.2f,%.2f)°/s\n",
                   acc_dev, gx_rps/DEG2RAD, gy_rps/DEG2RAD, gz_rps/DEG2RAD);
            t_last_warn = now_s;
        }
        still_reset(A);
        return -1;
    }

    /* Start accumulation on first call (or after a reset) */
    if( !A->active ){
        A->active = true;
        A->t0 = now_s;
        A->t_last_progress = now_s;
    }

    /* Accumulate raw gyro counts (and counts² for variance) */
    A->gyro_sum   [0] += (double)raw->gx;
    A->gyro_sum   [1] += (double)raw->gy;
    A->gyro_sum   [2] += (double)raw->gz;
    A->gyro_sum_sq[0] += (double)raw->gx * (double)raw->gx;
    A->gyro_sum_sq[1] += (double)raw->gy * (double)raw->gy;
    A->gyro_sum_sq[2] += (double)raw->gz * (double)raw->gz;

    /* Accumulate LSQ-corrected accel and accel² */
    A->acc_sum   [0] += (double)acc_b_lsq.x;
    A->acc_sum   [1] += (double)acc_b_lsq.y;
    A->acc_sum   [2] += (double)acc_b_lsq.z;
    A->acc_sum_sq[0] += (double)acc_b_lsq.x * (double)acc_b_lsq.x;
    A->acc_sum_sq[1] += (double)acc_b_lsq.y * (double)acc_b_lsq.y;
    A->acc_sum_sq[2] += (double)acc_b_lsq.z * (double)acc_b_lsq.z;
    A->acc_norm_sum     += (double)acc_norm;
    A->acc_norm_sum_sq  += (double)acc_norm * (double)acc_norm;
    A->gyro_norm_sum    += (double)gyro_norm;
    A->gyro_norm_sum_sq += (double)gyro_norm * (double)gyro_norm;

    A->N++;

    /* 1 Hz progress log */
    if (now_s - A->t_last_progress >= 1.0) {
        float elapsed = (float)(now_s - A->t0);
        printf("[CAL_BOOT progress] samples=%d stable_s=%.1f/%g accel_norm=%.3f gyro_norm=%.3f°/s\n",
               A->N, elapsed, still_required_s, acc_norm, gyro_norm/DEG2RAD);
        A->t_last_progress = now_s;
    }

    if((now_s - A->t0) < (double)still_required_s)
        return 0;

    /*--------------------Finalize calibration ----------------------*/

    double invN = 1.0 / (double)A->N;

    /* Means */
    double gx_mean = A->gyro_sum[0] * invN;
    double gy_mean = A->gyro_sum[1] * invN;
    double gz_mean = A->gyro_sum[2] * invN;
    vec3f acc_mean = v3((float)(A->acc_sum[0] * invN),
                        (float)(A->acc_sum[1] * invN),
                        (float)(A->acc_sum[2] * invN));

    /* Sample variance (E[x²] - E[x]², counts²) */
    double gx_var_raw = A->gyro_sum_sq[0]*invN - gx_mean*gx_mean;
    double gy_var_raw = A->gyro_sum_sq[1]*invN - gy_mean*gy_mean;
    double gz_var_raw = A->gyro_sum_sq[2]*invN - gz_mean*gz_mean;
    // Convert gyro variance from raw² counts to rad²/s²
    double gscale2 = (double)Lgz_inv * (double)Lgz_inv;
    double gx_var = gx_var_raw * gscale2;
    double gy_var = gy_var_raw * gscale2;
    double gz_var = gz_var_raw * gscale2;
    double ax_var = A->acc_sum_sq[0]*invN - (double)acc_mean.x*acc_mean.x;
    double ay_var = A->acc_sum_sq[1]*invN - (double)acc_mean.y*acc_mean.y;
    double az_var = A->acc_sum_sq[2]*invN - (double)acc_mean.z*acc_mean.z;
    double acc_norm_mean   = A->acc_norm_sum * invN;
    double acc_norm_var    = A->acc_norm_sum_sq*invN - acc_norm_mean*acc_norm_mean;
    double gyro_norm_mean  = A->gyro_norm_sum * invN;
    double gyro_norm_var   = A->gyro_norm_sum_sq*invN - gyro_norm_mean*gyro_norm_mean;

    /* Build a candidate imu_calib_t */
    imu_calib_t cand;
    imu_calib_set_defaults(&cand);

    /* 1) Gyro bias in raw counts (legacy g_cal) — also record in cand in SI */
    float gyro_bias_counts[3] = { (float)gx_mean, (float)gy_mean, (float)gz_mean };
    cand.gyro_bias[0] = gyro_bias_counts[0] * Lgz_inv;
    cand.gyro_bias[1] = gyro_bias_counts[1] * Lgz_inv;
    cand.gyro_bias[2] = gyro_bias_counts[2] * Lgz_inv;

    /* 2) Accel offset trim — enforce |a| = g along measured gravity direction.
     * NOTE: This is conservative; we do NOT assume axes are zero. We only push
     *       the measured gravity vector to the correct magnitude. */
    float acc_offset_trim[3] = {0,0,0};
    float n = v3_norm(acc_mean);
    if (n > 1e-3f) {
        vec3f g_dir = v3_scale(acc_mean, 1.0f/n);
        vec3f acc_exp = v3_scale(g_dir, GRAVITY);
        vec3f resid   = v3_sub(acc_mean, acc_exp);
        acc_offset_trim[0] = resid.x;
        acc_offset_trim[1] = resid.y;
        acc_offset_trim[2] = resid.z;
    }
    cand.accel_bias[0] = acc_offset_trim[0];
    cand.accel_bias[1] = acc_offset_trim[1];
    cand.accel_bias[2] = acc_offset_trim[2];

    /* 3) Quality metrics */
    cand.boot_accel_var[0]    = (float)fmax(0.0, ax_var);
    cand.boot_accel_var[1]    = (float)fmax(0.0, ay_var);
    cand.boot_accel_var[2]    = (float)fmax(0.0, az_var);
    cand.boot_gyro_var [0]    = (float)fmax(0.0, gx_var);
    cand.boot_gyro_var [1]    = (float)fmax(0.0, gy_var);
    cand.boot_gyro_var [2]    = (float)fmax(0.0, gz_var);
    cand.boot_accel_norm_mean = (float)acc_norm_mean;
    cand.boot_accel_norm_std  = (float)sqrt(fmax(0.0, acc_norm_var));
    cand.boot_gyro_norm_mean  = (float)gyro_norm_mean;
    cand.boot_gyro_norm_std   = (float)sqrt(fmax(0.0, gyro_norm_var));
    cand.boot_sample_count    = (uint32_t)A->N;
    cand.created_unix_s       = (uint64_t)time(NULL);
    cand.temperature_c        = 25.0f;
    cand.valid_flags          = CAL_FLAG_BOOT_OK;

    /* 4) Sanity-check the candidate before committing */
    int rej = imu_calib_validate(&cand);
    if (rej != 0) {
        printf("[CAL_BOOT rejected] reason=%d gyro_bias=(%.3f,%.3f,%.3f)°/s "
               "accel_bias=(%.3f,%.3f,%.3f) acc_norm=%.3f±%.3f var=(%.4f,%.4f,%.4f)\n",
               rej,
               cand.gyro_bias[0]/DEG2RAD, cand.gyro_bias[1]/DEG2RAD, cand.gyro_bias[2]/DEG2RAD,
               cand.accel_bias[0], cand.accel_bias[1], cand.accel_bias[2],
               cand.boot_accel_norm_mean, cand.boot_accel_norm_std,
               cand.boot_accel_var[0], cand.boot_accel_var[1], cand.boot_accel_var[2]);
        still_reset(A);
        return -2;
    }

    /* 5) Commit: legacy g_cal (used by calib_accel/calib_gyro) AND g_imu_cal */
    g_cal.gyro_bias_counts[0] = gyro_bias_counts[0];
    g_cal.gyro_bias_counts[1] = gyro_bias_counts[1];
    g_cal.gyro_bias_counts[2] = gyro_bias_counts[2];
    g_cal.accel_O[0] -= acc_offset_trim[0];
    g_cal.accel_O[1] -= acc_offset_trim[1];
    g_cal.accel_O[2] -= acc_offset_trim[2];
    g_cal.calibrated_once = 1;

    /* Keep mount/misalign/scale from g_imu_cal — they survive boot cal */
    {
        float saved_scale_a[3] = { g_imu_cal.accel_scale[0], g_imu_cal.accel_scale[1], g_imu_cal.accel_scale[2] };
        float saved_scale_g[3] = { g_imu_cal.gyro_scale[0],  g_imu_cal.gyro_scale[1],  g_imu_cal.gyro_scale[2] };
        float saved_mount[3][3], saved_mis[3][3];
        memcpy(saved_mount, g_imu_cal.mount_R_bv,     sizeof(saved_mount));
        memcpy(saved_mis,   g_imu_cal.accel_misalign, sizeof(saved_mis));
        g_imu_cal = cand;
        memcpy(g_imu_cal.mount_R_bv,     saved_mount, sizeof(saved_mount));
        memcpy(g_imu_cal.accel_misalign, saved_mis,   sizeof(saved_mis));
        memcpy(g_imu_cal.accel_scale,    saved_scale_a, sizeof(saved_scale_a));
        memcpy(g_imu_cal.gyro_scale,     saved_scale_g, sizeof(saved_scale_g));
    }

    /* 6) Persist atomically */
#if ENABLE_CAL_FILE_SAVE
    int save_rc = imu_calib_save_atomic(IMU_CALIB_PATH, &g_imu_cal);
    if (save_rc == 0) {
        g_last_calib_save_s = now_s;
        printf("[CAL_BOOT success] samples=%u path=%s gyro_bias=(%.3f,%.3f,%.3f)°/s "
               "accel_bias=(%.3f,%.3f,%.3f) acc_norm=%.3f±%.3f\n",
               cand.boot_sample_count, IMU_CALIB_PATH,
               cand.gyro_bias[0]/DEG2RAD, cand.gyro_bias[1]/DEG2RAD, cand.gyro_bias[2]/DEG2RAD,
               cand.accel_bias[0], cand.accel_bias[1], cand.accel_bias[2],
               cand.boot_accel_norm_mean, cand.boot_accel_norm_std);
    } else {
        printf("[CAL_BOOT success (no-persist)] samples=%u save_rc=%d gyro_bias=(%.3f,%.3f,%.3f)°/s\n",
               cand.boot_sample_count, save_rc,
               cand.gyro_bias[0]/DEG2RAD, cand.gyro_bias[1]/DEG2RAD, cand.gyro_bias[2]/DEG2RAD);
    }
#else
    printf("[CAL_BOOT success (persist disabled)] samples=%u\n", cand.boot_sample_count);
#endif

    g_imu_quality_degraded = 0;
    still_reset(A);
    return 1;
}

#if 0
/** Returns true if a still-window completed and applied an update */
static bool apply_stationary_calibration( still_accum_t *A, bool zupt_cond, 
    const struct mpu6050_sample *raw, 
    vec3f acc_b,
    quatf q_be,                 // body->ENU attitude (needed for accel trim )
    bool allow_accel_trim,      // false before nav_ready (gyro bias only)
    float still_required_s,     // e.g. 4.0 first-install, 3.0 runtime
    bool is_first_install,
    double now_s ) {
        int i;
        if( !zupt_cond ){
            still_reset(A);
            return false;
        }

        if( !A->active ){
            A->active = true;
            A->t0 = now_s;
            A->N = 0;
            A->gyro_sum[0]=A->gyro_sum[1]=A->gyro_sum[2] = 0.0;
            A->acc_sum[0]=A->acc_sum[1]=A->acc_sum[2] =0.0;
        }

        A->gyro_sum[0] += (double)raw->gx;
        A->gyro_sum[1] += (double)raw->gy;
        A->gyro_sum[2] += (double)raw->gz;

        A->acc_sum[0] += (double)acc_b.x;
        A->acc_sum[1] += (double)acc_b.y;
        A->acc_sum[2] += (double)acc_b.z;

        A->N++;

        double dt = now_s - A->t0;
        if( dt < (double)still_required_s ) return false;

        double invN = 1.0/(double)A->N;

        /** Gyro Bisa counts  */
        float meas_bg[3] = {
            (float)(A->gyro_sum[0] * invN),
            (float)(A->gyro_sum[1] * invN),
            (float)(A->gyro_sum[2] * invN)
        };

        // First Install: take mean directly. Runtime: gentle LPF update
        const float k_bg = is_first_install ? 1.0f : 0.05f;
        for( i=0; i<3; i++) {
            g_cal.gyro_bias_counts[i] = (1.0f - k_bg)*g_cal.gyro_bias_counts[i] + k_bg * meas_bg[i];
        }

        // ---2) Accel offset trim vs gravity (only if we trust attitude )---
        if( allow_accel_trim ){
            float R[9];
            R_from_q(q_be, R);

            /* Expected body specific force when stationary: R^T * [0,0,g] */
            vec3f fb_exp_b = v3(
                R[0]*0 + R[3]*0 + R[6]*GRAVITY,
                R[1]*0 + R[4]*0 + R[7]*GRAVITY,
                R[2]*0 + R[5]*0 + R[8]*GRAVITY
            );

            vec3f acc_mean  = v3(
                R[0]*0 + R[3]*0 + R[6]*GRAVITY,
                R[1]*0 + R[4]*0 + R[7]*GRAVITY,
                R[2]*0 + R[5]*0 + R[8]*GRAVITY
            );

            vec3f r = v3_sub(acc_mean, fb_exp_b);

            const float kO = is_first_install ? 0.30f : 0.05f; /* Conservative at runtime */
            g_cal.accel_O[0] -= kO * r.x;
            g_cal.accel_O[1] -= kO * r.y;
            g_cal.accel_O[2] -= kO * r.z;
        }

        still_reset(A);
        return true;
    }
#endif 

    /** ---------------------------------------GNSS yaw-rate learner (turn-segment) ----------------- 
     * Goal: learn a small gyro z scale correction (g_cal.gyro_z_scale) using GNSS heading (COG) while moving
     * Bias is learned via stillness (ZARU-like) using apply_stationary_calibration()
     * 
     * Approach: accumlate a "turn segment" while turning:
     * delta_psi_gnss = wrap(yaw_end - yaw_start)
     * delta_psi_gyro = integration of (gz_counts - bias_counts)*L dt (UNSCALED by gyro_z_scale)
     * scale_est = delta_psi_gnss/delta_psi_gyro
     * gyro_z_scale <- LPF(gyro_scale, scale_est)
     * 
     * This is robust because it uses integrated angle over seconds (reduces GNSS heading noise )
    */
   typedef struct {
    bool have_gnss;
    int64_t last_gnss_ns;
    float last_gnss_yaw;

    bool seg_active;
    double seg_tO_s;
    float seg_yaw_start;
    float seg_yaw_end;
    float seg_gyro_dpsi;
    double seg_last_update_s;

    double last_persist_s;
   }  yaw_learn_t;

   static void yaw_learn_reset_seg( yaw_learn_t *Y, double now_s ){
    Y->seg_active = false;
    Y->seg_tO_s = now_s;
    Y->seg_yaw_start = 0.0f;
    Y->seg_yaw_end = 0.0f;
    Y->seg_gyro_dpsi = 0.0f;
    Y->seg_last_update_s = now_s;
   }

   static void yaw_learn_on_gnss_fix( yaw_learn_t *Y, float yaw_enu_rad, int64_t fix_ns ) {
    Y->have_gnss = true;
    Y->last_gnss_ns = fix_ns;
    Y->last_gnss_yaw = yaw_enu_rad;
    if( Y->seg_active ){
        Y->seg_yaw_end = yaw_enu_rad;
    }
   }

   static void yaw_learn_try_finalize(yaw_learn_t *Y, double now_s ) {
    if ( !Y->seg_active ) return;

    double seg_dt = now_s - Y->seg_tO_s;
    if( seg_dt < YAW_LEARN_MIN_SEG_S ) return;

    // Finalize if too long 
    if (seg_dt > YAW_LEARN_MAX_SEG_S ) {
        yaw_learn_reset_seg(Y, now_s);
        return;
    }

    float dpsi_gnss = wrap_pi(Y->seg_yaw_end - Y->seg_yaw_start );
    float dpsi_gnss_deg = fabsf(dpsi_gnss)/DEG2RAD;

    if( dpsi_gnss_deg < YAW_LEARN_MIN_TURN_DEG ) return;
    if( fabsf(Y->seg_gyro_dpsi) < 0.10f ) return;

    // Scale estimate
    float scale_est = dpsi_gnss / Y->seg_gyro_dpsi;
    //Reject crazy estimates (GNSS glitch or slip )
    if (scale_est < 0.5f || scale_est > 1.5f ) {
        yaw_learn_reset_seg(Y, now_s);
        return;
    }

    //LPF update + clamp
    float s = g_cal.gyro_z_scale;
    s = (1.0f - YAW_LEARN_BETA_SCALE ) * s + (YAW_LEARN_BETA_SCALE) * scale_est;
    s = clampf(s, YAW_LEARN_SCALE_MIN, YAW_LEARN_SCALE_MAX );
    g_cal.gyro_z_scale = s;

    yaw_learn_reset_seg(Y, now_s);
   }

   static void yaw_learn_on_imu(
    yaw_learn_t *Y,
    double now_s,
    float dt,
    const struct mpu6050_sample *raw,
    bool gnss_ok, 
    float gnss_speed_mps,
    bool turning_ok
   ){
    // Always integrate gyro delta during an active segment.
    if( Y->seg_active ){
        /* Convert raw counts -> rad/s (UNSCALED) */
        const float L = (DEG2RAD / GYRO_LSB_PER_DPS ); // rad/s per count
        float gz_unscaled = ((float)raw->gz - g_cal.gyro_bias_counts[2]) * L;
        Y->seg_gyro_dpsi += gz_unscaled * dt;
        Y->seg_last_update_s = now_s;

        /*If GNSS went bad or speed low, finalize/reset gracefully */
        if( !gnss_ok || gnss_speed_mps < YAW_LEARN_MIN_SPEED_MPS ){
            yaw_learn_try_finalize(Y, now_s);
            return;
        }

        /* If turning stopped, try finalize */
        if( !turning_ok ){
            yaw_learn_try_finalize(Y, now_s);
        }
        return;
    }

    /* Start a segment only when GNSS is good, moving, and turning meaningfully */
    if(!gnss_ok) return;
    if (gnss_speed_mps < YAW_LEARN_MIN_SPEED_MPS ) return;
    if( !turning_ok )return;
    if( !Y->have_gnss ) return;

    Y->seg_active = true;
    Y->seg_tO_s = now_s;
    Y->seg_yaw_start = Y->last_gnss_yaw;
    Y->seg_yaw_end = Y->last_gnss_yaw;
    Y->seg_gyro_dpsi = 0.0f;
    Y->seg_last_update_s = now_s;

   }

   static void yaw_learn_maybe_persist(yaw_learn_t *Y, double now_s ){
    if((now_s - Y->last_persist_s ) <  YAW_LEARN_PERSIST_PERIOD_S ) return;
    /* Persist slow terms: biases already learned by stillness calibration; scale learned here */
        Y->last_persist_s = now_s;
   }


// ------------------------- 15-state INS EKF -------------------------

typedef struct {
    // Nominal
    vec3f p;     // ENU (m)
    vec3f v;     // ENU (m/s)
    quatf q;     // body->ENU

    // Bias estimates (nominal)
    vec3f ba;    // accel bias (m/s^2)
    vec3f bg;    // gyro bias (rad/s)

    // Error-state covariance (15x15)
    float P[15*15];
} ins15_t;

static void ins15_init(ins15_t *S) {
    memset(S, 0, sizeof(*S));
    S->q = q_identity();
    int i;

    // Conservative initial covariance (tune)
    const float p0 = IMU_COVAR_POS;          // m          
    const float v0 = IMU_COVAR_VEL;          // m/s       
    const float th0 = IMU_COVAR_ATT; // rad       
    const float ba0 = IMU_COVAR_BA;         // m/s^2      
    const float bg0 = IMU_COVAR_BG;        // rad/s    

    for (i=0;i<15;i++) S->P[i*15+i] = 1e-6f;
    for (i=0;i<3;i++) S->P[(0+i)*15 + (0+i)] = p0*p0;
    for (i=0;i<3;i++) S->P[(3+i)*15 + (3+i)] = v0*v0;
    for (i=0;i<3;i++) S->P[(6+i)*15 + (6+i)] = th0*th0;
    for (i=0;i<3;i++) S->P[(9+i)*15 + (9+i)] = ba0*ba0;
    for (i=0;i<3;i++) S->P[(12+i)*15 + (12+i)] = bg0*bg0;
}

static inline float compute_qscale(double outage_s) {
    if (outage_s < TIER_A) return QSCL_A;
    if (outage_s < TIER_B) return QSCL_B;
    if (outage_s < TIER_C) return QSCL_C;
    return QSCL_D;
}

static void mat15_mul(const float *A, const float *B, float *C) {
    int r, c, k;
    for (r=0;r<15;r++) {
        for (c=0;c<15;c++) {
            float s=0;
            for (k=0;k<15;k++) s += A[r*15+k]*B[k*15+c];
            C[r*15+c]=s;
        }
    }
}
static void mat15_T(const float *A, float *AT) {
    int r, c;
    for (r=0;r<15;r++) for (c=0;c<15;c++) AT[c*15+r]=A[r*15+c];
}
static void mat15_add(float *A, const float *B) { int i; for (i=0;i<15*15;i++) A[i]+=B[i]; }

// Mechanization + covariance propagation (discrete-time linearized, ENU, flat Earth)
static void ins15_predict(ins15_t *S, vec3f acc_meas_b, vec3f gyro_meas_b, float dt, float q_scale, vec3f *out_aw) {
    int i, r, c, k;
    // Remove estimated biases
    vec3f w = v3_sub(gyro_meas_b, S->bg);
    vec3f fb = v3_sub(acc_meas_b, S->ba);

    // Optional yaw deadband when near still (reduce yaw random walk when stopped)
    float an = v3_norm(acc_meas_b);
    bool near_still = fabsf(an - GRAVITY) < ACC_STILL_TOL;
    if (near_still && fabsf(w.z) < YAW_DEADBAND_RAD) w.z = 0.0f;

    // Attitude integration (small-angle)
    quatf dq = q_from_small_angle(v3_scale(w, dt));
    S->q = q_normalize(q_mul(S->q, dq));

    // Rotate specific force to ENU and subtract gravity
    float R[9]; R_from_q(S->q, R);
    vec3f f_enu = v3(
        R[0]*fb.x + R[1]*fb.y + R[2]*fb.z,
        R[3]*fb.x + R[4]*fb.y + R[5]*fb.z,
        R[6]*fb.x + R[7]*fb.y + R[8]*fb.z
    );
    vec3f a_enu = v3(f_enu.x, f_enu.y, f_enu.z - GRAVITY);

    // Clamp ENU acceleration to vehicle dynamics limits
    // Catches IMU spikes, calibration errors, and attitude-induced gravity leakage
    a_enu.x = clampf(a_enu.x, -MAX_ACCEL_ENU_MPS2, MAX_ACCEL_ENU_MPS2);
    a_enu.y = clampf(a_enu.y, -MAX_ACCEL_ENU_MPS2, MAX_ACCEL_ENU_MPS2);
    a_enu.z = clampf(a_enu.z, -MAX_ACCEL_ENU_MPS2, MAX_ACCEL_ENU_MPS2);

    // Kinematics
    S->p = v3_add(S->p, v3_add(v3_scale(S->v, dt), v3_scale(a_enu, 0.5f*dt*dt)));
    S->v = v3_add(S->v, v3_scale(a_enu, dt));

    if (out_aw) *out_aw = a_enu;

    // Linearized error dynamics matrix F (15x15) for flat ENU
    // x = [dp dv dtheta ba bg]
    float F[15*15]; memset(F, 0, sizeof(F));
    // dp_dot = dv
    for (i=0;i<3;i++) F[(0+i)*15 + (3+i)] = 1.0f;
    // dv_dot = -R*skew(fb)*dtheta - R*ba
    // dtheta_dot = -skew(w)*dtheta - bg
    // ba_dot = 0 (random walk), bg_dot = 0 (random walk)

    // Build A = -R*skew(fb)
    float fx=fb.x, fy=fb.y, fz=fb.z;
    float skew_f[9] = { 0, -fz,  fy,
                        fz,  0, -fx,
                       -fy, fx,  0 };
    float A[9]; // -R * skew(f)
    for (r=0;r<3;r++) for (c=0;c<3;c++) {
        float s=0;
        for (k=0;k<3;k++) s += R[r*3+k]*skew_f[k*3+c];
        A[r*3+c] = -s;
    }
    // dv wrt dtheta
    for (r=0;r<3;r++) for (c=0;c<3;c++) F[(3+r)*15 + (6+c)] = A[r*3+c];
    // dv wrt ba: -R
    for (r=0;r<3;r++) for (c=0;c<3;c++) F[(3+r)*15 + (9+c)] = -R[r*3+c];

    // dtheta wrt dtheta: -skew(w)
    float wx=w.x, wy=w.y, wz=w.z;
    float skew_w[9] = { 0, -wz,  wy,
                        wz,  0, -wx,
                       -wy, wx,  0 };
    for (r=0;r<3;r++) for (c=0;c<3;c++) F[(6+r)*15 + (6+c)] = -skew_w[r*3+c];
    // dtheta wrt bg: -I
    for (i=0;i<3;i++) F[(6+i)*15 + (12+i)] = -1.0f;

    // Discrete Phi ≈ I + F dt
    float Phi[15*15]; memset(Phi, 0, sizeof(Phi));
    for (i=0;i<15;i++) Phi[i*15+i] = 1.0f;
    for (r=0;r<15;r++) for (c=0;c<15;c++) Phi[r*15+c] += F[r*15+c]*dt;

    // Process noise (continuous densities, scaled during outages)
    // Tune these to your IMU
    const float sigma_a = IMU_SIGMA_ACCEL;     // m/s^2
    const float sigma_g = IMU_SIGMA_GYRO;    // rad/s
    const float sigma_ba = IMU_SIGMA_ACCEL_BIAS;   // m/s^2/sqrt(s)
    const float sigma_bg = IMU_SIGMA_GYRO_BIAS;   // rad/s/sqrt(s)

    float sa2 = (sigma_a*sigma_a) * q_scale;
    float sg2 = (sigma_g*sigma_g) * q_scale;
    float sba2 = (sigma_ba*sigma_ba) * q_scale;
    float sbg2 = (sigma_bg*sigma_bg) * q_scale;

    // Discrete Qd (simple diagonal approximation in error-state space)
    // dv driven by accel noise; dtheta driven by gyro noise
    float Q[15*15]; memset(Q, 0, sizeof(Q));
    for (i=0;i<3;i++) {
        Q[(3+i)*15 + (3+i)] = sa2 * dt;     // dv
        Q[(6+i)*15 + (6+i)] = sg2 * dt;     // dtheta
        Q[(9+i)*15 + (9+i)] = sba2 * dt;    // ba RW
        Q[(12+i)*15 + (12+i)] = sbg2 * dt;  // bg RW
    }
    // Position noise: double-integrated accel noise + position random walk
    // The random walk term prevents the filter from over-trusting IMU-propagated position
    const float sigma_p_rw = 0.015f; // m/sqrt(s) — position random walk (prevents P_pos shrinking too much)
    float sp2 = (sigma_p_rw * sigma_p_rw) * q_scale;
    for (i=0;i<3;i++) Q[(0+i)*15 + (0+i)] = sa2 * dt*dt*dt / 3.0f + sp2 * dt;

    // P = Phi P Phi^T + Q
    float PhiP[15*15], PhiT[15*15], PhiPPhiT[15*15];
    mat15_mul(Phi, S->P, PhiP);
    mat15_T(Phi, PhiT);
    mat15_mul(PhiP, PhiT, PhiPPhiT);
    memcpy(S->P, PhiPPhiT, sizeof(PhiPPhiT));
    mat15_add(S->P, Q);
}

static void ins15_inject(ins15_t *S, const float dx[15]) {
    // Nominal state correction using estimated error-state
    S->p.x += dx[0];  S->p.y += dx[1];  S->p.z += dx[2];
    S->v.x += dx[3];  S->v.y += dx[4];  S->v.z += dx[5];

    vec3f dth = v3(dx[6], dx[7], dx[8]);
    S->q = q_normalize(q_mul(S->q, q_from_small_angle(dth)));

    S->ba.x += dx[9];  S->ba.y += dx[10]; S->ba.z += dx[11];
    S->bg.x += dx[12]; S->bg.y += dx[13]; S->bg.z += dx[14];

    // Optional: Joseph-form covariance update would be better; we do standard KF update,
    // then reset: P = (I-KH)P(I-KH)^T + K R K^T (done in update), so no extra here.
}

// Read-only NIS computation — does NOT modify state or covariance
static bool kf_compute_nis_3(const ins15_t *S, const float H[3*15],
                              const float z[3], const float h[3],
                              const float Rdiag[3], float *out_NIS) {
    int r, c, k, i, j;
    // S33 = H P H^T + R  (3x3)
    float HP[3*15];
    for (r=0;r<3;r++)
        for (c=0;c<15;c++) {
            float s=0;
            for (k=0;k<15;k++) s += H[r*15+k]*S->P[k*15+c];
            HP[r*15+c]=s;
        }
    float S33[9];
    for (r=0;r<3;r++) for (c=0;c<3;c++) {
        float s=0;
        for (k=0;k<15;k++) s += HP[r*15+k]*H[c*15+k];
        if (r==c) s += Rdiag[r];
        S33[r*3+c]=s;
    }
    float Sinv[9];
    if (!mat3_inv(S33, Sinv)) return false;

    float nu[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };
    float tmp[3]={0};
    for(i=0;i<3;i++) for(j=0;j<3;j++) tmp[i] += nu[j]*Sinv[j*3+i];
    *out_NIS = nu[0]*tmp[0] + nu[1]*tmp[1] + nu[2]*tmp[2];
    return true;
}

static bool kf_update_3(ins15_t *S, const float H[3*15], const float z[3], const float h[3], const float Rdiag[3], float *out_NIS) {
    int i, r, c, k, j;
    // S = H P H^T + R (3x3)
    float HP[3*15];
    for (r=0;r<3;r++) {
        for (c=0;c<15;c++) {
            float s=0;
            for (k=0;k<15;k++) s += H[r*15+k]*S->P[k*15+c];
            HP[r*15+c]=s;
        }
    }
    float S33[9];
    for (r=0;r<3;r++) for (c=0;c<3;c++) {
        float s=0;
        for (k=0;k<15;k++) s += HP[r*15+k]*H[c*15+k];
        if (r==c) s += Rdiag[r];
        S33[r*3+c]=s;
    }
    float Sinv[9];
    if (!mat3_inv(S33, Sinv)) return false;

    float nu[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };

    if (out_NIS) {
        float tmp[3]={0};
        for(i=0;i<3;i++) for (j=0;j<3;j++) tmp[i] += nu[j]*Sinv[j*3+i];
        *out_NIS = nu[0]*tmp[0] + nu[1]*tmp[1] + nu[2]*tmp[2];
    }

    // K = P H^T Sinv  (15x3)
    float PHt[15*3];
    for (r=0;r<15;r++) for (c=0;c<3;c++) {
        float s=0;
        for (k=0;k<15;k++) s += S->P[r*15+k]*H[c*15+k];
        PHt[r*3+c]=s;
    }
    float K[15*3];
    for (r=0;r<15;r++) for (c=0;c<3;c++) {
        float s=0;
        for (k=0;k<3;k++) s += PHt[r*3+k]*Sinv[k*3+c];
        K[r*3+c]=s;
    }

    // dx = K nu
    float dx[15]={0};
    for (r=0;r<15;r++) dx[r] = K[r*3+0]*nu[0] + K[r*3+1]*nu[1] + K[r*3+2]*nu[2];
    ins15_inject(S, dx);

    // Joseph covariance update: P = (I-KH)P(I-KH)^T + K R K^T
    float KH[15*15]; memset(KH,0,sizeof(KH));
    for (r=0;r<15;r++) for (c=0;c<15;c++) {
        float s=0;
        for (k=0;k<3;k++) s += K[r*3+k]*H[k*15+c];
        KH[r*15+c]=s;
    }
    float I_KH[15*15]; memset(I_KH,0,sizeof(I_KH));
    for (i=0;i<15;i++) I_KH[i*15+i]=1.0f;
    for (i=0;i<15*15;i++) I_KH[i] -= KH[i];

    float tmp[15*15], I_KH_T[15*15], P1[15*15];
    mat15_mul(I_KH, S->P, tmp);
    mat15_T(I_KH, I_KH_T);
    mat15_mul(tmp, I_KH_T, P1);

    float KRKt[15*15]; memset(KRKt,0,sizeof(KRKt));
    for (r=0;r<15;r++) {
        for (c=0;c<15;c++) {
            float s=0;
            for (k=0;k<3;k++) {
                float Rk = Rdiag[k];
                s += K[r*3+k] * Rk * K[c*3+k];
            }
            KRKt[r*15+c]=s;
        }
    }
    for (i=0;i<15*15;i++) S->P[i] = P1[i] + KRKt[i];
    return true;
}

static bool ins15_update_gnss_pos(ins15_t *S, vec3f zpos, float Rpos, float *out_NIS) {
    int i;
    float H[3*15]; memset(H,0,sizeof(H));
    // z = p + n  => nu uses nominal p, and H maps error δp
    for (i=0;i<3;i++) H[i*15 + (0+i)] = 1.0f;

    float z[3] = { zpos.x, zpos.y, zpos.z };
    float h[3] = { S->p.x, S->p.y, S->p.z };
    float Rdiag[3] = { Rpos, Rpos, Rpos };
    return kf_update_3(S, H, z, h, Rdiag, out_NIS);
}

static bool ins15_update_gnss_vel(ins15_t *S, vec3f zvel, const float Rvdiag[3], float *out_NIS) {
    int i;
    float H[3*15]; memset(H,0,sizeof(H));
    for (i=0;i<3;i++) H[i*15 + (3+i)] = 1.0f; // δv
    float z[3] = { zvel.x, zvel.y, zvel.z };
    float h[3] = { S->v.x, S->v.y, S->v.z };
    return kf_update_3(S, H, z, h, Rvdiag, out_NIS);
}

// NIS-only (read-only) wrappers — compute innovation test without modifying state
static bool ins15_nis_gnss_pos(const ins15_t *S, vec3f zpos, float Rpos, float *out_NIS) {
    int i;
    float H[3*15]; memset(H,0,sizeof(H));
    for (i=0;i<3;i++) H[i*15 + (0+i)] = 1.0f;
    float z[3] = { zpos.x, zpos.y, zpos.z };
    float h[3] = { S->p.x, S->p.y, S->p.z };
    float Rdiag[3] = { Rpos, Rpos, Rpos };
    return kf_compute_nis_3(S, H, z, h, Rdiag, out_NIS);
}

static bool ins15_nis_gnss_vel(const ins15_t *S, vec3f zvel, const float Rvdiag[3], float *out_NIS) {
    int i;
    float H[3*15]; memset(H,0,sizeof(H));
    for (i=0;i<3;i++) H[i*15 + (3+i)] = 1.0f;
    float z[3] = { zvel.x, zvel.y, zvel.z };
    float h[3] = { S->v.x, S->v.y, S->v.z };
    return kf_compute_nis_3(S, H, z, h, Rvdiag, out_NIS);
}

static void ins15_update_zupt(ins15_t *S) {
    int i;
    // 3 sequential scalar updates on v components with z=0
    for (i=0;i<3;i++) {
        float H[3*15]; memset(H,0,sizeof(H));
        H[0*15 + (3+i)] = 1.0f;
        float z[3]={0,0,0};
        float h[3]={0,0,0};
        h[0] = (i==0)?S->v.x:(i==1)?S->v.y:S->v.z;
        float Rdiag[3]={R_ZUPT_V, 1e9f, 1e9f}; // only first row meaningful
        float NIS;
        (void)NIS;
        // Hack: reuse 3D update with large noise in unused dims; cheap & robust
        kf_update_3(S, H, z, h, Rdiag, NULL);
    }
}

static void ins15_update_zaru(ins15_t *S, vec3f gyro_meas_b ){
    int i;
    for( i=0; i<3; i++ ){
        float H[3*15]; memset(H, 0, sizeof(H));
        H[0*15 + (12+i)] = 1.0f;    //deltabg axis i

        float z[3] = {0,0,0};
        float h[3] = {0,0,0};

        float meas = (i==0)? gyro_meas_b.x:(i==1) ? gyro_meas_b.y:gyro_meas_b.z;
        float pred = (i==0)? S->bg.x:(i==1)?S->bg.y:S->bg.z;

        z[0] = meas;
        h[0] = pred;

        // Tune: smaller means stronger bias locking when still
        // Start conservative; adjust down if yaw still drifts at stops.

        const float R_W = 1e-5f; /* rad/s^2*/
        float Rdiag[3] = {R_W, 1e9f, 1e9f};
        kf_update_3(S, H, z, h, Rdiag, NULL);
    }
}

static void ins15_update_nhc(ins15_t *S) {
    int c;
    // Measurement: [v_y^b, v_z^b] ~ 0
    float R[9]; R_from_q(S->q, R);
    // v_b = R^T v_enu
    float RT[9] = { R[0],R[3],R[6],
                    R[1],R[4],R[7],
                    R[2],R[5],R[8] };
    vec3f vb = v3(
        RT[0]*S->v.x + RT[1]*S->v.y + RT[2]*S->v.z,
        RT[3]*S->v.x + RT[4]*S->v.y + RT[5]*S->v.z,
        RT[6]*S->v.x + RT[7]*S->v.y + RT[8]*S->v.z
    );

    // H for 2D measurement is (2 x 15). We'll embed into 3D update with dummy 3rd.
    // y-row measures vb.y; z-row measures vb.z.
    // vb = RT * v => d(vb) = RT * δv
    float H[3*15]; memset(H,0,sizeof(H));
    // row0 -> vb.y
    for (c=0;c<3;c++) H[0*15 + (3+c)] = RT[3 + c];
    // row1 -> vb.z
    for (c=0;c<3;c++) H[1*15 + (3+c)] = RT[6 + c];

    /**
     * d(vb)/d(theta) = skew(vb)
     * skew(vb) = [0 -vz vy
     *              vz 0 -vx
     *              -vy vx 0]
     * row0 corresponds to vb.y -> [-vy vx 0]
     */
    H[0*15 + (6+0)] = vb.z;
    H[0*15 + (6+1)] = 0.0f;
    H[0*15 + (6+2)] = -vb.x;

    // row 1 corresponds to vb.z -> [-vy, vx, 0]
    H[1*15 + (6+0)] = -vb.y;
    H[1*15 + (6+1)] = vb.x;
    H[1*15 + (6+2)] = 0.0f;

    float z[3] = {0,0,0};
    float h[3] = { vb.y, vb.z, 0.0f };
    float Rdiag[3] = { R_NHC_VY, R_NHC_VZ, 1e9f };
    kf_update_3(S, H, z, h, Rdiag, NULL);
}



// Scalar heading (yaw) measurement update through the EKF
// z_yaw = GPS course-over-ground (ENU convention), R_hdg = measurement variance (rad^2)
static void ins15_update_heading(ins15_t *S, float z_yaw, float R_hdg) {
    int r, c;
    float h_yaw = yaw_from_q(S->q);
    float nu = wrap_pi(z_yaw - h_yaw);

    // H = [0 0 0, 0 0 0, 0 0 1, 0 0 0, 0 0 0]  (scalar, 1x15)
    // Only element: H[8] = 1.0  (δθ_z)
    // S_scalar = H P H^T + R = P[8*15+8] + R_hdg
    float S_scalar = S->P[8*15 + 8] + R_hdg;
    if (S_scalar < 1e-12f) return;
    float Sinv = 1.0f / S_scalar;

    // K = P * H^T * Sinv  (15x1) — H^T has only element [8]=1
    float K[15];
    for (r=0; r<15; r++) K[r] = S->P[r*15 + 8] * Sinv;

    // dx = K * nu
    float dx[15];
    for (r=0; r<15; r++) dx[r] = K[r] * nu;
    ins15_inject(S, dx);

    // Joseph-form covariance update: P = (I-KH)P(I-KH)^T + K R K^T
    // KH is 15x15 with only column 8 nonzero: KH[r][8] = K[r]
    // (I-KH)[r][c] = I[r][c] - K[r]*delta(c,8)
    float Pnew[15*15];
    for (r=0; r<15; r++) {
        for (c=0; c<15; c++) {
            // (I-KH)*P row r, col c = P[r][c] - K[r]*P[8][c]
            Pnew[r*15+c] = S->P[r*15+c] - K[r]*S->P[8*15+c];
        }
    }
    // Now Pnew = (I-KH)*P. Apply *(I-KH)^T + KRK^T:
    float Pfinal[15*15];
    for (r=0; r<15; r++) {
        for (c=0; c<15; c++) {
            Pfinal[r*15+c] = Pnew[r*15+c] - Pnew[r*15+8]*K[c] + K[r]*R_hdg*K[c];
        }
    }
    memcpy(S->P, Pfinal, sizeof(Pfinal));
}

// ------------------------- GNSS helper (your ioctl pattern) -------------------------

static int open_imu(void) {
    int fd = open(IMU_DEVICE_PATH, O_RDONLY | O_CLOEXEC);
    return (fd < 0) ? -errno : fd;
}
static int open_gnss(void) {
    int fd = open(NEO6M_DEVICE_PATH, O_RDONLY | O_CLOEXEC);
    return (fd < 0) ? -errno : fd;
}

static int get_gnss_fix_ioctl(int fd, struct neo6m_gnss_fix *f) {
    if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, f) != 0) return -errno;
    return 0;
}

// ------------------------- Threading / Context -------------------------

typedef struct {
    // latest raw IMU
    struct mpu6050_sample imu_raw;
    bool have_imu;

    // latest GNSS snapshot (decoded)
    bool have_gnss;
    int  gnss_have_fix;
    double gnss_lat_rad;
    double gnss_lon_rad;
    double gnss_alt_m;
    float gnss_speed_mps;

    // optional heading/HDOP
    bool gnss_heading_valid;
    float gnss_heading_rad;
    bool gnss_hdop_valid;
    float gnss_hdop;

    // derived ENU velocity from speed+heading (if available)
    bool gnss_vel_valid;
    vec3f gnss_vel_enu;

    // for logging (store last GNSS even if no new fix this second)
    bool gnss_last_valid;
    int gnss_last_fix;
    double gnss_last_lat_deg, gnss_last_lon_deg, gnss_last_alt_m;
    float gnss_last_speed_mps, gnss_last_heading_deg, gnss_last_hdop;
    float gnss_last_course_deg, gnss_last_yaw_enu_deg;

    #if USE_2D_EKF
    // 2D EKF state (x=[p_x, p_y, v_x, v_y, theta], no biases)
    ins2d_t ins2d;
    quatf       att_q;       // legacy mirror of ahrs.q (kept for downstream readers)
    ahrs_state_t ahrs;       // production complementary attitude filter
    vec3f last_aw;
    #else
    // INS
    ins15_t ins;
    vec3f last_aw;
    #endif


    // ENU reference
    bool enu_ref_set;
    lla_t enu_ref_lla;
    ecef_t enu_ref_ecef;

    // nav state
    bool nav_ready;

    // GNSS presence / outage / reacquisition tracking
double last_gnss_meas_s;     // last time we RECEIVED a GNSS fix (raw measurement)
double last_gnss_used_s;     // last time we ACCEPTED (used) GNSS position update
double last_gnss_fused_s;    // last time we ENTERED the GNSS fusion path (for 10 s throttle)
bool   gnss_present;        // derived: (now - last_gnss_meas_s) <= GNSS_TIMEOUT_S
bool   gnss_valid;          // legacy: last accepted pos update (kept for logs)
int    reacq_left;          // >0 while in reacquisition mode (relaxed gating)
bool   reacq_active;        // latched for 1 Hz logs
bool   snap_applied;        // latched for 1 Hz logs
double outage_s;            // seconds since last_gnss_meas_s (raw outage duration)
float  qscale;              // process-noise scale (depends ONLY on gnss_present/outage_s)
    // ZUPT
    int zupt_count;

    // NIS for logging/diagnostics
float last_nis_pos;
float last_nis_vel;

// Latched per-1Hz log interval
int   last_gnss_used_pos;   // 1 if a GNSS pos update was accepted since last log line
int   last_gnss_used_vel;   // 1 if a GNSS vel update was accepted since last log line
    // last computed world acceleration (ENU)


    // logging
    FILE *logf;
    char log_filename[256];
    FILE *dbglog;
    char dbglog_filename[256];
    double last_log_s;

    int64_t last_gnss_fix_mono_ns;

    vec3f gnss_enu_last;
    bool gnss_enu_last_valid;

    // GNSS speed spike filter state
    float gnss_prev_speed_mps;  // previous accepted speed for rate-of-change check
    double gnss_prev_speed_time; // monotonic time of previous speed
    bool  gnss_prev_speed_valid; // false until first accepted fix
    bool  gnss_speed_rejected;   // latched for debug log: last fix had speed spike

    // threading
    pthread_mutex_t mtx;
    pthread_cond_t cv;
    bool running;

    // Stationary position anchor (eliminates drift when confirmed still)
    vec3f anchor_p;             // position when stillness first confirmed
    bool  anchor_set;           // true while vehicle is confirmed stationary
    int   sustained_still;      // counts how long we've been in ZUPT

    // Predict-only position snapshot (for diagnostic logging)
    vec3f predict_p;            // position right after ins15_predict(), before updates

    // GNSS alpha-beta tracker state (sawtooth correction)
    bool  ab_valid;             // true after first fix initialises the tracker
    vec3f ab_pos;               // tracker's smoothed position (ENU)
    vec3f ab_vel;               // tracker's smoothed velocity (ENU, m/s)
    double ab_time;             // timestamp of last tracker update

    // Calibration on Power On
    bool imu_cal_in_progress;
    bool imu_cal_done;
    double imu_cal_start_s;
    vec3f boot_acc_mean;        // mean accel from power-on calibration (for initial tilt)
    bool heading_observed;      // true after first GPS heading update at speed
    int  gnss_fix_count;        // counts valid GNSS fixes received (for cold-start skip)
    int  gnss_accepted_count;   // counts GNSS position updates accepted by EKF (for adaptive HDOP)
    double last_moving_s;       // last time EKF velocity exceeded motion threshold (for ZUPT inhibit)

} ctx_t;

static void make_log_file(ctx_t *C) {
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "mkdir -p %s", LOG_DIR);
    (void)system(cmd);

    time_t t = time(NULL);
    struct tm tmv;
    localtime_r(&t, &tmv);

    snprintf(C->log_filename, sizeof(C->log_filename),
             "%s/navlog_%04d-%02d-%02d_%02d-%02d-%02d.csv",
             LOG_DIR,
             tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec);

    C->logf = fopen(C->log_filename, "w");
    if (!C->logf) {
        fprintf(stderr, "ERROR: unable to create log file at %s: %s\n", C->log_filename, strerror(errno));
        exit(1);
    }

    fprintf(C->logf,
        "time_s,"
        "ekf_lat,ekf_lon,"
        "V_E,V_N,V_U,"
        "A_E,A_N,A_U,"
        "yaw_deg,"
        "ba_x,ba_y,ba_z,"
        "bg_x,bg_y,bg_z,"
        "imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,"
        "gnss_fix,lat_deg,lon_deg,alt_m,speed_mps,course_deg,yaw_enu_deg,hdop,"
        "gnss_E,gnss_N,gnss_U,err_E,err_N,err_H,"
        "gnss_present,gnss_valid,gnss_used_pos,gnss_used_vel,reacq_active,snap_applied,"
        "outage_s,qscale,nis_pos,nis_vel,gnss_meas_age_s,"
        "pred_E,pred_N,pred_U,zupt_active,anchor_active\n"
    );
    fflush(C->logf);

    fprintf(stderr, "Logging to %s\n", C->log_filename);
    C->last_log_s = 0.0;
}

static void make_debug_log_file(ctx_t *C){
    /* Assume make_log_file already created nav_log folder */
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);

    snprintf(C->dbglog_filename, sizeof(C->dbglog_filename),
            "/home/sijeo/nav_logs/debug_%04d%02d%02d_%02d%02d%02d.log",
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

    C->dbglog = fopen(C->dbglog_filename, "w");
    if( !C->dbglog ){
        perror("fopen dbglog ");
        return;
    }
    setvbuf(C->dbglog, NULL, _IOLBF, 0); /* Line Buffered */
}



// ------------------------- IMU thread -------------------------

static void* imu_thread(void *arg) {
    ctx_t *C = (ctx_t*)arg;
    int fd = open_imu();
    if (fd < 0) {
        fprintf(stderr, "open_imu failed: %d\n", fd);
        return NULL;
    }

    while (__atomic_load_n(&C->running, __ATOMIC_ACQUIRE)) {
        struct mpu6050_sample s;
        ssize_t n = read(fd, &s, sizeof(s));
        if (n == (ssize_t)sizeof(s)) {
            pthread_mutex_lock(&C->mtx);
            C->imu_raw = s;
            C->have_imu = true;
            pthread_cond_signal(&C->cv);
            pthread_mutex_unlock(&C->mtx);
        } else {
            usleep(1000);
        }
    }

    close(fd);
    return NULL;
}

// ------------------------- GNSS thread -------------------------

static pthread_mutex_t dbglog_mtx = PTHREAD_MUTEX_INITIALIZER;

static void dbg_printf(ctx_t *C, const char *fmt, ... ){
    if(!C || !C->dbglog ){
        return;
    }
    pthread_mutex_lock(&dbglog_mtx);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    fprintf(C->dbglog, "[%lld.%03lld]", (long long)ts.tv_sec, (long long)(ts.tv_nsec/1000000));

    va_list ap;
    va_start( ap, fmt );
    vfprintf(C->dbglog, fmt, ap);
    va_end(ap);

    fputc('\n', C->dbglog);
    pthread_mutex_unlock(&dbglog_mtx);
}

static void* gnss_thread(void *arg) {
    ctx_t *C = (ctx_t*)arg;
    int fd = open_gnss();
    if (fd < 0) {
        fprintf(stderr, "open_gnss failed: %d\n", fd);
        return NULL;
    }

    while (__atomic_load_n(&C->running, __ATOMIC_ACQUIRE)) {
        struct neo6m_gnss_fix f;
        int rc = get_gnss_fix_ioctl(fd, &f);
        if (rc != 0) {
            usleep(10000);
            continue;
        }

        pthread_mutex_lock(&C->mtx);

        C->have_gnss = false;
        C->gnss_have_fix = f.have_fix ? 1 : 0;
        double tmeas = now_sec();
        C->last_gnss_meas_s = tmeas;
        bool new_fix = true;
        if( C->gnss_have_fix ){
            if( C->last_gnss_fix_mono_ns == f.monotonic_ns ){
                new_fix = false;

            } else {
                C->last_gnss_fix_mono_ns = f.monotonic_ns;
            }
        }

        if (C->gnss_have_fix) {
            double lat_deg = (double)f.lat_e7 / 1e7;
            double lon_deg = (double)f.lon_e7 / 1e7;
            double alt_m   = (double)f.alt_mm / 1000.0;

            C->gnss_lat_rad = lat_deg * (M_PI/180.0);
            C->gnss_lon_rad = lon_deg * (M_PI/180.0);
            C->gnss_alt_m   = alt_m;
            float raw_speed = (float)f.speed_mmps / 1000.0f;

            // --- GNSS speed spike filter ---
            // Reject if: (a) exceeds absolute vehicle limit, or
            //            (b) rate-of-change exceeds max physically possible acceleration
            bool speed_ok = true;
            C->gnss_speed_rejected = false;

            if (raw_speed > MAX_GNSS_SPEED_MPS) {
                speed_ok = false;  // absolute cap
            } else if (C->gnss_prev_speed_valid) {
                double dt_gnss = tmeas - C->gnss_prev_speed_time;
                if (dt_gnss > 0.1 && dt_gnss < 10.0) {
                    float delta_v = fabsf(raw_speed - C->gnss_prev_speed_mps);
                    float max_delta = MAX_GNSS_ACCEL_MPS2 * (float)dt_gnss;
                    if (delta_v > max_delta) {
                        speed_ok = false;  // rate-of-change exceeded
                    }
                }
            }

            if (speed_ok) {
                C->gnss_speed_mps = raw_speed;
                C->gnss_prev_speed_mps = raw_speed;
                C->gnss_prev_speed_time = tmeas;
                C->gnss_prev_speed_valid = true;
            } else {
                // Speed spike: keep previous accepted speed, invalidate heading+velocity
                C->gnss_speed_rejected = true;
                dbg_printf(C, "GNSS_SPIKE speed=%.2f prev=%.2f REJECTED", raw_speed,
                           C->gnss_prev_speed_valid ? C->gnss_prev_speed_mps : -1.0f);
                // Use last good speed for logging, but mark heading/vel invalid below
            }

#if HAVE_GNSS_HDOP
            // Expected fields if you extended your driver:
            //   f.hdop_valid (bool-like), f.hdop_x100 (uint16/uint8)
            // If not present, set HAVE_GNSS_HDOP=0.
            if (f.hdop_valid) {
                C->gnss_hdop = (float)f.hdop_x100 / 100.0f;
                C->gnss_hdop_valid = true;
            } else {
                C->gnss_hdop_valid = false;
            }
#else
            C->gnss_hdop_valid = false;
#endif

#if HAVE_GNSS_HEADING
            // Expected fields if you extended your driver:
            //   f.heading_valid (bool-like), f.course_deg_e5 (int32, deg*1e5)
            // When speed spike is detected, also reject heading and derived velocity
            // (COG is unreliable during speed glitches)
            if (f.heading_valid && C->gnss_speed_mps > 0.5f && !C->gnss_speed_rejected) {
                float heading_deg = (float)f.course_deg_e5 / 1e5f;
                float yaw_enu = (0.5*M_PI)-(heading_deg * DEG2RAD);
                yaw_enu = wrap_pi(yaw_enu);

                C->gnss_heading_rad = yaw_enu;
                C->gnss_heading_valid = true;

                C->gnss_last_course_deg = heading_deg;
                C->gnss_last_yaw_enu_deg = yaw_enu/DEG2RAD;

                // derive ENU velocity (course over ground)
                float spd = C->gnss_speed_mps;
                float psi = C->gnss_heading_rad;
                C->gnss_vel_enu = v3(spd*cosf(psi), spd*sinf(psi), 0.0f);
                C->gnss_vel_valid = true;
            } else {
                C->gnss_heading_valid = false;
                C->gnss_vel_valid = false;
            }
#else
            C->gnss_heading_valid = false;
            C->gnss_vel_valid = false;
#endif

            C->have_gnss = (C->gnss_have_fix && new_fix) ;
// Mark GNSS measurement presence (independent of EKF gating)



            // Save for logging
            C->gnss_last_lat_deg = lat_deg;
            C->gnss_last_lon_deg = lon_deg;
            C->gnss_last_alt_m = alt_m;
            C->gnss_last_speed_mps = C->gnss_speed_mps;
            C->gnss_last_fix = 1;
            C->gnss_last_heading_deg = C->gnss_heading_valid ? (C->gnss_heading_rad / DEG2RAD) : 0.0f;
            C->gnss_last_hdop = C->gnss_hdop_valid ? C->gnss_hdop : -1.0f;
            C->gnss_last_valid = true;

            // Count valid fixes (for cold-start skip in fusion thread)
            if (new_fix) C->gnss_fix_count++;
        } else {
            // No fix: still keep last values for logging
            C->gnss_last_fix = 0;
            C->gnss_last_valid = true;
        }

        pthread_cond_signal(&C->cv);
        pthread_mutex_unlock(&C->mtx);

        usleep(100000); // ~10 Hz poll (GNSS typically 1 Hz)
    }

    close(fd);
    return NULL;
}

// ------------------------- Fusion thread -------------------------

static void* fusion_thread(void *arg) {
    int i;
    ctx_t *C = (ctx_t*)arg;
    static yaw_learn_t yawL = {0};
    uint64_t last_tick_ns = monotonic_ns();

    while (__atomic_load_n(&C->running, __ATOMIC_ACQUIRE)) {
        pthread_mutex_lock(&C->mtx);

        // Wait for IMU sample (or timeout)
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 5*1000*1000;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        (void)pthread_cond_timedwait(&C->cv, &C->mtx, &ts);

        uint64_t now_ns = monotonic_ns();
        float dt = (float)((now_ns - last_tick_ns) * 1e-9);
        if (dt <= 0.0f || dt > 1.0f) dt = DT_IMU_DEFAULT;
        last_tick_ns = now_ns;

        if (!C->have_imu) {
            pthread_mutex_unlock(&C->mtx);
            continue;
        }

        // Calibrate IMU through the centralized preprocessor (g_cal + g_imu_cal).
        // Output is in vehicle frame (mount_R_bv applied) and SI units, with
        // gravity still present in accel_vehicle.
        vec3f acc_b, gyro_b;
        {
            float a_v[3], g_v[3];
            imu_preprocess_sample(&g_imu_cal, &C->imu_raw, a_v, g_v);
            acc_b  = v3(a_v[0], a_v[1], a_v[2]);
            gyro_b = v3(g_v[0], g_v[1], g_v[2]);
        }

        // --- IMU signal processing pipeline ---
        // Stage 1: median filter (spike removal)
        {
            static float ax_buf[MEDFILT_LEN], ay_buf[MEDFILT_LEN], az_buf[MEDFILT_LEN];
            static float gx_buf[MEDFILT_LEN], gy_buf[MEDFILT_LEN], gz_buf[MEDFILT_LEN];
            static int   mf_idx = 0;
            static bool  mf_rdy = false;

            ax_buf[mf_idx] = acc_b.x;  ay_buf[mf_idx] = acc_b.y;  az_buf[mf_idx] = acc_b.z;
            gx_buf[mf_idx] = gyro_b.x; gy_buf[mf_idx] = gyro_b.y; gz_buf[mf_idx] = gyro_b.z;
            mf_idx = (mf_idx + 1) % MEDFILT_LEN;
            if (mf_idx == 0) mf_rdy = true;

            vec3f acc_med = acc_b, gyro_med = gyro_b;
            if (mf_rdy) {
                acc_med  = v3(medf5(ax_buf), medf5(ay_buf), medf5(az_buf));
                gyro_med = v3(medf5(gx_buf), medf5(gy_buf), medf5(gz_buf));
            }

            // Stage 2: IIR low-pass filter (vibration reduction)
            static vec3f acc_lpf, gyro_lpf;
            static bool  lpf_init = false;
            if (!lpf_init) { acc_lpf = acc_med; gyro_lpf = gyro_med; lpf_init = true; }
            acc_lpf.x  = IMU_LPF_ALPHA * acc_med.x  + (1.0f - IMU_LPF_ALPHA) * acc_lpf.x;
            acc_lpf.y  = IMU_LPF_ALPHA * acc_med.y  + (1.0f - IMU_LPF_ALPHA) * acc_lpf.y;
            acc_lpf.z  = IMU_LPF_ALPHA * acc_med.z  + (1.0f - IMU_LPF_ALPHA) * acc_lpf.z;
            gyro_lpf.x = IMU_LPF_ALPHA * gyro_med.x + (1.0f - IMU_LPF_ALPHA) * gyro_lpf.x;
            gyro_lpf.y = IMU_LPF_ALPHA * gyro_med.y + (1.0f - IMU_LPF_ALPHA) * gyro_lpf.y;
            gyro_lpf.z = IMU_LPF_ALPHA * gyro_med.z + (1.0f - IMU_LPF_ALPHA) * gyro_lpf.z;

            // Attitude-based vertical direction in body frame.
            // R maps body→ENU; column 2 of R = [R[2], R[5], R[8]] is the body-frame
            // up-axis expressed in ENU, i.e. R^T * [0,0,1] is up in body coords.
            float Rmat[9];
        #if USE_2D_EKF
            /* Use the AHRS quaternion (single source of truth for attitude).
             * att_q is kept in sync by ahrs_update but lives one IMU step
             * behind during the very first sample — use ahrs.q to avoid that. */
            R_from_q(C->ahrs.q, Rmat);
        #else
            R_from_q(C->ins.q, Rmat);
        #endif
            vec3f up_body = v3(Rmat[2], Rmat[5], Rmat[8]);

            // Stage 3: vertical and total acceleration magnitude
            // Only the vertical component is relevant for bump detection.
            // Cornering adds centripetal force horizontally — using total |a|
            // would falsely trigger at every turn (e.g. 60 km/h / 30 m radius
            // gives centripetal ≈ 9 m/s², |a_total| ≈ 13.5 m/s², deviation ≈ 3.7 m/s²).
            float acc_vert  = v3_dot(acc_lpf, up_body);   // component along gravity axis
            vec3f acc_horiz = v3_sub(acc_lpf, v3_scale(up_body, acc_vert)); // lateral+forward
            float acc_mag   = v3_norm(acc_lpf);           // kept for logging only

            // Stage 4: gyro vibration energy (high-frequency component power)
            vec3f gyro_hf = v3_sub(gyro_med, gyro_lpf);
            float gyro_vib_e = gyro_hf.x*gyro_hf.x + gyro_hf.y*gyro_hf.y + gyro_hf.z*gyro_hf.z;

            // Stage 5: bump detection — VERTICAL acceleration deviates from g.
            // Cornering does not affect acc_vert (centripetal is horizontal),
            // so only genuine road bumps trigger this gate.
            bool bump = (fabsf(acc_vert - GRAVITY) > BUMP_ACC_THR_MPS2);

            // Stage 6: vertical-only bump clamping with CORRECT symmetric range.
            // Expected vertical specific force on level ground = +GRAVITY (≈+9.81).
            // Allow ± BUMP_THR around that, NOT ± (GRAVITY + BUMP_THR) which let
            // through pure-negative readings as if they were valid. With the
            // proper window, gravity is locked in and only vibration-driven
            // excursions are limited. Horizontal dynamics preserved.
            // Gyro is always kept — attitude must track through the bump.
            gyro_b = gyro_lpf;
            // ---- Bump statistics for diagnostics & quality scoring ----
            static int    bump_count_in_window = 0;
            static int    sample_count_in_window = 0;
            static float  bump_max_dev = 0.0f;
            static double bump_window_t0 = 0.0;
            double t_bump_now = now_sec();
            if (bump_window_t0 == 0.0) bump_window_t0 = t_bump_now;
            sample_count_in_window++;
            if (bump) {
                bump_count_in_window++;
                float dev = fabsf(acc_vert - GRAVITY);
                if (dev > bump_max_dev) bump_max_dev = dev;
            }
            if (t_bump_now - bump_window_t0 >= 10.0) {
                float duty = (sample_count_in_window > 0)
                           ? (100.0f * bump_count_in_window / (float)sample_count_in_window) : 0.0f;
                dbg_printf(C, "BUMP_STATS count=%d/%d duty=%.1f%% max_dev=%.2f a_norm=%.2f g_norm=%.2f",
                           bump_count_in_window, sample_count_in_window,
                           duty, bump_max_dev, acc_mag, sqrtf(gyro_vib_e));
                // If bump is firing on >25% of samples the IMU/mount is bad —
                // do not silently clamp; mark quality degraded so the EKF
                // increases its process noise and avoids over-trusting accel.
                if (duty > 25.0f) {
                    if (!g_imu_quality_degraded) {
                        dbg_printf(C, "IMU_QUALITY degraded (bump_duty=%.1f%%) — inflating Q", duty);
                        printf("[IMU_QUALITY degraded] bump_duty=%.1f%% over 10 s\n", duty);
                    }
                    g_imu_quality_degraded = 1;
                    g_imu_cal.valid_flags |= CAL_FLAG_DEGRADED;
                } else if (duty < 5.0f && g_imu_quality_degraded) {
                    g_imu_quality_degraded = 0;
                    g_imu_cal.valid_flags &= ~CAL_FLAG_DEGRADED;
                    dbg_printf(C, "IMU_QUALITY good (bump_duty=%.1f%%)", duty);
                }
                bump_count_in_window = 0;
                sample_count_in_window = 0;
                bump_max_dev = 0.0f;
                bump_window_t0 = t_bump_now;
            }

            if (!bump) {
                acc_b = acc_lpf;
            } else {
                float vert_clamped = clampf(acc_vert,
                                            GRAVITY - BUMP_ACC_THR_MPS2,
                                            GRAVITY + BUMP_ACC_THR_MPS2);
                acc_b = v3_add(acc_horiz, v3_scale(up_body, vert_clamped));
#if ENABLE_IMU_DEBUG_LOGS
                dbg_printf(C, "BUMP: |a|=%.2f a_vert=%.2f dev=%.2f vib_e=%.4f -> clamped to [%.2f,%.2f]",
                           acc_mag, acc_vert, acc_vert - GRAVITY, gyro_vib_e,
                           GRAVITY - BUMP_ACC_THR_MPS2, GRAVITY + BUMP_ACC_THR_MPS2);
#endif
            }
        }

        float acc_norm = v3_norm(acc_b);

        // ZUPT inhibition: multiple signals to prevent false stillness during driving.
        // Problem: on smooth roads, calibrated IMU looks nearly stationary, and GNSS speed
        // is often 0 (TAU1204 doesn't always report speed). ZUPT then fires while driving,
        // killing velocity and freezing position — destroying dead-reckoning.
        //
        // Solution: track when EKF velocity was last significant. If the vehicle was recently
        // moving, ZUPT cannot fire even if IMU looks "still". This breaks the ZUPT→zero-vel
        // death spiral because EKF velocity persists from the last good state.
        double tnow_zupt = now_sec();
        float ekf_speed = 0.0f;
        #if USE_2D_EKF
            ekf_speed = sqrtf(C->ins2d.v_E*C->ins2d.v_E + C->ins2d.v_N*C->ins2d.v_N);
        #else
            ekf_speed = v3_norm(C->ins.v);
        #endif
        if (ekf_speed > 0.5f) {
            C->last_moving_s = tnow_zupt;
        }
        bool gnss_shows_motion = (C->gnss_have_fix && C->gnss_speed_mps > 1.0f);
        bool recently_moving = (C->last_moving_s > 0 && (tnow_zupt - C->last_moving_s) < 30.0);
        bool zupt_cond =
            !(gnss_shows_motion || recently_moving) &&
            (fabsf(acc_norm - GRAVITY) < ZUPT_ACC_THR) &&
            (fabsf(gyro_b.x) < ZUPT_GYRO_THR) &&
            (fabsf(gyro_b.y) < ZUPT_GYRO_THR) &&
            (fabsf(gyro_b.z) < ZUPT_GYRO_THR);

#if ENABLE_RUNTIME_GYRO_CAL
        /* Runtime gyro-bias refinement: only triggers when EKF and GNSS both
         * agree the vehicle is stationary, accel is near gravity, and gyro
         * magnitude is low. Updates are slow (alpha=0.5 %), persisted at most
         * once per RT_CAL_MIN_SAVE_INTERVAL_S. */
        if (C->imu_cal_done && C->nav_ready) {
            float gyro_v[3] = { gyro_b.x, gyro_b.y, gyro_b.z };
            float accel_v[3]= { acc_b.x,  acc_b.y,  acc_b.z };
            bool gnss_speed_low = (!C->gnss_have_fix) ||
                                   (C->gnss_speed_mps < RT_CAL_GNSS_SPEED_MAX);
            (void)runtime_gyro_cal_update(gyro_v, accel_v,
                                          (ekf_speed < RT_CAL_EKF_SPEED_MAX),
                                          gnss_speed_low,
                                          tnow_zupt);
        }
#endif

            if (zupt_cond) {
                C->zupt_count++;
            } else {
                C->zupt_count = 0;
            }

        /* -------------------Power on IMU calibration (once per boot; no file persistence )-----------------------*/
        static still_accum_t cal_accum_boot;
        double tcal_now = now_sec();

        if (!C->imu_cal_done) {
            if (!C->imu_cal_in_progress) {
                C->imu_cal_in_progress = true;
                C->imu_cal_start_s = tcal_now;
                still_reset(&cal_accum_boot);
                printf("[CAL] Power on IMU Calibration started: Keep vehicle still... \n");
            }

            // Blink LED while calibration
            cal_led_update(true, false, tcal_now);

            int boot_rc = apply_poweron_calibration(&cal_accum_boot, &C->imu_raw, acc_b,
                                                    BOOT_CAL_DURATION_S, tcal_now);
            if (boot_rc == 1) {
                C->imu_cal_done = true;
                C->imu_cal_in_progress = false;
                cal_led_update(false, true, tcal_now); // steady ON

                // Re-calibrate to get corrected gravity vector for initial tilt estimation
                vec3f acc_corrected;
                calib_accel(&C->imu_raw, &acc_corrected);
                C->boot_acc_mean = acc_corrected;

                printf("[CAL] Power-On IMU calibration complete.\n");
                printf("       gyro_bias_counts = [%.3f, %.3f, %.3f]\n",
                       g_cal.gyro_bias_counts[0], g_cal.gyro_bias_counts[1], g_cal.gyro_bias_counts[2]);
                printf("       accel_O = [%.6f, %.6f, %.6f]\n",
                       g_cal.accel_O[0], g_cal.accel_O[1], g_cal.accel_O[2]);
                printf("       boot_acc_mean = [%.4f, %.4f, %.4f]\n",
                       C->boot_acc_mean.x, C->boot_acc_mean.y, C->boot_acc_mean.z);
            } else if (boot_rc == -2) {
                /* Cal completed but failed sanity. If a previously-saved cal
                 * is on disk it's already loaded (see main); otherwise we
                 * continue with safe defaults and mark IMU quality degraded. */
                C->imu_cal_done = true;       // stop blocking nav
                C->imu_cal_in_progress = false;
                g_imu_quality_degraded = 1;
                g_imu_cal.valid_flags |= CAL_FLAG_DEGRADED;
                cal_led_update(false, true, tcal_now);
                printf("[CAL_BOOT rejected] proceeding with prior/default calibration — "
                       "IMU quality marked degraded.\n");
            }
            /* boot_rc == 0 (in progress) or -1 (motion) — keep blinking */
        } else {
            cal_led_update(false, true, tcal_now);
        }




        // Initialize nav after MIN_FIXES good GNSS fixes (skip cold-start, check HDOP)
        if (!C->nav_ready) {
            // Diagnostic: print nav-init gate status every 2 seconds
            {
                static double t_navwait = 0;
                double tnw = now_sec();
                if (tnw - t_navwait > 2.0) {
                    printf("[NAV_WAIT] have_gnss=%d have_fix=%d fix_count=%d hdop_valid=%d hdop=%.2f (need: fixes>=%d hdop<=%.1f)\n",
                        C->have_gnss, C->gnss_have_fix, C->gnss_fix_count,
                        C->gnss_hdop_valid, C->gnss_hdop_valid ? C->gnss_hdop : -1.0f,
                        NAV_INIT_MIN_FIXES, (float)NAV_INIT_HDOP_MAX);
                    t_navwait = tnw;
                }
            }
            // Wait for enough good GNSS fixes to skip cold-start transient,
            // then set ENU reference AND init nav from the SAME fix (atomic).
            if (C->have_gnss && C->gnss_have_fix &&
                C->gnss_fix_count >= NAV_INIT_MIN_FIXES &&
                (!C->gnss_hdop_valid || C->gnss_hdop <= NAV_INIT_HDOP_MAX)) {

                // ---- Set ENU reference from THIS fix ----
                C->enu_ref_lla.lat = C->gnss_lat_rad;
                C->enu_ref_lla.lon = C->gnss_lon_rad;
                C->enu_ref_lla.h   = C->gnss_alt_m;
                C->enu_ref_ecef    = lla2ecef(C->enu_ref_lla);
                C->enu_ref_set     = true;
                # if USE_2D_EKF
                    ins2d_init(&C->ins2d);
                    C->ins2d.p_E = 0.0f;
                    C->ins2d.p_N = 0.0f;
                    C->ins2d.v_E = 0.0f;
                    C->ins2d.v_N = 0.0f;

                    // Initialize heading from GNSS if available, else 0 with wide covariance
                    if (C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN) {
                        C->ins2d.psi = C->gnss_heading_rad;
                        // Tighten heading covariance — TAU1204 dual-band COG at speed >1 m/s is ~3°
                        C->ins2d.P[4*7+4] = (5.0f * DEG2RAD) * (5.0f * DEG2RAD);   // 5° 1-sigma
                        C->heading_observed = true;
                    } else {
                        C->ins2d.psi = 0.0f;
                        // Widen heading covariance
                        C->ins2d.P[4*7+4] = (60.0f * DEG2RAD) * (60.0f * DEG2RAD); // 60° 1-sigma
                        dbg_printf(C, "NAV_INIT no heading — psi P widened to (60 deg)^2");
                    }
                /* Initialize the AHRS complementary filter from the stationary
                 * boot accelerometer mean and (optional) GNSS course. The AHRS
                 * owns roll/pitch/yaw from this point on; att_q is kept as a
                 * legacy mirror for any downstream code still using it. */
                float boot_a[3] = { C->boot_acc_mean.x, C->boot_acc_mean.y, C->boot_acc_mean.z };
                int   yaw_valid = (C->heading_observed) ? 1 : 0;
                float init_yaw  = (yaw_valid) ? C->gnss_heading_rad : 0.0f;
                ahrs_init_from_accel(&C->ahrs, boot_a, init_yaw, yaw_valid);
                /* Keep ins2d.psi in agreement with the AHRS yaw at init. */
                C->ins2d.psi = C->ahrs.yaw;
                C->att_q     = C->ahrs.q;
                dbg_printf(C, "NAV_INIT_AHRS roll=%.2f pitch=%.2f yaw=%.2f yaw_valid=%d quality=%d",
                           C->ahrs.roll/DEG2RAD, C->ahrs.pitch/DEG2RAD, C->ahrs.yaw/DEG2RAD,
                           yaw_valid, (int)C->ahrs.quality);
                #else
                    dbg_printf(C, "NAV_INIT ENU_REF set from fix #%d lat=%.9f lon=%.9f alt=%.3f hdop=%.1f",
                    C->gnss_fix_count,
                    C->enu_ref_lla.lat * (180.0/M_PI),
                    C->enu_ref_lla.lon * (180.0/M_PI),
                    C->enu_ref_lla.h,
                    C->gnss_hdop_valid ? C->gnss_hdop : -1.0f);

                // ---- Init EKF state ----
                ins15_init(&C->ins);
                // ENU origin = this fix, so initial position is exactly (0,0,0)
                C->ins.p = v3(0.0f, 0.0f, 0.0f);
                C->ins.v = v3(0,0,0);

                // Widen initial position covariance based on HDOP
                float hdop_init = C->gnss_hdop_valid ? C->gnss_hdop : 2.0f;
                float p0_h = hdop_init * 2.5f;   // horizontal 1-sigma (m)
                float p0_v = hdop_init * 5.0f;    // vertical 1-sigma  (m)
                C->ins.P[0*15+0] = p0_h * p0_h;   // East
                C->ins.P[1*15+1] = p0_h * p0_h;   // North
                C->ins.P[2*15+2] = p0_v * p0_v;   // Up

                if (C->gnss_heading_valid) {
                    yaw_learn_on_gnss_fix(&yawL, C->gnss_heading_rad, C->last_gnss_fix_mono_ns);
                }
                float yaw0 = 0.0f;
                if (C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN) {
                    yaw0 = C->gnss_heading_rad;
                    C->heading_observed = true;
                }

                // Initialize roll/pitch from boot accelerometer gravity vector
                float roll0 = 0.0f, pitch0 = 0.0f;
                float amag = v3_norm(C->boot_acc_mean);
                if (amag > 1.0f) {
                    vec3f a = C->boot_acc_mean;
                    pitch0 = atan2f(-a.x, sqrtf(a.y*a.y + a.z*a.z));
                    roll0  = atan2f(a.y, a.z);
                    dbg_printf(C, "NAV_INIT tilt: roll=%.2f pitch=%.2f yaw=%.2f deg",
                               roll0/DEG2RAD, pitch0/DEG2RAD, yaw0/DEG2RAD);
                }
                C->ins.q = q_from_euler(roll0, pitch0, yaw0);

                // Start biases at 0 (already calibrated raw -> SI).
                C->ins.ba = v3(0,0,0);
                C->ins.bg = v3(0,0,0);

                // If heading was NOT observed at init, widen yaw covariance
                // so the filter can converge heading from subsequent GNSS COG updates
                if (!C->heading_observed) {
                    float yaw_unc = 45.0f * DEG2RAD;  // 45° 1-sigma
                    C->ins.P[8*15+8] = yaw_unc * yaw_unc;
                    dbg_printf(C, "NAV_INIT no heading — yaw P widened to (%.0f deg)^2", 45.0f);
                }
            #endif 
                C->nav_ready = true;
                double t0 = now_sec();
                C->last_gnss_meas_s = t0;
                C->last_gnss_used_s = t0;
                C->gnss_present     = true;
                C->gnss_valid       = true;
                C->reacq_left       = 0;
                C->reacq_active     = false;
                C->snap_applied     = false;
                C->outage_s         = 0.0;
                C->qscale           = 1.0f;
                C->last_nis_pos = 0.0f;
                C->last_nis_vel = 0.0f;

            #if USE_2D_EKF
                dbg_printf(C, "NAV_READY (2D) p=(0,0) P_pos=%.1f m^2", C->ins2d.P[0*7+0]);
            #else
                dbg_printf(C, "NAV_READY p=(0,0,0) P_pos_h=%.1f P_pos_v=%.1f m^2",
                    p0_h * p0_h, p0_v * p0_v);
            #endif

                C->have_gnss = false; // consume
            }

            pthread_mutex_unlock(&C->mtx);
            continue;
        }

        // GNSS presence/outage tracking (decoupled from gating)
double tnow = now_sec();

if (C->last_gnss_meas_s <= 0.0) {
    // no GNSS measurement ever received yet (shouldn't happen after nav_ready, but keep safe)
    C->gnss_present = false;
    C->outage_s += dt;
} else {
    double age = tnow - C->last_gnss_meas_s;
    if (age <= GNSS_TIMEOUT_S) {
        C->gnss_present = true;
        C->outage_s = 0.0;
    } else {
        C->gnss_present = false;
        C->outage_s = age;
    }
}

double nav_age = tnow - C->last_gnss_used_s;
if (nav_age <= GNSS_TIMEOUT_S) {
    C->outage_s = 0.0;
    C->qscale = 1.0f;
} else {
    C->outage_s = nav_age;
    C->qscale = compute_qscale(C->outage_s);
}
/* If the IMU quality is currently degraded (continuous bumps / vibration /
 * boot-cal failed), inflate process noise so the EKF doesn't over-trust
 * the predict step. Capped at 3× the base outage scale. */
if (g_imu_quality_degraded) {
    C->qscale *= 2.0f;
    if (C->qscale > 12.0f) C->qscale = 12.0f;
}
#if USE_2D_EKF
    // Update attitude quaternion (for gravity compensation).
    // Subtract estimated gyro-z bias on the yaw axis so heading drift in att_q
    // matches the EKF heading state. Pitch/roll have no bias estimate in 2D
    // (filter doesn't track gx/gy biases), so leave those axes uncorrected —
    // pitch/roll drift gets absorbed by the b_a estimate via gravity compensation.
    /* ---------------- AHRS / complementary attitude filter ----------------
     * Runs at IMU rate, BEFORE the EKF predict step. The AHRS owns roll,
     * pitch and yaw. Roll/pitch are corrected from gravity (gated), yaw is
     * corrected from GNSS course (gated). The 2D EKF receives only
     *   - forward acceleration (vehicle X, gravity removed)
     *   - yaw rate           (gyro Z)
     *   - AHRS yaw via a measurement update further down                 */
    float a_v[3] = { acc_b.x,  acc_b.y,  acc_b.z  };
    float g_v[3] = { gyro_b.x, gyro_b.y, gyro_b.z };
    ahrs_update(&C->ahrs, a_v, g_v, dt,
                C->gnss_heading_valid ? 1 : 0,
                C->gnss_heading_rad,
                C->gnss_speed_mps,
                C->gnss_hdop_valid ? C->gnss_hdop : 99.0f);
    C->att_q = C->ahrs.q;            /* keep legacy mirror in sync */

    /* If the AHRS quality is anything other than GOOD, raise the global IMU
     * quality flag so the existing qscale-inflation path (further up) widens
     * EKF process noise. This blocks false-confident predictions during
     * bumps and during accel/yaw-rejected windows. */
    if (C->ahrs.quality == AHRS_QUALITY_BUMP ||
        C->ahrs.quality == AHRS_QUALITY_BAD) {
        g_imu_quality_degraded = 1;
    }

    /* Gravity-compensated vehicle-frame linear acceleration */
    float lin_v[3];
    ahrs_remove_gravity(&C->ahrs, a_v, lin_v);
    /* Sanity clamp on forward and lateral components (z is unused by 2D EKF) */
    lin_v[0] = clampf(lin_v[0], -MAX_ACCEL_ENU_MPS2, MAX_ACCEL_ENU_MPS2);
    lin_v[1] = clampf(lin_v[1], -MAX_ACCEL_ENU_MPS2, MAX_ACCEL_ENU_MPS2);
    C->last_aw = v3(lin_v[0], lin_v[1], lin_v[2]);

    /* For ground vehicle (NHC), forward = vehicle X axis */
    float a_fwd  = lin_v[0];

    /* Yaw rate from gyro Z (vehicle frame). ins2d_predict subtracts b_g
     * internally so we do not subtract it here. */
    float omega_z = gyro_b.z;

    /* ---- 2D EKF prediction ---- */
    ins2d_predict(&C->ins2d, a_fwd, omega_z, dt, C->qscale);

    /* Snapshot position right after predict (before measurement updates) */
    C->predict_p = v3(C->ins2d.p_E, C->ins2d.p_N, 0.0f);

    /* ---- Yaw measurement update from AHRS (keeps EKF psi coupled to AHRS) ----
     * AHRS yaw is integrated from gyro and corrected by GNSS course; without
     * this measurement the EKF psi state would integrate gyro independently
     * and could diverge from the AHRS during long DR. R_yaw is widened when
     * the AHRS itself hasn't seen a yaw observation recently. */
    {
        float R_yaw = AHRS_TO_EKF_R_YAW;
        if (C->ahrs.quality == AHRS_QUALITY_YAW_UNOBSERVED) R_yaw *= 25.0f;   // (5°)² → (25°)²
        if (C->ahrs.quality == AHRS_QUALITY_BAD)            R_yaw *= 100.0f;  // (5°)² → (50°)²
        float yaw_diff = wrap_pi(C->ahrs.yaw - C->ins2d.psi);
        if (fabsf(yaw_diff) > AHRS_YAW_INNOV_MAX) {
            /* Large disagreement — log and force EKF yaw to AHRS to avoid divergence */
            dbg_printf(C, "AHRS_EKF_YAW_RESYNC ahrs=%.2f ekf=%.2f diff=%.2f",
                       C->ahrs.yaw/DEG2RAD, C->ins2d.psi/DEG2RAD, yaw_diff/DEG2RAD);
            C->ins2d.psi = C->ahrs.yaw;
        } else {
            ins2d_update_yaw_from_ahrs(&C->ins2d, C->ahrs.yaw, R_yaw);
        }
    }

    // NHC: only when confirmed moving and heading is trusted
    if (C->heading_observed && ekf_speed > 1.0f) {
        ins2d_update_nhc(&C->ins2d);
    }

    //ZUPT
    if (C->zupt_count >= ZUPT_COUNT_REQUIRED) {
        ins2d_update_zupt(&C->ins2d);
        // Position anchor when sustained still 
        C->sustained_still++;
        if (!C->anchor_set && C->sustained_still > 20) { 
            C->anchor_p = v3(C->ins2d.p_E, C->ins2d.p_N, 0.0f);
            C->anchor_set = true;
        }
        if (C->anchor_set) {
            ins2d_update_gnss_pos(&C->ins2d, C->anchor_p.x, C->anchor_p.y, 0.25f, NULL);
        }
     } else
        {
            C->sustained_still = 0;
            C->anchor_set = false;

        }
        // Velocity management: hard cap + outage decay
        // Without these, IMU bias + heading drift integrates unbounded into v
        // and the position diverges by km after a 60-s outage.
        float vnorm = sqrtf(C->ins2d.v_E*C->ins2d.v_E + C->ins2d.v_N*C->ins2d.v_N);

        // Hard cap at typical road-vehicle max
        if (vnorm > MAX_GNSS_SPEED_MPS) {
            float scale = MAX_GNSS_SPEED_MPS / vnorm;
            C->ins2d.v_E *= scale;
            C->ins2d.v_N *= scale;
            vnorm = MAX_GNSS_SPEED_MPS;
        }

        // Velocity decay strategy (with 10-s GNSS fusion period):
        //   Between fusion events the predict runs for ~10 s before any
        //   correction. IMU bias error integrates linearly over that gap, so
        //   we apply a continuous gentle decay (proportional to time since
        //   last fusion) to bound runaway. The decay is light enough that
        //   real motion is preserved over 10 s but caps drift after that.
        double time_since_fuse = (C->last_gnss_fused_s > 0.0)
                                ? (now_sec() - C->last_gnss_fused_s) : 0.0;
        if (time_since_fuse > 5.0) {
            // Light damping ramps up after 5 s without a fusion event.
            // 0.995/sec for 5–15 s gap; 0.99/sec for >15 s; 0.95/sec for >60 s.
            float decay_per_s = 0.995f;
            if (time_since_fuse > 15.0) decay_per_s = 0.99f;
            if (time_since_fuse > 60.0) decay_per_s = 0.95f;
            float decay = powf(decay_per_s, dt);
            C->ins2d.v_E *= decay;
            C->ins2d.v_N *= decay;
        } else if (vnorm < 0.10f) {
            // Near-stationary: stronger decay to suppress residual creep
            float decay = (vnorm < 0.02f) ? 0.90f : VEL_DECAY;
            C->ins2d.v_E *= decay;
            C->ins2d.v_N *= decay;
        }

#else
// GNSS Yaw rate learner: Feed IMU every step (turn-segment integration)
// Compute GNSS quality gates
bool gnss_ok = (C->gnss_present && C->gnss_valid && C->gnss_have_fix && C->gnss_heading_valid &&
                (C->gnss_speed_mps >= YAW_LEARN_MIN_SPEED_MPS) && 
                (!C->gnss_hdop_valid || (C->gnss_hdop <= YAW_LEARN_MAX_HDOP)));
        //Turning gate based on UNSCALED gyro-z (bias removed, before scale trim )
        const float Lgz = (DEG2RAD / GYRO_LSB_PER_DPS ); // rad/s per count
        float gz_unscaled_radps = ((float)C->imu_raw.gz - g_cal.gyro_bias_counts[2]) * Lgz;
        bool turning_ok = fabsf(gz_unscaled_radps) > YAW_LEARN_TURN_RATE_THR;
        yaw_learn_on_imu(&yawL, tnow, dt, &C->imu_raw, gnss_ok, C->gnss_speed_mps, turning_ok);

        // Predict INS
        ins15_predict(&C->ins, acc_b, gyro_b, dt, C->qscale, &C->last_aw);

        // Snapshot position right after predict (before any measurement updates)
        C->predict_p = C->ins.p;

        // Apply NHC only after heading has been observed from GPS
        // (NHC with wrong heading causes rapid divergence)
        if (C->heading_observed) {
            ins15_update_nhc(&C->ins);
        }


        // ZUPT detection
       
        if (zupt_cond) C->zupt_count++;
        else C->zupt_count = 0;

        if (C->zupt_count >= ZUPT_COUNT_REQUIRED) {
            ins15_update_zupt(&C->ins);
            ins15_update_zaru(&C->ins, gyro_b);
            // Keep counter saturated so ZUPT fires EVERY step while still
            // (previously reset to 0, causing 5-sample gaps with unconstrained drift)
            if (C->zupt_count > ZUPT_COUNT_REQUIRED + 100)
                C->zupt_count = ZUPT_COUNT_REQUIRED; // cap to avoid overflow
        }

        // --- Stationary position anchor ---
        // When ZUPT has been active for a sustained period, anchor position
        // via an EKF-consistent position measurement update (prevents random walk)
        if (C->zupt_count >= ZUPT_COUNT_REQUIRED) {
            C->sustained_still++;
            if (!C->anchor_set && C->sustained_still > 20) { // ~200ms confirmed still
                C->anchor_p = C->ins.p;
                C->anchor_set = true;
            }
            // Apply position anchor as a tight EKF position measurement
            // This is EKF-consistent: tells the filter "position hasn't changed"
            if (C->anchor_set) {
                const float R_anchor = 0.25f; // (0.5 m)^2 — tight but not aggressive
                ins15_update_gnss_pos(&C->ins, C->anchor_p, R_anchor, NULL);
            }
        } else {
            C->sustained_still = 0;
            C->anchor_set = false;
        }

        // Velocity management: decay + cap
        float vnorm = v3_norm(C->ins.v);

        // Hard velocity cap: no road vehicle exceeds ~55 m/s (~200 km/h)
        // During GNSS outage, bad IMU integration can push velocity to absurd values
        if (vnorm > MAX_GNSS_SPEED_MPS) {
            C->ins.v = v3_scale(C->ins.v, MAX_GNSS_SPEED_MPS / vnorm);
            vnorm = MAX_GNSS_SPEED_MPS;
        }

        // Velocity decay during GNSS outage: very gentle, TIME-BASED.
        // DR relies on IMU-propagated velocity. Aggressive decay defeats dead-reckoning.
        // The EKF Q-inflation already models growing uncertainty during outage.
        // Only apply mild decay to prevent truly runaway drift after very long outages.
        if (!C->gnss_present && C->outage_s > 30.0) {
            // 99% retention per second → halves in ~70 seconds. Gentle enough for DR.
            C->ins.v = v3_scale(C->ins.v, powf(0.99f, dt));
        } else if (vnorm < 0.10f) {
            // Near-stationary: strong decay to suppress residual creep
            float decay = (vnorm < 0.02f) ? 0.90f : VEL_DECAY;
            C->ins.v = v3_scale(C->ins.v, decay);
        }
#endif 
        // GNSS fusion (pos + optional vel + optional yaw complementary)
        if (C->have_gnss && C->gnss_have_fix) {
            if( C->gnss_heading_valid ){
                yaw_learn_on_gnss_fix(&yawL, C->gnss_heading_rad, C->last_gnss_fix_mono_ns);
            }
            lla_t L = { C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m };
            ecef_t E = lla2ecef(L);
            double enu_d[3];
            ecef2enu(E, C->enu_ref_lla, C->enu_ref_ecef, enu_d);
            vec3f zpos_raw = v3((float)enu_d[0], (float)enu_d[1], (float)enu_d[2]);

            // --- Alpha-beta sawtooth correction ---
            // 2nd-order tracker: maintains smoothed position AND velocity.
            // Eliminates N-sample oscillation that a simple 1st-order blend creates.
            vec3f zpos;
            double tnow_gnss = C->last_gnss_meas_s;
            if (C->ab_valid) {
                float dt_s = (float)(tnow_gnss - C->ab_time);
                if (dt_s > 0.0f && dt_s < AB_MAX_DT) {
                    // Predict step: extrapolate using tracker's own velocity
                    vec3f predicted = v3_add(C->ab_pos, v3_scale(C->ab_vel, dt_s));
                    // Residual: how far raw fix is from prediction
                    vec3f residual = v3_sub(zpos_raw, predicted);
                    float res_norm = v3_norm(residual);

                    if (res_norm < AB_MAX_RESIDUAL) {
                        // Correct step: update position and velocity from residual
                        zpos       = v3_add(predicted, v3_scale(residual, AB_ALPHA));
                        C->ab_vel  = v3_add(C->ab_vel, v3_scale(residual, AB_BETA / dt_s));
                    } else {
                        // Large residual — reset tracker (real manoeuvre, reacq, etc.)
                        zpos      = zpos_raw;
                        C->ab_vel = C->gnss_vel_valid ? C->gnss_vel_enu : v3(0,0,0);
                        dbg_printf(C, "AB_RESET residual=%.2f m", res_norm);
                    }
                } else {
                    // Gap too long — reset tracker
                    zpos      = zpos_raw;
                    C->ab_vel = C->gnss_vel_valid ? C->gnss_vel_enu : v3(0,0,0);
                    dbg_printf(C, "AB_RESET dt=%.2f s", dt_s);
                }
            } else {
                // First fix — initialise tracker
                zpos      = zpos_raw;
                C->ab_vel = C->gnss_vel_valid ? C->gnss_vel_enu : v3(0,0,0);
            }
            // Update tracker state
            C->ab_pos   = zpos;
            C->ab_time  = tnow_gnss;
            C->ab_valid = true;

            C->gnss_enu_last = zpos;
            C->gnss_enu_last_valid = true;
            dbg_printf(C, "AB_TRACK raw=(%.3f,%.3f,%.3f) smooth=(%.3f,%.3f,%.3f) res=%.3f",
                       zpos_raw.x, zpos_raw.y, zpos_raw.z,
                       zpos.x, zpos.y, zpos.z,
                       v3_norm(v3_sub(zpos_raw, zpos)));
            dbg_printf(C, "GNSS_FUSE ENU_REF lat=%.9f lon=%.9f alt=%.3f meas_age=%.3f present=%d",
            C->enu_ref_lla.lat * (180.0/M_PI),
            C->enu_ref_lla.lon * (180.0/M_PI),
            C->enu_ref_lla.h,
            (float)(now_sec() - C->last_gnss_meas_s),
            C->gnss_present ? 1 : 0);
        #if USE_2D_EKF
            dbg_printf(C, "GNSS_FUSE zpos_ENU=(%.3f,%.3f,%.3f) ins2d.p=(%.3f,%.3f)",
                       zpos.x, zpos.y, zpos.z,
                       C->ins2d.p_E, C->ins2d.p_N);
        #else
            dbg_printf(C, "GNSS_FUSE zpos_ENU=(%.3f,%.3f,%.3f) ins.p=(%.3f,%.3f,%.3f)",
                       zpos.x, zpos.y, zpos.z,
                       C->ins.p.x, C->ins.p.y, C->ins.p.z);
        #endif
            float hdop = (C->gnss_hdop_valid ? C->gnss_hdop : 1.0f);

            #if USE_2D_EKF
                double tnow2 = now_sec();

                // -------- GNSS fusion period throttle (10 s) --------
                // Skip the EKF fusion path if we have fused within the last
                // GNSS_FUSION_PERIOD_S seconds. Predict still runs at IMU rate
                // (the throttle does not affect the predict cadence). The
                // alpha-beta tracker above still uses every fix for logging.
                if (C->last_gnss_fused_s > 0.0 &&
                    (tnow2 - C->last_gnss_fused_s) < GNSS_FUSION_PERIOD_S) {
                    // Intermediate fix — ignore for fusion; consume so the
                    // measurement isn't re-tried next IMU step.
                    static double t_skip_last = 0.0;
                    if (tnow2 - t_skip_last > 2.0) {
                        dbg_printf(C, "GNSS_FUSE_SKIP age_since_fuse=%.2fs (period=%.1fs)",
                                   (float)(tnow2 - C->last_gnss_fused_s),
                                   (float)GNSS_FUSION_PERIOD_S);
                        t_skip_last = tnow2;
                    }
                    C->have_gnss = false;
                    goto gnss_done;
                }
                // We are going to fuse this fix — stamp the time NOW so the
                // next 10 s of fixes are skipped regardless of accept/reject.
                C->last_gnss_fused_s = tnow2;

                float Rpos = INS2D_R_POS * hdop * hdop;

                // -------- Reacquisition mode --------
                // If we haven't ACCEPTED a GNSS fix for >5s, relax R and gate
                // for the next REACQ_STEPS attempts so the filter can re-lock.
                if ((tnow2 - C->last_gnss_used_s) > REACQ_MIN_OUTAGE_S && C->reacq_left <= 0) {
                    C->reacq_left = REACQ_STEPS;
                }
                float gate_thr = INS2D_CHI2_2DOF_GATE;
                if (C->reacq_left > 0) {
                    Rpos *= REACQ_R_MULT;
                    gate_thr = REACQ_CHI2_GATE;
                    C->reacq_active = true;   // latched for 1 Hz log
                }

                // -------- Snap-to-GNSS safety net --------
                // After SNAP_MIN_OUTAGE_S without an accepted fix and a large
                // innovation, teleport position (and velocity) to GNSS so the
                // filter can resume — otherwise the NIS gate stays closed
                // forever once the EKF has drifted by >100m.
                float innov_e = zpos.x - C->ins2d.p_E;
                float innov_n = zpos.y - C->ins2d.p_N;
                float innov_h = sqrtf(innov_e*innov_e + innov_n*innov_n);
                bool allow_snap = ((tnow2 - C->last_gnss_used_s) > SNAP_MIN_OUTAGE_S);
                dbg_printf(C, "GNSS_INNOV(2D) innov=(%.2f,%.2f) |innov|=%.2f hdop=%.2f qS=%.2f reacq=%d",
                           innov_e, innov_n, innov_h, hdop, C->qscale, C->reacq_left);

                if (allow_snap && (!C->gnss_hdop_valid || hdop <= SNAP_HDOP_MAX) && innov_h > SNAP_INNOV_M) {
                    C->ins2d.p_E = zpos.x;
                    C->ins2d.p_N = zpos.y;
                    if (C->gnss_vel_valid && !C->gnss_speed_rejected) {
                        C->ins2d.v_E = C->gnss_vel_enu.x;
                        C->ins2d.v_N = C->gnss_vel_enu.y;
                    }
                    // Inflate position covariance so subsequent updates aren't over-confident
                    if (C->ins2d.P[0*7+0] < 25.0f) C->ins2d.P[0*7+0] = 25.0f;
                    if (C->ins2d.P[1*7+1] < 25.0f) C->ins2d.P[1*7+1] = 25.0f;
                    // Also pull heading toward GNSS course if available — keep AHRS in sync.
                    if (C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN) {
                        C->ins2d.psi = C->gnss_heading_rad;
                        C->ahrs.yaw  = C->gnss_heading_rad;
                        C->ahrs.q    = q_from_euler(C->ahrs.roll, C->ahrs.pitch, C->ahrs.yaw);
                        C->ahrs.last_yaw_obs_s = now_sec();
                        C->att_q = C->ahrs.q;
                    }
                    C->last_gnss_used_s = tnow2;
                    C->gnss_valid = true;
                    C->last_gnss_used_pos = 1;
                    C->snap_applied = true;
                    C->last_nis_pos = 0.0f;
                    C->gnss_accepted_count++;
                    C->reacq_left = 0;
                    // Reset alpha-beta tracker so it re-seeds from snapped position
                    C->ab_pos  = zpos;
                    C->ab_time = tnow_gnss;
                    C->ab_vel  = C->gnss_vel_valid ? C->gnss_vel_enu : v3(0,0,0);
                    dbg_printf(C, "SNAP_2D innov=%.1fm outage=%.1fs -> teleport to GNSS",
                               innov_h, (float)(tnow2 - C->last_gnss_used_s));
                } else {
                    // -------- Position update with NIS gate BEFORE applying --------
                    // ins2d_update_gnss_pos applies the update unconditionally and
                    // returns NIS as a diagnostic. Compute NIS first so an outlier
                    // never reaches the state.
                    float nis_pos = 0.0f;
                    bool ok_nis = ins2d_nis_gnss_pos(&C->ins2d, zpos.x, zpos.y, Rpos, &nis_pos);
                    bool gate_pos = ok_nis && (nis_pos < gate_thr);
                    C->last_nis_pos = nis_pos;

                    if (gate_pos) {
                        ins2d_update_gnss_pos(&C->ins2d, zpos.x, zpos.y, Rpos, NULL);
                        C->last_gnss_used_s = tnow2;
                        C->gnss_valid = true;
                        C->last_gnss_used_pos = 1;
                        C->gnss_accepted_count++;
                    }
                    if (C->reacq_left > 0) C->reacq_left--;
                }

                // -------- Velocity update with NIS gate --------
                if (C->gnss_vel_valid && !C->gnss_speed_rejected) {
                    float R_vel = INS2D_R_VEL;
                    if (C->gnss_speed_mps < 1.0f) R_vel *= 10.0f;  // course-derived vel is noisy at low speed
                    float nis_vel = 0.0f;
                    bool ok_nis_v = ins2d_nis_gnss_vel(&C->ins2d, C->gnss_vel_enu.x, C->gnss_vel_enu.y, R_vel, &nis_vel);
                    bool gate_vel = ok_nis_v && (nis_vel < INS2D_CHI2_2DOF_GATE);
                    C->last_nis_vel = nis_vel;
                    if (gate_vel) {
                        ins2d_update_gnss_vel(&C->ins2d, C->gnss_vel_enu.x, C->gnss_vel_enu.y, R_vel, NULL);
                        C->last_gnss_used_vel = 1;
                        C->last_gnss_used_s = tnow2;
                        C->gnss_valid = true;
                    }
                } else {
                    C->last_nis_vel = 0.0f;
                }

                // -------- Heading update (critical for 2D EKF stability) --------
                // The AHRS already applies a gated GNSS yaw correction every
                // IMU step (5 % alpha), and the AHRS yaw is then injected into
                // the EKF via ins2d_update_yaw_from_ahrs above. We still keep
                // a direct EKF heading update here for fast initial lock — the
                // R sigma is computed dynamically from speed (TAU1204 dual-band).
                if (C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN)
                {
                    float sigma_hdg = fmaxf(0.5f, 5.0f / C->gnss_speed_mps) * DEG2RAD;
                    float R_hdg = sigma_hdg * sigma_hdg;
                    ins2d_update_heading(&C->ins2d, C->gnss_heading_rad, R_hdg, NULL);
                    /* Keep legacy mirror in sync with AHRS — AHRS itself was
                     * already pulled toward this fix inside ahrs_update. */
                    C->att_q = C->ahrs.q;
                    if (!C->heading_observed) {
                        C->heading_observed = true;
                        dbg_printf(C, "HEADING_OBSERVED at speed=%.2f m/s — heading fusion enabled", C->gnss_speed_mps);
                    }
                }
                #else 

            // Adaptive HDOP gate: relaxed for first few fixes, strict after convergence
            float hdop_limit = (C->gnss_accepted_count < GNSS_HDOP_CONVERGE_FIXES)
                             ? NAV_INIT_HDOP_MAX       // 10.0 — accept anything to converge
                             : GNSS_FUSION_HDOP_MAX;   //  2.5 — quality gate after convergence
            if (C->gnss_hdop_valid && hdop > hdop_limit) {
                dbg_printf(C, "GNSS_HDOP_REJECT hdop=%.2f limit=%.1f accepted=%d",
                           hdop, hdop_limit, C->gnss_accepted_count);
                C->have_gnss = false; // consumed — skip this fix
                goto gnss_done;
            }

            float Rpos = R_GNSS_POS_VAR * hdop * hdop;

            // Reacquisition: if we haven't ACCEPTED GNSS for a while, relax gating and inflate R
            double tnow2 = now_sec();
            if ((tnow2 - C->last_gnss_used_s) > REACQ_MIN_OUTAGE_S && C->reacq_left <= 0) {
                C->reacq_left = REACQ_STEPS;
            }

            float gate_thr = CHI2_3DOF_GATE;
            if (C->reacq_left > 0) {
                 Rpos *= REACQ_R_MULT;
                gate_thr = REACQ_CHI2_GATE;
                C->reacq_active = true; // latched for 1 Hz logs
            }

            // Optional snap-to-GNSS safety net after long outage and large innovation
            float innov_e = zpos.x - C->ins.p.x;
            float innov_n = zpos.y - C->ins.p.y;
            float innov_h = sqrtf(innov_e*innov_e + innov_n*innov_n);
            dbg_printf(C, "GNSS_INNOV innov_ENU=(%.3f,%.3f) norm=%.3f hdop=%.2f qscale=%.2f",
            innov_e, innov_n, innov_h, hdop, C->qscale);
            bool allow_snap = ((tnow2 - C->last_gnss_used_s) > SNAP_MIN_OUTAGE_S);
            if (allow_snap && (!C->gnss_hdop_valid || hdop <= SNAP_HDOP_MAX) && innov_h > SNAP_INNOV_M) {
                // Snap position (and velocity if available) to GNSS to guarantee re-lock.
                C->ins.p = zpos;
                if (C->gnss_vel_valid && !C->gnss_speed_rejected) C->ins.v = C->gnss_vel_enu;

                // Inflate position covariance a bit after snap
                for (i=0;i<3;i++) {
                int ii = i*15 + i;
                    if (C->ins.P[ii] < (25.0f)) C->ins.P[ii] = 25.0f; // ~5 m std
                }

                C->last_gnss_used_s = tnow2;
                C->gnss_valid = true;
                C->last_gnss_used_pos = 1;
                C->snap_applied = true;
                C->last_nis_pos = 0.0f;
                C->gnss_accepted_count++;
                C->reacq_left = 0; // done
                // Reset alpha-beta tracker so it re-seeds from snapped position
                C->ab_pos  = zpos;
                C->ab_time = tnow_gnss;
                C->ab_vel  = C->gnss_vel_valid ? C->gnss_vel_enu : v3(0,0,0);
            } else {
                // Compute NIS first (read-only) then gate before applying update
                float nis_pos = 0.0f;
                bool ok_nis = ins15_nis_gnss_pos(&C->ins, zpos, Rpos, &nis_pos);
                bool gate_pos = ok_nis && (nis_pos < gate_thr);
                C->last_nis_pos = nis_pos;

                if (gate_pos) {
                    ins15_update_gnss_pos(&C->ins, zpos, Rpos, NULL);
                    C->last_gnss_used_s = tnow2;
                    C->gnss_valid = true;
                    C->last_gnss_used_pos = 1;
                    C->gnss_accepted_count++;
                }

                if (C->reacq_left > 0) {
                    // count down each GNSS attempt during reacq, regardless of accept
                    C->reacq_left--;
                }
            }

            // Velocity update if valid
            if (C->gnss_vel_valid) {
                float Rv[3] = { RV_VEL_E, RV_VEL_N, RV_VEL_U };

                // Optional: Inflate velocity  noise when speed is low (course-based velocity gets noisy )
                if( C->gnss_speed_mps < 1.0f ){
                    Rv[0] *= 10.0f; Rv[1] *= 10.0f;
                }
                // Compute NIS first (read-only) then gate before applying update
                float nis_vel = 0.0f;
                bool ok_nis_v = ins15_nis_gnss_vel(&C->ins, C->gnss_vel_enu, Rv, &nis_vel);
                bool gate_vel = ok_nis_v && (nis_vel < CHI2_3DOF_GATE);
                C->last_nis_vel = nis_vel;

                if( gate_vel ){
                    ins15_update_gnss_vel(&C->ins, C->gnss_vel_enu, Rv, NULL);
                    C->last_gnss_used_vel = 1;
                    C->last_gnss_used_s = tnow2;
                    C->gnss_valid = true;
                }
            } else {
                C->last_nis_vel = 0.0f;
            }

            // GPS heading measurement update through the EKF
            // TAU1204 dual-band COG accuracy ≈ inversely proportional to speed
            // sigma ≈ max(0.3°, 3°/speed) — TAU1204 dual-band COG is very clean
            if (C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN && hdop <= YAW_FUSION_HDOP_MAX) {
                float sigma_hdg_deg = fmaxf(0.3f, 3.0f / C->gnss_speed_mps);
                float sigma_hdg_rad = sigma_hdg_deg * DEG2RAD;
                float R_hdg = sigma_hdg_rad * sigma_hdg_rad;
                ins15_update_heading(&C->ins, C->gnss_heading_rad, R_hdg);
                if (!C->heading_observed) C->heading_observed = true;
            }
            #endif 
            gnss_done:
            C->have_gnss = false; // consumed
        } else {
            // Dead-reckoning status prints at ~2 Hz (enhanced for DR debugging)
            static double t_last = 0.0;
            double t = now_sec();
            if (t - t_last > 0.5) {
            #if USE_2D_EKF
                float dr_speed = sqrtf(C->ins2d.v_E*C->ins2d.v_E + C->ins2d.v_N*C->ins2d.v_N);
                float since_moving = (C->last_moving_s > 0) ? (float)(t - C->last_moving_s) : -1.0f;
                dbg_printf(C, "DR(2D): outage=%.1fs qS=%.1f p=(%.1f,%.1f) v=(%.2f,%.2f) "
                        "|v|=%.2f psi=%.1f zupt=%d anchor=%d since_mov=%.1fs gnss_spd=%.1f",
                        C->outage_s, C->qscale,
                        C->ins2d.p_E, C->ins2d.p_N,
                        C->ins2d.v_E, C->ins2d.v_N,
                        dr_speed,
                        C->ins2d.psi / DEG2RAD,
                        (C->zupt_count >= ZUPT_COUNT_REQUIRED) ? 1 : 0,
                        C->anchor_set ? 1 : 0,
                        since_moving,
                        C->gnss_speed_mps);
            #else
                float dr_speed = v3_norm(C->ins.v);
                float since_moving = (C->last_moving_s > 0) ? (float)(t - C->last_moving_s) : -1.0f;
                dbg_printf(C, "DR: outage=%.1fs qS=%.1f p=(%.1f,%.1f,%.1f) v=(%.2f,%.2f,%.2f) "
                        "|v|=%.2f zupt=%d anchor=%d since_mov=%.1fs gnss_spd=%.1f",
                        C->outage_s, C->qscale,
                        C->ins.p.x, C->ins.p.y, C->ins.p.z,
                        C->ins.v.x, C->ins.v.y, C->ins.v.z,
                        dr_speed,
                        (C->zupt_count >= ZUPT_COUNT_REQUIRED) ? 1 : 0,
                        C->anchor_set ? 1 : 0,
                        since_moving,
                        C->gnss_speed_mps);
            #endif
                t_last = t;
            }
        }

        // ENU output: CSV file only at 1 Hz (handled in the 1 Hz CSV block below)
        // No stdout output — all position data goes to the log file.

        // 1 Hz CSV logging
        double tlog = now_sec();
        if (tlog - C->last_log_s >= 1.0) {
            if (C->logf) {
            #if USE_2D_EKF
           
                float yaw_deg = C->ins2d.psi / DEG2RAD;

                //Convert EKF position to lat/lon for CSV (Google Maps compatible)
                double ekf_lat_deg = 0.0, ekf_lon_deg = 0.0;
                if (C->enu_ref_set) {
                    ecef_t ekf_ecef = enu2ecef(
                        (double)C->ins2d.p_E, (double)C->ins2d.p_N, 0.0,
                        C->enu_ref_lla, C->enu_ref_ecef);
                    lla_t ekf_lla = ecef2lla(ekf_ecef);
                    ekf_lat_deg = ekf_lla.lat * (180.0 / M_PI);
                    ekf_lon_deg = ekf_lla.lon * (180.0 / M_PI);
                }

                fprintf(C->logf, "%.3f, %.9f,%.9f, %.3f,%.3f,0.0, %.3f,%.3f,0.0,%.2f,"
                    "%.6f,0.0,0.0,0.0,0.0,%.6f,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%d,%.7f,%.7f,%.2f,%.3f,%.2f,%.2f,%.2f,"
                    "%.3f,%.3f,0.0,%.3f,%.3f,%.3f,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%.3f,%.2f,%.2f,%.2f,%.3f,"
                    "%.3f,%.3f,0.0,%d,%d\n",
                    tlog,
                    ekf_lat_deg, ekf_lon_deg,
                    C->ins2d.v_E, C->ins2d.v_N,
                    C->last_aw.x, C->last_aw.y,
                    yaw_deg,
                    C->ins2d.b_a, C->ins2d.b_g,
                    (int)C->imu_raw.ax, (int)C->imu_raw.ay, (int)C->imu_raw.az,
                    (int)C->imu_raw.gx, (int)C->imu_raw.gy, (int)C->imu_raw.gz,
                    C->gnss_last_fix,
                    C->gnss_last_lat_deg, C->gnss_last_lon_deg, C->gnss_last_alt_m,
                    C->gnss_last_speed_mps,
                    C->gnss_last_course_deg, C->gnss_last_yaw_enu_deg, C->gnss_last_hdop,
                    C->gnss_enu_last.x, C->gnss_enu_last.y, 
                    C->ins2d.p_E - C->gnss_enu_last.x, 
                    C->ins2d.p_N - C->gnss_enu_last.y,
                    sqrtf(powf(C->ins2d.p_E - C->gnss_enu_last.x, 2) + powf(C->ins2d.p_N - C->gnss_enu_last.y, 2)),
                    C->gnss_present ? 1 : 0,
                    C->gnss_valid ? 1 : 0,
                    C->last_gnss_used_pos,
                    C->last_gnss_used_vel,
                    C->reacq_active ? 1 : 0,
                    C->snap_applied ? 1 : 0,
                    C->outage_s, C->qscale,
                    C->last_nis_pos, C->last_nis_vel,
                    C->gnss_present ? (float)(tlog - C->last_gnss_meas_s) : -1.0f,
                    C->predict_p.x, C->predict_p.y,
                    (C->zupt_count >= ZUPT_COUNT_REQUIRED) ? 1 : 0,
                    C->anchor_set ? 1 : 0
                );
            #else

                yaw_learn_maybe_persist(&yawL, tlog);
                float yaw_deg = yaw_from_q(C->ins.q) / DEG2RAD;
                float gnss_meas_age_s = (C->last_gnss_meas_s > 0.0) ? (float)(tlog - C->last_gnss_meas_s) : -1.0f;
                float gnssE = 0, gnssN=0, gnssU=0, errE=0, errN=0, errH=0;
                if( C->gnss_enu_last_valid ){
                    gnssE = C->gnss_enu_last.x;
                    gnssN = C->gnss_enu_last.y;
                    gnssU = C->gnss_enu_last.z;
                    errE = C->ins.p.x - gnssE;
                    errN = C->ins.p.y - gnssN;
                    errH = sqrtf(errE*errE + errN*errN);
                }

                // Convert EKF ENU position to lat/lon for CSV (Google Maps compatible)
                double ekf_lat_deg = 0.0, ekf_lon_deg = 0.0;
                if (C->enu_ref_set) {
                    ecef_t ekf_ecef = enu2ecef(
                        (double)C->ins.p.x, (double)C->ins.p.y, (double)C->ins.p.z,
                        C->enu_ref_lla, C->enu_ref_ecef);
                    lla_t ekf_lla = ecef2lla(ekf_ecef);
                    ekf_lat_deg = ekf_lla.lat * (180.0 / M_PI);
                    ekf_lon_deg = ekf_lla.lon * (180.0 / M_PI);
                }

                fprintf(C->logf,
                    "%.3f,"
                    "%.9f,%.9f,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%.2f,"
                    "%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%d,%.7f,%.7f,%.2f,%.3f,%.2f,%.2f,%.2f,"
                    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%.3f,%.2f,%.2f,%.2f,%.3f,"
                    "%.3f,%.3f,%.3f,%d,%d\n",
                    tlog,
                    ekf_lat_deg, ekf_lon_deg,
                    C->ins.v.x, C->ins.v.y, C->ins.v.z,
                    C->last_aw.x, C->last_aw.y, C->last_aw.z,
                    yaw_deg,
                    C->ins.ba.x, C->ins.ba.y, C->ins.ba.z,
                    C->ins.bg.x, C->ins.bg.y, C->ins.bg.z,
                    (int)C->imu_raw.ax, (int)C->imu_raw.ay, (int)C->imu_raw.az,
                    (int)C->imu_raw.gx, (int)C->imu_raw.gy, (int)C->imu_raw.gz,
                    C->gnss_last_fix,
                    C->gnss_last_lat_deg, C->gnss_last_lon_deg, C->gnss_last_alt_m,
                    C->gnss_last_speed_mps,
                    C->gnss_last_course_deg,
                    C->gnss_last_yaw_enu_deg,
                    C->gnss_last_hdop,
                    gnssE, gnssN, gnssU, errE, errN, errH,
                    C->gnss_present ? 1 : 0,
                    C->gnss_valid ? 1 : 0,
                    C->last_gnss_used_pos,
                    C->last_gnss_used_vel,
                    C->reacq_active ? 1 : 0,
                    C->snap_applied ? 1 : 0,
                    C->outage_s, C->qscale,
                    C->last_nis_pos, C->last_nis_vel,
                    gnss_meas_age_s,
                    C->predict_p.x, C->predict_p.y, C->predict_p.z,
                    (C->zupt_count >= ZUPT_COUNT_REQUIRED) ? 1 : 0,
                    C->anchor_set ? 1 : 0
                );

            #endif 
                fflush(C->logf);

                // reset per-log latches
                C->last_gnss_used_pos = 0;
                C->last_gnss_used_vel = 0;
                C->reacq_active = false;
                C->snap_applied = false;
            }
            C->last_log_s = tlog;
        }

        pthread_mutex_unlock(&C->mtx);
    }

    return NULL;
}

// ------------------------- Main / signal handling -------------------------

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

int main(void) {
    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    ctx_t C;
    memset(&C, 0, sizeof(C));
    pthread_mutex_init(&C.mtx, NULL);
    pthread_cond_init(&C.cv, NULL);

#if USE_2D_EKF
    ins2d_init(&C.ins2d);
    C.att_q = q_identity();
    memset(&C.ahrs, 0, sizeof(C.ahrs));
    C.ahrs.q = q_identity();           /* initialized flag stays 0 until NAV_INIT */
#else
    ins15_init(&C.ins);
#endif

    C.running = true;
    C.enu_ref_set = false;
    C.nav_ready = false;
    C.gnss_valid = false;
C.gnss_present = false;
C.last_gnss_meas_s = 0.0;
C.last_gnss_used_s = 0.0;
C.last_gnss_fused_s = 0.0;
C.reacq_left = 0;
C.reacq_active = false;
C.snap_applied = false;
C.outage_s = 0.0;
C.qscale = 1.0f;
C.zupt_count = 0;
    C.last_nis_pos = 0.0f;
    C.last_nis_vel = 0.0f;
    C.last_aw = v3(0,0,0);
C.last_gnss_used_pos = 0;
C.last_gnss_used_vel = 0;
C.last_gnss_fix_mono_ns = -1;
C.gnss_enu_last_valid = false;
C.anchor_set = false;
C.sustained_still = 0;
C.predict_p = v3(0,0,0);
C.gnss_prev_speed_valid = false;
C.gnss_prev_speed_mps = 0.0f;
C.gnss_prev_speed_time = 0.0;
C.gnss_speed_rejected = false;

    make_log_file(&C);
    make_debug_log_file(&C);
    cal_set_defaults_from_lsq();

    /* ---- Load production calibration (imu_calib_t) ---- */
    imu_calib_set_defaults(&g_imu_cal);
    {
        imu_calib_t loaded;
        int rc = imu_calib_load(IMU_CALIB_PATH, &loaded);
        if (rc == 0) {
            g_imu_cal = loaded;
            /* Mirror the SI bias values into the legacy counts representation so
             * calib_gyro()/calib_accel() reflect the saved state. */
            const float counts_per_rps = GYRO_LSB_PER_DPS / DEG2RAD;
            g_cal.gyro_bias_counts[0] = g_imu_cal.gyro_bias[0] * counts_per_rps;
            g_cal.gyro_bias_counts[1] = g_imu_cal.gyro_bias[1] * counts_per_rps;
            g_cal.gyro_bias_counts[2] = g_imu_cal.gyro_bias[2] * counts_per_rps;
            g_cal.accel_O[0] -= g_imu_cal.accel_bias[0];
            g_cal.accel_O[1] -= g_imu_cal.accel_bias[1];
            g_cal.accel_O[2] -= g_imu_cal.accel_bias[2];
            time_t cal_t = (time_t)g_imu_cal.created_unix_s;
            double age_s = (cal_t > 0) ? difftime(time(NULL), cal_t) : -1.0;
            printf("[CAL_LOAD ok] path=%s age_s=%.0f gyro_bias=(%.3f,%.3f,%.3f)°/s "
                   "accel_bias=(%.3f,%.3f,%.3f) flags=0x%x\n",
                   IMU_CALIB_PATH, age_s,
                   g_imu_cal.gyro_bias[0]/DEG2RAD, g_imu_cal.gyro_bias[1]/DEG2RAD, g_imu_cal.gyro_bias[2]/DEG2RAD,
                   g_imu_cal.accel_bias[0], g_imu_cal.accel_bias[1], g_imu_cal.accel_bias[2],
                   g_imu_cal.valid_flags);
        } else if (rc == -1 && errno == ENOENT) {
            printf("[CAL_DEFAULT] no saved calibration at %s — using identity calibration\n",
                   IMU_CALIB_PATH);
        } else {
            printf("[CAL_LOAD rejected] reason=%d path=%s — using identity calibration; "
                   "EKF process noise will be inflated until boot cal completes.\n",
                   rc, IMU_CALIB_PATH);
            imu_calib_set_defaults(&g_imu_cal);
            g_imu_quality_degraded = 1;     /* be cautious until boot cal validates */
        }
        imu_calib_sync_from_g_cal(&g_imu_cal);
    }

    printf("[CAL] Power-on IMU calibration will run on every boot (will refine and persist).\n");

    C.imu_cal_done =false;
    C.imu_cal_in_progress = false;
    C.imu_cal_start_s = 0.0;

    cal_led_init();

    pthread_t th_imu, th_gnss, th_fuse;
    if (pthread_create(&th_imu, NULL, imu_thread, &C) != 0) { perror("imu_thread"); return 1; }
    if (pthread_create(&th_gnss, NULL, gnss_thread, &C) != 0) { perror("gnss_thread"); return 1; }
    if (pthread_create(&th_fuse, NULL, fusion_thread, &C) != 0) { perror("fusion_thread"); return 1; }

    // Block until SIGINT/SIGTERM
    while (!g_stop) pause();

    __atomic_store_n(&C.running, false, __ATOMIC_RELEASE);
    pthread_join(th_imu, NULL);
    pthread_join(th_gnss, NULL);
    pthread_join(th_fuse, NULL);

    pthread_mutex_destroy(&C.mtx);
    pthread_cond_destroy(&C.cv);
    
    if (C.logf) fclose(C.logf);
    cal_led_deinit();

    /* Final persistence: flush any runtime-cal refinements to disk on clean exit. */
#if ENABLE_CAL_FILE_SAVE
    if ((g_imu_cal.valid_flags & (CAL_FLAG_BOOT_OK | CAL_FLAG_RUNTIME_OK)) != 0) {
        if (imu_calib_save_atomic(IMU_CALIB_PATH, &g_imu_cal) == 0) {
            printf("[CAL_EXIT] saved calibration to %s (flags=0x%x)\n",
                   IMU_CALIB_PATH, g_imu_cal.valid_flags);
        }
    }
#endif
    return 0;
}
