// dr_dead_reckoning_app_ins15.c
//
// 15-state INS (Error-State EKF) + GNSS (Neo6M) dead-reckoning app for RPi3B + MPU6050
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
//   - neo6m_gnss_ioctl.h (struct neo6m_gnss_fix; this app expects optional fields for hdop/heading)
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
// Reacquisition / snap parameters (GNSS returns after outage)
#define REACQ_MIN_OUTAGE_S     3.0f     // start reacq if outage longer than this
#define REACQ_STEPS           8        // number of GNSS attempts in reacq mode
#define REACQ_R_MULT          100.0f   // inflate Rpos during reacq
#define REACQ_CHI2_GATE       200.0f   // relaxed gate during reacq (3 dof)
#define SNAP_MIN_OUTAGE_S     15.0f    // allow snap after long outage
#define SNAP_INNOV_M          120.0f   // if horizontal innovation exceeds, snap to GNSS
#define SNAP_HDOP_MAX         3.0f     // require decent HDOP for snap

#define GRAVITY             9.80665f
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD ( (float)M_PI / 180.0f )

// If your neo6m_gnss_fix was extended with HDOP + heading/course fields, enable these.
#ifndef HAVE_GNSS_HDOP
#define HAVE_GNSS_HDOP      1
#endif
#ifndef HAVE_GNSS_HEADING
#define HAVE_GNSS_HEADING   1
#endif

// Yaw dead-band for near-stationary conditions (limits gyro z noise from integrating into yaw)
#define YAW_DEADBAND_RAD    (0.03f)           // ~1.7 deg/s
#define ACC_STILL_TOL       (0.10f*GRAVITY)
#define GYRO_STILL_TOL_RAD  (3.0f * DEG2RAD)

// ZUPT detection thresholds
#define ZUPT_ACC_THR        (0.40f)           // | |a|-g | < thr
#define ZUPT_GYRO_THR       (5.0f * DEG2RAD)
#define ZUPT_COUNT_REQUIRED 5

// Mild velocity decay (helps bound drift when filter is "almost stopped")
#define VEL_DECAY           0.98f
#define VEL_EPS             1e-3f

// Outage-tiered Q inflation
#define TIER_A  2.0f
#define TIER_B  10.0f
#define TIER_C  60.0f
#define QSCL_A  2.0f
#define QSCL_B  5.0f
#define QSCL_C  10.0f
#define QSCL_D  20.0f

// GNSS gating and fade-in
#define CHI2_3DOF_GATE      50.0f            // ~99.5% for 3 DOF
#define R_GNSS_POS_VAR      4.0f             // base (m^2), scaled by HDOP^2
#define FADE_IN_FACT_INIT   4.0f
#define FADE_IN_STEPS       3

// Measurement noise
#define RV_VEL_E            (0.09f)          // (0.30 m/s)^2
#define RV_VEL_N            (0.09f)
#define RV_VEL_U            (0.36f)          // (0.60 m/s)^2
#define R_NHC_VY            (0.01f)          // (m/s)^2
#define R_NHC_VZ            (0.04f)
#define R_ZUPT_V            (0.0004f)        // (m/s)^2

// GNSS yaw fusion (optional complementary yaw correction)
#define YAW_FUSION_SPEED_MIN    (1.0f)       // m/s
#define YAW_FUSION_HDOP_MAX     (4.0f)
#define YAW_FUSION_ALPHA        (0.05f)      // fraction per GNSS update

// Logging
#define LOG_DIR             "/home/sijeo/nav_logs"

// MPU6050 scale (assuming ±500 dps for gyro here; adjust to your config)
#define GYRO_LSB_PER_DPS    65.5f

#define aWGS        6378137.0
#define fWGS        (1.0/298.257223563)
#define bWGS        (6378137.0 * (1 - fWGS))
#define  e2         ((aWGS*aWGS - bWGS*bWGS) / (aWGS*aWGS))


#define CAL_FILE_PATH   "/home/sijeo/dr_vehicle/imu_cal.bin"
#define CAL_MAGIC       0x43414C31u /* 'CAL1' */
#define CAL_VERSION     1u

/**--------------------------GNSS yaw-rate learner tuning --------------------
 * Learn gyro Z scale using GNSS course/heading during turns.
 * Units rad/s, m/s, seconds
 */
#define YAW_LEARN_MIN_SPEED_MPS     2.0f
#define YAW_LEARN_MAX_HDOP          2.5f
#define YAW_LEARN_TURN_RATE_THR     0.03f   // ~ 1.7deg/s
#define YAW_LEARN_MIN_TURN_DEG      20.0f
#define YAW_LEARN_MIN_SEG_S         3.0f
#define YAW_LEARN_MAX_SEG_S         40.0f
#define YAW_LEARN_BETA_SCALE        0.05f   // LPF update for scale
#define YAW_LEARN_SCALE_MIN         0.85f
#define YAW_LEARN_SCALE_MAX         1.15f
#define YAW_LEARN_PERSIST_PERIOD_S  60.0


#define CAL_LED_GPIO        13     //BCM GPIO 13
#define CAL_LED_BLINK_HZ    1.0     // 2.0Hz blink while calibration pending

#define BOOT_CAL_N_SAMPLES  500     // e.g. 500 samples (tune based on IMU rate )

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

    out[0] = -s0*dx + c0*dy;
    out[1] = -c1*c0*dx - c1*s0*dy + s1*dz;
    out[2] =  s1*c0*dx + s1*s0*dy + c1*dz;
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

/* ---- tincy CRC32 ( good enough for a small blob ) ---- */
#if 0
static uint32_t crc32_simple( const void *data, size_t nbytes ) {
    size_t i;
    int b;
    const uint8_t *p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    for( i=0; i<nbytes; i++ ){
        crc ^= p[i];
        for( b=0; b<8; b++ ){
            uint32_t m = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & m);
        }
    }
    return ~crc;
}
#endif

static void cal_set_defaults_from_lsq( void ) {
    memset(&g_cal, 0, sizeof(g_cal));
    g_cal.magic = CAL_MAGIC;
    g_cal.version = CAL_VERSION;

    /* Seed with your current LSQ Values (so the behavior deosn't regress )*/
    g_cal.accel_C[0][0] = 0.0005964462462844185f; g_cal.accel_C[0][1] = -9.211739001666678e-07f; g_cal.accel_C[0][2] = 1.5763305507490624e-05f;
    g_cal.accel_C[1][0] = -1.1573398643476584e-06f; g_cal.accel_C[1][1] = 0.0006037351040586055f; g_cal.accel_C[1][2] = 3.881537441769146e-07f;
    g_cal.accel_C[2][0] = -3.851697134466662e-05f; g_cal.accel_C[2][1] = -3.2356391081574615e-05f; g_cal.accel_C[2][2] =  0.0005895175306304627f;

    g_cal.accel_O[0] = -0.1329121010477303f;
    g_cal.accel_O[1] = -0.047222673271787766f;
    g_cal.accel_O[2] = 1.257425727446983f;

    g_cal.gyro_bias_counts[0] = -153.461f;
    g_cal.gyro_bias_counts[1] = 69.446f;
    g_cal.gyro_bias_counts[2] = 992.782f;

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

/* ---------------- Stillness based calibration ( first install + runtime ) -------------------*/
typedef struct {
    bool active;
    int N;
    double t0;
    double gyro_sum[3];     /* raw counts */
    double acc_sum[3];      /* m/s^2 (after LSQ) */

}still_accum_t;

static void still_reset(still_accum_t *A ){
    A->active = false;
    A->N = 0;
    A->t0 = 0.0;
    A->gyro_sum[0] = A->gyro_sum[1] = A->gyro_sum[2] = 0.0;
    A->acc_sum[0] = A->acc_sum[1] = A->acc_sum[2] = 0.0;

}

static bool apply_poweron_calibration(still_accum_t *A, 
    const struct mpu6050_sample *raw,
    vec3f acc_b_lsq, 
    float still_required_s, 
    double now_s )
{
    /* Start accumulation on first call */
    if( !A->active ){
        A->active = true;
        A->t0 = now_s;
        A->N = 0;
        A->gyro_sum[0] = A->gyro_sum[1] = A->gyro_sum[2] = 0.0;
        A->acc_sum[0] = A->acc_sum[1] = A->acc_sum[2] = 0.0;
    }

    /* Accumulate raw gyro counts */
    A->gyro_sum[0] += (double)raw->gx;
    A->gyro_sum[1] += (double)raw->gy;
    A->gyro_sum[2] += (double)raw->gz;

    /*Accumulate LSQ-corrected accle(m/s2)*/
    A->acc_sum[0] += (double)acc_b_lsq.x;
    A->acc_sum[1] += (double)acc_b_lsq.y;
    A->acc_sum[2] += (double)acc_b_lsq.z;
    
    A->N++;

    /* Check time condition only */
    if((now_s - A->t0) < (double)still_required_s)
        return false;
    
    /*--------------------Finalize calibration ----------------------*/

    double invN = 1.0 / (double)A->N;

    /*1) Gyro bias in raw counts */
    g_cal.gyro_bias_counts[0] = (float)(A->gyro_sum[0] * invN);
    g_cal.gyro_bias_counts[1] = (float)(A->gyro_sum[1] * invN);
    g_cal.gyro_bias_counts[2] = (float)(A->gyro_sum[2] * invN);

    /**
     * 2) Accel bias trim (no attitude needed )
     * Enforce |acc| = g along measured gravity direction 
     */
    vec3f acc_mean = v3((float)(A->acc_sum[0] * invN),
                        (float)(A->acc_sum[1] * invN),
                        (float)(A->acc_sum[2] * invN));

    float n = v3_norm(acc_mean);
    if( n > 1e-3f ){
        vec3f g_dir = v3_scale(acc_mean, 1.0f/n);   // measured gravity direction 
        vec3f acc_exp = v3_scale(g_dir, GRAVITY);   // Expected Gravity vector
        vec3f resid = v3_sub(acc_mean, acc_exp);    // bias-like residual

        /* accel_b = C *  raw + O */
        g_cal.accel_O[0] -= resid.x;
        g_cal.accel_O[1] -= resid.y;
        g_cal.accel_O[2] -= resid.z;
    }
    still_reset(A);
    return true;
 
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
    const float p0 = 5.0f;          // m
    const float v0 = 1.0f;          // m/s
    const float th0 = 5.0f*DEG2RAD; // rad
    const float ba0 = 0.5f;         // m/s^2
    const float bg0 = 0.01f;        // rad/s

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
    const float sigma_a = 0.50f;     // m/s^2
    const float sigma_g = 0.010f;    // rad/s
    const float sigma_ba = 0.010f;   // m/s^2/sqrt(s)
    const float sigma_bg = 0.005f;   // rad/s/sqrt(s)

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
    // also inject into dp via integration of dv noise (rough)
    for (i=0;i<3;i++) Q[(0+i)*15 + (0+i)] = sa2 * dt*dt*dt / 3.0f;

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

    // INS
    ins15_t ins;

    // ENU reference
    bool enu_ref_set;
    lla_t enu_ref_lla;
    ecef_t enu_ref_ecef;

    // nav state
    bool nav_ready;

    // GNSS presence / outage / reacquisition tracking
double last_gnss_meas_s;     // last time we RECEIVED a GNSS fix (raw measurement)
double last_gnss_used_s;     // last time we ACCEPTED (used) GNSS position update
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
    vec3f last_aw;

    // logging
    FILE *logf;
    char log_filename[256];
    FILE *dbglog;
    char dbglog_filename[256];
    double last_log_s;

    int64_t last_gnss_fix_mono_ns;

    vec3f gnss_enu_last;
    bool gnss_enu_last_valid;
    

    // threading
    pthread_mutex_t mtx;
    pthread_cond_t cv;
    bool running;

    // Calibration on Power On
    bool imu_cal_in_progress;
    bool imu_cal_done;
    double imu_cal_start_s;
    
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
        "E,N,U,"
        "V_E,V_N,V_U,"
        "A_E,A_N,A_U,"
        "yaw_deg,"
        "ba_x,ba_y,ba_z,"
        "bg_x,bg_y,bg_z,"
        "imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,"
        "gnss_fix,lat_deg,lon_deg,alt_m,speed_mps,course_deg,yaw_enu_deg,hdop,"
        "gnss_E,gnss_N,gnss_U,err_E,err_N,err_H,"
        "gnss_present,gnss_valid,gnss_used_pos,gnss_used_vel,reacq_active,snap_applied,"
        "outage_s,qscale,nis_pos,nis_vel,gnss_meas_age_s\n"
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
            C->gnss_speed_mps = (float)f.speed_mmps / 1000.0f;

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
            if (f.heading_valid && C->gnss_speed_mps > 0.5f) {
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

            // Set ENU reference on first ever fix
            if (!C->enu_ref_set) {
                C->enu_ref_lla = (lla_t){C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m};
                C->enu_ref_ecef = lla2ecef(C->enu_ref_lla);
                C->enu_ref_set = true;
                dbg_printf(C, "ENU_REF_SET lat=%.9f lon=%.9f alt=%.3f",
                C->enu_ref_lla.lat * (180.0/M_PI), 
                C->enu_ref_lla.lon * (180.0/M_PI),
                C->enu_ref_lla.h); /* Print in Degrees for Sanity*/
            }
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

        // Calibrate IMU
        vec3f acc_b, gyro_b;
        calib_accel(&C->imu_raw, &acc_b);
        calib_gyro(&C->imu_raw, &gyro_b);

        float acc_norm = v3_norm(acc_b);

        // ZUPT-like stillness condition (used for both constraints and calibration)
         bool zupt_cond =
            (fabsf(acc_norm - GRAVITY) < ZUPT_ACC_THR) &&
            (fabsf(gyro_b.x) < ZUPT_GYRO_THR) &&
            (fabsf(gyro_b.y) < ZUPT_GYRO_THR) &&
            (fabsf(gyro_b.z) < ZUPT_GYRO_THR);

        /* -------------------Power on IMU calibration (once per boot; no file persistence )-----------------------*/
        static still_accum_t cal_accum_boot;
        double tcal_now = now_sec();

        while ( !C->imu_cal_done ){
            if(!C->imu_cal_in_progress ){
                C->imu_cal_in_progress = true;
                C->imu_cal_start_s = tcal_now;
                still_reset(&cal_accum_boot);
                printf("[CAL] Power on IMU Calibration started: Keep vehicle still... \n");
            }

            // Blink LED while calibration 
            cal_led_update(true, false, tcal_now);

            bool done = apply_poweron_calibration(&cal_accum_boot, &C->imu_raw, acc_b, 10.0f, tcal_now );
            if(done) {
                C->imu_cal_done = true;
                C->imu_cal_in_progress = false;
                cal_led_update(false, true, tcal_now); // stead ON
                printf("[CAL] Power-On IMU calibration complete. \n");
                printf("       gyro_bias_counts = [%.3f, %.3f, %.3f]\n", g_cal.gyro_bias_counts[0], g_cal.gyro_bias_counts[1],
                g_cal.gyro_bias_counts[2]);
                printf("       accel_O = [%.6f, %.6f, %.6f]\n", g_cal.accel_O[0], g_cal.accel_O[1], g_cal.accel_O[2]);
            }

        } 
    cal_led_update(false, true, tcal_now);




        // Initialize nav on first GNSS fix (wait until ENU origin set AND have current GNSS fix)
        if (!C->nav_ready) {
            if (C->enu_ref_set && C->have_gnss && C->gnss_have_fix) {
                lla_t L = { C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m };
                ecef_t E = lla2ecef(L);
                dbg_printf(C, "NAV_INIT using ENU_REF lat=%.9f lon=%.9f alt=%.3f",
                C->enu_ref_lla.lat * (180.0/M_PI),
                C->enu_ref_lla.lon * (180.0/M_PI),
                C->enu_ref_lla.h);
                double enu_d[3];
                ecef2enu(E, C->enu_ref_lla, C->enu_ref_ecef, enu_d);
                dbg_printf(C, "NAV_INIT zpos_ENU=(%.3f,%.3f,%.3f)", enu_d[0],enu_d[1],enu_d[2]);
                ins15_init(&C->ins);
                C->ins.p = v3((float)enu_d[0], (float)enu_d[1], (float)enu_d[2]);
                C->ins.v = v3(0,0,0);
                //C->ins.q = q_identity();
                if( C->gnss_heading_valid ){
                    yaw_learn_on_gnss_fix(&yawL, C->gnss_heading_rad, C->last_gnss_fix_mono_ns);
                }
                float yaw0 = 0.0f;
                if(C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN ) {
                    yaw0 = C->gnss_heading_rad;
                }
                C->ins.q = q_normalize(q_from_yaw_delta(yaw0));

                // Start biases at 0 (since you already calibrated raw -> SI).
                C->ins.ba = v3(0,0,0);
                C->ins.bg = v3(0,0,0);

                C->nav_ready = true;
                // GNSS measurement just initialized navigation -> treat as present+used
                double t0 = now_sec();
                C->last_gnss_meas_s = t0;
                C->last_gnss_used_s = t0;
                C->gnss_present     = true;
                C->gnss_valid       = true;   // legacy: last accepted update
                C->reacq_left       = 0;
                C->reacq_active     = false;
                C->snap_applied     = false;
                C->outage_s         = 0.0;
                C->qscale           = 1.0f;
                C->last_nis_pos = 0.0f;
                C->last_nis_vel = 0.0f;

                dbg_printf(C, "NAV_READY ins.p=(%.3f,%.3f,%.3f)",C->ins.p.x, C->ins.p.y, C->ins.p.z);

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

        // Apply constraints every IMU step
        ins15_update_nhc(&C->ins);

        // ZUPT detection
       
        if (zupt_cond) C->zupt_count++;
        else C->zupt_count = 0;

        if (C->zupt_count >= ZUPT_COUNT_REQUIRED) {
            ins15_update_zupt(&C->ins);
            ins15_update_zaru(&C->ins, gyro_b);
            C->zupt_count = 0;
        }

        // Velocity decay when *almost* stopped (helps fight tiny residuals)
        float vnorm = v3_norm(C->ins.v);
        const float VEL_DAMP_THR = 0.10f;
        if (vnorm < VEL_DAMP_THR) {
            C->ins.v = v3_scale(C->ins.v, VEL_DECAY);
        }

        // GNSS fusion (pos + optional vel + optional yaw complementary)
        if (C->have_gnss && C->gnss_have_fix) {
            if( C->gnss_heading_valid ){
                yaw_learn_on_gnss_fix(&yawL, C->gnss_heading_rad, C->last_gnss_fix_mono_ns);
            }
            lla_t L = { C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m };
            ecef_t E = lla2ecef(L);
            double enu_d[3];
            ecef2enu(E, C->enu_ref_lla, C->enu_ref_ecef, enu_d);
            vec3f zpos = v3((float)enu_d[0], (float)enu_d[1], (float)enu_d[2]);
            C->gnss_enu_last = zpos;
            C->gnss_enu_last_valid = true;
            dbg_printf(C, "GNSS_FUSE ENU_REF lat=%.9f lon=%.9f alt=%.3f meas_age=%.3f present=%d",
            C->enu_ref_lla.lat * (180.0/M_PI),
            C->enu_ref_lla.lon * (180.0/M_PI),
            C->enu_ref_lla.h,
            (float)(now_sec() - C->last_gnss_meas_s),
            C->gnss_present ? 1 : 0);
            dbg_printf(C, "GNSS_FUSE zpos_ENU=(%.3f,%.3f,%.3f) ins.p=(%.3f,%.3f,%.3f)",
            zpos.x, zpos.y, zpos.z, 
            C->ins.p.x, C->ins.p.y, C->ins.p.z);
            float hdop = (C->gnss_hdop_valid ? C->gnss_hdop : 1.0f);
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
                if (C->gnss_vel_valid) C->ins.v = C->gnss_vel_enu;

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
                C->reacq_left = 0; // done
            } else {
                float nis_pos = 0.0f;
                bool ok_pos = ins15_update_gnss_pos(&C->ins, zpos, Rpos, &nis_pos);
                bool gate_pos = ok_pos && (nis_pos < gate_thr);
                C->last_nis_pos = nis_pos;

                if (gate_pos) {
                    C->last_gnss_used_s = tnow2;
                    C->gnss_valid = true;
                    C->last_gnss_used_pos = 1;
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
                float nis_vel = 0.0f;
                bool ok_vel = ins15_update_gnss_vel(&C->ins, C->gnss_vel_enu, Rv, &nis_vel);
                bool gate_vel = ok_vel && (nis_vel < CHI2_3DOF_GATE);
                C->last_nis_vel = nis_vel;
                
                if( gate_vel ){
                    C->last_gnss_used_vel = 1;
                    C->last_gnss_used_s = tnow2;
                    C->gnss_valid = true;
                }
            } else {
                C->last_nis_vel = 0.0f;
            }

            // Optional yaw complementary correction using GNSS course (if available)
            if (C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN && hdop <= YAW_FUSION_HDOP_MAX) {
                float yaw_est = yaw_from_q(C->ins.q);
                float yaw_gnss = C->gnss_heading_rad;
                float dpsi = wrap_pi(yaw_gnss - yaw_est);
                float dpsi_corr = YAW_FUSION_ALPHA * dpsi;
                C->ins.q = q_normalize(q_mul(q_from_yaw_delta(dpsi_corr), C->ins.q));
            }
            C->have_gnss = false; // consumed
        } else {
            // Dead-reckoning status prints at ~2 Hz
            static double t_last = 0.0;
            double t = now_sec();
            if (t - t_last > 0.5) {
                dbg_printf(C, "DR: outage=%.1fs qS=%.1f p=(%.1f,%.1f,%.1f) v=(%.2f,%.2f,%.2f)\n",
                        C->outage_s, C->qscale,
                        C->ins.p.x, C->ins.p.y, C->ins.p.z,
                        C->ins.v.x, C->ins.v.y, C->ins.v.z);
                t_last = t;
            }
        }

        // Output ENU only (after nav is ready)
        printf("%.3f,%.3f,%.3f\n", C->ins.p.x, C->ins.p.y, C->ins.p.z);
        fflush(stdout);

        // 1 Hz CSV logging
        double tlog = now_sec();
        if (tlog - C->last_log_s >= 1.0) {
            yaw_learn_maybe_persist(&yawL, tlog);
            if (C->logf) {
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

                fprintf(C->logf,
                    "%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%.2f,"
                    "%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%d,%.7f,%.7f,%.2f,%.3f,%.2f,%.2f,%.2f,"
                    "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%.3f,%.2f,%.2f,%.2f,%.3f\n",
                    tlog,
                    C->ins.p.x, C->ins.p.y, C->ins.p.z,
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
                    gnss_meas_age_s
                );
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

    ins15_init(&C.ins);

    C.running = true;
    C.enu_ref_set = false;
    C.nav_ready = false;
    C.gnss_valid = false;
C.gnss_present = false;
C.last_gnss_meas_s = 0.0;
C.last_gnss_used_s = 0.0;
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

    make_log_file(&C);
    make_debug_log_file(&C);
    cal_set_defaults_from_lsq();

    printf("[CAL] Power-on IMU calibration will run on every boot (no file persistence).\n");

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
    return 0;
}
