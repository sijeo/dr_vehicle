
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
#include <sys/ioctl.h>

#include "mpu6050_ioctl.h"
#include "neo6m_gnss_ioctl.h"


// ------------------------- Configuration -------------------------

#define IMU_DEVICE_PATH     "/dev/mpu6050-0"
#define NEO6M_DEVICE_PATH   "/dev/neo6m0"

#define IMU_HZ              100.0f
#define DT_IMU_DEFAULT      (1.0f/IMU_HZ)

#define GNSS_TIMEOUT_S      2.0f

#define GRAVITY             9.80665f
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD ( (float)M_PI / 180.0f )
#define RAD2DEG ( 180.0f/(float)M_PI )

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
#define VEL_EPS             0.01f
#define MAX_VELOCITY        50.0f   //m/s

// Outage-tiered Q inflation
#define TIER_A  2.0f
#define TIER_B  10.0f
#define TIER_C  60.0f
#define QSCL_A  2.0f
#define QSCL_B  5.0f
#define QSCL_C  10.0f
#define QSCL_D  20.0f

// GNSS gating and fade-in
#define CHI2_3DOF_GATE      16.2f            // ~99.5% for 3 DOF
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

#define aWGS       6378137.0
#define fWGS       (1.0/298.257223563)
#define bWGS       (6378137.0 * (1 - fWGS))
#define e2         ((aWGS*aWGS - bWGS*bWGS) / (aWGS*aWGS))


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
static inline vec3f v3_normalize(vec3f a ){
    float n = v3_norm(a);
    return (n > 1e-12f) ? v3_scale(a, 1.0f/n) : a;
}

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

static inline quatf q_from_ yaw_pitch_roll( float yaw, float pitch, float roll ){
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    return (quatf){cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy };
}

static inline void R_from_q(quatf q, float R[9]) {
    // ENU: columns are body axes in world? Here: R maps body->world: v_w = R * v_b
    q = q_normalize(q);
    float w=q.w, x=q.x, y=q.y, z=q.z;
    R[0] = 1 - 2*(y*y + z*z);  R[1] = 2*(x*y - w*z);      R[2] = 2*(x*z + w*y);
    R[3] = 2*(x*y + w*z);      R[4] = 1 - 2*(x*x + z*z);  R[5] = 2*(y*z - w*x);
    R[6] = 2*(x*z - w*y);      R[7] = 2*(y*z + w*x);      R[8] = 1 - 2*(x*x + y*y);
}

static inline quatf q_from_yaw(float yaw ) {
    return q_normalize((quatf){cosf(yaw/2), 0, 0, sinf(yaw/2)});
}

static inline float yaw_from_q(quatf q) {
    // yaw about +Z in ENU
    q = q_normalize(q);
    float siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    return atan2f(siny_cosp, cosy_cosp);
}

static inline quatf q_from_yaw_delta(float dpsi) {
    float half = 0.5f*dpsi;
    return q_normalize((quatf){cosf(half), 0.0f, 0.0f, sinf(half)});
}

static inline quatf q_nlerp(quatf a, quatf b, float dt ){
    /* Normalized linear interpolation (adequate for small t )*/
    quatf q = (quatf){
        a.w + dt*(b.w - a.w),
        a.x + dt*(b.x - a.x),
        a.y + dt*(b.y - a.y),
        a.z + dt*(b.z - a.z), 
    };
    return q_normalize(q);
}

static void ins15_tilt_correction(ins15_t *s vec3f acc_meas_b, vec3f gyro_meas_b, float alpha) {
/* User accelerometer to stabilize roll/pitch when dynamics are low */
/* Keeps current yaw (no magnetometer ), only corrects tilt */
float an = v3_norm(acc_meas_b);
float wn = v3_norm(gyro_meas_b);
if( an < 1e-3f ) return;

bool low_dyn = (fabs(an - GRAVITY) < 0.35f) &&(wn < 0.35f);
if( !low_dyn ) return ;

/* Accel-based tilt (body frame: x forward, y right, z down/up depending on the mount )
* This formula assumez +Z points up  in the body after mounting matrizx. if +Z is down flip az.
*/
float ax = acc_meas_b.x, ay = acc_meas_b.y, az = acc_meas_b.z;
/* Derive roll/pitch that make body-Z align with gravity direction */
float roll = atan2f(ay, az);
float pitch = atan2f(-ax, sqrtf(ay*ay + az*az) + 1e-6f);

float yaw = yaw_from_q(s->q);
quatf q_tilt = q_from_yaw_pitch_roll(yaw, pitch, roll);

/* small blend toward accel tilt */
s->q = q_nlerp(s->q, q_tilt, clampf(alpha, 0.0f, 0.2f));
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
    double slat = sin(ref.lat), clat = cos(ref.lat);
    double slon = sin(ref.lon), clon = cos(ref.lon);
    double dx = e.x - e0.x;
    double dy = e.y - e0.y;
    double dz = e.z - e0.z;

    out[0] = -slon*dx + clon*dy;   /**< East */
    out[1] = -slat*clon*dx - slat*slon*dy + clat*dz; /** NORTH */
    out[2] =  clat*clon*dx + clat*slon*dy + slat*dz; /** UP */
}

/** Convert ENU to LLA for debugging  */
static void enu2lla( vec3f enu, lla_t ref_lla, ecef_t ref_ecef, double *lat, double *lon, double *alt){
    double sin_lat = sin(ref_lla.lat);
    double cos_lat = cos(ref_lla.lat);
    double sin_lon = sin(ref_lla.lon);
    double cos_lon = cos(ref_lla.lon);

    /* ENU to ECEF offset */
    double dx = -sin_lon*enu.x - cos_lon*sin_lat*enu.y + cos_lon*cos_lat*enu.z;
    double dy = cos_lon*enu.x - sin_lon*sin_lat*enu.y + sin_lon*cos_lat*enu.z;
    double dz = cos_lat*enu.y + sin_lat*enu.z;

    /** Add to reference ECEF */
    ecef_t e = { ref_ecef.x + dx, ref_ecef.y + dy, ref_ecef.z + dz };

    /** Convert back to LLA */
    double p = sqrt(e.x*e.x + e.y*e.y);
    double theta = atan2(e.z * aWGS, p * bWGS);
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    *lat = atan2(e.z + e2 * bWGS * sin_theta*sin_theta*sin_theta, p - e2 * aWGS * cos_theta*cos_theta*cos_theta);
    *lon = atan2(e.y, e.x);

    double sin_lat2 = sin(*lat);
    double N = aWGS / sqrt(1.0f - e2*sin_lat2*sin_lat2);
    *alt = p / cos(*lat) - N;
}

// ------------------------- Calibration (from your current app) -------------------------
// (Keep your LSQ accel matrix + offset; and gyro bias in raw counts.)

static const float ACCEL_C[3][3] = {
    {0.0005964462462844185f, -9.211739001666678e-07f, 1.5763305507490624e-05f},
    {-1.1573398643476584e-06f, 0.0006037351040586055f, 3.881537441769146e-07f},
    {-3.851697134466662e-05f, -3.2356391081574615e-05f, 0.0005895175306304627f}
};

static const float ACCEL_O[3] = { -0.1329121010477303f, -0.047222673271787766f, 1.257425727446983f };

// Gyro bias in raw counts (constant here; can be learned slowly in future)
static const float GYRO_B[3] = { 153.461f, 69.446f, 992.782f };

static const float MOUNT_MATRIX[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};

static void calib_accel(const struct mpu6050_sample *raw, vec3f *acc_mps2) {
    float a0 = (float)raw->ax, a1 = (float)raw->ay, a2 = (float)raw->az;
    float x = ACCEL_C[0][0]*a0 + ACCEL_C[0][1]*a1 + ACCEL_C[0][2]*a2 + ACCEL_O[0];
    float y = ACCEL_C[1][0]*a0 + ACCEL_C[1][1]*a1 + ACCEL_C[1][2]*a2 + ACCEL_O[1];
    float z = ACCEL_C[2][0]*a0 + ACCEL_C[2][1]*a1 + ACCEL_C[2][2]*a2 + ACCEL_O[2];

    *acc_mps2 = v3(
        MOUNT_MATRIX[0]*x + MOUNT_MATRIX[1]*y + MOUNT_MATRIX[2]*z,
        MOUNT_MATRIX[3]*x + MOUNT_MATRIX[4]*y + MOUNT_MATRIX[5]*z,
        MOUNT_MATRIX[6]*x + MOUNT_MATRIX[7]*y + MOUNT_MATRIX[8]*z
    );
}

static void calib_gyro(const struct mpu6050_sample *raw, vec3f *gyro_radps) {
    float g0 = (float)raw->gx - GYRO_B[0];
    float g1 = (float)raw->gy - GYRO_B[1];
    float g2 = (float)raw->gz - GYRO_B[2];
    float dps0 = g0 / GYRO_LSB_PER_DPS;
    float dps1 = g1 / GYRO_LSB_PER_DPS;
    float dps2 = g2 / GYRO_LSB_PER_DPS;
    float x = dps0 * DEG2RAD;
    float y = dps1 * DEG2RAD;
    float z = dps2 * DEG2RAD;

    /* Apply Mounting alignment */
    *gyro_radps = v3(
        MOUNT_MATRIX[0]*x + MOUNT_MATRIX[1]*y + MOUNT_MATRIX[2]*z,
        MOUNT_MATRIX[3]*x + MOUNT_MATRIX[4]*y + MOUNT_MATRIX[5]*z,
        MOUNT_MATRIX[6]*x + MOUNT_MATRIX[7]*y + MOUNT_MATRIX[8]*z
    );
}

// ------------------------- 15-state INS EKF -------------------------



static void ins15_init(ins15_t *S) {
    int i;
    memset(S, 0, sizeof(*S));
    S->q = q_identity();

    // Conservative initial covariance (tune)
    const float p0 = 25.0f;          // m
    const float v0 = 1.0f;          // m/s
    const float th0 = powf(10.0f*DEG2RAD, 2); // rad
    const float ba0 = 0.25f;         // m/s^2
    const float bg0 = powf(0.01f*DEG2RAD, 2);        // rad/s

    for (i=0;i<15;i++) S->P[i*15+i] = 1e-6f;   /* Small but non zero */
    for (i=0;i<3;i++) S->P[(0+i)*15 + (0+i)] = p0;
    for (i=0;i<3;i++) S->P[(3+i)*15 + (3+i)] = v0;
    for (i=0;i<3;i++) S->P[(6+i)*15 + (6+i)] = th0;
    for (i=0;i<3;i++) S->P[(9+i)*15 + (9+i)] = ba0;
    for (i=0;i<3;i++) S->P[(12+i)*15 + (12+i)] = bg0;
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
            for ( k=0;k<15;k++) s += A[r*15+k]*B[k*15+c];
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
    vec3f w_dt = v3_scale(w, dt);
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
   
    S->v = v3_add(S->v, v3_scale(a_enu, dt));

    /* Limit Velocity during outages */
    float v_norm = v3_norm(S->v);
    if( v_norm > MAX_VELOCITY ){
        S->v = v3_scale(S->v, MAX_VELOCITY/v_norm);
    }

     S->p = v3_add(S->p, v3_scale(S->v, dt));

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
    float A[9] = {0}; // -R * skew(f)
    
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
    const float sigma_a = 0.05f;     // m/s^2
    const float sigma_g = 0.001f;    // rad/s
    const float sigma_ba = 0.001f;   // m/s^2/sqrt(s)
    const float sigma_bg = 0.0001f;   // rad/s/sqrt(s)

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
    quatf dq = q_from_small_angle(dth);
    S->q = q_normalize(q_mul(dq, S->q));

    S->ba.x += dx[9];  S->ba.y += dx[10]; S->ba.z += dx[11];
    S->bg.x += dx[12]; S->bg.y += dx[13]; S->bg.z += dx[14];

    // Optional: Joseph-form covariance update would be better; we do standard KF update,
    // then reset: P = (I-KH)P(I-KH)^T + K R K^T (done in update), so no extra here.
}

static bool kf_update_3(ins15_t *S, const float H[3*15], const float z[3], const float h[3], const float Rdiag[3], float *out_NIS) {
    // S = H P H^T + R (3x3)
    float HP[3*15];
    int r, c, k, i, j;
    for (r=0;r<3;r++) {
        for ( c=0;c<15;c++) {
            float s=0;
            for ( k=0;k<15;k++) s += H[r*15+k]*S->P[k*15+c];
            HP[r*15+c]=s;
        }
    }
    float S33[9];
    for ( r=0;r<3;r++) for ( c=0;c<3;c++) {
        float s=0;
        for ( k=0;k<15;k++) s += HP[r*15+k]*H[c*15+k];
        if (r==c) s += Rdiag[r];
        S33[r*3+c]=s;
    }
    float Sinv[9];
    if (!mat3_inv(S33, Sinv)) return false;

    float nu[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };

    if (out_NIS) {
        float tmp[3]={0};
        for (i=0;i<3;i++) for ( j=0;j<3;j++) tmp[i] += nu[j]*Sinv[j*3+i];
        *out_NIS = nu[0]*tmp[0] + nu[1]*tmp[1] + nu[2]*tmp[2];
    }

    // K = P H^T Sinv  (15x3)
    float PHt[15*3];
    for ( r=0;r<15;r++) for (c=0;c<3;c++) {
        float s=0;
        for ( k=0;k<15;k++) s += S->P[r*15+k]*H[c*15+k];
        PHt[r*3+c]=s;
    }
    float K[15*3];
    for ( r=0;r<15;r++) for ( c=0;c<3;c++) {
        float s=0;
        for (k=0;k<3;k++) s += PHt[r*3+k]*Sinv[k*3+c];
        K[r*3+c]=s;
    }

    // dx = K nu
    float dx[15]={0};
    for ( r=0;r<15;r++) dx[r] = K[r*3+0]*nu[0] + K[r*3+1]*nu[1] + K[r*3+2]*nu[2];
    ins15_inject(S, dx);

    // Joseph covariance update: P = (I-KH)P(I-KH)^T + K R K^T
    float KH[15*15]; memset(KH,0,sizeof(KH));
    for ( r=0;r<15;r++) for ( c=0;c<15;c++) {
        float s=0;
        for ( k=0;k<3;k++) s += K[r*3+k]*H[k*15+c];
        KH[r*15+c]=s;
    }
    float I_KH[15*15]; memset(I_KH,0,sizeof(I_KH));
    for ( i=0;i<15;i++) I_KH[i*15+i]=1.0f;
    for ( i=0;i<15*15;i++) I_KH[i] -= KH[i];

    float tmp[15*15], I_KH_T[15*15], P1[15*15];
    mat15_mul(I_KH, S->P, tmp);
    mat15_T(I_KH, I_KH_T);
    mat15_mul(tmp, I_KH_T, P1);

    float KRKt[15*15]; memset(KRKt,0,sizeof(KRKt));
    for ( r=0;r<15;r++) {
        for ( c=0;c<15;c++) {
            float s=0;
            for ( k=0;k<3;k++) {
                float Rk = Rdiag[k];
                s += K[r*3+k] * Rk * K[c*3+k];
            }
            KRKt[r*15+c]=s;
        }
    }
    for ( i=0;i<15*15;i++) S->P[i] = P1[i] + KRKt[i];
    return true;
}

static bool ins15_update_gnss_pos(ins15_t *S, vec3f zpos, float Rpos, float *out_NIS) {
    float H[3*15]; memset(H,0,sizeof(H));
    int i;
    // z = p + n  => nu uses nominal p, and H maps error δp
    for ( i=0;i<3;i++) H[i*15 + (0+i)] = 1.0f;

    float z[3] = { zpos.x, zpos.y, zpos.z };
    float h[3] = { S->p.x, S->p.y, S->p.z };
    float Rdiag[3] = { Rpos, Rpos, Rpos };
    return kf_update_3(S, H, z, h, Rdiag, out_NIS);
}

static bool ins15_update_gnss_vel(ins15_t *S, vec3f zvel, const float Rvdiag[3], float *out_NIS) {
    float H[3*15]; memset(H,0,sizeof(H));
    int i;
    for ( i=0;i<3;i++) H[i*15 + (3+i)] = 1.0f; // δv
    float z[3] = { zvel.x, zvel.y, zvel.z };
    float h[3] = { S->v.x, S->v.y, S->v.z };
    return kf_update_3(S, H, z, h, Rvdiag, out_NIS);
}

static void ins15_update_zupt(ins15_t *S) {
    // 3 sequential scalar updates on v components with z=0
    int i;
    for ( i=0;i<3;i++) {
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

static void ins15_update_nhc(ins15_t *S) {
    // Measurement: [v_y^b, v_z^b] ~ 0
    float R[9]; R_from_q(S->q, R);
    int c;
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
    for ( c=0;c<3;c++) H[0*15 + (3+c)] = RT[3 + c];
    // row1 -> vb.z
    for ( c=0;c<3;c++) H[1*15 + (3+c)] = RT[6 + c];

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
    uint64_t gnss_fix_ns;   /*<< Monotonic timestamp from driver when fix parsed.*/
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

    // INS
    ins15_t ins;

    // ENU reference
    bool enu_ref_set;
    lla_t enu_ref_lla;
    ecef_t enu_ref_ecef;

    // nav state
    bool nav_ready;

    // GNSS outage tracking
    double last_gnss_s;
    bool gnss_valid;
    bool gnss_just_returned;
    int fade_in_left;
    double outage_s;
    float qscale;

    // ZUPT
    int zupt_count;
    float acc_variance;
    float gyro_variance;

    // NIS for logging/diagnostics
    float last_nis_pos;
    float last_nis_vel;

    // last computed world acceleration (ENU)
    vec3f last_aw;

    // logging
    FILE *logf;
    char log_filename[256];
    double last_log_s;

    // threading
    pthread_mutex_t mtx;
    pthread_cond_t cv;
    bool running;
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
        "gnss_fix,lat_deg,lon_deg,alt_m,speed_mps,heading_deg,hdop,"
        "gnss_valid,outage_s,qscale,nis_pos,nis_vel,acc_var,gyro_var,zupt_count\n"
    );
    fflush(C->logf);

    fprintf(stderr, "Logging to %s\n", C->log_filename);
    C->last_log_s = 0.0;
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
        if (rc < 0) {
            usleep(10000);
            continue;
        }

        pthread_mutex_lock(&C->mtx);

        C->have_gnss = false;
        C->gnss_have_fix = f.have_fix ? 1 : 0;

        if (C->gnss_have_fix) {
            double lat_deg = (double)f.lat_e7 / 1e7;
            double lon_deg = (double)f.lon_e7 / 1e7;
            double alt_m   = (double)f.alt_mm / 1000.0;

            C->gnss_lat_rad = lat_deg * (M_PI/180.0);
            C->gnss_lon_rad = lon_deg * (M_PI/180.0);
            C->gnss_alt_m   = alt_m;
            C->gnss_fix_ns = (uint64_t)f.monotonic_ns;
            C->gnss_speed_mps = (float)f.speed_mmps / 1000.0f;

#if HAVE_GNSS_HDOP
            // Expected fields if you extended your driver:
            //   f.hdop_valid (bool-like), f.hdop_x10 (uint16/uint8)
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
            //   f.heading_valid (bool-like), f.heading_deg_e5 (int32, deg*1e5)
            if (f.heading_valid && C->gnss_speed_mps > 0.5f) {
                float heading_deg = (float)f.course_deg_e5 / 1e5f;
                C->gnss_heading_rad = heading_deg * DEG2RAD;
                C->gnss_heading_valid = true;

                // derive ENU velocity (course over ground)
                float spd = C->gnss_speed_mps;
                float psi = C->gnss_heading_rad;
                C->gnss_vel_enu = v3(spd*sinf(psi), spd*cosf(psi), 0.0f);
                C->gnss_vel_valid = true;
            } else {
                C->gnss_heading_valid = false;
                C->gnss_vel_valid = false;
            }
#else
            C->gnss_heading_valid = false;
            C->gnss_vel_valid = false;
#endif

            C->have_gnss = true;

            // Save for logging
            C->gnss_last_lat_deg = lat_deg;
            C->gnss_last_lon_deg = lon_deg;
            C->gnss_last_alt_m = alt_m;
            C->gnss_last_speed_mps = C->gnss_speed_mps;
            C->gnss_last_fix = 1;
            C->gnss_last_heading_deg = C->gnss_heading_valid ? (C->gnss_heading_rad * RAD2DEG) : 0.0f;
            C->gnss_last_hdop = C->gnss_hdop_valid ? C->gnss_hdop : -1.0f;
            C->gnss_last_valid = true;

            // Set ENU reference on first ever fix
            if (!C->enu_ref_set) {
                C->enu_ref_lla = (lla_t){C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m};
                C->enu_ref_ecef = lla2ecef(C->enu_ref_lla);
                C->enu_ref_set = true;
                fprintf(stderr, "ENU origin set from first GNSS fix.\n");
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
    ctx_t *C = (ctx_t*)arg;
    uint64_t last_tick_ns = monotonic_ns();
    uint64_t imu_count = 0;

    while (__atomic_load_n(&C->running, __ATOMIC_ACQUIRE)) {
        pthread_mutex_lock(&C->mtx);

        // Wait for IMU sample (or timeout)
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 10*1000*1000;   /**< 10ms timeout  */
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        (void)pthread_cond_timedwait(&C->cv, &C->mtx, &ts);

        uint64_t now_ns = monotonic_ns();
        float dt = (float)((now_ns - last_tick_ns) * 1e-9);
        if (dt <= 0.0f || dt > 1.0f) dt = DT_IMU_DEFAULT;
        last_tick_ns = now_ns;
        imu_count++;

        if (!C->have_imu) {
            pthread_mutex_unlock(&C->mtx);
            continue;
        }

        // Calibrate IMU
        vec3f acc_b, gyro_b;
        calib_accel(&C->imu_raw, &acc_b);
        calib_gyro(&C->imu_raw, &gyro_b);

        float acc_norm = v3_norm(acc_b);

        /* Update Variance for ZUPT detection */
        C->acc_variance = 0.95f * C->acc_variance + 0.05f * powf(acc_norm - GRAVITY, 2);
        float gyro_norm = v3_norm(gyro_b);
        C->gyro_variance = 0.95f * C->gyro_variance + 0.05f * powf(gyro_norm, 2);

        // Initialize nav on first GNSS fix (wait until ENU origin set AND have current GNSS fix)
        if (!C->nav_ready) {
            if (C->enu_ref_set && C->have_gnss && C->gnss_have_fix) {
                lla_t L = { C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m };
                ecef_t E = lla2ecef(L);
                double enu_d[3];
                ecef2enu(E, C->enu_ref_lla, C->enu_ref_ecef, enu_d);

                ins15_init(&C->ins);
                C->ins.p = v3((float)enu_d[0], (float)enu_d[1], (float)enu_d[2]);
                C->ins.v = v3(0,0,0);
                //Initialize attitude from GNSS heading if available
                if(C->gnss_heading_valid) {
                    C->ins.q = q_from_yaw(C->gnss_heading_rad);
                } else {
                    C->ins.q = q_identity();  /*<< Assume Level and north facing */
                }

                // Start biases at 0 (since you already calibrated raw -> SI).
                C->ins.ba = v3(0,0,0);
                C->ins.bg = v3(0,0,0);

                C->nav_ready = true;
                C->gnss_valid = true;
                C->gnss_just_returned = false;
                C->fade_in_left = 0;
                C->outage_s = 0.0;
                C->qscale = 1.0f;
                C->last_gnss_s = now_sec();
                C->last_nis_pos = 0.0f;
                C->last_nis_vel = 0.0f;

                fprintf(stderr, "Nav initialized. ENU p=(%.2f,%.2f,%.2f)\n",
                        C->ins.p.x, C->ins.p.y, C->ins.p.z);

                C->have_gnss = false; // consume
            }

            pthread_mutex_unlock(&C->mtx);
            continue;
        }

        // GNSS outage tracking
        double tnow = now_sec();
        if (!C->gnss_valid || (tnow - C->last_gnss_s) > GNSS_TIMEOUT_S) {
            if (C->gnss_valid) {
                C->gnss_valid = false;
                C->gnss_just_returned = false;
            }
            C->outage_s += dt;
        } else {
            C->outage_s = 0.0;
        }
        C->qscale = C->gnss_valid ? 1.0f : compute_qscale(C->outage_s);

        // Predict INS
        ins15_predict(&C->ins, acc_b, gyro_b, dt, C->qscale, &C->last_aw);

        // Roll/Pitch stabilization using accelerometer when dynamics are low
        ins15_tilt_correction(&C->ins, acc_b, gyro_b, 0.02f);

        // Apply constraints every IMU step
        ins15_update_nhc(&C->ins);

        // ZUPT detection
        bool zupt_cond =
            (C->acc_variance < 0.1f) &&
            (C->gyro_variance < 0.01f) &&
            (fabsf(acc_norm - GRAVITY) < ZUPT_ACC_THR) &&
            (fabsf(gyro_b.x) < ZUPT_GYRO_THR) &&
            (fabsf(gyro_b.y) < ZUPT_GYRO_THR) &&
            (fabsf(gyro_b.z) < ZUPT_GYRO_THR);

        if (zupt_cond) C->zupt_count++;
        else C->zupt_count = 0;

        if (C->zupt_count >= ZUPT_COUNT_REQUIRED) {
            ins15_update_zupt(&C->ins);
            C->zupt_count = 0;
        }

        // Velocity decay when *almost* stopped (helps fight tiny residuals)
        float vnorm = v3_norm(C->ins.v);
        if (vnorm < VEL_EPS) {
            C->ins.v = v3_scale(C->ins.v, VEL_DECAY);
        }

        // GNSS fusion (pos + optional vel + optional yaw complementary)
        if (C->have_gnss && C->gnss_have_fix) {
            lla_t L = { C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m };
            ecef_t E = lla2ecef(L);
            double enu_d[3];
            ecef2enu(E, C->enu_ref_lla, C->enu_ref_ecef, enu_d);
            vec3f zpos = v3((float)enu_d[0], (float)enu_d[1], (float)enu_d[2]);

            /**
             * Time align  GNSSS to "now" using speed/course and monotonic timestamps (no replay buffer)
             * If GNSS fix was parsed earlier than now, forward-extrapolate the position by dt_lat
             */
            uint64_t now_ns = monotonic_ns();
            if(C->gnss_fix_ns != 0 && now_ns > C->gnss_fix_ns && C->gnss_vel_valid ){
                float dt_lat = (float)((double) (now_ns - C->gnss_fix_ns) * 1e-9);
                dt_lat = clampf(dt_lat, 0.0f, 1.5f);
                zpos.x += C->gnss_vel_enu.x * dt_lat;
                zpos.y += C->gnss_vel_enu.y * dt_lat;
                // Inflate Rpos with latency induced uncertainity
                float lat_sig = C->gnss_speed_mps *dt_lat * 0.7f /*<< 0.7 typical along track uncertainity */
                Rpos += lat_sig * lat_sig;
            }

            float hdop = (C->gnss_hdop_valid ? C->gnss_hdop : 1.0f);
            float Rpos = R_GNSS_POS_VAR * hdop * hdop;

            if (!C->gnss_valid) {
                C->gnss_just_returned = true;
                C->fade_in_left = FADE_IN_STEPS;
            }
            if (C->gnss_just_returned) Rpos *= FADE_IN_FACT_INIT * (1.0f - (float)C->fade_in_left / FADE_IN_STEPS );

            float nis_pos = 0.0f;
            bool ok_pos = ins15_update_gnss_pos(&C->ins, zpos, Rpos, &nis_pos);
            bool gate_pos = ok_pos && (nis_pos < CHI2_3DOF_GATE);
            C->last_nis_pos = nis_pos;

            if (gate_pos) {
                C->last_gnss_s = tnow;
                C->gnss_valid = true;
                C->outage_s = 0.0;
                C->qscale = 1.0f;

                if (imu_count % 100 == 0 ){
                    fprintf(stderr, "GNSS: NIS=%.2f, pos=(%.1f, %.1f, %.1f), hdop=%.1f\n", nis_pos, zpos.x, zpos.y, zpos.z, hdop);
                }
            } else {
                // reject -> do not mark gnss_valid true
            }

            // Velocity update if valid
            if (C->gnss_vel_valid) {
                float Rv[3] = { RV_VEL_E, RV_VEL_N, RV_VEL_U };
                float nis_vel = 0.0f;
                bool ok_vel = ins15_update_gnss_vel(&C->ins, C->gnss_vel_enu, Rv, &nis_vel);
                //bool gate_vel = ok_vel && (nis_vel < CHI2_3DOF_GATE);
                C->last_nis_vel = nis_vel;
                (void)ok_vel;
            } else {
                C->last_nis_vel = 0.0f;
            }

            // Optional yaw complementary correction using GNSS course (if available)
            if (C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN && hdop <= YAW_FUSION_HDOP_MAX) {
                float yaw_est = yaw_from_q(C->ins.q);
                float yaw_gnss = C->gnss_heading_rad;
                float dpsi = wrap_pi(yaw_gnss - yaw_est);
                float dpsi_corr = YAW_FUSION_ALPHA * dpsi;
                C->ins.q = q_normalize(q_mul(C->ins.q, q_from_yaw_delta(dpsi_corr)));
            }

            if (C->gnss_just_returned && gate_pos) {
                C->fade_in_left--;
                if (C->fade_in_left <= 0) C->gnss_just_returned = false;
            }

            if (gate_pos) {
                fprintf(stderr, "GNSS pos upd OK: NIS=%.2f Rpos=%.2f hdop=%.2f\n", nis_pos, Rpos, hdop);
            } else {
                fprintf(stderr, "GNSS pos upd REJ: NIS=%.2f Rpos=%.2f hdop=%.2f\n", nis_pos, Rpos, hdop);
            }

            C->have_gnss = false; // consumed
        } else {
            // Dead-reckoning status prints at ~2 Hz
            if (imu_count % 50 == 0) {
                fprintf(stderr, "DR: outage=%.1fs qS=%.1f p=(%.1f,%.1f,%.1f) v=(%.2f,%.2f,%.2f)\n",
                        C->outage_s, C->qscale,
                        C->ins.p.x, C->ins.p.y, C->ins.p.z,
                        C->ins.v.x, C->ins.v.y, C->ins.v.z);
            }
        }

        // Output ENU only (after nav is ready)
        printf("%.3f,%.3f,%.3f\n", C->ins.p.x, C->ins.p.y, C->ins.p.z);
        fflush(stdout);

        // 1 Hz CSV logging
        double tlog = now_sec();
        if (tlog - C->last_log_s >= 1.0) {
            if (C->logf) {
                float yaw_deg = yaw_from_q(C->ins.q) / DEG2RAD;
                fprintf(C->logf,
                    "%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%.2f,"
                    "%.6f,%.6f,%.6f,"
                    "%.6f,%.6f,%.6f,"
                    "%d,%d,%d,%d,%d,%d,"
                    "%d,%.7f,%.7f,%.2f,%.3f,%.2f,%.2f,"
                    "%d,%.3f,%.2f,%.2f,%.2f,%.6f,%.6f,%d\n",
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
                    C->gnss_last_heading_deg,
                    C->gnss_last_hdop,
                    C->gnss_valid ? 1 : 0,
                    C->outage_s, C->qscale,
                    C->last_nis_pos, C->last_nis_vel,
                    C->acc_variance, C->gyro_variance,
                    C->zupt_count

                );
                fflush(C->logf);
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
    C.gnss_just_returned = false;
    C.fade_in_left = 0;
    C.outage_s = 0.0;
    C.qscale = 1.0f;
    C.zupt_count = 0;
    C.acc_variance = 1.0f;
    C.gyro_variance = 1.0f;
    C.last_nis_pos = 0.0f;
    C.last_nis_vel = 0.0f;
    C.last_aw = v3(0,0,0);

    make_log_file(&C);

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

    return 0;
}
