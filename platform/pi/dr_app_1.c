/**
 * @file dr_app_1.c
 * @brief Dead Reckoning application source for ground vehicle using MPU6050 IMU and NEO-6M GNSS.
 * with a 15-state error-state Kalman Filter.
 * 
 * @details 
 * Algorithm (loop @ ~100ms):
 * 1) Load/open devices: /dev/mpu6050-* (IRQ FIFO or on-demand) and /dev/neo6m0
 * 2) Calibrate IMU when stationary (estimate gyro/accel biases)
 * 3) Wait for the first GNSS fix; set ENU origin form that LLA.
 * 4) Each 100ms: read one IMU sample, correct using calibration, run EKF prediction.
 * 5) When a fresh GNSS fix is available: read and run EKF update. 
 * 6) Emit CSV over stdout each loop; also appen 1Hz CSV to SD card file .
 * If GNSS has no fix, the predicted state is used for output/logging.
 * 
 * Build:
 * gcc -o dr_app_1 dr_app_1.c -lm
 * 
 * 
 */
#define GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "../core/dr_types.h"
#include "../core/dr_ekf.h"
#include "../core/dr_math.h"
#include "../core/dr_cal.h"

#include "mpu6050_ioctl.h"
#include "dr_app_config.h"
#include "neo6m_gnss_ioctl.h"

/*--------------------Timing Helpers------------------*/
/**
 * @brief Sleep for ms milliseconds (montonic).
 */
static void msleep(unsigned int ms)
{
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* -------------------------MPU6050 char-device --------------------------*/
/**
 * @brief Discover first /dev/mpu6050-* node and open it read-only.
 * @return fd >= 0 on success; negative errno on failure.
 */
static int open_mpu6050(void){
    glob_t g;
    if(glob("/dev/mpu6050-*", 0, NULL, &g) !=0 || ( g.gl_pathc == 0)){
        globfree(&g);
        return -ENOENT;
    }
    int fd = open(g.gl_pathv[0], O_RDONLY | O_CLOEXEC);
    int err = (fd <0) ? -errno : 0;
    globfree(&g);
    return (fd < 0) ? err : fd;
}

/**
 * @brief Convert raw MPU6050 LSB to SI units using full-scale settings.
 * @param fs GET_FS result (accel g-range, gyro dps-range)
 * @param s raw sample
 * @param gyro_radps [out] gyro in rad/s
 * @param accel_mps2 [out] accel in m/s^2
 */
static void mpu6050_raw_to_si(const struct mpu6050_fs* fs, const struct mpu6050_sample* s,float gyro_radps[3],
    float accel_mps2[3])
{
    /* LSB per full scale: 32768 counts span +-FS */
    const float g_range = (fs->accel == ACCEL_2G) ? 2.0f :
                            (fs->accel == ACCEL_4G) ? 4.0f :
                            (fs->accel == ACCEL_8G) ? 8.0f : 16.0f;
    const float dps_range = (fs->gyro == GYRO_250DPS) ? 250.0f :
                            (fs->gyro == GYRO_500DPS) ? 500.0f :
                            (fs->gyro == GYRO_1000DPS) ? 1000.0f : 2000.0f;
    const float a_lsb_to_mps2 = (g_range * GRAVITY_CONST) / 32768.0f; // m/s^2 per LSB
    const float g_lsb_to_rad = ((dps_range * (float)M_PI / 180.0f) / 32768.0f); // rad/s per LSB

    accel_mps2[0] = s->ax * a_lsb_to_mps2;
    accel_mps2[1] = s->ay * a_lsb_to_mps2;
    accel_mps2[2] = s->az * a_lsb_to_mps2;

    gyro_radps[0] = s->gx * g_lsb_to_rad;
    gyro_radps[1] = s->gy * g_lsb_to_rad;
    gyro_radps[2] = s->gz * g_lsb_to_rad;

}

/**
 * @brief Read one synchronous snapshot of MPU6050 data.
 * @param fd MPU6050 char-device fd
 * @param out [out] filled mpu6050_sample struct 
 * @return 0 on success; negative errno on failure. 
 */
static int mpu6050_read_once(int fd, struct mpu6050_sample* out){
    ssize_t r = read(fd, out, sizeof(struct mpu6050_sample));
    if ( r == (ssize_t)sizeof(struct mpu6050_sample)) {
        return 0;
    }
    return (r <0) ? -errno : -EIO;
}

/* -------------------------NEO-6M GNSS char-device --------------------------*/
static int open_gnss(void ){
    int fd = open(NEO6M_DEVICE_PATH, O_RDONLY | O_CLOEXEC);
    return (fd <0) ? -errno : fd;
}

/**
 * @brief Read latest GNSS fix via ioctl (non-blocking best-effort).
 * @return 1 if have-fix, 0 if no-fix, negative errno on error.
 */
static int get_gnss_fix(int fd, int *have_fix, int *utcY, int *utcM, int *utcd,
    int *utcH, int *utcm, int *utcs, int *utcms, int64_t *tmono_ns, double *lat_deg,
    double *lon_deg, double *alt_m, float *spd_mps) {
        struct neo6m_gnss_fix f;
        if( ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, &f) != 0){
            return -errno;
        }

        *have_fix = f.have_fix ? 1 : 0;
        *tmono_ns = f.monotonic_ns;
        *utcY = f.utc_year;
        *utcM = f.utc_mon;
        *utcd = f.utc_day; 
        *utcH = f.utc_hour;
        *utcm = f.utc_min;
        *utcs = f.utc_sec;
        *utcms = f.utc_millis;
        *spd_mps = (float)f.speed_mmps / 1000.0f;
        if ( !*have_fix ) {
            return 0;
        }
        *lat_deg = f.lat_e7 / 1e7;
        *lon_deg = f.lon_e7 / 1e7;
        *alt_m = f.alt_mm / 1000.0;
        return 1;
    }

/*----------------------Geodesy (WGS84)----------------------*/
typedef struct { double lat_deg, lon_deg, alt_m; } geodetic_t;
typedef struct { double x, y, z; } ecef_t;

static const double WGS84_A = 6378137.0;          /**< WGS-84 Earth semimajor axis (m) */
static const double WGS84_F = 1.0 / 298.257223563; /**< WGS-84 Earth flattening */

#define WGS84_E2 (WGS84_F * (2.0 - WGS84_F)) /**< Square of eccentricity */

/**
 * @brief LLA to ECEF conversion.
 * 
 */
static ecef_t lla_ecef( geodetic_t lla ) {
    double lat = lla.lat_deg * (M_PI / 180.0);
    double lon = lla.lon_deg * (M_PI / 180.0);
    double s = sin(lat);
    double c = cos(lat);
    double sl = sin(lon);
    double cl = cos(lon);
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * s * s);
    return (ecef_t){
        .x = (N + lla.alt_m) * c * cl,
        .y = (N + lla.alt_m) * c * sl,
        .z = (N * (1.0 - WGS84_E2) + lla.alt_m) * s
    };
}

/**@brief Build ENU rotation (ECEF to ENU) at ref LLA */
static void ecef_to_enu_R ( geodetic_t ref, double R[9])
{
    double lat = ref.lat_deg * (M_PI / 180.0);
    double lon = ref.lon_deg * (M_PI / 180.0);
    double s_lat = sin(lat);
    double c_lat = cos(lat);
    double s_lon = sin(lon);
    double c_lon = cos(lon);

    R[0] = -s_lon;        R[1] = c_lon;         R[2] = 0.0;
    R[3] = -s_lat * c_lon; R[4] = -s_lat * s_lon; R[5] = c_lat;
    R[6] = c_lat * c_lon;  R[7] = c_lat * s_lon;  R[8] = s_lat;
}

/** @brief ECEF delta to ENU */
static void ecef_delta_to_enu(geodetic_t ref, ecef_t e, ecef_t e0, float enu[3]){
    double R[9]; 
    ecef_to_enu_R(ref, R);
    double dx = e.x - e0.x;
    double dy = e.y - e0.y;
    double dz = e.z - e0.z;
    enu[0] = (float)(R[0]*dx + R[1]*dy + R[2]*dz); // east
    enu[1] = (float)(R[3]*dx + R[4]*dy + R[5]*dz); // north
    enu[2] = (float)(R[6]*dx + R[7]*dy + R[8]*dz); // up
}

/** @brief ENU to LLA using ref and its ECEF */
static geodetic_t enu_to_lla(geodetic_t ref, ecef_t e0, const float enu[3]){
    double R[9];
    ecef_to_enu_R(ref, R);
    // transpose R for ENU to ECEF
    double dx = R[0]*enu[0] + R[3]*enu[1] + R[6]*enu[2];
    double dy = R[1]*enu[0] + R[4]*enu[1] + R[7]*enu[2];
    double dz = R[2]*enu[0] + R[5]*enu[1] + R[8]*enu[2];
    ecef_t e = { .x = e0.x + dx, .y = e0.y + dy, .z = e0.z + dz };

    /* ECEF to LLA (Bowring-ish)*/
    double a = WGS84_A, e2 = WGS84_E2, b = a * sqrt(1.0 - e2);
    double ep2 = (a*a - b*b) / (b*b);
    double p = sqrt(e.x*e.x + e.y*e.y);
    double th = atan2(a * e.z, b * p);
    double sth = sin(th), cth = cos(th);
    double lon = atan2(e.y, e.x);
    double lat = atan2(e.z + ep2 * b * sth * sth * sth,
                       p - e2 * a * cth * cth * cth);
    double s = sin(lat), N = a / sqrt(1.0 - e2 * s * s);
    geodetic_t lla = {
        .lat_deg = lat * (180.0 / M_PI),
        .lon_deg = lon * (180.0 / M_PI),
        .alt_m = p / cos(lat) - N
    };
    return lla;
}

/* ------------------------Moving Average (IMU)------------------------*/
typedef struct {
    int n, idx;
    float buf[IMU_AVG_WINDOW][6]; // gx,gy,gz,ax,ay,az

}imu_ma_t;

static void imu_ma_init(imu_ma_t* ma){
    memset(ma, 0, sizeof(imu_ma_t));
}

static void imu_ma_push(imu_ma_t* ma, const float g[3], const float a[3]){
    memcpy(ma->buf[ma->idx], g, 3*sizeof(float));
    memcpy(ma->buf[ma->idx] + 3, a, 3*sizeof(float));
    ma->idx = (ma->idx + 1) % IMU_AVG_WINDOW;
    if (ma->n < IMU_AVG_WINDOW) ma->n++;
}

static void imu_ma_get(const imu_ma_t* ma, float g[3], float a[3]){
    float s[6] = {0}; int k, i, N = ma->n ? ma->n : 1;
    for(i = 0; i< ma->n; i++) {
        for( k = 0; k < 6; k++) {
            s[k] += ma->buf[i][k];
        }
    }
    for( k = 0; k <3; k++) {
        g[k] = s[k] / N;
        a[k] = s[k + 3] / N;
    }
}

/* ---------------------- Calibration (stationary) ----------------------*/
/**
 * @brief Stationary bias estimate (gyro, accel)
 * @param fd MPU6050 char-device fd
 * @param fs full-scale config (affects raw-to-si conversion)
 * @param bg [out] gyro bias (rad/s)
 * @param ba [out] accel bias (m/s^2)
 * @return 0 on success; negative errno on failure.
 */
static int calibrate_stationary(int fd, const struct mpu6050_fs* fs, float bg[3], float ba[3]){
    const int N = CAL_SAMPLES;
    double sumg[3] = {0}, suma[3] = {0};
    struct mpu6050_sample s;
    int i, ret;

    for( i = 0; i < N; i++) {
        ret = mpu6050_read_once(fd, &s);
        if ( ret != 0 ) {
            return ret;
        }
        float g[3], a[3];
        mpu6050_raw_to_si(fs, &s, g, a);
        sumg[0] += g[0]; sumg[1] += g[1]; sumg[2] += g[2];
        suma[0] += a[0]; suma[1] += a[1]; suma[2] += a[2];
        msleep(10); /* ~100Hz independent of the loop */
    }
    /* Average */
    bg[0] = (float)(sumg[0] / N);
    bg[1] = (float)(sumg[1] / N);
    bg[2] = (float)(sumg[2] / N);
    ba[0] = (float)(suma[0] / N);
    ba[1] = (float)(suma[1] / N);
    ba[2] = (float)(suma[2] / N);
    return 0;
}

/*-----------------------Yaw/Pitch/Roll-----------------------*/
/**
 * @brief Convert quaternion to ZYX Euler angles (yaw, pitch, roll).
 */
static void quat_to_euler_deg(dr_quatf_t q, float* yaw_deg, float* pitch_deg, float* roll_deg){
    float R[9]; 
    dr_R_from_q(q, R);
    *roll_deg = (float)atan2f(R[5], R[8]) * (180.0f / (float)M_PI);
    *pitch_deg = (float)asinf(-R[2]) * (180.0f / (float)M_PI);
    *yaw_deg = (float)atan2f(R[1], R[0]) * (180.0f / (float)M_PI);
}

/* ----------------------- Main Application -----------------------*/
int main (void) {
    /* 1) Open devices */
    int fd_imu = open_mpu6050();
    if (fd_imu < 0) {
        fprintf(stderr, "Failed to open MPU6050: %s\n", strerror(-fd_imu));
        return 1;
    }
    int fd_gnss = open_gnss();
    if (fd_gnss < 0) {
        fprintf(stderr, "Failed to open NEO-6M GNSS: %s\n", strerror(-fd_gnss));
        close(fd_imu);
        return 1;
    }

    /* Get full scale (for raw->SI) - we don't modify it here */
    struct mpu6050_fs fs;
    memset(&fs, 0, sizeof(fs));
    if (ioctl(fd_imu, MPU6050_IOC_GET_FS, &fs) != 0) {
        fprintf(stderr, "Failed to get MPU6050 full-scale: %s\n", strerror(errno));
        close(fd_imu);
        close(fd_gnss);
        return 1;
    }

    /* 2) Calibrate IMU when stationary */
    float ba[3], bg[3];
    memset(ba, 0, 3*sizeof(float));
    memset(bg, 0, 3*sizeof(float));
    printf("Calibrating IMU (stationary) ...\n");
    if ( calibrate_stationary(fd_imu, &fs, bg, ba) != 0 ) {
        fprintf(stderr, "Failed to calibrate MPU6050: %s\n", strerror(errno));
        close(fd_imu);
        close(fd_gnss);
        return 1;
    }

    printf("Calibration complete.\n");
    /* 3) Wait for first GNSS fix (ENU origin) */
    geodetic_t ref;
    ecef_t ref_ecef;
    memset(&ref, 0, sizeof(ref));
    memset(&ref_ecef, 0, sizeof(ref_ecef));
    fprintf(stderr, "Waiting for first GNSS fix...\n");
    while (1) {
        int have_fix = 0;
        int Y, M, d, H, m, s, ms;
        int64_t tns;
        double lat, lon, alt;
        float spd;
        int ret = get_gnss_fix(fd_gnss, &have_fix, &Y, &M, &d, &H, &m, &s, &ms, &tns,
            &lat, &lon, &alt, &spd);
        if ( ret < 0 ) {
            fprintf(stderr, "Failed to get GNSS fix: %s\n", strerror(-ret));
            close(fd_imu);
            close(fd_gnss);
            return 1;
        }
        if (ret > 0 && have_fix) {
            ref = (geodetic_t){ .lat_deg = lat, .lon_deg = lon, .alt_m = alt };
            ref_ecef = lla_ecef(ref);
            fprintf(stderr, "Got first GNSS fix: lat=%.7f deg, lon=%.7f deg, alt=%.3f m\n",
                lat, lon, alt);
            break;
        }
        msleep(500);
    }
    /* 4) Init EKF config/state */
    dr_ekf_t ekf;
    dr_ekf_config_t cfg = {
        .gravity = {0.0f, 0.0f, -GRAVITY_CONST},
        .sigma_g = SIGMA_G_RADPS,
        .sigma_a = SIGMA_A_MPS2,
        .sigma_bg = SIGMA_BG_RADPS,
        .sigma_ba = SIGMA_BA_MPS2,
        .p0_pos = P0_POS_STD_M,
        .p0_vel = P0_VEL_STD_MPS,
        .p0_ang = P0_ANG_STD_RAD,
        .p0_ba = P0_BA_STD_MPS2,
        .p0_bg = P0_BG_STD_RADPS
    };
    dr_nominal_state_t x0;
    memset(&x0, 0, sizeof(x0));
    memcpy(x0.ba, ba, sizeof(ba));
    memcpy(x0.bg, bg, sizeof(bg));
    dr_ekf_init(&ekf, &cfg, &x0);
    printf("EKF initialized.\n");
    ekf.cfg = cfg;  /* ensure config is live in ekf */

    /* Moving Average */
    imu_ma_t imu_ma;
    imu_ma_init(&imu_ma);

    /* CSV to stdout (USB) */
    printf(DR_CSV_HEADER);
    fflush(stdout);

    /* SD Card CSV once per second */
    const char *sd_csv = "/home/sijeo/dr_vehicle_log.csv";
    FILE* f_sd = fopen(sd_csv, "a");
    if ( f_sd == NULL ) {
        fprintf(stderr, "Failed to open SD card log file %s: %s\n", sd_csv, strerror(errno));
    }

    /* NEW: GNSS raw log */
    const char *gnss_csv = "/home/sijeo/gnss_log.csv";
    FILE* f_gnss = fopen(gnss_csv, "a");
    if ( f_gnss == NULL ) {
        fprintf(stderr, "Failed to open GNSS log file %s: %s\n", gnss_csv, strerror(errno));
    }
    else {fprintf(f_gnss, "utc,mono_ns,lat_deg,lon_deg,alt_m,spd_mps,have_fix\n"); fflush(f_gnss); }
    time_t last_sd_time = 0;

    /* Main loop @ 100 ms*/
    const float dt = 1.0f / IMU_LOOP_HZ;
    for(;;) {
        struct mpu6050_sample sample;
        if ( mpu6050_read_once(fd_imu, &sample) != 0 ) {
            fprintf(stderr, "Failed to read MPU6050 sample: %s\n", strerror(errno));
            break;
        }
        float g[3], a[3];
        mpu6050_raw_to_si(&fs, &sample, g, a);
        imu_ma_push(&imu_ma, g, a);
        float g_avg[3], a_avg[3];
        imu_ma_get(&imu_ma, g_avg, a_avg);

        dr_imu_sample_t imu_sample = {
            .gyro = { g_avg[0] - bg[0], g_avg[1] - bg[1], g_avg[2] - bg[2] },
            .accel = { a_avg[0] - ba[0], a_avg[1] - ba[1], a_avg[2] - ba[2] }
        };

        /* Predict */
        dr_ekf_predict(&ekf, &imu_sample, dt);

        /* GNSS update (best effort)*/
        int have_fix = 0; int Y, M, d, H, m, s, ms; int64_t tns; double lat = 0, lon = 0, alt = 0; float spd = 0;
        int ret = get_gnss_fix(fd_gnss, &have_fix, &Y, &M, &d, &H, &m, &s, &ms, &tns,
            &lat, &lon, &alt, &spd);
        if (fd_gnss)
        {
            char utc[40] = "0000-00-00T00:00:00.000Z";
            if (have_fix) {
                snprintf(utc, sizeof(utc), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                    Y, M, d, H, m, s, ms);
            }
            fprintf(f_gnss, "%s,%lld,%.8f,%.8f,%.3f,%.3f,%d\n",
                utc, (long long)tns, lat, lon, alt, spd, have_fix);
            fflush(f_gnss);
        }

        if ( ret > 0 && have_fix ){
            ecef_t e = lla_ecef( (geodetic_t){ .lat_deg = lat, .lon_deg = lon, .alt_m = alt } );
            dr_gps_pos_t z;
            ecef_delta_to_enu(ref, e, ref_ecef, z.pos);
            (void)dr_ekf_update_gps_pos(&ekf, &z, (float[9]){
                GPS_POS_VAR_M2, 0.0f, 0.0f,
                0.0f, GPS_POS_VAR_M2, 0.0f,
                0.0f, 0.0f, GPS_POS_VAR_M2
            });
        }

        /* Output: Current nominal state (predicted if no fix)*/
        float yaw_deg, pitch_deg, roll_deg;
        quat_to_euler_deg(ekf.x.q, &yaw_deg, &pitch_deg, &roll_deg);
        geodetic_t lla = enu_to_lla(ref, ref_ecef, ekf.x.p);
        char utc[40] = "0000-00-00T00:00:00.000Z";
        if( ret > 0 ) {
            snprintf(utc, sizeof(utc), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                Y, M, d, H, m, s, ms);
        }
        printf(DR_CSV_FORMAT,
            utc,
            (long long)tns,
            lla.lat_deg,
            lla.lon_deg,
            lla.alt_m,
            ekf.x.p[0],
            ekf.x.p[1],
            ekf.x.p[2],
            ekf.x.v[0],
            ekf.x.v[1],
            ekf.x.v[2],
            yaw_deg,
            pitch_deg,
            roll_deg
        );
        fflush(stdout);

        /* SD Card logging */
        time_t now = time(NULL);
        if ( f_sd != NULL && now != last_sd_time ) {
            fprintf(f_sd, "%ld,%.f,%.8f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",(long)now,
                lla.lat_deg,
                lla.lon_deg,
                lla.alt_m,
                ekf.x.v[0],
                ekf.x.v[1],
                ekf.x.v[2],
                yaw_deg,
                pitch_deg,
                roll_deg
            );
            fflush(f_sd);
        }

        msleep((unsigned)(1000.0f / IMU_LOOP_HZ + 0.5f)); /* ~100ms */

    }
    if(f_sd != NULL) {
        fclose(f_sd);
    }
    if(f_gnss != NULL) {
        fclose(f_gnss);
    }
    close(fd_imu);
    close(fd_gnss);
    return 0;

}