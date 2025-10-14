/**
 * @file dr_app.c
 * @brief Dead-Reckoning application for ground vehicles (MPU6050 + NEO-6M + EKF)
 * 
 * @details
 * Pipeline:
 * 1. Discover MPU6050 IIO device and NEO-6M char device. 
 * 2. Calibrate IMU when stationary: Estimate gyro bias and accel bias so that a_b = [0,0,g].
 * 3. Block until first GNSS fix; use as ENU reference (lat0, lon0, alt0) and seed EKF.
 * 4. Loop at configured rate (IMU_LOOP_HZ): read IMU -> moving average -> EKF predict.
 * 5. When a fresh GNSS fix arrives: convert LLA -> ECEF -> ENU, EKF update with position.
 * 6. Emit CSV: UTC, t_mono_ns, lat, lon, h, ENU(x,y,z), vel( x, y, z), yaw, pitch, roll.\
 * 
 * Build:
 * gcc -O2 -Wall -Wextra -o dr_app dr_app.c -I./core -I./platform/pi -lm
 */

#define GNU_SOURCE
#include <errno.h>
#include <fcntl.h> // for O_CLOEXEC
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
#include <dirent.h>
#include <sys/stat.h>

#include "../../core/dr_types.h"
#include "../../core/dr_ekf.h"
#include "../../core/dr_math.h"
#include "../../core/dr_cal.h"

#include "dr_app_config.h"
#include "neo6m_gnss_ioctl.h"
#include <math.h>

/*----------------Small Helpers ------------------ */

/**
 * @brief Sleep for specified milliseconds.
 */
static void msleep(unsigned ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

/**
 * @brief Read a single integer from a sysfs file. (e.g. IIO *_raw)
 * 
 * @return 0 on success and errno on failuare.
 */
static int read_int_from_file(const char *path, int *out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -errno;
    }
    int v = 0;
    int n = fscanf(f, "%d", &v);
    fclose(f);
    if (n != 1) {
        return -EINVAL;
    }
    *out = v;
    return 0;
}

/**
 * @brief Read a double from a sysfs file (e.g. IIO scale)
 */
static int read_double_from_file(const char *path, double *out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -errno;
    }
    double v = 0;
    int n = fscanf(f, "%lf", &v);
    fclose(f);
    if (n != 1) {
        return -EINVAL;
    }
    *out = v;
    return 0;
}

/**
 * @brief Find the IIO device directory for the MPU6050 (by name)
 * @param out_path Buffer to receive director path (e.g. /sys/bus/iio/devices/iio:deviceX)
 * @param cap size of out_path buffer
 * @return 0 on success and -errno on failure.
 */
static int find_mpu6050_iio(char *out_path, size_t cap)
{
    const char *base = "/sys/bus/iio/devices";
    DIR *d = opendir(base);
    if (!d) {
        return -errno;
    }
    struct dirent *e;
    int ret = -ENOENT;
    while ((e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, "iio:device", 10) == 0) {
            char p[512];
            snprintf(p, sizeof(p), "%s/%s/name", base, e->d_name);
            FILE *f = fopen(p, "r");
            if (!f) continue;
            char name[128] = {0};
            if (fgets(name, sizeof(name), f) != NULL) {
                // Remove newline
                name[strcspn(name, "\n")] = 0;
                if (strcmp(name, MPU6050_IIO_NAME) == 0) {
                    // Found it
                    snprintf(out_path, cap, "%s/%s", base, e->d_name);
                    ret = 0;
                    fclose(f);
                    break;
                }
            }
            fclose(f);
        }
    }
    closedir(d);
    return ret;
}

/**
 * @brief Read one IMU sample (raw->SI) from IIO sysfs
 * @param devpath IIO device directory
 * @param[out] accel_mps2 body specific force (m/s^2)
 * @param[out] gyro_radps body angular rate (rad/s)
 * @return 0 on success and -errno on failure.
 */
static int read_imu_once(const char *devpath, float gyro_radps[3], float accel_mps2[3])
{
    char path[512];
    int raw;
    double scale;
    int i;

    /* Accelerometer X Y Z*/
    const char *acc_nodes[3] = {"in_accel_x", "in_accel_y", "in_accel_z"};
    for (i = 0; i < 3; i++) {
        snprintf(path, sizeof(path), "%s/%s_raw", devpath, acc_nodes[i]);
        if (read_int_from_file(path, &raw))
        {
            return -EIO;
        }
        snprintf(path, sizeof(path), "%s/%s_scale", devpath, acc_nodes[i]);
        if (read_double_from_file(path, &scale))
        {
            return -EIO;
        }
        accel_mps2[i] = (float)(raw * scale);
    }

    /* Gyro X Y Z*/
    const char *gyro_nodes[3] = {"in_gyro_x", "in_gyro_y", "in_gyro_z"};
    for (i = 0; i < 3; i++) {
        snprintf(path, sizeof(path), "%s/%s_raw", devpath, gyro_nodes[i]);
        if (read_int_from_file(path, &raw))
        {
            return -EIO;
        }
        snprintf(path, sizeof(path), "%s/%s_scale", devpath, gyro_nodes[i]);
        if (read_double_from_file(path, &scale))
        {
            return -EIO;
        }
        gyro_radps[i] = (float)(raw * scale);
    }

    return 0;

}

/*---------------Geodesy (WGS-84)-------------------------*/
typedef struct {
    double lat_deg, lon_deg, alt_m;
} geodetic_t;

typedef struct {
    double x, y, z;
} ecef_t;

static const double WGS84_A = 6378137.0;          // Semi-major axis
static const double WGS84_F = 1.0 / 298.257223563; // Flattening
#define WGS84_E2 (2.0 * WGS84_F - WGS84_F * WGS84_F) // Eccentricity squared

/**
 * @brief Convert geodetic (lat, lon, alt) LLA to ECEF (x, y, z)
 */
static ecef_t lla_to_ecef(geodetic_t lla)
{
    double lat_rad = lla.lat_deg * M_PI / 180.0;
    double lon_rad = lla.lon_deg * M_PI / 180.0;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);

    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat); // Prime vertical radius of curvature

    ecef_t ecef;
    ecef.x = (N + lla.alt_m) * cos_lat * cos_lon;
    ecef.y = (N + lla.alt_m) * cos_lat * sin_lon;
    ecef.z = (N * (1 - WGS84_E2) + lla.alt_m) * sin_lat;

    return ecef;
}

/**
 * @brief Convert ECEF (x, y, z) to geodetic LLA (Bowring's method)
 * 
 */
static geodetic_t ecef_to_lla(ecef_t ecef)
{
    double a = WGS84_A;
    double e2 = WGS84_E2;
    double b = a * sqrt(1 - e2);
    double ep2 = (a * a - b * b) / (b * b);
    double p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    double th = atan2(a * ecef.z, b * p);
    double sth = sin(th);
    double cth = cos(th);
    
    geodetic_t lla;
    lla.lon_deg = atan2(ecef.y, ecef.x) * 180.0 / M_PI;
    double num = ecef.z + ep2 * b * sth * sth * sth;
    double den = p - e2 * a * cth * cth * cth;
    double lat_rad = atan2(num, den);
    double s = sin(lat_rad);
    double N = a / sqrt(1 - e2 * s * s);
    lla.lat_deg = lat_rad * 180.0 / M_PI;
    lla.alt_m = p / cos(lat_rad) - N;
    return lla;
}

/**
 * @brief Build ENU (East-North-Up) rotation matrix from reference LLA
 * @param R 3x3 row-major, maps ECEF delta to ENU
 * 
 */
static void ecef_to_enu_matrix(geodetic_t ref, double R[9])
{
    double lat_rad = ref.lat_deg * M_PI / 180.0;
    double lon_rad = ref.lon_deg * M_PI / 180.0;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);

    // Row-major
    R[0] = -sin_lon;        R[1] = cos_lon;        R[2] = 0.0;
    R[3] = -sin_lat * cos_lon; R[4] = -sin_lat * sin_lon; R[5] = cos_lat;
    R[6] = cos_lat * cos_lon; R[7] = cos_lat * sin_lon; R[8] = sin_lat;
}

/**
 * @brief ECEF delta -> ENU at ref
 */
static void ecef_delta_to_enu(geodetic_t ref, ecef_t e, const ecef_t e0, float enu[3])
{
    double R[9];
    ecef_to_enu_matrix(ref, R);
    double dx = e.x - e0.x;
    double dy = e.y - e0.y;
    double dz = e.z - e0.z;
    enu[0] = (float)(R[0] * dx + R[1] * dy + R[2] * dz); // East
    enu[1] = (float)(R[3] * dx + R[4] * dy + R[5] * dz); // North
    enu[2] = (float)(R[6] * dx + R[7] * dy + R[8] * dz); // Up
}

/**
 * @brief ENU->ECEF (delta), then back to geodetic using ref
 */
static geodetic_t enu_to_lla(geodetic_t ref, const ecef_t e0, const float enu[3])
{
    double R[9];
    ecef_to_enu_matrix(ref, R);
    /* inv(R) = R^T for rotation */
    double dx = R[0] * enu[0] + R[3] * enu[1] + R[6] * enu[2];
    double dy = R[1] * enu[0] + R[4] * enu[1] + R[7] * enu[2];
    double dz = R[2] * enu[0] + R[5] * enu[1] + R[8] * enu[2];
    ecef_t e = (ecef_t){e0.x + dx, e0.y + dy, e0.z + dz};
    return ecef_to_lla(e);
}

/* -------------------GNSS access --------------------*/

/**
 * @brief Open the NEO-6M chardev.
 */
static int open_gnss(void)
{
    int fd = open(NEO6M_GNSS_CHARDEV_NAME, O_RDONLY | O_CLOEXEC);
    return (fd < 0) ? -errno : fd;
}

/**
 * @brief Non-blocking (best-effort) read of latest GNSS fix via ioctl.
 * @return 1 if we have a valid fix and LLA, 0 if no fix yet, <0 on error.
 */
static int get_gnss_fix(int fd, geodetic_t *lla, int *have_fix, int *utc_y, int *utc_m, int *utc_d, 
            int *utc_H, int *utc_M, int *utc_S, int *utc_ms, int64_t *tmono_ns, float *spd_mps)
{
    struct neo6m_gnss_fix f;
    if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, &f) != 0) {
        return -errno;
    }
    *have_fix = (f.have_fix) ? 1 : 0; // fix or no fix
    *tmono_ns = f.monotonic_ns;
    *utc_y = f.utc_year; *utc_m = f.utc_mon; *utc_d = f.utc_day;
    *utc_H = f.utc_hour; *utc_M = f.utc_min; *utc_S = f.utc_sec; *utc_ms = f.utc_millis;
    *spd_mps = (float)f.speed_mmps / 1000.0f;
    if( !*have_fix ) {
        return 0; // no fix yet
    }   
    /* lat/lon in deg*1e7, alt in mm -> meters */
    lla->lat_deg = f.lat_e7 / 1e7;
    lla->lon_deg = f.lon_e7 / 1e7;
    lla->alt_m = f.alt_mm / 1000.0;
       return 1; // have fix
}

/*-------------Simple Moving Average -----------------------*/
typedef struct {
    int n;
    int idx;
    float buf[IMU_AVG_WINDOW][6];
} imu_ma_t;

static void imu_ma_init(imu_ma_t *ma) {
    memset(ma, 0, sizeof(*ma));
}

static void imu_ma_push(imu_ma_t *ma, const float g[3], const float a[3])
{
    memcpy(ma->buf[ma->idx], g, 3 * sizeof(float));
    memcpy(ma->buf[ma->idx] + 3, a, 3 * sizeof(float));
    ma->idx = (ma->idx + 1) % IMU_AVG_WINDOW;
    if( ma->n < IMU_AVG_WINDOW )
    {
        ma->n++;
    }
}

static void imu_ma_get(const imu_ma_t *ma, float g_avg[3], float a_avg[3])
{
    float s[6] = {0};
    int i, k;
    int N = (ma->n > 0) ? ma->n : 1;
    for ( i = 0; i < ma->n; i++) {
        for ( k = 0; k < 6; k++ ) {
            s[k] += ma->buf[i][k];
        }
    }
    for ( k = 0; k < 3; k++ ) {
        g_avg[k] = s[k] / N;
        a_avg[k] = s[k + 3] / N;
    }
}

/*----------------------Calibration(Stationary)----------------------*/

/**
 * @brief Estimate gyro bias and accel bias when device is still on level ground.
 * @param devpath IIO device path
 * @param[out] bg gyro bias (rad/s), ba accel bias (m/s^2)
 * @return 0 on success and -errno on failure.
 */

 static int calibrate_imu_stationary(const char *devpath, float bg[3], float ba[3])
 {
    float g[3], a[3];
    double sum_g[3] = {0}, sum_a[3] = {0};
    int i, k;
    for(i=0; i < CAL_SAMPLES; ++i)
    {
        if (read_imu_once(devpath, g, a) != 0) {
            return -EIO;
        }
        for (k = 0; k < 3; k++) {
            sum_g[k] += g[k];
            sum_a[k] += a[k];
        }
        msleep(10); /* ~100Hz; independent of IMU_LOOP_HZ */

    }
    for (k = 0; k < 3; k++) {
        bg[k] = (float)(sum_g[k] / CAL_SAMPLES);
        ba[k] = (float)(sum_a[k] / CAL_SAMPLES);
    }
    return 0;
 }

 /*------------------Attitude Helpers --------------------*/
 static void quat_to_euler_deg(dr_quatf_t q, float *yaw, float *pitch, float *roll)
 {
    /* ZYX euler (yaw around +Z, pitch around +Y, roll around +X)*/
    float R[9]; dr_R_from_q(q, R);
    *roll = (float)(atan2f(R[5], R[8]) * 180.0 / M_PI);
    *pitch = (float)(asinf(-R[2]) * 180.0 / M_PI);
    *yaw = (float)(atan2f(R[1], R[0]) * 180.0 / M_PI);
 }

 /*----------Main-----------------*/
 int main(void)
 {
    float bg[3] = {0}, ba[3] = {0}; /* gyro bias (rad/s), accel bias (m/s^2) */
    geodetic_t ref_lla = (geodetic_t){0, 0, 0};
    ecef_t ref_ecef = (ecef_t){0, 0, 0};
    geodetic_t cur; 
    int have = 0, y,m,d,H,M,S,ms;
    int64_t tns = 0;
    float spd = 0.0f;
    int rc;
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
    /* GPS R (position) */
    float Rpos[9] = {GPS_POS_VAR_M2,0,0, 0,GPS_POS_VAR_M2,0, 0,0,GPS_POS_VAR_M2}; /* GPS position measurement noise */
    /* Main Loop @ IMU_LOOP_HZ */
    const float dt = 1.0f / IMU_LOOP_HZ;
    imu_ma_t imu_ma;;
    float g[3], a[3], g_f[3], a_f[3], g_cal[3], a_cal[3];
    dr_imu_sample_t imu = {.gyro = {g_cal[0], g_cal[1], g_cal[2]},
                           .accel = {a_cal[0], a_cal[1], a_cal[2]}};
    int k;
    

    /* 1) Discover Devices */
    char iio_path[512];
    if (find_mpu6050_iio(iio_path, sizeof(iio_path)) != 0) {
        fprintf(stderr, "MPU6050 IIO device not found. Ensure driver is loaded and device is connected.\n");
        return EXIT_FAILURE;
    }

    int fd_gnss = open_gnss();
    if (fd_gnss < 0) {
        fprintf(stderr, "Failed to open GNSS device %s: %s\n", NEO6M_GNSS_CHARDEV_NAME, strerror(-fd_gnss));
        return EXIT_FAILURE;
    }

    printf("Found MPU6050 IIO at %s\n", iio_path);
    /* 2) Calibrate IMU (Stationary ) */
    if( calibrate_imu_stationary(iio_path, bg, ba) ) {
        fprintf(stderr, "IMU calibration failed.\n");
        return 1;
    }

    /* 3) Wait for first GNSS fix to define ENU reference */
    fprintf(stderr, "Waiting for first GNSS fix...\n");
    for(;;){
        rc = get_gnss_fix(fd_gnss, &cur, &have, &y,&m,&d,&H,&M,&S,&ms, &tns, &spd);
        if( rc < 0 ) {
            fprintf(stderr, "GNSS read error: %s\n", strerror(-rc));
            return EXIT_FAILURE;
        }
        if( (rc > 0) && (have )) {
            ref_lla = cur;
            ref_ecef = lla_to_ecef(ref_lla);
            fprintf(stderr, "Got GNSS fix: lat=%.7f lon=%.7f alt=%.2f m\n", ref_lla.lat_deg, ref_lla.lon_deg, ref_lla.alt_m);
            break;
        }
        msleep(200);
    }
    fprintf(stderr, "Fix: lat=%.7f lon=%.7f alt=%.2f m -> ENU Origin Set \n", ref_lla.lat_deg, ref_lla.lon_deg, ref_lla.alt_m); 
    
    /*4) Initialize EKF */
    x0.q = dr_quat_identity();
    memcpy(x0.ba, ba, sizeof(ba));
    memcpy(x0.bg, bg, sizeof(bg));
    dr_ekf_init(&ekf, &cfg, &x0);
    imu_ma_init(&imu_ma);

    /* CSV Header */
    printf(DR_CSV_HEADER);
    fflush(stdout);

    for(;;)
    {
        /* IMU Read and Average */
        
        if ( read_imu_once(iio_path, g, a)) { fprintf(stderr, "IMU read error\n"); break;}
        imu_ma_push(&imu_ma, g, a);
        imu_ma_get(&imu_ma, g_f, a_f);
        for( k = 0; k < 3; k++){ g_cal[k] = g_f[k] - bg[k]; a_cal[k] = a_f[k] - ba[k];}

        /* Predict */
        dr_ekf_predict(&ekf, &imu, dt);

        /*GNSS update (best-effort)*/
        have = 0; tns = 0; spd = 0.0f;
        rc = get_gnss_fix(fd_gnss, &cur, &have, &y, &m, &d, &H, &M, &S, &ms, &tns, &spd);
        if( rc > 0 && have ){
            /* Convert to ENU */
            ecef_t e = lla_to_ecef(cur);
            dr_gps_pos_t z;
            ecef_delta_to_enu(ref_lla, e, ref_ecef, z.pos);
            (void)dr_ekf_update_gps_pos(&ekf, &z, Rpos);
        }
        /* Output CSV (EKF nominal to ENU, plus back to LLA for convenience)*/
        float yaw, pitch, roll; 
        quat_to_euler_deg(ekf.x.q, &yaw, &pitch, &roll);
        geodetic_t out_lla = enu_to_lla(ref_lla, ref_ecef, ekf.x.p);
        char utc[40] = "0000-00-00T00:00:00.000Z";
        if( rc > 0 ){
            snprintf(utc, sizeof(utc), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",y,m,d,H,M,S,ms);
        }
        printf(DR_CSV_FORMAT,utc, (long long)tns, out_lla.lat_deg, out_lla.lon_deg, out_lla.alt_m,
                                    ekf.x.p[0], ekf.x.p[1], ekf.x.p[2], 
                                    ekf.x.v[0], ekf.x.v[1], ekf.x.v[2],
                                    yaw, pitch, roll);
        fflush(stdout);

        msleep(MAIN_LOOP_DELAY_MS);
    }

    close(fd_gnss);
    return 0;

}