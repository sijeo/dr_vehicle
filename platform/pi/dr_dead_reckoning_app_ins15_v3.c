// dr_dead_reckoning_app_ins15_v3.c — vehicle dead reckoning (ISM330BX + GNSS)
//
// Rewritten 2026-11 around the IMU chain proven in test_app/ (imu_streamer +
// imu_rotation_dashboard bench tests). The previous 6k-line version (2D/15-state
// EKFs, own boot cal and AHRS, FRD mount default) is in git history.
//
// Pipeline (one IMU thread, everything at the IMU rate, ~240 Hz):
//
//   mpu6050_char.ko  HW FIFO, reconstructed sample times, SFLP gyro bias
//     -> imu_core    accel scale, sensor->vehicle mount, boot gyro cal,
//                    median + LPF, quality flags           (test_app/imu_core.c)
//     -> imu_fusion  Mahony AHRS with the live SFLP gyro bias, gravity
//                    removal into the heading-free level frame (test_app/imu_fusion.c)
//     -> dr_nav      planar vehicle EKF: position, speed, heading, yaw-bias,
//                    accel-bias, yaw-scale; GNSS pos/vel, ZUPT + ZARU at stops,
//                    NIS gating, re-anchoring                (dr_nav.c)
//   gnss thread  -> /dev/neo6m0 (v2 ioctl, seq-based freshness, validation)
//   main thread  -> LED, push button, mount-calibration trigger, fsync
//
// Heading drift: the AHRS yaw drifts ~0.3 deg/min with the SFLP bias alone
// (bench, 2026-10-30). The nav heading is corrected by GNSS velocity while
// driving and by ZARU (zero yaw rate) at every stop, which also estimates the
// residual yaw bias, so the drift during GNSS outages stays small.
//
// No screen: see platform/pi/DR_APP_README.md for the LED patterns, the push
// button, the in-vehicle mount calibration, the log files and the vehicle
// test plan.
//
// Build (Pi or cross):  make -C platform/pi app_v4
// Run:                  sudo ./dr_dead_reckoning_app --help
//
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "mpu6050_ioctl.h"
#include "neo6m_gnss_ioctl.h"
#include "dr_nav.h"
#include "imu_core.h"
#include "imu_fusion.h"
#include "imu_mount.h"
#include "imu_wire.h"

#define D2R (M_PI / 180.0)
#define R2D (180.0 / M_PI)

/* =========================================================================
 * Configuration (defaults; see --help)
 * ========================================================================= */
#define DEF_IMU_DEV        "/dev/mpu6050-0"
#define DEF_GNSS_DEV       "/dev/neo6m0"
#define DEF_BASE_DIR       "/home/sijeo/dr_vehicle"
#define DEF_LOG_DIR        "/home/sijeo/nav_logs"
#define DEF_LED_GPIO       13          /* BCM, existing calibration LED */
#define DEF_BUTTON_GPIO    26          /* BCM, momentary switch to GND, pull-up in config.txt */

/* GNSS validation (carried over from the previous app) */
#define GNSS_HDOP_MAX_GOOD     2.0f
#define GNSS_HDOP_MAX_USABLE   5.0f
#define MAX_GNSS_SPEED_MPS     45.0f
#define MAX_GNSS_ACCEL_MPS2    8.0f
#define MAX_GNSS_POS_JUMP_M    60.0
#define GNSS_MAX_DT_S          5.0
#define GNSS_COURSE_SPEED_MIN  3.0f    /* m/s: below this the course is not used */
#define GNSS_ALIGN_SPEED       4.0f    /* m/s: heading initialised from the course above this */
#define GNSS_TIMEOUT_S         2.0     /* no used GNSS for this long -> DR mode */

/* Stationary detection (vehicle, engine may be idling) */
#define STILL_GYRO_DPS         1.0     /* filtered |gyro| */
#define STILL_ACC_STD          0.20    /* m/s^2, 2 s window */
#define STILL_ACC_STD_NO_GNSS  0.08    /* tighter when GNSS cannot confirm speed ~0 */
#define STILL_GNSS_SPEED       0.30    /* m/s */
#define STILL_HOLD_S           1.0

/* Mount calibration (in-vehicle procedure) */
#define MC_STILL_S             5.0
#define MC_TURN_MIN_DEG        150.0
#define MC_TURN_MAX_DEG        210.0
#define MC_FWD_MIN_ACC         0.8     /* m/s^2 horizontal */
#define MC_FWD_MAX_YAWRATE     3.0     /* deg/s */
#define MC_FWD_WINDOW_S        4.0     /* use the first seconds after pulling away */
#define MC_FWD_NEED_S          1.0
#define MC_TIMEOUT_S           900.0

typedef struct {
    const char *imu_dev, *gnss_dev, *log_dir, *base_dir;
    char mount_file[256], cal_file[256], state_file[256], trigger_file[256];
    int led_gpio, button_gpio, gpio_base;
    int gyro_fs_dps, sflp_mask;
    double gnss_latency_s, gnss_pos_sigma, gnss_vel_sigma;
    int nav_log_hz, imu_log_hz;
    const char *udp_host;
    int udp_port;
    int outage_period_s, outage_len_s;
    bool mount_cal_at_start, mount_frd;
    float ahrs_kp, ahrs_ki;
} opts_t;

/* =========================================================================
 * Shared state
 * ========================================================================= */
enum { MODE_INIT = 0, MODE_WAIT_GNSS, MODE_ALIGNING, MODE_NAV_GNSS, MODE_NAV_DR, MODE_ERROR };
static const char *mode_name[] = { "INIT", "WAIT_GNSS", "ALIGNING", "NAV_GNSS", "NAV_DR", "ERROR" };

enum { MC_OFF = 0, MC_LEVEL, MC_TURN, MC_FORWARD };          /* LED shows the step number */
enum { MC_RES_NONE = 0, MC_RES_OK, MC_RES_FAIL };

typedef struct {
    double t;                        /* CLOCK_BOOTTIME s of the measurement (latency removed) */
    int pos_fresh, vel_fresh, have_fix;
    double lat, lon, alt, e, n;
    float speed, course_deg, hdop, r_scale;
    bool course_valid;
    int reject;                      /* 0 = accepted */
    char reason[48];
} gnss_meas_t;

#define GQ_LEN 16

typedef struct {
    opts_t o;
    volatile sig_atomic_t stop;

    /* IMU chain (IMU thread only, except the control handler which runs there too) */
    int imu_fd;
    bool imu_ext;
    imu_config_t cfg;
    imu_pipeline_t pl;
    imu_fusion_t fu;
    float mount_preset[3][3];
    bool mount_from_file;
    dr_nav_t nav;

    /* GNSS mailbox + ENU origin (mutex) */
    pthread_mutex_t mtx;
    gnss_meas_t gq[GQ_LEN];
    int gq_head, gq_tail;
    bool origin_set;
    double lat0, lon0, alt0, m_per_rad_lat, m_per_rad_lon;
    bool gnss_dev_ok;

    /* status published to the main thread (LED) */
    volatile int mode;
    volatile int mc_step, mc_result;
    volatile int imu_ok;
    volatile int req_mount_cal, req_mount_abort, req_skip_turn;
    volatile int marker;

    /* logs */
    FILE *navlog, *dbg, *imubin;
    char stamp[32];
    pthread_mutex_t dbg_mtx;

    /* optional live link */
    int udp_sock;
    struct sockaddr_in udp_peer;
} app_t;

static app_t g_app;

/* =========================================================================
 * Small helpers
 * ========================================================================= */
static double boottime_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static void dbg(app_t *A, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void dbg(app_t *A, const char *fmt, ...) {
    va_list ap;
    char line[512];
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    pthread_mutex_lock(&A->dbg_mtx);
    fprintf(stderr, "[dr %9.3f] %s\n", boottime_s(), line);
    if (A->dbg) { fprintf(A->dbg, "%.3f %s\n", boottime_s(), line); fflush(A->dbg); }
    pthread_mutex_unlock(&A->dbg_mtx);
}

static float gyro_lsb_per_dps_for(unsigned e) {
    static const float mdps[] = { 8.75f, 17.5f, 35.0f, 70.0f };      /* 250/500/1000/2000 dps */
    return e < 4 ? 1000.0f / mdps[e] : 0.0f;
}

/* ---- local tangent plane (WGS84), origin at the first fix ---- */
static void origin_set_locked(app_t *A, double lat, double lon, double alt) {
    const double a = 6378137.0, e2 = 6.69437999014e-3;
    double s = sin(lat * D2R), w = sqrt(1.0 - e2 * s * s);
    A->lat0 = lat; A->lon0 = lon; A->alt0 = alt;
    A->m_per_rad_lat = a * (1.0 - e2) / (w * w * w);
    A->m_per_rad_lon = a / w * cos(lat * D2R);
    A->origin_set = true;
}
static void ll_to_en(const app_t *A, double lat, double lon, double *e, double *n) {
    *n = (lat - A->lat0) * D2R * A->m_per_rad_lat;
    *e = (lon - A->lon0) * D2R * A->m_per_rad_lon;
}
static void en_to_ll(const app_t *A, double e, double n, double *lat, double *lon) {
    *lat = A->lat0 + n / A->m_per_rad_lat * R2D;
    *lon = A->lon0 + e / A->m_per_rad_lon * R2D;
}

/* =========================================================================
 * GPIO (sysfs) — LED out, button in (active low)
 * ========================================================================= */
static int gpio_open(int gpio, const char *dir) {
    char path[96];
    int fd, i;
    if (gpio < 0) return -1;
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd >= 0) { dprintf(fd, "%d", gpio); close(fd); }
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    for (i = 0; i < 50 && access(path, W_OK) != 0; i++) usleep(2000);
    fd = open(path, O_WRONLY);
    if (fd >= 0) { if (write(fd, dir, strlen(dir)) < 0) { /* already set */ } close(fd); }
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    return open(path, strcmp(dir, "out") == 0 ? O_WRONLY : O_RDONLY);
}
static void gpio_write(int fd, int on) {
    if (fd < 0) return;
    lseek(fd, 0, SEEK_SET);
    if (write(fd, on ? "1" : "0", 1) < 0) { /* best effort */ }
}
static int gpio_read(int fd) {
    char c = '1';
    if (fd < 0) return -1;
    lseek(fd, 0, SEEK_SET);
    if (read(fd, &c, 1) != 1) return -1;
    return c == '1';
}

/* =========================================================================
 * Files: logs, saved state
 * ========================================================================= */
static void make_dir(const char *d) {
    char cmd[400];
    snprintf(cmd, sizeof(cmd), "mkdir -p '%s'", d);
    if (system(cmd) != 0) { /* reported when the open fails */ }
}

static void open_logs(app_t *A) {
    char path[400];
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    strftime(A->stamp, sizeof(A->stamp), "%Y-%m-%d_%H-%M-%S", &tmv);
    make_dir(A->o.log_dir);

    snprintf(path, sizeof(path), "%s/debug_%s.log", A->o.log_dir, A->stamp);
    A->dbg = fopen(path, "w");

    snprintf(path, sizeof(path), "%s/navlog_%s.csv", A->o.log_dir, A->stamp);
    A->navlog = fopen(path, "w");
    if (A->navlog) {
        fprintf(A->navlog,
            "time_s,mode,ekf_lat,ekf_lon,E,N,speed_mps,heading_enu_deg,heading_cog_deg,"
            "bg_dps,ba_mps2,sg,sig_pos_m,sig_heading_deg,"
            "gnss_fix,gnss_fresh,lat_deg,lon_deg,alt_m,gnss_speed_mps,course_deg,hdop,gnss_E,gnss_N,"
            "err_E,err_N,err_H,gnss_reject,gnss_used_pos,gnss_used_vel,nis_pos,nis_vel,"
            "snap_applied,reacq_active,outage_s,outage_sim,zupt_active,zaru_active,"
            "a_fwd_mps2,w_up_dps,roll_deg,pitch_deg,ahrs_yaw_deg,temp_c,imu_quality,imu_cal_state,"
            "ahrs_bias_sflp,driver_dropped,fifo_overrun,mountcal_step,marker\n");
        fflush(A->navlog);
    }
    if (A->o.imu_log_hz > 0) {
        snprintf(path, sizeof(path), "%s/imuframes_%s.bin", A->o.log_dir, A->stamp);
        A->imubin = fopen(path, "wb");
    }
    dbg(A, "logs: %s/{navlog,debug,imuframes}_%s.*", A->o.log_dir, A->stamp);
}

typedef struct { bool valid; double lat, lon, psi, bg, ba, sg; long wall; } saved_state_t;

static saved_state_t state_load(const char *path) {
    saved_state_t s = { 0 };
    char tag[32] = "";
    FILE *fp = fopen(path, "r");
    if (!fp) return s;
    if (fscanf(fp, "%31s %lf %lf %lf %lf %lf %lf %ld", tag, &s.lat, &s.lon, &s.psi, &s.bg, &s.ba, &s.sg, &s.wall) == 8 &&
        strcmp(tag, "dr_state_v1") == 0 && isfinite(s.lat) && isfinite(s.psi) && fabs(s.bg) < 0.01 &&
        fabs(s.ba) < 1.0 && fabs(s.sg) < 0.05)
        s.valid = true;
    fclose(fp);
    return s;
}

static void state_save(app_t *A, double lat, double lon) {
    char tmp[300];
    FILE *fp;
    snprintf(tmp, sizeof(tmp), "%s.tmp", A->o.state_file);
    fp = fopen(tmp, "w");
    if (!fp) return;
    fprintf(fp, "dr_state_v1 %.9f %.9f %.6f %.9e %.6f %.6f %ld\n", lat, lon, A->nav.x[DR_PSI],
            A->nav.x[DR_BG], A->nav.x[DR_BA], A->nav.x[DR_SG], (long)time(NULL));
    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
    if (rename(tmp, A->o.state_file) != 0) { /* keep the old one */ }
}

/* =========================================================================
 * GNSS thread
 * ========================================================================= */
static int gnss_validate(const gnss_meas_t *prev, gnss_meas_t *cur) {
    if (!cur->have_fix) { snprintf(cur->reason, sizeof(cur->reason), "no_fix"); return 1; }
    if (!isfinite(cur->lat) || !isfinite(cur->lon) || fabs(cur->lat) > 90 || fabs(cur->lon) > 180 ||
        (fabs(cur->lat) < 1e-6 && fabs(cur->lon) < 1e-6)) {
        snprintf(cur->reason, sizeof(cur->reason), "latlon_range"); return 2;
    }
    if (!isfinite(cur->alt) || cur->alt < -500 || cur->alt > 9000) { snprintf(cur->reason, sizeof(cur->reason), "alt_range"); return 3; }
    if (!isfinite(cur->speed) || cur->speed < 0 || cur->speed > MAX_GNSS_SPEED_MPS) {
        snprintf(cur->reason, sizeof(cur->reason), "speed_range %.1f", (double)cur->speed); return 4;
    }
    if (cur->hdop > GNSS_HDOP_MAX_USABLE) { snprintf(cur->reason, sizeof(cur->reason), "hdop %.1f", (double)cur->hdop); return 5; }
    if (prev && cur->pos_fresh) {
        double dt = cur->t - prev->t;
        if (dt > 0 && dt <= GNSS_MAX_DT_S) {
            double jump = hypot(cur->e - prev->e, cur->n - prev->n);
            if (jump > MAX_GNSS_POS_JUMP_M || jump / dt > MAX_GNSS_SPEED_MPS) {
                snprintf(cur->reason, sizeof(cur->reason), "pos_jump %.1fm/%.2fs", jump, dt); return 6;
            }
            if (fabsf(cur->speed - prev->speed) / (float)dt > MAX_GNSS_ACCEL_MPS2) {
                snprintf(cur->reason, sizeof(cur->reason), "speed_spike %.1f->%.1f", (double)prev->speed, (double)cur->speed); return 7;
            }
        }
    }
    cur->r_scale = (cur->hdop <= GNSS_HDOP_MAX_GOOD) ? 1.0f : (cur->hdop <= 4.0f ? cur->hdop * cur->hdop / 4.0f : 6.25f);
    return 0;
}

static void *gnss_thread(void *arg) {
    app_t *A = (app_t *)arg;
    int fd = open(A->o.gnss_dev, O_RDONLY | O_CLOEXEC);
    uint32_t last_pos = 0, last_vel = 0, last_utc = 0xFFFFFFFFu;
    bool use_v2 = true, have_prev = false;
    gnss_meas_t prev;
    memset(&prev, 0, sizeof(prev));
    if (fd < 0) {
        dbg(A, "GNSS: cannot open %s (%s) — DR only from saved state", A->o.gnss_dev, strerror(errno));
        return NULL;
    }
    A->gnss_dev_ok = true;
    {
        struct neo6m_gnss_fix_v2 p;
        if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX_V2, &p) != 0 && errno == ENOTTY) use_v2 = false;
        dbg(A, "GNSS: %s ioctl", use_v2 ? "v2 (sequence counters)" : "v1 (UTC change detection)");
    }
    while (!A->stop) {
        struct neo6m_gnss_fix_v2 f;
        gnss_meas_t m;
        memset(&f, 0, sizeof(f));
        memset(&m, 0, sizeof(m));
        if (use_v2) {
            if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX_V2, &f) != 0) { usleep(50000); continue; }
            m.pos_fresh = f.pos_valid && f.pos_seq != last_pos;
            m.vel_fresh = f.vel_valid && f.vel_seq != last_vel;
            last_pos = f.pos_seq; last_vel = f.vel_seq;
        } else {
            struct neo6m_gnss_fix f1;
            uint32_t utc;
            if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, &f1) != 0) { usleep(50000); continue; }
            memcpy(&f, &f1, sizeof(f1));                 /* v1 fields are a prefix of v2 */
            utc = (uint32_t)f1.utc_hour * 3600000u + f1.utc_min * 60000u + f1.utc_sec * 1000u + f1.utc_millis;
            m.pos_fresh = m.vel_fresh = f1.have_fix && utc != last_utc;
            last_utc = utc;
        }
        if (!m.pos_fresh && !m.vel_fresh) { usleep(20000); continue; }

        m.t = boottime_s() - A->o.gnss_latency_s;
        m.have_fix = f.have_fix;
        m.lat = f.lat_e7 * 1e-7; m.lon = f.lon_e7 * 1e-7; m.alt = f.alt_mm * 1e-3;
        m.speed = f.speed_mmps * 1e-3f;
        m.course_deg = f.course_deg_e5 * 1e-5f;
        m.course_valid = f.heading_valid && m.speed >= GNSS_COURSE_SPEED_MIN;
        m.hdop = f.hdop_valid ? f.hdop_x100 * 0.01f : -1.0f;
        m.r_scale = 1.0f;

        pthread_mutex_lock(&A->mtx);
        if (!A->origin_set && m.have_fix && m.pos_fresh && fabs(m.lat) > 1e-6 &&
            m.hdop <= GNSS_HDOP_MAX_USABLE) {             /* hdop < 0: not reported by the receiver */
            origin_set_locked(A, m.lat, m.lon, m.alt);
            dbg(A, "ENU origin %.7f %.7f %.1f m", m.lat, m.lon, m.alt);
        }
        if (A->origin_set) ll_to_en(A, m.lat, m.lon, &m.e, &m.n);
        m.reject = A->origin_set ? gnss_validate(have_prev ? &prev : NULL, &m) : 8;
        if (!A->origin_set) snprintf(m.reason, sizeof(m.reason), "no_origin");
        if (((A->gq_head + 1) % GQ_LEN) != A->gq_tail) {
            A->gq[A->gq_head] = m;
            A->gq_head = (A->gq_head + 1) % GQ_LEN;
        }
        pthread_mutex_unlock(&A->mtx);
        if (m.reject == 0 && m.pos_fresh) { prev = m; have_prev = true; }
        else if (m.reject && m.reject != 1 && m.reject != 8) dbg(A, "GNSS reject: %s", m.reason);
        usleep(20000);
    }
    close(fd);
    return NULL;
}

/* =========================================================================
 * IMU device setup
 * ========================================================================= */
static int imu_setup(app_t *A) {
    struct mpu6050_fs fs;
    struct mpu6050_filter_cfg fc;
    __u32 odr = 0, fmt = MPU6050_FMT_EXT, who = 0;
    int r, c;

    A->imu_fd = open(A->o.imu_dev, O_RDONLY | O_CLOEXEC);
    if (A->imu_fd < 0) { dbg(A, "IMU: cannot open %s: %s", A->o.imu_dev, strerror(errno)); return -1; }
    ioctl(A->imu_fd, MPU6050_IOC_GET_WHOAMI, &who);
    imu_config_set_dr_defaults(&A->cfg);

    memset(&fs, 0, sizeof(fs));
    if (ioctl(A->imu_fd, MPU6050_IOC_GET_FS, &fs) == 0) {
        unsigned want = A->o.gyro_fs_dps == 250 ? GYRO_250DPS : A->o.gyro_fs_dps == 1000 ? GYRO_1000DPS :
                        A->o.gyro_fs_dps == 2000 ? GYRO_2000DPS : GYRO_500DPS;
        if (fs.gyro != want) {
            fs.gyro = want;
            if (ioctl(A->imu_fd, MPU6050_IOC_SET_FS, &fs) != 0) dbg(A, "IMU: SET_FS failed: %s", strerror(errno));
            ioctl(A->imu_fd, MPU6050_IOC_GET_FS, &fs);
        }
        A->cfg.gyro_lsb_per_dps = gyro_lsb_per_dps_for(fs.gyro);
        if (fs.accel != ACCEL_8G && fs.accel <= ACCEL_16G) {
            static const float fs_g[] = { 2.0f, 4.0f, 8.0f, 16.0f };
            for (r = 0; r < 3; r++) for (c = 0; c < 3; c++) A->cfg.accel_C[r][c] *= fs_g[fs.accel] / 8.0f;
        }
    }
    if (ioctl(A->imu_fd, MPU6050_IOC_GET_ODR, &odr) == 0 && odr > 0) A->cfg.sample_rate_hz = (float)odr;

    if (ioctl(A->imu_fd, MPU6050_IOC_GET_FILTER, &fc) == 0) {
        fc.sflp_enable = 1;
        fc.sflp_fifo_mask = (__u32)A->o.sflp_mask;
        fc.fifo_stream = 1;
        if (ioctl(A->imu_fd, MPU6050_IOC_SET_FILTER, &fc) != 0) dbg(A, "IMU: SET_FILTER failed: %s", strerror(errno));
        ioctl(A->imu_fd, MPU6050_IOC_GET_FILTER, &fc);
        dbg(A, "IMU: chain gyroLPF1=%d accelLPF2=%d SFLP=%u mask=0x%x read=%s", fc.gyro_lpf1_bw, fc.accel_lpf2_bw,
            fc.sflp_enable, fc.sflp_fifo_mask, fc.fifo_stream ? "HW FIFO" : "register poll");
    } else {
        dbg(A, "IMU: driver has no filter ioctl — old mpu6050_char.ko? (SFLP gyro bias unavailable)");
    }
    A->imu_ext = ioctl(A->imu_fd, MPU6050_IOC_SET_READ_FMT, &fmt) == 0;
    dbg(A, "IMU: WHO_AM_I=0x%02X ODR=%.0f Hz gyro %.3f LSB/dps read=%s", (unsigned)who, (double)A->cfg.sample_rate_hz,
        (double)A->cfg.gyro_lsb_per_dps, A->imu_ext ? "extended" : "legacy");

    imu_mount_build(A->o.mount_frd, 0, 0, 0, A->mount_preset);
    memcpy(A->cfg.mount_R_bv, A->mount_preset, sizeof(A->mount_preset));
    A->mount_from_file = imu_mount_load(A->o.mount_file, A->cfg.mount_R_bv) == 0;
    dbg(A, "mount: %s", A->mount_from_file ? A->o.mount_file : "NOT CALIBRATED (preset only) - run the mount calibration");

    if (imu_pipeline_init(&A->pl, &A->cfg, A->o.cal_file, false) != 0) { dbg(A, "IMU: pipeline init failed"); return -1; }
    imu_fusion_init(&A->fu, A->cfg.mount_R_bv, A->o.ahrs_kp, A->o.ahrs_ki);
    return 0;
}

/* =========================================================================
 * Mount calibration (IMU thread)
 * ========================================================================= */
typedef struct {
    int step;
    double t_start, t_still0, t_moving0, yaw_since_level;
    double sum[3]; int n;
    float f1[3], f2[3], fwd[3];
    bool have_f2;
    double fwd_t; int fwd_n; double fwd_sum[3];
    float up[3];
} mountcal_t;

static void mc_set_up(mountcal_t *M) {
    float u1[3], u2[3], t[3];
    float n1 = sqrtf(M->f1[0] * M->f1[0] + M->f1[1] * M->f1[1] + M->f1[2] * M->f1[2]);
    int i;
    for (i = 0; i < 3; i++) u1[i] = M->f1[i] / n1;
    if (M->have_f2) {
        float n2 = sqrtf(M->f2[0] * M->f2[0] + M->f2[1] * M->f2[1] + M->f2[2] * M->f2[2]), nt;
        for (i = 0; i < 3; i++) { u2[i] = M->f2[i] / n2; t[i] = u1[i] + u2[i]; }
        nt = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
        for (i = 0; i < 3; i++) M->up[i] = t[i] / nt;
    } else {
        memcpy(M->up, u1, sizeof(u1));
    }
}

static void mc_finish(app_t *A, mountcal_t *M, bool ok, const char *why) {
    A->mc_step = MC_OFF;
    A->mc_result = ok ? MC_RES_OK : MC_RES_FAIL;
    M->step = MC_OFF;
    if (!ok) dbg(A, "MOUNTCAL failed: %s", why);
}

static void mc_solve_apply(app_t *A, mountcal_t *M) {
    float D[3][3], Rn[3][3];
    imu_mount_cal_info_t info;
    char why[128];
    int r, c, k;
    if (imu_mount_solve(M->f1, M->have_f2 ? M->f2 : NULL, M->fwd, true, D, &info, why, sizeof(why)) != 0) {
        mc_finish(A, M, false, why);
        return;
    }
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++) {
            Rn[r][c] = 0.0f;
            for (k = 0; k < 3; k++) Rn[r][c] += D[r][k] * A->pl.cfg.mount_R_bv[k][c];
        }
    imu_mount_apply(&A->pl, &A->fu, Rn, A->o.mount_file);
    A->mount_from_file = true;
    /* biases learnt in the old frame no longer apply */
    dr_nav_seed_biases(&A->nav, 0.0, 0.0, 0.0);
    dbg(A, "MOUNTCAL done: correction roll %.2f pitch %.2f yaw %.2f deg, slope %.2f deg, |g| %.3f -> %s",
        (double)info.euler_deg[0], (double)info.euler_deg[1], (double)info.euler_deg[2], (double)info.slope_deg,
        (double)info.g_mps2, A->o.mount_file);
    mc_finish(A, M, true, NULL);
}

/* Runs every IMU sample while a calibration is active. acc_v: current
 * vehicle-frame specific force (filtered); still: stationary detector. */
static void mc_update(app_t *A, mountcal_t *M, double t, const float acc_v[3], bool still,
                      double w_up_dps, double dt, double gnss_speed, bool gnss_recent) {
    int i;
    if (A->req_mount_abort) { A->req_mount_abort = 0; mc_finish(A, M, false, "aborted (button)"); return; }
    if (t - M->t_start > MC_TIMEOUT_S) { mc_finish(A, M, false, "timeout"); return; }

    switch (M->step) {
    case MC_LEVEL:
    case MC_TURN:
        if (M->step == MC_TURN) {
            if (!still) M->yaw_since_level += w_up_dps * dt;
            if (A->req_skip_turn) {
                A->req_skip_turn = 0;
                M->step = MC_FORWARD; A->mc_step = MC_FORWARD;
                M->fwd_n = 0; M->t_moving0 = -1;
                mc_set_up(M);
                dbg(A, "MOUNTCAL step 2 skipped (single position): now pull away forward");
                return;
            }
        }
        if (!still) { M->n = 0; M->t_still0 = -1; break; }
        if (M->t_still0 < 0) { M->t_still0 = t; M->n = 0; memset(M->sum, 0, sizeof(M->sum)); }
        for (i = 0; i < 3; i++) M->sum[i] += acc_v[i];
        M->n++;
        if (t - M->t_still0 < MC_STILL_S) break;
        if (M->step == MC_LEVEL) {
            for (i = 0; i < 3; i++) M->f1[i] = (float)(M->sum[i] / M->n);
            M->step = MC_TURN; A->mc_step = MC_TURN;
            M->yaw_since_level = 0; M->t_still0 = -1;
            dbg(A, "MOUNTCAL step 1 captured (%.3f %.3f %.3f): turn around and park on the same spot",
                (double)M->f1[0], (double)M->f1[1], (double)M->f1[2]);
        } else {
            double yaw = fabs(M->yaw_since_level);
            if (yaw >= MC_TURN_MIN_DEG && yaw <= MC_TURN_MAX_DEG) {
                for (i = 0; i < 3; i++) M->f2[i] = (float)(M->sum[i] / M->n);
                M->have_f2 = true;
                M->step = MC_FORWARD; A->mc_step = MC_FORWARD;
                M->fwd_n = 0; M->t_moving0 = -1;
                mc_set_up(M);
                dbg(A, "MOUNTCAL step 2 captured after %.0f deg turn: now pull away forward", yaw);
            } else if (yaw > 20.0) {
                dbg(A, "MOUNTCAL step 2: turned %.0f deg, need ~180 — adjust and stop again", yaw);
                M->t_still0 = -1;
                M->n = 0;
            }
        }
        break;

    case MC_FORWARD: {
        /* From standstill the first horizontal force is the acceleration in the
         * direction of travel: use the first seconds after pulling away. */
        /* Qualify on the change from the last still capture (same spot), so a
         * sloped parking spot does not fake or skew the forward direction. */
        const float *ref = M->have_f2 ? M->f2 : M->f1;
        float d[3], h[3], a;
        (void)gnss_speed; (void)gnss_recent;
        if (still) { M->t_moving0 = -1; M->fwd_n = 0; memset(M->fwd_sum, 0, sizeof(M->fwd_sum)); break; }
        if (M->t_moving0 < 0) M->t_moving0 = t;
        if (t - M->t_moving0 > MC_FWD_WINDOW_S) break;              /* wait for the next start */
        for (i = 0; i < 3; i++) d[i] = acc_v[i] - ref[i];
        a = d[0] * M->up[0] + d[1] * M->up[1] + d[2] * M->up[2];
        for (i = 0; i < 3; i++) h[i] = d[i] - a * M->up[i];
        if (sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]) >= MC_FWD_MIN_ACC && fabs(w_up_dps) < MC_FWD_MAX_YAWRATE) {
            for (i = 0; i < 3; i++) M->fwd_sum[i] += acc_v[i];      /* raw: imu_mount_solve takes the difference */
            M->fwd_n++;
        }
        if (M->fwd_n >= (int)(MC_FWD_NEED_S * A->cfg.sample_rate_hz)) {
            for (i = 0; i < 3; i++) M->fwd[i] = (float)(M->fwd_sum[i] / M->fwd_n);
            mc_solve_apply(A, M);
        }
        break;
    }
    default:
        break;
    }
}

/* =========================================================================
 * IMU / navigation thread
 * ========================================================================= */
typedef struct {
    double t;
    int used_pos, used_vel, snap, reject, fresh;
    double nis_pos, nis_vel;
    gnss_meas_t last;
    bool have_last;
    double t_last_used;
} gnss_status_t;

static void handle_udp_ctl(app_t *A) {
    char buf[512], reply[512];
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    ssize_t n;
    if (A->udp_sock < 0) return;
    n = recvfrom(A->udp_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT, (struct sockaddr *)&from, &fl);
    if (n <= 0 || from.sin_addr.s_addr != A->udp_peer.sin_addr.s_addr) return;
    buf[n] = '\0';
    if (strncmp(buf, "IMUCTL ", 7) != 0) return;
    imu_mount_handle_ctl(buf + 7, &A->pl, &A->fu, A->mount_preset, A->o.mount_file, NULL, NULL, reply, sizeof(reply));
    if (strstr(reply, "MOUNT_DELTA OK") || strstr(reply, "MOUNT_RESET OK")) {
        A->mount_from_file = true;
        dr_nav_seed_biases(&A->nav, 0.0, 0.0, 0.0);
        dbg(A, "mount changed from the dashboard: %s", reply);
    }
    sendto(A->udp_sock, reply, strlen(reply), 0, (struct sockaddr *)&from, fl);
}

static void *imu_thread(void *arg) {
    app_t *A = (app_t *)arg;
    dr_nav_cfg_t ncfg;
    mountcal_t M;
    gnss_status_t G;
    saved_state_t saved = state_load(A->o.state_file);
    uint64_t last_t_ns = 0;
    double t_prev = 0, t_still_since = -1, t_nav_log = 0, t_imu_log = 0, t_flush = 0, t_state = 0, t_status = 0;
    double t_imu_ready = -1, still_w_sum = 0, t_gbias_since = -1;
    int still_w_n = 0, last_marker = 0;
    bool moved_since_start = false, zupt_now = false, zaru_now = false;
    uint8_t frame[IMU_WIRE_V2_FRAME_SIZE];

    memset(&M, 0, sizeof(M));
    memset(&G, 0, sizeof(G));
    dr_nav_default_cfg(&ncfg);
    dr_nav_init(&A->nav, &ncfg);
    if (saved.valid) {
        dr_nav_seed_biases(&A->nav, saved.bg, saved.ba, saved.sg);
        dbg(A, "saved state: lat %.7f lon %.7f heading %.1f deg, bg %.4f dps", saved.lat, saved.lon,
            saved.psi * R2D, saved.bg * R2D);
    }

    while (!A->stop) {
        struct mpu6050_ext_sample es;
        struct mpu6050_sample ls;
        imu_raw_sample_t raw = { 0 };
        imu_sflp_input_t sfi;
        imu_output_t out;
        imu_fusion_output_t fo;
        double t, dt, w_up, a_fwd, g_ref;
        float wv[3], R[3][3];
        ssize_t n;
        bool still;
        int i, r;

        memset(&sfi, 0, sizeof(sfi));
        if (A->imu_ext) {
            n = read(A->imu_fd, &es, sizeof(es));
            if (n != (ssize_t)sizeof(es)) { if (n < 0 && errno != EINTR) { dbg(A, "IMU read: %s", strerror(errno)); usleep(10000); } continue; }
            raw = (imu_raw_sample_t){ .ax = es.ax, .ay = es.ay, .az = es.az, .gx = es.gx, .gy = es.gy, .gz = es.gz };
            last_t_ns = (uint64_t)es.t_ns > last_t_ns ? (uint64_t)es.t_ns : last_t_ns + 1;
            sfi.have_ext = true; sfi.flags = es.flags; sfi.seq = es.seq; sfi.dropped = es.dropped;
            sfi.sflp_seq = es.sflp_seq; sfi.temp = es.temp;
            memcpy(sfi.quat_h, es.quat_h, sizeof(sfi.quat_h));
            memcpy(sfi.gravity, es.gravity, sizeof(sfi.gravity));
            memcpy(sfi.gbias, es.gbias, sizeof(sfi.gbias));
        } else {
            n = read(A->imu_fd, &ls, sizeof(ls));
            if (n != (ssize_t)sizeof(ls)) { if (n < 0 && errno != EINTR) usleep(10000); continue; }
            raw = (imu_raw_sample_t){ .ax = ls.ax, .ay = ls.ay, .az = ls.az, .gx = ls.gx, .gy = ls.gy, .gz = ls.gz };
            last_t_ns = (uint64_t)ls.t_ns > last_t_ns ? (uint64_t)ls.t_ns : last_t_ns + 1;
            sfi.temp = ls.temp;
        }
        if (imu_pipeline_process(&A->pl, &raw, last_t_ns, &out) != 0) continue;
        g_ref = (A->pl.cal.valid && A->pl.cal.boot_acc_norm_mean > 1.0f) ? A->pl.cal.boot_acc_norm_mean
                                                                          : A->cfg.gravity_mps2;
        imu_fusion_update(&A->fu, &sfi, &out, (float)g_ref, &fo);
        t = (double)last_t_ns * 1e-9;
        dt = t_prev > 0 ? t - t_prev : 0.0;
        t_prev = t;

        /* IMU readiness: boot calibration done, or the live SFLP gyro bias for 5 s */
        if (fo.flags & IMU_FF_AHRS_SFLP_BIAS) { if (t_gbias_since < 0) t_gbias_since = t; }
        else t_gbias_since = -1;
        if (t_imu_ready < 0 && (fo.flags & IMU_FF_AHRS_VALID) &&
            (out.cal_state == IMU_CAL_VALID || (t_gbias_since >= 0 && t - t_gbias_since > 5.0))) {
            t_imu_ready = t;
            A->imu_ok = 1;
            dbg(A, "IMU ready (%s), g_ref %.3f", out.cal_state == IMU_CAL_VALID ? "boot cal valid" : "SFLP gyro bias", g_ref);
        }

        /* vertical yaw rate (SFLP-bias-corrected gyro, levelled by the AHRS) and level forward accel */
        if (fo.flags & IMU_FF_AHRS_SFLP_BIAS) {
            for (r = 0; r < 3; r++) {
                wv[r] = -fo.gbias_sflp_dps[r] * (float)D2R;
                for (i = 0; i < 3; i++) wv[r] += A->pl.cfg.mount_R_bv[r][i] * out.gyro_sensor_raw_rps[i];
            }
        } else {
            memcpy(wv, out.gyro_vehicle_unfiltered_rps, sizeof(wv));
        }
        {
            float w = fo.q_ahrs[0], x = fo.q_ahrs[1], y = fo.q_ahrs[2], z = fo.q_ahrs[3];
            R[2][0] = 2 * (x * z - w * y); R[2][1] = 2 * (y * z + w * x); R[2][2] = 1 - 2 * (x * x + y * y);
        }
        w_up = R[2][0] * wv[0] + R[2][1] * wv[1] + R[2][2] * wv[2];
        a_fwd = fo.lin_acc_level_mps2[0];

        /* stationary detector */
        {
            bool gnss_recent = G.have_last && (boottime_s() - G.last.t) < 2.0 && G.last.reject == 0;
            double acc_std = fmax(out.accel_std[0], fmax(out.accel_std[1], out.accel_std[2]));
            bool imu_still = out.gyro_norm_filtered_rps * R2D < STILL_GYRO_DPS &&
                             acc_std < (gnss_recent ? STILL_ACC_STD : STILL_ACC_STD_NO_GNSS);
            bool gnss_slow = !gnss_recent || G.last.speed < STILL_GNSS_SPEED;
            if (imu_still && gnss_slow) { if (t_still_since < 0) t_still_since = t; }
            else t_still_since = -1;
            still = t_still_since >= 0 && (t - t_still_since) >= STILL_HOLD_S;
            if (!still && A->imu_ok && t_still_since < 0 && gnss_recent && G.last.speed > 1.0) moved_since_start = true;
            if (!still && A->imu_ok && !gnss_recent && fabs(a_fwd) > 1.0) moved_since_start = true;
        }

        /* ---- navigation ----
         * Always the real measurements, also when parked: the model removes
         * bg / ba, so a parked vehicle keeps its heading while ZARU/ZUPT tune them. */
        if (A->imu_ok) dr_nav_predict(&A->nav, a_fwd, w_up, dt);

        zupt_now = zaru_now = false;
        if (A->imu_ok && still) {
            still_w_sum += w_up; still_w_n++;
            if (still_w_n >= (int)(A->cfg.sample_rate_hz / 10.0f)) {
                dr_nav_update_zupt(&A->nav);
                dr_nav_update_zaru(&A->nav, still_w_sum / still_w_n);
                zupt_now = zaru_now = true;
                still_w_sum = 0; still_w_n = 0;
            }
        } else {
            still_w_sum = 0; still_w_n = 0;
        }

        /* GNSS measurements from the mailbox */
        for (;;) {
            gnss_meas_t m;
            bool have = false;
            pthread_mutex_lock(&A->mtx);
            if (A->gq_tail != A->gq_head) { m = A->gq[A->gq_tail]; A->gq_tail = (A->gq_tail + 1) % GQ_LEN; have = true; }
            pthread_mutex_unlock(&A->mtx);
            if (!have) break;
            G.last = m; G.have_last = true; G.fresh = 1;
            if (m.reject) { G.reject = 1; continue; }
            if (A->o.outage_period_s > 0 && t_imu_ready > 0 &&
                fmod(t - t_imu_ready, A->o.outage_period_s) > A->o.outage_period_s - A->o.outage_len_s) continue;
            if (!A->imu_ok) continue;
            {
                double lag = fmin(fmax(boottime_s() - m.t, 0.0), 1.0);
                double v = A->nav.x[DR_V], psi = A->nav.x[DR_PSI];
                double sig = A->o.gnss_pos_sigma * sqrt(m.r_scale), nis = 0;
                if (m.pos_fresh) {
                    double e = m.e, nn = m.n;
                    if (A->nav.aligned) { e += v * cos(psi) * lag; nn += v * sin(psi) * lag; }
                    r = dr_nav_update_pos(&A->nav, e, nn, sig, &nis);
                    G.nis_pos = nis;
                    if (r == DR_UPD_APPLIED) { G.used_pos = 1; G.t_last_used = t; }
                    if (r == DR_UPD_REANCHORED && A->nav.aligned) { G.snap = 1; G.t_last_used = t; dbg(A, "re-anchored to GNSS (NIS %.1f)", nis); }
                }
                if (m.vel_fresh) {
                    double psi_g = M_PI / 2.0 - m.course_deg * D2R;     /* compass course -> ENU heading */
                    double ve = m.speed * cos(psi_g), vn = m.speed * sin(psi_g);
                    if (!A->nav.aligned && A->nav.positioned && m.course_valid && m.speed > GNSS_ALIGN_SPEED) {
                        dr_nav_align(&A->nav, psi_g, m.speed, 3.0 * D2R, 0.3);
                        dbg(A, "heading aligned from GNSS course %.1f deg at %.1f m/s", (double)m.course_deg, (double)m.speed);
                    } else if (A->nav.aligned) {
                        if (m.course_valid && m.speed > GNSS_COURSE_SPEED_MIN)
                            r = dr_nav_update_vel(&A->nav, ve, vn, A->o.gnss_vel_sigma, &nis), G.nis_vel = nis;
                        else
                            r = dr_nav_update_speed(&A->nav, m.speed, A->o.gnss_vel_sigma);
                        if (r == DR_UPD_APPLIED) { G.used_vel = 1; G.t_last_used = t; }
                    }
                }
            }
        }

        /* heading from the saved state when the vehicle has not moved since power-up */
        if (A->imu_ok && !A->nav.aligned && saved.valid && !moved_since_start) {
            bool use = false;
            if (A->nav.positioned) {
                double e, nn, ok;
                pthread_mutex_lock(&A->mtx);
                ll_to_en(A, saved.lat, saved.lon, &e, &nn);
                pthread_mutex_unlock(&A->mtx);
                ok = hypot(e - A->nav.x[DR_PE], nn - A->nav.x[DR_PN]);
                use = ok < 20.0;
            } else if (t - t_imu_ready > 30.0) {           /* no GNSS at all: start from the saved position */
                pthread_mutex_lock(&A->mtx);
                if (!A->origin_set) origin_set_locked(A, saved.lat, saved.lon, 0.0);
                pthread_mutex_unlock(&A->mtx);
                {
                    double e, nn;
                    ll_to_en(A, saved.lat, saved.lon, &e, &nn);
                    dr_nav_set_position(&A->nav, e, nn, 10.0);
                }
                use = true;
            }
            if (use) {
                dr_nav_align(&A->nav, saved.psi, 0.0, 5.0 * D2R, 0.1);
                dbg(A, "heading %.1f deg taken from the saved state (vehicle not moved)", saved.psi * R2D);
            }
        }

        /* mode for the LED */
        {
            int mode;
            if (!A->imu_ok) mode = MODE_INIT;
            else if (!A->nav.positioned) mode = MODE_WAIT_GNSS;
            else if (!A->nav.aligned) mode = MODE_ALIGNING;
            else mode = (G.t_last_used > 0 && t - G.t_last_used < GNSS_TIMEOUT_S) ? MODE_NAV_GNSS : MODE_NAV_DR;
            if (mode != A->mode) dbg(A, "mode %s -> %s", mode_name[A->mode], mode_name[mode]);
            A->mode = mode;
        }

        /* mount calibration */
        if (A->req_mount_cal) {
            A->req_mount_cal = 0;
            memset(&M, 0, sizeof(M));
            M.step = MC_LEVEL; M.t_start = t; M.t_still0 = -1; M.t_moving0 = -1;
            A->mc_step = MC_LEVEL; A->mc_result = MC_RES_NONE;
            dbg(A, "MOUNTCAL started: park on level ground, stay still for %.0f s", MC_STILL_S);
        }
        if (M.step != MC_OFF)
            mc_update(A, &M, t, out.accel_vehicle_filtered, still, w_up * R2D, dt,
                      G.have_last ? G.last.speed : 0.0, G.have_last && boottime_s() - G.last.t < 2.0);

        /* ---- outputs ---- */
        if (A->udp_sock >= 0) {
            size_t fsz = 0;
            if (imu_wire_encode_v2(&out, &A->pl.cal, &fo, frame, sizeof(frame), &fsz) == 0)
                sendto(A->udp_sock, frame, fsz, 0, (struct sockaddr *)&A->udp_peer, sizeof(A->udp_peer));
            handle_udp_ctl(A);
        }
        if (A->imubin && A->o.imu_log_hz > 0 && t - t_imu_log >= 1.0 / A->o.imu_log_hz) {
            size_t fsz = 0;
            t_imu_log = t;
            if (imu_wire_encode_v2(&out, &A->pl.cal, &fo, frame, sizeof(frame), &fsz) == 0) fwrite(frame, 1, fsz, A->imubin);
        }
        if (A->navlog && t - t_nav_log >= 1.0 / A->o.nav_log_hz) {
            double lat = NAN, lon = NAN, eE = NAN, eN = NAN, eH = NAN, outage = 0;
            const gnss_meas_t *gm = &G.last;
            bool outage_sim = A->o.outage_period_s > 0 && t_imu_ready > 0 &&
                              fmod(t - t_imu_ready, A->o.outage_period_s) > A->o.outage_period_s - A->o.outage_len_s;
            t_nav_log = t;
            if (A->origin_set && A->nav.positioned) en_to_ll(A, A->nav.x[DR_PE], A->nav.x[DR_PN], &lat, &lon);
            if (G.have_last && gm->reject == 0 && A->nav.positioned) {
                eE = A->nav.x[DR_PE] - gm->e; eN = A->nav.x[DR_PN] - gm->n; eH = hypot(eE, eN);
            }
            if (A->nav.aligned) outage = G.t_last_used > 0 ? t - G.t_last_used : 0;
            if (A->marker != last_marker) last_marker = A->marker;
            fprintf(A->navlog,
                "%.3f,%s,%.8f,%.8f,%.3f,%.3f,%.3f,%.2f,%.2f,%.5f,%.4f,%.5f,%.2f,%.2f,"
                "%d,%d,%.8f,%.8f,%.2f,%.3f,%.2f,%.2f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,%d,%d,%d,%.2f,%.2f,"
                "%d,%d,%.2f,%d,%d,%d,"
                "%.4f,%.4f,%.3f,%.3f,%.2f,%.2f,%.0f,%d,"
                "%d,%u,%d,%d,%d\n",
                t, mode_name[A->mode], lat, lon, A->nav.x[DR_PE], A->nav.x[DR_PN], A->nav.x[DR_V],
                A->nav.x[DR_PSI] * R2D, fmod(450.0 - A->nav.x[DR_PSI] * R2D, 360.0),
                A->nav.x[DR_BG] * R2D, A->nav.x[DR_BA], A->nav.x[DR_SG],
                sqrt(fmax(A->nav.P[DR_PE][DR_PE], A->nav.P[DR_PN][DR_PN])), sqrt(A->nav.P[DR_PSI][DR_PSI]) * R2D,
                G.have_last ? gm->have_fix : 0, G.fresh, G.have_last ? gm->lat : NAN, G.have_last ? gm->lon : NAN,
                G.have_last ? gm->alt : NAN, G.have_last ? (double)gm->speed : NAN,
                G.have_last ? (double)gm->course_deg : NAN, G.have_last ? (double)gm->hdop : NAN,
                G.have_last ? gm->e : NAN, G.have_last ? gm->n : NAN,
                eE, eN, eH, G.reject, G.used_pos, G.used_vel, G.nis_pos, G.nis_vel,
                G.snap, G.snap, outage, outage_sim ? 1 : 0, zupt_now || still, zaru_now || still,
                a_fwd, w_up * R2D, (double)fo.euler_ahrs_deg[0], (double)fo.euler_ahrs_deg[1],
                (double)fo.euler_ahrs_deg[2], (double)fo.temp_c, (double)out.quality_score, (int)out.cal_state,
                (fo.flags & IMU_FF_AHRS_SFLP_BIAS) ? 1 : 0, fo.driver_dropped,
                (fo.flags & IMU_FF_FIFO_OVERRUN) ? 1 : 0, A->mc_step, last_marker);
            G.fresh = G.reject = G.used_pos = G.used_vel = G.snap = 0;
        }
        if (t - t_flush >= 1.0) {
            t_flush = t;
            if (A->navlog) fflush(A->navlog);
            if (A->imubin) fflush(A->imubin);
        }
        /* save the state while parked (heading + biases for the next start) */
        if (still && A->nav.aligned && A->origin_set && t - t_state > 10.0) {
            double lat, lon;
            t_state = t;
            en_to_ll(A, A->nav.x[DR_PE], A->nav.x[DR_PN], &lat, &lon);
            state_save(A, lat, lon);
        }
        if (t - t_status >= 10.0) {
            t_status = t;
            dbg(A, "status %s v=%.2f hdg=%.1f bg=%.4fdps ba=%.3f sig_pos=%.1fm AHRS r/p=%.1f/%.1f T=%.1fC drop=%u",
                mode_name[A->mode], A->nav.x[DR_V], fmod(450.0 - A->nav.x[DR_PSI] * R2D, 360.0),
                A->nav.x[DR_BG] * R2D, A->nav.x[DR_BA], sqrt(A->nav.P[DR_PE][DR_PE]),
                (double)fo.euler_ahrs_deg[0], (double)fo.euler_ahrs_deg[1], (double)fo.temp_c, fo.driver_dropped);
        }
    }

    if (A->nav.aligned && A->origin_set) {
        double lat, lon;
        en_to_ll(A, A->nav.x[DR_PE], A->nav.x[DR_PN], &lat, &lon);
        state_save(A, lat, lon);
    }
    return NULL;
}

/* =========================================================================
 * Main thread: LED, button, trigger file, fsync
 * ========================================================================= */
/* LED pattern for the current state, as on/off at time t (s) */
static int led_pattern(const app_t *A, double t, double result_until, int result) {
    double ph;
    if (result_until > t) return result == MC_RES_OK ? 1 : (fmod(t, 0.2) < 0.1);   /* 3 s: on = OK, 5 Hz = failed */
    if (A->mode == MODE_ERROR) return fmod(t, 0.2) < 0.1;                           /* 5 Hz: fatal */
    if (A->mc_step != MC_OFF) {                                                     /* N flashes + pause = step N */
        double cyc = 0.3 * A->mc_step + 1.5;
        ph = fmod(t, cyc);
        return ph < 0.3 * A->mc_step && fmod(ph, 0.3) < 0.15;
    }
    /* mount never calibrated: a double flash every 3 s on top of the normal pattern */
    if (!A->mount_from_file && A->mode != MODE_INIT) {
        ph = fmod(t, 3.0);
        if (ph < 0.6) return (ph < 0.1) || (ph >= 0.25 && ph < 0.35);
    }
    switch (A->mode) {
    case MODE_INIT:      return fmod(t, 1.0) < 0.5;                 /* 1 Hz: IMU starting / calibrating */
    case MODE_WAIT_GNSS:
    case MODE_ALIGNING:  return fmod(t, 1.0) < 0.1;                 /* short flash: waiting for GNSS / first drive */
    case MODE_NAV_GNSS:  return 1;                                  /* steady: navigating with GNSS */
    case MODE_NAV_DR:    return fmod(t, 1.0) > 0.1;                 /* on with a short gap: dead reckoning */
    default:             return 0;
    }
}

static void on_signal(int sig) { (void)sig; g_app.stop = 1; }

static void usage(const char *p) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  --imu-dev PATH       %s\n"
        "  --gnss-dev PATH      %s\n"
        "  --base-dir DIR       %s (mount, gyro cal, saved state, START_MOUNT_CAL trigger)\n"
        "  --log-dir DIR        %s\n"
        "  --led-gpio N         %d (BCM, -1 = none)\n"
        "  --button-gpio N      %d (BCM, active low, -1 = none)\n"
        "  --gpio-base N        0 (sysfs offset, e.g. 512 on kernel >= 6.6)\n"
        "  --gyro-fs DPS        500\n"
        "  --sflp-mask M        4 (SFLP gyro bias only)\n"
        "  --gnss-latency S     0.10\n"
        "  --gnss-pos-sigma M   1.5\n"
        "  --gnss-vel-sigma MPS 0.15\n"
        "  --nav-log-hz N       10\n"
        "  --imu-log-hz N       50 (imuframes_*.bin for the dashboard, 0 = off)\n"
        "  --udp HOST:PORT      stream live frames to imu_rotation_dashboard.py\n"
        "  --outage-test P,L    ignore GNSS for L s every P s (DR accuracy test)\n"
        "  --mount-cal          start the mount calibration at power-up\n"
        "  --mount frd          preset for a z-down/y-right installation (default flu)\n",
        p, DEF_IMU_DEV, DEF_GNSS_DEV, DEF_BASE_DIR, DEF_LOG_DIR, DEF_LED_GPIO, DEF_BUTTON_GPIO);
}

int main(int argc, char **argv) {
    app_t *A = &g_app;
    pthread_t th_imu, th_gnss;
    int led_fd, btn_fd, btn_low = 0, btn_cnt = 0, i;
    double press_t0 = -1, result_until = 0, t_sync = 0;
    int last_result = MC_RES_NONE, shutdown_req = 0;
    static const struct option opts[] = {
        { "imu-dev", required_argument, NULL, 1 }, { "gnss-dev", required_argument, NULL, 2 },
        { "base-dir", required_argument, NULL, 3 }, { "log-dir", required_argument, NULL, 4 },
        { "led-gpio", required_argument, NULL, 5 }, { "button-gpio", required_argument, NULL, 6 },
        { "gpio-base", required_argument, NULL, 7 }, { "gyro-fs", required_argument, NULL, 8 },
        { "sflp-mask", required_argument, NULL, 9 }, { "gnss-latency", required_argument, NULL, 10 },
        { "gnss-pos-sigma", required_argument, NULL, 11 }, { "gnss-vel-sigma", required_argument, NULL, 12 },
        { "nav-log-hz", required_argument, NULL, 13 }, { "imu-log-hz", required_argument, NULL, 14 },
        { "udp", required_argument, NULL, 15 }, { "outage-test", required_argument, NULL, 16 },
        { "mount-cal", no_argument, NULL, 17 }, { "mount", required_argument, NULL, 18 },
        { "help", no_argument, NULL, 'h' }, { NULL, 0, NULL, 0 }
    };

    memset(A, 0, sizeof(*A));
    A->o.imu_dev = DEF_IMU_DEV; A->o.gnss_dev = DEF_GNSS_DEV;
    A->o.base_dir = DEF_BASE_DIR; A->o.log_dir = DEF_LOG_DIR;
    A->o.led_gpio = DEF_LED_GPIO; A->o.button_gpio = DEF_BUTTON_GPIO;
    A->o.gyro_fs_dps = 500; A->o.sflp_mask = 4;
    A->o.gnss_latency_s = 0.10; A->o.gnss_pos_sigma = 1.5; A->o.gnss_vel_sigma = 0.15;
    A->o.nav_log_hz = 10; A->o.imu_log_hz = 50;
    A->o.ahrs_kp = 0.3f; A->o.ahrs_ki = 0.0f;           /* vehicle: softer accel correction, bias from SFLP */
    A->udp_sock = -1;
    for (;;) {
        int c = getopt_long(argc, argv, "h", opts, NULL);
        if (c == -1) break;
        switch (c) {
        case 1: A->o.imu_dev = optarg; break;
        case 2: A->o.gnss_dev = optarg; break;
        case 3: A->o.base_dir = optarg; break;
        case 4: A->o.log_dir = optarg; break;
        case 5: A->o.led_gpio = atoi(optarg); break;
        case 6: A->o.button_gpio = atoi(optarg); break;
        case 7: A->o.gpio_base = atoi(optarg); break;
        case 8: A->o.gyro_fs_dps = atoi(optarg); break;
        case 9: A->o.sflp_mask = (int)strtol(optarg, NULL, 0); break;
        case 10: A->o.gnss_latency_s = atof(optarg); break;
        case 11: A->o.gnss_pos_sigma = atof(optarg); break;
        case 12: A->o.gnss_vel_sigma = atof(optarg); break;
        case 13: A->o.nav_log_hz = atoi(optarg) > 0 ? atoi(optarg) : 10; break;
        case 14: A->o.imu_log_hz = atoi(optarg); break;
        case 15: {
            static char host[64];
            const char *colon = strrchr(optarg, ':');
            if (!colon) { usage(argv[0]); return 2; }
            snprintf(host, sizeof(host), "%.*s", (int)(colon - optarg), optarg);
            A->o.udp_host = host; A->o.udp_port = atoi(colon + 1);
            break;
        }
        case 16: if (sscanf(optarg, "%d,%d", &A->o.outage_period_s, &A->o.outage_len_s) != 2 ||
                     A->o.outage_len_s >= A->o.outage_period_s) { usage(argv[0]); return 2; } break;
        case 17: A->o.mount_cal_at_start = true; break;
        case 18: A->o.mount_frd = strcmp(optarg, "frd") == 0; break;
        default: usage(argv[0]); return c == 'h' ? 0 : 2;
        }
    }
    snprintf(A->o.mount_file, sizeof(A->o.mount_file), "%s/imu_mount.txt", A->o.base_dir);
    snprintf(A->o.cal_file, sizeof(A->o.cal_file), "%s/imu_calibration.bin", A->o.base_dir);
    snprintf(A->o.state_file, sizeof(A->o.state_file), "%s/dr_state.txt", A->o.base_dir);
    snprintf(A->o.trigger_file, sizeof(A->o.trigger_file), "%s/START_MOUNT_CAL", A->o.base_dir);
    make_dir(A->o.base_dir);

    setvbuf(stderr, NULL, _IOLBF, 0);
    pthread_mutex_init(&A->mtx, NULL);
    pthread_mutex_init(&A->dbg_mtx, NULL);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    open_logs(A);
    dbg(A, "dr app start; base %s; outage test %d/%d s", A->o.base_dir, A->o.outage_len_s, A->o.outage_period_s);

    led_fd = gpio_open(A->o.led_gpio >= 0 ? A->o.led_gpio + A->o.gpio_base : -1, "out");
    btn_fd = gpio_open(A->o.button_gpio >= 0 ? A->o.button_gpio + A->o.gpio_base : -1, "in");
    if (A->o.led_gpio >= 0 && led_fd < 0) dbg(A, "LED GPIO %d not available", A->o.led_gpio);
    if (A->o.button_gpio >= 0 && btn_fd < 0) dbg(A, "button GPIO %d not available (use START_MOUNT_CAL file)", A->o.button_gpio);

    if (A->o.udp_host) {
        A->udp_sock = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        A->udp_peer.sin_family = AF_INET;
        A->udp_peer.sin_port = htons((uint16_t)A->o.udp_port);
        if (A->udp_sock < 0 || inet_pton(AF_INET, A->o.udp_host, &A->udp_peer.sin_addr) != 1) {
            dbg(A, "bad --udp %s", A->o.udp_host);
            if (A->udp_sock >= 0) close(A->udp_sock);
            A->udp_sock = -1;
        } else {
            dbg(A, "live frames -> %s:%d", A->o.udp_host, A->o.udp_port);
        }
    }

    if (imu_setup(A) != 0) {
        A->mode = MODE_ERROR;
        for (i = 0; i < 300 && !A->stop; i++) { gpio_write(led_fd, led_pattern(A, boottime_s(), 0, 0)); usleep(20000); }
        dbg(A, "fatal: IMU not available");
        return 1;
    }
    if (A->o.mount_cal_at_start) A->req_mount_cal = 1;
    pthread_create(&th_gnss, NULL, gnss_thread, A);
    pthread_create(&th_imu, NULL, imu_thread, A);

    while (!A->stop) {
        double t = boottime_s();
        int lv;

        if (A->mc_result != last_result) {
            last_result = A->mc_result;
            if (last_result != MC_RES_NONE) result_until = t + 3.0;
        }
        if (press_t0 > 0 && t - press_t0 >= 10.0) gpio_write(led_fd, 0);        /* release = shutdown */
        else if (press_t0 > 0 && t - press_t0 >= 3.0) gpio_write(led_fd, 1);    /* release = mount cal */
        else gpio_write(led_fd, led_pattern(A, t, result_until, last_result));

        /* button: <1 s = marker (or skip step 2 during calibration), >=3 s = start/abort calibration */
        lv = gpio_read(btn_fd);
        if (lv >= 0) {
            int low = (lv == 0);
            btn_cnt = (low == btn_low) ? 0 : btn_cnt + 1;
            if (btn_cnt >= 3) {                           /* 60 ms debounce */
                btn_low = low; btn_cnt = 0;
                if (low) press_t0 = t;
                else if (press_t0 > 0) {
                    double held = t - press_t0;
                    press_t0 = -1;
                    if (held >= 10.0) {               /* clean shutdown before cutting power */
                        dbg(A, "button held %.0f s: shutting down", held);
                        shutdown_req = 1;
                        A->stop = 1;
                    } else if (held >= 3.0) {
                        if (A->mc_step != MC_OFF) A->req_mount_abort = 1;
                        else A->req_mount_cal = 1;
                    } else if (held >= 0.05) {
                        if (A->mc_step == MC_TURN) A->req_skip_turn = 1;
                        else { A->marker++; dbg(A, "MARKER %d (button)", A->marker); }
                    }
                }
            }
        }
        if (access(A->o.trigger_file, F_OK) == 0) {
            unlink(A->o.trigger_file);
            if (A->mc_step == MC_OFF) A->req_mount_cal = 1;
        }
        if (t - t_sync > 10.0) {                         /* power can be cut at any time in a vehicle */
            t_sync = t;
            if (A->navlog) fsync(fileno(A->navlog));
            if (A->imubin) fsync(fileno(A->imubin));
        }
        usleep(20000);
    }

    pthread_join(th_imu, NULL);
    pthread_join(th_gnss, NULL);
    gpio_write(led_fd, 0);
    if (A->navlog) { fflush(A->navlog); fsync(fileno(A->navlog)); fclose(A->navlog); }
    if (A->imubin) fclose(A->imubin);
    dbg(A, "dr app stop");
    if (A->dbg) fclose(A->dbg);
    if (shutdown_req) {
        sync();
        if (system("/sbin/shutdown -h now") != 0) { /* nothing else to do */ }
    }
    return 0;
}
