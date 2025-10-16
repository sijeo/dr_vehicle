// SPDX-License-Identifier: MIT
//
// mpu6050_test.c — Userspace tester for MPU6050 integer-only kernel driver
//
// Features:
//  - Userspace stationary calibration (uses double for accuracy).
//  - App-paced sampling at --period-ms (e.g., 100 ms).
//  - Driver ODR set via ioctl (--odr), recommended higher than app period.
//  - Drains device each tick, uses latest (or --avg to average within tick).
//  - CSV output: raw & bias-corrected LSB, optional SI scaling.
//
// Build:
//   gcc -O2 -Wall -o mpu6050_test mpu6050_test.c
//
// Example:
//   sudo ./mpu6050_test /dev/mpu6050-0 
//        --odr 200 --period-ms 100 
//        --cal-ms 3000 --g-lsb 0 0 16384 
//        --accel-fs 4 --gyro-fs 500 --si --avg > imu.csv
//
// Notes:
//   - For 2g FS, 1 g ≈ 16384 LSB; 4g→8192; 8g→4096; 16g→2048.
//   - Keep sensor very still during --cal-ms window; g-lsb sign/axis should
//     match the actual orientation (e.g., +Z up -> 0 0 +16384 at 2g).
//   - SI conversion is optional and done in userspace only.

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "../mpu6050_ioctl.h"

static volatile int g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

// ---- time helpers -----------------------------------------------------------
static void sleep_until_next_tick(struct timespec *t, uint32_t period_ms) {
    t->tv_nsec += (long)period_ms * 1000000L;
    while (t->tv_nsec >= 1000000000L) { t->tv_nsec -= 1000000000L; t->tv_sec += 1; }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t, NULL);
}

// ---- FIFO drain helpers -----------------------------------------------------
static int read_one(int fd, struct mpu6050_sample *s) {
    ssize_t r = read(fd, s, sizeof(*s));
    if (r == (ssize_t)sizeof(*s)) return 1;
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
    return -1; // error/partial: treat as error
}

static int drain_latest(int fd, struct mpu6050_sample *out) {
    struct mpu6050_sample s; int got = 0;
    for (;;) {
        int r = read_one(fd, &s);
        if (r < 0) return -1;
        if (r == 0) break;
        *out = s; got = 1;
    }
    return got;
}

static int drain_accumulate(int fd, // average all samples currently queued
                            double *ax, double *ay, double *az,
                            double *gx, double *gy, double *gz,
                            int64_t *ts_ns, int *count, int *temp_last) {
    struct mpu6050_sample s; int got = 0;
    *ax = *ay = *az = *gx = *gy = *gz = 0.0;
    *count = 0; *ts_ns = 0; *temp_last = 0;
    for (;;) {
        int r = read_one(fd, &s);
        if (r < 0) return -1;
        if (r == 0) break;
        *ax += s.ax; *ay += s.ay; *az += s.az;
        *gx += s.gx; *gy += s.gy; *gz += s.gz;
        *ts_ns = s.t_ns;        // use timestamp of last sample in batch
        *temp_last = s.temp;     // and last raw temp
        ++(*count);
        got = 1;
    }
    return got ? 1 : 0;
}

// ---- calibration ------------------------------------------------------------
struct biases {
    double ax, ay, az; // accel bias in raw LSB (to subtract)
    double gx, gy, gz; // gyro  bias in raw LSB (to subtract)
};

static int calibrate_stationary(int fd, uint32_t cal_ms,
                                double g_lsb_x, double g_lsb_y, double g_lsb_z,
                                struct biases *b) {
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    double ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    uint64_t n = 0;
    struct pollfd p = { .fd = fd, .events = POLLIN };
    struct mpu6050_sample s;

    // Warm up: give the driver a moment to fill FIFO
    (void)poll(&p, 1, 50);

    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (long)((now.tv_sec - start.tv_sec) * 1000L
                          + (now.tv_nsec - start.tv_nsec) / 1000000L);
        if (elapsed_ms >= (long)cal_ms) break;

        if (poll(&p, 1, 50) <= 0) continue;

        while (read(fd, &s, sizeof(s)) == (ssize_t)sizeof(s)) {
            ax += (double)s.ax; ay += (double)s.ay; az += (double)s.az;
            gx += (double)s.gx; gy += (double)s.gy; gz += (double)s.gz;
            ++n;
        }
    }
    if (n == 0) return -1;

    ax /= (double)n; ay /= (double)n; az /= (double)n;
    gx /= (double)n; gy /= (double)n; gz /= (double)n;

    // Stationary assumptions:
    //   gyro_bias = mean(gyro)
    //   accel_bias = mean(accel) - expected_g_in_raw
    b->gx = gx; b->gy = gy; b->gz = gz;
    b->ax = ax - g_lsb_x; b->ay = ay - g_lsb_y; b->az = az - g_lsb_z;
    return 0;
}

// ---- SI scaling (userspace only) -------------------------------------------
// Convert LSB to SI units given full-scale range
static double accel_lsb_per_g(int accel_fs_g) {
    // FS options: 2,4,8,16 g; LSB/g = 16384,8192,4096,2048 respectively
    switch (accel_fs_g) {
        case 2:  return 16384.0;
        case 4:  return 8192.0;
        case 8:  return 4096.0;
        case 16: return 2048.0;
        default: return 8192.0; // default to 4g if unspecified
    }
}
static double gyro_lsb_per_dps(int gyro_fs_dps) {
    // FS options: 250,500,1000,2000 dps; LSB/°/s = 131,65.5,32.8,16.4
    switch (gyro_fs_dps) {
        case 250:  return 131.0;
        case 500:  return 65.5;
        case 1000: return 32.8;
        case 2000: return 16.4;
        default:   return 65.5; // default to 500 dps if unspecified
    }
}

// ---- main -------------------------------------------------------------------
static void usage(const char *argv0) {
    fprintf(stderr,
        "Usage: %s /dev/mpu6050-0 [options]\n"
        "Options:\n"
        "  --odr N              Set driver sampling rate in Hz (e.g., 200)\n"
        "  --period-ms M        App read period in ms (default 100)\n"
        "  --cal-ms K           Stationary calibration window in ms (0=skip)\n"
        "  --g-lsb X Y Z        Expected gravity in raw LSB during calibration\n"
        "  --avg                Average all samples available each tick (default: latest sample)\n"
        "  --accel-fs g         Accel full-scale in g (2|4|8|16) for SI conversion\n"
        "  --gyro-fs dps        Gyro full-scale in dps (250|500|1000|2000)\n"
        "  --si                 Also print SI columns (m/s^2, rad/s)\n"
        "  --duration S         Stop after S seconds (default: run until Ctrl-C)\n",
        argv0);
}

int main(int argc, char **argv) {
    if (argc < 2) { usage(argv[0]); return 1; }

    const char *dev = argv[1];
    uint32_t odr = 200;            // driver sampling Hz
    uint32_t period_ms = 100;      // app tick
    uint32_t cal_ms = 0;           // 0 => skip calibration
    double g_lsb_x = 0, g_lsb_y = 0, g_lsb_z = 0;
    int do_avg = 0;
    int accel_fs = 4;              // g
    int gyro_fs  = 500;            // dps
    int print_si = 0;
    int duration_s = -1;
    int i;
    struct biases b;
    const double ALSB = accel_lsb_per_g(accel_fs);
    const double GLSB = gyro_lsb_per_dps(gyro_fs);
    const double g = 9.80665;
    const double d2r = 3.14159265358979323846 / 180.0;

    struct timespec next;
    time_t start_sec = 0;
    int timed_out = 0;
    int printed = 0;
    // Average all samples available at this tick
    double ax, ay, az, gx, gy, gz;
    int64_t ts_ns = 0;
    int count = 0, temp_last = 0, got = 0;
    double axc = 0.0, ayc = 0.0, azc = 0.0;
    double gxc = 0.0, gyc = 0.0, gzc = 0.0;
    double ax_ms2 = 0.0, ay_ms2 = 0.0, az_ms2 = 0.0;
    double gx_rads = 0.0, gy_rads = 0.0, gz_rads = 0.0;

    for (i = 2; i < argc; ++i) {
        if (!strcmp(argv[i], "--odr") && i+1 < argc) odr = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--period-ms") && i+1 < argc) period_ms = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cal-ms") && i+1 < argc) cal_ms = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--g-lsb") && i+3 < argc) {
            g_lsb_x = strtod(argv[++i], NULL);
            g_lsb_y = strtod(argv[++i], NULL);
            g_lsb_z = strtod(argv[++i], NULL);
        } else if (!strcmp(argv[i], "--avg")) do_avg = 1;
        else if (!strcmp(argv[i], "--accel-fs") && i+1 < argc) accel_fs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--gyro-fs") && i+1 < argc)  gyro_fs  = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--si")) print_si = 1;
        else if (!strcmp(argv[i], "--duration") && i+1 < argc) duration_s = atoi(argv[++i]);
        else { usage(argv[0]); return 1; }
    }

    signal(SIGINT, on_sigint);

    int fd = open(dev, O_RDONLY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 1; }

    // Configure driver ODR
    if (ioctl(fd, MPU6050_IOC_SET_ODR, &odr) != 0)
        perror("ioctl SET_ODR");

    // Optional: set full-scale ranges via ioctl if your app also owns that
    // (keeping defaults here, but you can expose MPU6050_IOC_SET_FS if needed)

    // Optional calibration
    memset(&b, 0, sizeof(b));
    if (cal_ms > 0) {
        fprintf(stderr, "Calibrating for %u ms. Keep device still...\n", cal_ms);
        if (calibrate_stationary(fd, cal_ms, g_lsb_x, g_lsb_y, g_lsb_z, &b) == 0) {
            fprintf(stderr, "Calibration done.\n"
                            "  Accel bias (LSB): [%.2f %.2f %.2f]\n"
                            "  Gyro  bias (LSB): [%.2f %.2f %.2f]\n",
                    b.ax, b.ay, b.az, b.gx, b.gy, b.gz);
        } else {
            fprintf(stderr, "Calibration failed (no samples).\n");
        }
    }

    // CSV header
    if (print_si) {
        printf("ts_ns,ax,ay,az,gx,gy,gz,temp,"
               "ax_corr,ay_corr,az_corr,gx_corr,gy_corr,gz_corr,"
               "ax_ms2,ay_ms2,az_ms2,gx_rads,gy_rads,gz_rads\n");
    } else {
        printf("ts_ns,ax,ay,az,gx,gy,gz,temp,ax_corr,ay_corr,az_corr,gx_corr,gy_corr,gz_corr\n");
    }
    fflush(stdout);

    
    clock_gettime(CLOCK_MONOTONIC, &next);
    start_sec = next.tv_sec;

    while (!g_stop) {
        

        if (do_avg) {
            

            if (drain_accumulate(fd, &ax, &ay, &az, &gx, &gy, &gz, &ts_ns, &count, &temp_last) < 0) {
                perror("read");
                break;
            }
            if (count > 0) {
                ax /= (double)count; ay /= (double)count; az /= (double)count;
                gx /= (double)count; gy /= (double)count; gz /= (double)count;

                axc = ax - b.ax; ayc = ay - b.ay; azc = az - b.az;
                gxc = gx - b.gx; gyc = gy - b.gy; gzc = gz - b.gz;

                if (print_si) {
                    ax_ms2 = (axc / ALSB) * g;
                    ay_ms2 = (ayc / ALSB) * g;
                    az_ms2 = (azc / ALSB) * g;
                    gx_rads = (gxc / GLSB) * d2r;
                    gy_rads = (gyc / GLSB) * d2r;
                    gz_rads = (gzc / GLSB) * d2r;

                    printf("%lld,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                        (long long)ts_ns,
                        ax, ay, az, gx, gy, gz, temp_last,
                        axc, ayc, azc, gxc, gyc, gzc,
                        ax_ms2, ay_ms2, az_ms2, gx_rads, gy_rads, gz_rads);
                } else {
                    printf("%lld,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        (long long)ts_ns,
                        ax, ay, az, gx, gy, gz, temp_last,
                        axc, ayc, azc, gxc, gyc, gzc);
                }
                printed = 1;
            }
        } else {
            // Use the latest sample currently available
            struct mpu6050_sample s;
            got = drain_latest(fd, &s);
            if (got < 0) { perror("read"); break; }
            if (got > 0) {
                axc = (double)s.ax - b.ax;
                ayc = (double)s.ay - b.ay;
                azc = (double)s.az - b.az;
                gxc = (double)s.gx - b.gx;
                gyc = (double)s.gy - b.gy;
                gzc = (double)s.gz - b.gz;

                if (print_si) {
                    ax_ms2 = (axc / ALSB) * g;
                    ay_ms2 = (ayc / ALSB) * g;
                    az_ms2 = (azc / ALSB) * g;
                    gx_rads = (gxc / GLSB) * d2r;
                    gy_rads = (gyc / GLSB) * d2r;
                    gz_rads = (gzc / GLSB) * d2r;

                    printf("%lld,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                        (long long)s.t_ns,
                        s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp,
                        axc, ayc, azc, gxc, gyc, gzc,
                        ax_ms2, ay_ms2, az_ms2, gx_rads, gy_rads, gz_rads);
                } else {
                    printf("%lld,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        (long long)s.t_ns,
                        s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp,
                        axc, ayc, azc, gxc, gyc, gzc);
                }
                printed = 1;
            }
        }

        fflush(stdout);
        sleep_until_next_tick(&next, period_ms);

        if (duration_s > 0 && (int)(next.tv_sec - start_sec) >= duration_s) break;
    }

    close(fd);
    return 0;
}
