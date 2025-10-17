// SPDX-License-Identifier: MIT
// Minimal MPU6050 reader for the mpu6050_char driver.
// - Does NOT modify sensor/driver settings.
// - Optional WHO_AM_I read (sanity check).
// - Userspace stationary calibration (--cal-ms + --g-lsb).
// - App-paced read loop (--period-ms, --duration).
//
// Build:
//   gcc -O2 -Wall -o mpu6050_reader_simple mpu6050_reader_simple.c
//
// Example:
//   sudo ./mpu6050_reader_simple /dev/mpu6050-0 \
//       --period-ms 100 \
//       --cal-ms 3000 --g-lsb 0 0 8192 \
//       --accel-fs 4 --gyro-fs 500 --si --whoami

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "mpu6050_ioctl.h"  // from your driver tree

// If your UAPI uses 't_ns' instead of 'ts_ns', compile with -DSAMPLE_TS_FIELD=t_ns
#ifndef SAMPLE_TS_FIELD
#define SAMPLE_TS_FIELD ts_ns
#endif

#define SAMPLE_TS(s) ((long long)((s).SAMPLE_TS_FIELD))

static volatile int g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* --- time helper: absolute sleep to avoid drift --- */
static void sleep_until_next_tick(struct timespec *t, unsigned period_ms) {
    t->tv_nsec += (long)period_ms * 1000000L;
    while (t->tv_nsec >= 1000000000L) { t->tv_nsec -= 1000000000L; t->tv_sec += 1; }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t, NULL);
}

/* --- simple one-sample read (works in on-demand or IRQ mode) --- */
static int read_one(int fd, struct mpu6050_sample *s) {
    ssize_t r = read(fd, s, sizeof(*s));
    if (r == (ssize_t)sizeof(*s)) return 1;
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
    return -1;
}

/* --- calibration (stationary) --- */
struct biases { double ax, ay, az, gx, gy, gz; };

static int calibrate_stationary(int fd, unsigned cal_ms,
                                double g_lsb_x, double g_lsb_y, double g_lsb_z,
                                struct biases *b)
{
    struct timespec start, now; clock_gettime(CLOCK_MONOTONIC, &start);
    double ax=0, ay=0, az=0, gx=0, gy=0, gz=0; uint64_t n=0;

    // small warmup
    struct timespec tmp = start; sleep_until_next_tick(&tmp, 20);

    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed = (long)((now.tv_sec - start.tv_sec)*1000L +
                              (now.tv_nsec - start.tv_nsec)/1000000L);
        if (elapsed >= (long)cal_ms) break;

        struct mpu6050_sample s;
        if (read_one(fd, &s) == 1) {
            ax += (double)s.ax; ay += (double)s.ay; az += (double)s.az;
            gx += (double)s.gx; gy += (double)s.gy; gz += (double)s.gz;
            ++n;
        } else {
            // avoid hammering the bus
            struct timespec t = now; sleep_until_next_tick(&t, 5);
        }
    }

    if (n == 0) return -1;
    ax/=n; ay/=n; az/=n; gx/=n; gy/=n; gz/=n;

    b->gx = gx; b->gy = gy; b->gz = gz;               // expect ~0 at rest
    b->ax = ax - g_lsb_x; b->ay = ay - g_lsb_y; b->az = az - g_lsb_z; // remove gravity
    return 0;
}

/* --- userspace-only SI conversions (no driver changes) --- */
static double accel_lsb_per_g(int fs_g) {
    switch (fs_g) { case 2: return 16384.0; case 4: return 8192.0;
                    case 8: return 4096.0; case 16: return 2048.0;
                    default: return 8192.0; }
}
static double gyro_lsb_per_dps(int fs_dps) {
    switch (fs_dps) { case 250: return 131.0; case 500: return 65.5;
                      case 1000: return 32.8; case 2000: return 16.4;
                      default: return 65.5; }
}

static void usage(const char *p) {
    fprintf(stderr,
        "Usage: %s /dev/mpu6050-0 [options]\n"
        "  --period-ms M       Sample every M ms (default 100)\n"
        "  --duration S        Stop after S seconds (default: Ctrl-C)\n"
        "  --cal-ms K          Stationary calibration window in ms (0=skip)\n"
        "  --g-lsb X Y Z       Gravity in RAW LSB during cal (match FS & orientation)\n"
        "  --accel-fs g        Accel FS for SI (2|4|8|16). Used only for conversion.\n"
        "  --gyro-fs dps       Gyro FS for SI (250|500|1000|2000).\n"
        "  --si                Also print SI columns (m/s^2, rad/s)\n"
        "  --whoami            Read and print WHO_AM_I (no settings changed)\n",
        p);
}

int main(int argc, char **argv) {
    if (argc < 2) { usage(argv[0]); return 1; }

    const char *devnode = argv[1];
    unsigned period_ms = 100;
    unsigned duration_s = 0;   // 0 => run until Ctrl-C
    unsigned cal_ms = 0;
    double g_lsb_x=0, g_lsb_y=0, g_lsb_z=0;
    int accel_fs = 4, gyro_fs = 500;
    int print_si = 0, do_whoami = 0;

    for (int i=2; i<argc; ++i) {
        if (!strcmp(argv[i], "--period-ms") && i+1<argc) period_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--duration")   && i+1<argc) duration_s = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cal-ms")     && i+1<argc) cal_ms     = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--g-lsb")      && i+3<argc) {
            g_lsb_x = strtod(argv[++i], NULL);
            g_lsb_y = strtod(argv[++i], NULL);
            g_lsb_z = strtod(argv[++i], NULL);
        } else if (!strcmp(argv[i], "--accel-fs") && i+1<argc) accel_fs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--gyro-fs")    && i+1<argc) gyro_fs  = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--si")) print_si = 1;
        else if (!strcmp(argv[i], "--whoami")) do_whoami = 1;
        else { usage(argv[0]); return 1; }
    }

    int fd = open(devnode, O_RDONLY);     // blocking open is fine for both modes
    if (fd < 0) { perror("open"); return 1; }

    if (do_whoami) {
        uint32_t who = 0;
        if (ioctl(fd, MPU6050_IOC_GET_WHOAMI, &who) == 0)
            fprintf(stderr, "WHO_AM_I = 0x%02X\n", (unsigned)(who & 0xFF));
        else
            perror("ioctl GET_WHOAMI");
    }

    // Optional calibration (userspace only; no driver changes)
    struct biases bias = {0};
    if (cal_ms > 0) {
        fprintf(stderr, "Calibrating for %u ms. Keep device still...\n", cal_ms);
        if (calibrate_stationary(fd, cal_ms, g_lsb_x, g_lsb_y, g_lsb_z, &bias) == 0) {
            fprintf(stderr, "Bias accel (LSB): [%.2f %.2f %.2f], gyro (LSB): [%.2f %.2f %.2f]\n",
                    bias.ax, bias.ay, bias.az, bias.gx, bias.gy, bias.gz);
        } else {
            fprintf(stderr, "Calibration failed (no samples read).\n");
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

    const double ALSB = accel_lsb_per_g(accel_fs);
    const double GLSB = gyro_lsb_per_dps(gyro_fs);
    const double g = 9.80665, d2r = 3.14159265358979323846/180.0;

    signal(SIGINT, on_sigint);

    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    time_t start_sec = next.tv_sec;

    while (!g_stop) {
        struct mpu6050_sample s;
        int r = read_one(fd, &s);
        if (r < 0) { perror("read"); break; }
        if (r > 0) {
            // bias-corrected (still raw LSB)
            double axc = (double)s.ax - bias.ax;
            double ayc = (double)s.ay - bias.ay;
            double azc = (double)s.az - bias.az;
            double gxc = (double)s.gx - bias.gx;
            double gyc = (double)s.gy - bias.gy;
            double gzc = (double)s.gz - bias.gz;

            if (print_si) {
                double ax_ms2 = (axc / ALSB) * g;
                double ay_ms2 = (ayc / ALSB) * g;
                double az_ms2 = (azc / ALSB) * g;
                double gx_rs  = (gxc / GLSB) * d2r;
                double gy_rs  = (gyc / GLSB) * d2r;
                double gz_rs  = (gzc / GLSB) * d2r;

                printf("%lld,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                       SAMPLE_TS(s), s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp,
                       axc, ayc, azc, gxc, gyc, gzc,
                       ax_ms2, ay_ms2, az_ms2, gx_rs, gy_rs, gz_rs);
            } else {
                printf("%lld,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                       SAMPLE_TS(s), s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp,
                       axc, ayc, azc, gxc, gyc, gzc);
            }
            fflush(stdout);
        }

        sleep_until_next_tick(&next, period_ms);
        if (duration_s && (unsigned)(next.tv_sec - start_sec) >= duration_s) break;
    }

    close(fd);
    return 0;
}
