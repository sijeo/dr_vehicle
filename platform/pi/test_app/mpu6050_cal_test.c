// gcc -O2 -Wall -o mpu6050_read_100ms tools/mpu6050_read_100ms.c
// Example:
//   sudo ./mpu6050_read_100ms /dev/mpu6050-0 --odr 200 --period_ms 100 
//        --cal_ms 3000 --g_raw 0 0 16384 --commit_cal
// Notes:
//   * This app owns the 100 ms tick. Driver ODR can be higher (e.g., 200 Hz) for low-jitter latest-sample reads.
//   * Calibration is done in userspace by averaging stationary samples and then committing via SET_CAL.

#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "../mpu6050_ioctl.h"

static void sleep_until_next_tick(struct timespec *t, uint32_t period_ms) 
{
    t->tv_nsec += (long)period_ms * 1000000L;
    while (t->tv_nsec >= 1000000000L) { 
        t->tv_nsec -= 1000000000L; 
        t->tv_sec += 1; 
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t, NULL);
}

static int drain_latest(int fd, struct mpu6050_sample *out) 
{
    struct mpu6050_sample s; 
    int got = 0;
    
    for (;;) {
        ssize_t r = read(fd, &s, sizeof(s));
        if (r == (ssize_t)sizeof(s)) { 
            *out = s; 
            got = 1; 
            continue; 
        }
        if (r < 0) break; // EAGAIN => empty
        break;            // anything else => stop
    }
    return got; // 1 if we captured at least one sample
}

static int calibrate_stationary_userspace(int fd, uint32_t cal_ms, float graw[3], struct mpu6050_cal_pair *out)
{
    // Ensure device is nonblocking and driver is sampling (set ODR beforehand)
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    double ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0; 
    uint64_t n = 0;
    struct pollfd p = { .fd = fd, .events = POLLIN };
    struct mpu6050_sample s;

    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (long)((now.tv_sec - start.tv_sec) * 1000L + (now.tv_nsec - start.tv_nsec) / 1000000L);
        if (elapsed_ms >= (long)cal_ms) break;
        if (poll(&p, 1, 50) <= 0) continue;
        
        while (read(fd, &s, sizeof(s)) == (ssize_t)sizeof(s)) {
            ax += s.ax; 
            ay += s.ay; 
            az += s.az;
            gx += s.gx; 
            gy += s.gy; 
            gz += s.gz;
            n++;
        }
    }
    
    if (n == 0) return -1;

    ax /= (double)n; 
    ay /= (double)n; 
    az /= (double)n;
    gx /= (double)n; 
    gy /= (double)n; 
    gz /= (double)n;

    memset(out, 0, sizeof(*out));
    out->gyro.bias[0] = (float)gx; 
    out->gyro.bias[1] = (float)gy; 
    out->gyro.bias[2] = (float)gz;
    out->gyro.scale[0] = out->gyro.scale[1] = out->gyro.scale[2] = 1.0f;

    out->accel.bias[0] = (float)(ax - graw[0]);
    out->accel.bias[1] = (float)(ay - graw[1]);
    out->accel.bias[2] = (float)(az - graw[2]);
    out->accel.scale[0] = out->accel.scale[1] = out->accel.scale[2] = 1.0f;
    
    return 0;
}

int main(int argc, char **argv)
{
    struct mpu6050_sample s;
     uint32_t odr = 200;        // driver sampling Hz
    uint32_t period_ms = 100;  // our app tick
    uint32_t cal_ms = 0;       // 0 => no calibration
    float graw[3] = {0, 0, 0}; // expected stationary gravity in RAW LSB
    int commit_cal = 0;
    int i, fd;
     struct mpu6050_cal_pair cal;
     struct timespec next; 

    if (argc < 2) {
        fprintf(stderr, "Usage: %s /dev/mpu6050-0 [--odr N] [--period_ms M] [--cal_ms K] [--g_raw gx gy gz] [--commit_cal]\n", argv[0]);
        return 1;
    }

    for (i = 2; i < argc; ++i) {
        if (!strcmp(argv[i], "--odr") && i + 1 < argc) {
            odr = (uint32_t)atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--period_ms") && i + 1 < argc) {
            period_ms = (uint32_t)atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--cal_ms") && i + 1 < argc) {
            cal_ms = (uint32_t)atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--g_raw") && i + 3 < argc) {
            graw[0] = strtof(argv[++i], NULL);
            graw[1] = strtof(argv[++i], NULL);
            graw[2] = strtof(argv[++i], NULL);
        } else if (!strcmp(argv[i], "--commit_cal")) {
            commit_cal = 1;
        }
    }

    fd = open(argv[1], O_RDONLY | O_NONBLOCK);
    if (fd < 0) { 
        perror("open"); 
        return 1; 
    }

    if (ioctl(fd, MPU6050_IOC_SET_ODR, &odr) != 0) {
        perror("ioctl SET_ODR");
    }

    // Optional: run userspace stationary calibration
    if (cal_ms > 0) {
        fprintf(stderr, "Calibrating for %u ms. Keep device still...\n", cal_ms);
       
        
        if (calibrate_stationary_userspace(fd, cal_ms, graw, &cal) == 0) {
            fprintf(stderr, "Cal done. Accel bias = [%.1f %.1f %.1f], Gyro bias = [%.1f %.1f %.1f]\n",
                cal.accel.bias[0], cal.accel.bias[1], cal.accel.bias[2],
                cal.gyro.bias[0], cal.gyro.bias[1], cal.gyro.bias[2]);
                
            if (commit_cal) {
                if (ioctl(fd, MPU6050_IOC_SET_CAL, &cal) != 0) {
                    perror("ioctl SET_CAL");
                } else {
                    fprintf(stderr, "Calibration committed to driver.\n");
                }
            }
        } else {
            fprintf(stderr, "Calibration failed (no samples).\n");
        }
    }

    printf("ts_ns,ax,ay,az,gx,gy,gz,temp,ax_corr,ay_corr,az_corr,gx_corr,gy_corr,gz_corr\n");

    
    clock_gettime(CLOCK_MONOTONIC, &next);
    
    for (;;) {
        memset(&s, 0, sizeof(s));
        if (drain_latest(fd, &s)) {
            printf("%lld,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                (long long)s.t_ns,
                s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp,
                s.ax_corr, s.ay_corr, s.az_corr, s.gx_corr, s.gy_corr, s.gz_corr);
            fflush(stdout);
        }
        sleep_until_next_tick(&next, period_ms);
    }
}
