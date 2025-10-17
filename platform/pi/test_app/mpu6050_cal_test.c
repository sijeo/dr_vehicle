// SPDX-License-Identifier: MIT
// Simple raw MPU6050 reader for mpu6050_char driver
// Works in on-demand snapshot mode (default) or IRQ FIFO mode.

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include "mpu6050_ioctl.h"   // must match your driver’s UAPI

static volatile int stop = 0;
static void handle_sigint(int sig) { (void)sig; stop = 1; }

int main(int argc, char *argv[])
{
    const char *dev = (argc > 1) ? argv[1] : "/dev/mpu6050-0";
    unsigned interval_ms = 100;  // default 10 Hz
    if (argc > 2) interval_ms = atoi(argv[2]);

    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    printf("Reading MPU6050 raw data from %s every %u ms (Ctrl+C to stop)\n",
           dev, interval_ms);
    printf("  %-8s %-8s %-8s | %-8s %-8s %-8s | %-8s\n",
           "ax", "ay", "az", "gx", "gy", "gz", "temp");

    signal(SIGINT, handle_sigint);

    struct mpu6050_sample s;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!stop) {
        ssize_t n = read(fd, &s, sizeof(s));
        if (n != sizeof(s)) {
            if (errno == EAGAIN) continue;
            perror("read");
            break;
        }

        // Convert raw temperature to °C (optional)
        // Datasheet: Temp_in_C = (TEMP_OUT / 340) + 36.53
        double temp_c = ((double)s.temp / 340.0) + 36.53;

        printf("%8d %8d %8d | %8d %8d %8d | %8.2f °C\n",
               s.ax, s.ay, s.az, s.gx, s.gy, s.gz, temp_c);
        fflush(stdout);

        // sleep until next period (absolute to avoid drift)
        next.tv_nsec += interval_ms * 1000000L;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    close(fd);
    printf("\nStopped.\n");
    return 0;
}
