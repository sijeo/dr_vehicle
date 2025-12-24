/*
 * neo6m_driver_test.c - Userspace test tool for neo6m_gnss_serdev
 *
 * Opens /dev/neo6m0, polls ioctl snapshot, prints every update.
 * Prints "No fix acquired yet..." until have_fix==1.
 *
 * Build:
 *   gcc -O2 -Wall -Wextra -o neo6m_driver_test neo6m_driver_test.c
 *
 * Run:
 *   sudo ./neo6m_driver_test
 */

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "../neo6m_gnss_ioctl.h"

static volatile sig_atomic_t g_run = 1;

static void on_sigint(int sig)
{
    (void)sig;
    g_run = 0;
}

static void print_fix(const struct neo6m_gnss_fix *f)
{
    const double lat = f->lat_e7 / 1e7;
    const double lon = f->lon_e7 / 1e7;
    const double alt_m = f->alt_mm / 1000.0;
    const double spd = f->speed_mmps / 1000.0;

    const double hdop = f->hdop_x100 / 100.0;
    const double heading = f->course_deg_e5 / 1e5;

    printf("fix=%d lat=%.7f lon=%.7f alt=%.3f m spd=%.3f m/s "
           "UTC=%04u-%02u-%02u %02u:%02u:%02u.%03u  "
           "HDOP=%s%.2f  Heading=%s%.5f\n",
           (int)f->have_fix,
           lat, lon, alt_m, spd,
           (unsigned)f->utc_year, (unsigned)f->utc_mon, (unsigned)f->utc_day,
           (unsigned)f->utc_hour, (unsigned)f->utc_min, (unsigned)f->utc_sec,
           (unsigned)f->utc_millis,
           f->hdop_valid ? "" : "NA/",
           hdop,
           f->heading_valid ? "" : "NA/",
           heading);
    fflush(stdout);
}

int main(void)
{
    signal(SIGINT, on_sigint);

    int fd = open("/dev/neo6m0", O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        perror("open(/dev/neo6m0)");
        return 1;
    }

    printf("NEO6M driver test started (Ctrl+C to stop)\n");

    struct neo6m_gnss_fix f = {0};
    struct neo6m_gnss_fix prev = {0};

    bool had_fix = false;

    while (g_run) {
        if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, &f) < 0) {
            if (errno == EINTR)
                continue;
            perror("ioctl(NEO6M_GNSS_IOC_GET_FIX)");
            break;
        }

        if (!f.have_fix) {
            /* Print a non-spammy status line */
            static int tick = 0;
            if ((tick++ % 4) == 0) {
                printf("No fix acquired yet...\r");
                fflush(stdout);
            }
            had_fix = false;
        } else {
            if (!had_fix) {
                printf("\n✅ GNSS fix acquired!\n");
                had_fix = true;
            }

            /* Print only when snapshot changes */
            if (memcmp(&f, &prev, sizeof(f)) != 0) {
                print_fix(&f);
                prev = f;
            }
        }

        usleep(250000); /* 250 ms poll */
    }

    printf("\nStopping.\n");
    close(fd);
    return 0;
}
