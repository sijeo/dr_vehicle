#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include "../neo6m_gnss_ioctl.h"

static volatile int keep_running = 1;

/**
 * @brief Signal handler for Ctrl+C (SIGINT)
 */
static void sigint_handler(int signo)
{
    (void)signo;
    keep_running = 0;
}

/**
 * @brief Compare two fix structures to detect change.
 * @return 1 if different enough to reprint, 0 otherwise.
 */
static int fix_changed(const struct neo6m_gnss_fix *a,
                       const struct neo6m_gnss_fix *b)
{
    return memcmp(a, b, sizeof(*a)) != 0;
}

int main(void)
{
    int fd;
    struct neo6m_gnss_fix fix = {0};
    struct neo6m_gnss_fix prev_fix = {0};
    int had_fix = 0; /* track if we've had a fix before */

    signal(SIGINT, sigint_handler);

    fd = open("/dev/neo6m0", O_RDONLY);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    printf("Starting GNSS monitor — press Ctrl+C to stop\n");

    while (keep_running) {
        /* Request latest fix from kernel driver */
        if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, &fix) < 0) {
            if (errno == EINTR)
                continue; /* interrupted by signal */
            perror("ioctl");
            break;
        }

        /* If we have a fix, print it once or when it changes */
        if (fix.have_fix) {
            if (!had_fix) {
                printf("\n✅ GNSS fix acquired!\n");
                had_fix = 1;
            }

            if (fix_changed(&fix, &prev_fix)) {
                printf("fix=%d lat=%.7f lon=%.7f alt=%.3f m speed=%.3f m/s "
                       "UTC=%04u-%02u-%02u %02u:%02u:%02u.%03u\n",
                       fix.have_fix,
                       fix.lat_e7 / 1e7,
                       fix.lon_e7 / 1e7,
                       fix.alt_mm / 1000.0,
                       fix.speed_mmps / 1000.0,
                       fix.utc_year, fix.utc_mon, fix.utc_day,
                       fix.utc_hour, fix.utc_min, fix.utc_sec, fix.utc_millis);
                fflush(stdout);
                prev_fix = fix;
            }
        } else {
            /* No fix yet — print once every few cycles to avoid spam */
            static int counter = 0;
            if ((counter++ % 4) == 0) {
                printf("No fix acquired yet...\r");
                fflush(stdout);
            }
        }

        /* Sleep half a second (GNSS updates ~1 Hz) */
        usleep(500000);
    }

    printf("\nStopping GNSS monitor.\n");
    close(fd);
    return 0;
}

