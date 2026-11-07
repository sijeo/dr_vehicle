/*
 * test_neo6m_gnss_serdev.c - Userspace test tool for neo6m_gnss_serdev
 *
 * Opens /dev/neo6m0, polls the ioctl snapshot (v2 if the module supports it,
 * else v1) and prints every update, including the v2 freshness counters
 * (pos_seq / vel_seq) that the DR application relies on.
 *
 * "Inappropriate ioctl for device" (ENOTTY) means the loaded module was built
 * from a different neo6m_gnss_ioctl.h than this program: the ioctl numbers
 * encode the struct sizes. Compare the numbers printed at start with the
 * "ioctl ABI:" line in dmesg, then rebuild/reload the module (make in
 * platform/pi builds it) and rebuild this tool from the same tree.
 *
 * Build:
 *   gcc -O2 -Wall -Wextra -o test_neo6m_gnss_serdev test_neo6m_gnss_serdev.c
 *
 * Run:
 *   sudo ./test_neo6m_gnss_serdev [/dev/neo6m0]
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

static void print_fix(const struct neo6m_gnss_fix_v2 *f, bool v2)
{
    printf("fix=%d lat=%.7f lon=%.7f alt=%.3f m spd=%.3f m/s "
           "UTC=%04u-%02u-%02u %02u:%02u:%02u.%03u  HDOP=%s%.2f  Course=%s%.5f",
           (int)f->have_fix,
           f->lat_e7 / 1e7, f->lon_e7 / 1e7, f->alt_mm / 1000.0, f->speed_mmps / 1000.0,
           (unsigned)f->utc_year, (unsigned)f->utc_mon, (unsigned)f->utc_day,
           (unsigned)f->utc_hour, (unsigned)f->utc_min, (unsigned)f->utc_sec,
           (unsigned)f->utc_millis,
           f->hdop_valid ? "" : "NA/", f->hdop_x100 / 100.0,
           f->heading_valid ? "" : "NA/", f->course_deg_e5 / 1e5);
    if (v2)
        printf("  pos_seq=%u%s vel_seq=%u%s nmea_seq=%u",
               (unsigned)f->pos_seq, f->pos_valid ? "" : "(invalid)",
               (unsigned)f->vel_seq, f->vel_valid ? "" : "(invalid)", (unsigned)f->nmea_seq);
    printf("\n");
    fflush(stdout);
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/neo6m0";
    struct neo6m_gnss_fix_v2 f, prev;
    bool v2 = true, had_fix = false;
    int fd;

    signal(SIGINT, on_sigint);
    printf("ioctl ABI of this build: GET_FIX=0x%08lx (%zu B)  GET_FIX_V2=0x%08lx (%zu B)\n",
           (unsigned long)NEO6M_GNSS_IOC_GET_FIX, sizeof(struct neo6m_gnss_fix),
           (unsigned long)NEO6M_GNSS_IOC_GET_FIX_V2, sizeof(struct neo6m_gnss_fix_v2));

    fd = open(dev, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        perror(dev);
        return 1;
    }

    memset(&f, 0, sizeof(f));
    if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX_V2, &f) < 0 && errno == ENOTTY) {
        struct neo6m_gnss_fix f1;
        v2 = false;
        if (ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, &f1) < 0 && errno == ENOTTY) {
            fprintf(stderr,
                "\nBoth ioctls rejected (ENOTTY): the loaded neo6m_gnss_serdev.ko was built from a different\n"
                "neo6m_gnss_ioctl.h. Check: dmesg | grep 'ioctl ABI'  (missing line = module older than\n"
                "this fix). Rebuild the module (cd platform/pi && make), then:\n"
                "  sudo rmmod neo6m_gnss_serdev && sudo insmod neo6m_gnss_serdev.ko\n");
            close(fd);
            return 2;
        }
    }
    printf("NEO6M driver test started using the %s ioctl (Ctrl+C to stop)\n", v2 ? "v2" : "v1 (old module)");

    memset(&prev, 0, sizeof(prev));
    while (g_run) {
        int rc;
        memset(&f, 0, sizeof(f));
        if (v2) {
            rc = ioctl(fd, NEO6M_GNSS_IOC_GET_FIX_V2, &f);
        } else {
            struct neo6m_gnss_fix f1;
            rc = ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, &f1);
            memcpy(&f, &f1, sizeof(f1));             /* v1 fields are a prefix of v2 */
        }
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            perror("ioctl");
            break;
        }

        if (!f.have_fix) {
            static int tick = 0;
            if ((tick++ % 4) == 0) {
                printf("No fix acquired yet...%s\r", v2 ? "" : " (old module: no seq counters)");
                fflush(stdout);
            }
            had_fix = false;
        } else {
            if (!had_fix) {
                printf("\nGNSS fix acquired!\n");
                had_fix = true;
            }
            if (memcmp(&f, &prev, sizeof(f)) != 0) {     /* print only when the snapshot changes */
                print_fix(&f, v2);
                prev = f;
            }
        }
        usleep(250000);
    }

    printf("\nStopping.\n");
    close(fd);
    return 0;
}
