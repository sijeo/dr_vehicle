// SPDX-License-Identifier: GPL-2.0
/*
 * imulog_to_csv.c
 *
 * Converts an imulog_*.bin file (produced by dr_dead_reckoning_app_ins15_v3)
 * into a pandas-friendly CSV. Self-contained — no dependency on the main
 * app or the kernel headers. The .bin format definitions here MUST stay in
 * sync with the runtime structs in dr_dead_reckoning_app_ins15_v3.c; both
 * are versioned via IMULOG_VERSION.
 *
 * Build (host or Pi):
 *     gcc -O2 -Wall -Wextra -o imulog_to_csv imulog_to_csv.c
 *
 * Usage:
 *     ./imulog_to_csv <input.bin> [output.csv]
 *
 * If output.csv is omitted, writes to stdout.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#define IMULOG_MAGIC          "IMULOG01"
#define IMULOG_VERSION        1u

/* Kept in one-to-one sync with dr_dead_reckoning_app_ins15_v3.c */
struct __attribute__((packed)) imulog_header {
    char     magic[8];
    uint32_t version;
    uint32_t record_size;
    uint32_t sample_rate_hz;
    uint32_t reserved0;
    int64_t  wall_clock_ns;
    int64_t  monotonic_ns_at_open;
    float    accel_range_g;
    float    gyro_range_dps;
    float    accel_lsb_per_g;
    float    gyro_lsb_per_dps;
    char     imu_chip[16];
    char     reserved1[48];
};

struct __attribute__((packed)) imulog_record {
    int64_t  t_ns;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int16_t  temp;
    uint16_t flags;
    float    ax_mps2, ay_mps2, az_mps2;
    float    gx_rps,  gy_rps,  gz_rps;
    float    ekf_speed_mps;
    float    ekf_yaw_rad;
    float    ekf_yaw_rate_rps;
    float    gnss_fix_age_s;
    float    gnss_speed_mps;
    uint8_t  label_hint;
    uint8_t  reserved[3];
};

/* Compile-time size guard — struct layout drift is the #1 way this tool
 * silently produces garbage. Update these values AND bump IMULOG_VERSION
 * if you change either struct. */
_Static_assert(sizeof(struct imulog_header) == 120,
               "imulog_header must be 120 bytes");
_Static_assert(sizeof(struct imulog_record) ==  72,
               "imulog_record must be 72 bytes");

static const char *label_name(uint8_t lbl) {
    switch (lbl) {
    case 0:   return "STATIONARY";
    case 1:   return "CRUISING";
    case 2:   return "TURNING";
    case 3:   return "ACCEL_BRAKE";
    case 4:   return "ROUGH_ROAD";
    case 255: return "UNKNOWN";
    default:  return "INVALID";
    }
}

static void print_header_summary(const struct imulog_header *h, FILE *stream) {
    char chip[17] = {0};
    memcpy(chip, h->imu_chip, sizeof(h->imu_chip));
    fprintf(stream,
        "# imulog v%u  chip=%s  rate=%u Hz  record_size=%u B\n"
        "# accel: ±%.1fg (%.3f LSB/g)   gyro: ±%.0f dps (%.3f LSB/dps)\n"
        "# wall_clock_ns=%lld  monotonic_ns_at_open=%lld\n",
        h->version, chip, h->sample_rate_hz, h->record_size,
        (double)h->accel_range_g, (double)h->accel_lsb_per_g,
        (double)h->gyro_range_dps, (double)h->gyro_lsb_per_dps,
        (long long)h->wall_clock_ns, (long long)h->monotonic_ns_at_open);
}

int main(int argc, char **argv) {
    if (argc < 2 || argc > 3) {
        fprintf(stderr, "usage: %s <input.bin> [output.csv]\n", argv[0]);
        return 2;
    }

    FILE *in = fopen(argv[1], "rb");
    if (!in) {
        fprintf(stderr, "open %s: %s\n", argv[1], strerror(errno));
        return 1;
    }

    struct imulog_header h;
    if (fread(&h, sizeof(h), 1, in) != 1) {
        fprintf(stderr, "read header: %s\n", strerror(errno));
        fclose(in);
        return 1;
    }
    if (memcmp(h.magic, IMULOG_MAGIC, 8) != 0) {
        fprintf(stderr, "bad magic (not an imulog file)\n");
        fclose(in);
        return 1;
    }
    if (h.version != IMULOG_VERSION) {
        fprintf(stderr, "version mismatch: file=%u expected=%u\n",
                h.version, IMULOG_VERSION);
        fclose(in);
        return 1;
    }
    if (h.record_size != sizeof(struct imulog_record)) {
        fprintf(stderr, "record_size mismatch: file=%u expected=%zu — "
                "rebuild this tool against a matching header\n",
                h.record_size, sizeof(struct imulog_record));
        fclose(in);
        return 1;
    }

    FILE *out = stdout;
    if (argc == 3) {
        out = fopen(argv[2], "w");
        if (!out) {
            fprintf(stderr, "open %s: %s\n", argv[2], strerror(errno));
            fclose(in);
            return 1;
        }
    }

    /* Preserve the header metadata as CSV comment lines — most CSV readers
     * (pandas read_csv with comment='#') ignore these. */
    print_header_summary(&h, out);

    fprintf(out,
        "t_ns,ax,ay,az,gx,gy,gz,temp,"
        "gnss_valid,gnss_fresh,nav_ready,have_fix,"
        "ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps,"
        "ekf_speed_mps,ekf_yaw_rad,ekf_yaw_rate_rps,"
        "gnss_fix_age_s,gnss_speed_mps,"
        "label_hint,label_name\n");

    struct imulog_record r;
    unsigned long n = 0;
    while (fread(&r, sizeof(r), 1, in) == 1) {
        fprintf(out,
            "%lld,"
            "%d,%d,%d,%d,%d,%d,%d,"
            "%u,%u,%u,%u,"
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.4f,%.6f,%.6f,"
            "%.3f,%.3f,"
            "%u,%s\n",
            (long long)r.t_ns,
            r.ax, r.ay, r.az, r.gx, r.gy, r.gz, r.temp,
            (unsigned)((r.flags >> 0) & 1u),
            (unsigned)((r.flags >> 1) & 1u),
            (unsigned)((r.flags >> 2) & 1u),
            (unsigned)((r.flags >> 3) & 1u),
            (double)r.ax_mps2, (double)r.ay_mps2, (double)r.az_mps2,
            (double)r.gx_rps,  (double)r.gy_rps,  (double)r.gz_rps,
            (double)r.ekf_speed_mps,
            (double)r.ekf_yaw_rad,
            (double)r.ekf_yaw_rate_rps,
            (double)r.gnss_fix_age_s,
            (double)r.gnss_speed_mps,
            (unsigned)r.label_hint, label_name(r.label_hint));
        n++;
    }

    fprintf(stderr, "imulog_to_csv: %lu records converted\n", n);

    if (out != stdout) fclose(out);
    fclose(in);
    return 0;
}
