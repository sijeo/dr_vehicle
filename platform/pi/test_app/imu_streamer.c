/*
 * imu_streamer.c — ISM330BX test streamer (Raspberry Pi -> PC over UDP)
 * =====================================================================
 *
 * Reads /dev/mpu6050-0 (ISM330BX behind mpu6050_char.ko), runs the IMU
 * pipeline, and sends one UDP frame per sample to a PC dashboard. It exists
 * to prove that the IMU readings are right before the algorithm moves into
 * the DR app.
 *
 * Data path
 * ---------
 *   chip (on-chip filters + SFLP) -> HW FIFO -> driver (extended read format)
 *     -> imu_core   : accel scale (accel_C/O), sensor->vehicle FLU mount,
 *                     boot gyro-bias calibration, median + LPF, quality flags
 *     -> imu_fusion : host Mahony AHRS (primary attitude), SFLP decode and
 *                     consistency check, gravity removal (vehicle + level frame)
 *     -> UDP frame v2 (format version 3) or v1 for the old dashboard
 *   The same UDP socket also receives control commands from the dashboard
 *   (mount calibration, see below).
 *
 *   SFLP = the ISM330BX "sensor fusion low power" block. The chip fuses accel
 *   + gyro into a game rotation vector (quaternion, yaw origin arbitrary, no
 *   magnetometer), a gravity vector and a gyro-bias estimate; the driver
 *   attaches the latest values to every accel/gyro sample.
 *   Status from the 2026-10-27 recordings:
 *     - SFLP gyro bias: correct (matches the boot calibration to 0.004 dps).
 *     - SFLP gravity: correct once read in Z,Y,X order and negated (done in
 *       imu_fusion.c).
 *     - SFLP quaternion: follows the gyro but its gravity reference was ~90 deg
 *       off. imu_fusion therefore checks it continuously ("SFLP vs gravity" on
 *       the dashboard); above 10 deg it is flagged INCONSISTENT and the host
 *       AHRS is used for gravity removal instead.
 *
 *   Frames: vehicle FLU (x forward, y left, z up). Angles follow REP-103:
 *   roll + = right side down, pitch + = nose down, yaw + = left turn.
 *   After 2 s at rest the streamer prints "[FLU check]": gravity must be on +z.
 *
 * Mount (sensor -> vehicle) — order of precedence
 * ------------------------------------------------
 *   1. --mount / --mount-rpy on the command line (the mount file is ignored).
 *   2. ./imu_mount.txt (or --mount-file), written by the dashboard's mount
 *      calibration wizard.
 *   3. FLU identity (the board is mounted FLU).
 *   Guided calibration: imu_rotation_dashboard.py starts a wizard at launch
 *   (or press 'm') that prompts LEVEL -> TURN 180 -> FORWARD (tip nose down, or
 *   drive forward). It sends the correction here as
 *       IMUCTL MOUNT_DELTA r00 r01 r02 r10 r11 r12 r20 r21 r22
 *   The streamer applies R_bv <- delta * R_bv live, saves it to the mount file,
 *   re-stamps the boot-calibration file so it stays valid, and replies
 *   "IMUACK MOUNT_DELTA OK ...". Other commands: MOUNT_RESET (back to the
 *   --mount preset), GET_MOUNT, SFLP_RESET. Only packets from --host are
 *   accepted. Re-run the wizard whenever the board is moved or remounted.
 *
 * Which dashboard
 * ---------------
 *   imu_rotation_dashboard.py  needs --wire v2 (the default). Mount wizard,
 *                              AHRS + SFLP cubes, attitude, bias-corrected gyro,
 *                              gravity-free level-frame accel, FIFO health;
 *                              records every sample to CSV and can --replay it.
 *   imu_dashboard.py           (older) needs --wire v1. Calibration progress,
 *                              raw/filtered accel+gyro, noise statistics; it
 *                              cannot show SFLP/fusion output or calibrate the mount.
 *   Both listen on UDP 5005 by default; run only one per port, and allow
 *   inbound UDP 5005 through the PC firewall.
 *
 * Options (default -> recommended)
 * --------------------------------
 *   --host IP          PC address (required). Control packets are accepted
 *                      only from this address.
 *   --port N           5005. Must match the dashboard's --port.
 *   --device PATH      /dev/mpu6050-0.
 *   --cal-file PATH    ./imu_calibration.bin (boot gyro-bias calibration).
 *                      Reused only if the config CRC matches; changing FS,
 *                      rate, mount (from the command line) or cutoff forces
 *                      a fresh calibration.
 *   --recalibrate      ignore the saved calibration. Keep the IMU completely
 *                      still until the state reaches VALID (about 22 s).
 *   --rate HZ          default = driver ODR (240). Only override for a
 *                      non-standard ODR; it must match the chip.
 *   --cutoff HZ        6. Software LPF after the median filter. Keep 5-10 Hz
 *                      for vehicle dynamics; must be < 0.45 x rate.
 *   --gyro-fs DPS      unchanged (DT gives 2000). Recommended: 500. At 2000
 *                      one LSB is 70 mdps, coarser than the gyro noise; 500
 *                      gives 17.5 mdps and still covers vehicle rates.
 *                      DRIVER-WIDE: any other reader (DR app) must use the
 *                      same LSB/dps while this is set.
 *   --gyro-lpf N       driver default 6. On-chip gyro LPF1 BW code, -1 = off.
 *                      @240 Hz ODR: 0..3 = 96 Hz, 4 = 78, 5 = 53, 6 = 27.3,
 *                      7 = 14.2 Hz. Recommended 6; 7 on rough roads.
 *   --accel-lpf N      driver default 2. On-chip accel LPF2, -1 = off.
 *                      0..7 = ODR/4, /10, /20, /45, /100, /200, /400, /800.
 *                      Recommended 2 (12 Hz @240 Hz); 3 (5.3 Hz) when engine
 *                      vibration is strong. Lower = more delay.
 *   --sflp-odr HZ      driver default 120. One of 15/30/60/120/240/480,
 *                      clamped to <= ODR. Recommended 120.
 *   --sflp-mask M      driver default 7. bit0 quaternion, bit1 gravity,
 *                      bit2 gyro bias. Recommended 7 at 400 kHz I2C. If the
 *                      dashboard still shows FIFO OVERRUN, use 6 (drop the
 *                      quaternion, the least reliable output) or 4.
 *   --no-sflp          turn SFLP off (host AHRS only). For A/B comparison.
 *   --no-fifo          register polling instead of the HW FIFO. Debug only:
 *                      polling can lose samples when the reader is late.
 *   --mount flu|frd    flu (identity). The ISM330BX here is mounted FLU.
 *                      Use frd only for a board installed z-down / y-right.
 *                      The wizard corrects a wrong preset anyway.
 *   --mount-rpy R,P,Y  0,0,0. Fixed mount trim in degrees (overrides the
 *                      mount file). Prefer the dashboard wizard.
 *   --mount-file PATH  ./imu_mount.txt. Where the wizard result is kept.
 *   --ahrs-kp K        0.5. Mahony accel-correction gain. 0.2-0.5 in a moving
 *                      vehicle (less pulled by acceleration), up to 1.0 on
 *                      a bench.
 *   --ahrs-ki K        0.01. Mahony integral (residual gyro bias); 0 = off.
 *   --wire v1|v2       v2. v2 for imu_rotation_dashboard.py, v1 for
 *                      imu_dashboard.py.
 *
 *   On-chip settings (--gyro-lpf, --accel-lpf, --sflp-*, --no-sflp,
 *   --no-fifo, --gyro-fs) change the driver and persist after the streamer
 *   exits, until changed again or the module is reloaded. They can also be
 *   set in /sys/class/mpu6050/mpu6050-0/ or as insmod parameters.
 *   FIFO throughput: the driver reads 4 FIFO words per I2C transfer
 *   (fifo_burst_words=4) and needs dtparam=i2c_arm_baudrate=400000. If
 *   overruns persist, check the bus speed (README).
 *
 * Examples
 * --------
 *   # 1) Recommended: start the dashboard, then the streamer; follow the
 *   #    on-screen mount calibration (LEVEL -> TURN 180 -> FORWARD, ENTER)
 *   PC : python imu_rotation_dashboard.py --csv bench.csv
 *   Pi : ./imu_streamer --host 192.168.68.100 --gyro-fs 500
 *
 *   # 2) Fresh gyro-bias calibration after a temperature change or new FS
 *   Pi : ./imu_streamer --host 192.168.68.100 --gyro-fs 500 --recalibrate
 *
 *   # 3) Rough road / engine running: narrower on-chip filters, softer AHRS
 *   Pi : ./imu_streamer --host 192.168.68.100 --gyro-fs 500 \
 *            --gyro-lpf 7 --accel-lpf 3 --ahrs-kp 0.3
 *
 *   # 4) Dashboard without the start-up wizard (mount already calibrated)
 *   PC : python imu_rotation_dashboard.py --no-wizard
 *
 *   # 5) Fixed trim from the command line instead of the wizard
 *   Pi : ./imu_streamer --host 192.168.68.100 --gyro-fs 500 --mount-rpy 1.20,-0.85,0.40
 *
 *   # 6) A/B: host AHRS only (SFLP cube greys out)
 *   Pi : ./imu_streamer --host 192.168.68.100 --no-sflp
 *
 *   # 7) Legacy dashboard (noise / calibration view, no SFLP)
 *   PC : python imu_dashboard.py --port 5005 --csv noise.csv
 *   Pi : ./imu_streamer --host 192.168.68.100 --wire v1
 *
 *   # 8) Offline review of a recording (no Pi needed)
 *   PC : python imu_rotation_dashboard.py --replay bench.csv --speed 2
 *
 * What "correct" looks like
 * -------------------------
 *   [FLU check] OK, |g| within 1 %; after the wizard: level -> roll/pitch ~0,
 *   nose down -> pitch +, right side down -> roll +, left turn -> yaw +;
 *   rate ~240 Hz with "driver dropped" not increasing and no FIFO OVERRUN;
 *   at rest the level-frame accel stays within about ±0.03 m/s² (see also
 *   /sys/class/mpu6050/mpu6050-0/stream_stats).
 */

#include "imu_core.h"
#include "imu_fusion.h"
#include "imu_wire.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>



/* This is the existing kernel-driver ABI used by the DR application. */
#include "../mpu6050_ioctl.h"

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

static uint64_t monotonic_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void usage(const char *prog){
    fprintf(stderr,
        "Usage: %s --host RECEIVER IP [options]\n "
    " --host IP         UDP receiver address (required)\n "
    " --port N          UDP port, default 5005\n "
    " --device PATH     IMU device, default /dev/mpu6050-0\n"
    " --cal-file PATH   calibration blob, default ./imu_calibration.bin\n"
    " --recalibrate     ignore saved calibration and run stationary boot calibration\n"
    " --rate HZ         expected IMU rate, default = driver ODR\n"
    " --cutoff HZ       LPF cutoff, default 6Hz\n"
    " --gyro-fs DPS     set gyro full scale 250/500/1000/2000 (driver-wide!)\n"
    " --gyro-lpf N      on-chip gyro LPF1 bandwidth 0..7, -1 = off\n"
    " --accel-lpf N     on-chip accel LPF2 0..7 (ODR/4,10,20,45,100,200,400,800), -1 = off\n"
    " --sflp-odr HZ     SFLP rate 15/30/60/120/240/480\n"
    " --sflp-mask M     SFLP FIFO outputs: bit0 quat, bit1 gravity, bit2 gyro bias\n"
    " --no-sflp         disable the on-chip SFLP fusion\n"
    " --no-fifo         register polling instead of the HW FIFO stream\n"
    " --mount flu|frd   sensor axes vs vehicle FLU frame, default flu (identity)\n"
    " --mount-rpy R,P,Y mount trim in degrees from field calibration (ZYX, REP-103)\n"
    " --mount-file PATH mount from the dashboard calibration, default ./imu_mount.txt\n"
    " --ahrs-kp K       host Mahony proportional gain, default 0.5\n"
    " --ahrs-ki K       host Mahony integral gain, default 0.01\n"
    " --wire v1|v2      UDP frame format, default v2 (v1 = legacy imu_dashboard.py)\n",
    prog);
}

static const char *fs_enum_name_gyro(unsigned e) {
    static const char *n[] = { "250", "500", "1000", "2000" };
    return e < 4 ? n[e] : "?";
}

/* ISM330BX / LSM6DSV gyro sensitivity (DS14588): 8.75/17.5/35/70 mdps/LSB. */
static float gyro_lsb_per_dps_for(unsigned gyro_enum) {
    static const float mdps[] = { 8.75f, 17.5f, 35.0f, 70.0f };
    return gyro_enum < 4 ? 1000.0f / mdps[gyro_enum] : 0.0f;
}

/* mount_R_bv = Rz(yaw) * Ry(pitch) * Rx(roll) * R_preset  (sensor -> vehicle FLU). */
static void build_mount(bool frd, float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]) {
    const float d2r = IMU_DEG2RAD;
    float cr = cosf(roll_deg * d2r),  sr = sinf(roll_deg * d2r);
    float cp = cosf(pitch_deg * d2r), sp = sinf(pitch_deg * d2r);
    float cy = cosf(yaw_deg * d2r),   sy = sinf(yaw_deg * d2r);
    const float T[3][3] = {
        { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
        { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
        { -sp,     cp * sr,                cp * cr                },
    };
    /* FRD sensor (z down, y right): 180 deg about x maps it onto FLU. */
    const float P[3] = { 1.0f, frd ? -1.0f : 1.0f, frd ? -1.0f : 1.0f };
    int r, c;
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++) R[r][c] = T[r][c] * P[c];
}

/* Once the IMU has been still for 2 s, confirm gravity lands on vehicle +z
 * (i.e. the FLU mount is right) and report the residual tilt. */
typedef struct {
    double   sum[3];
    uint32_t n;
    bool     done;
} flu_check_t;

static void flu_check_update(flu_check_t *c, const imu_output_t *o, float rate_hz) {
    const float still_std = 0.05f;   /* m/s^2 */
    int i;

    if (c->done) return;
    if (o->gyro_norm_filtered_rps > 1.0f * IMU_DEG2RAD || o->accel_std[0] > still_std ||
        o->accel_std[1] > still_std || o->accel_std[2] > still_std) {
        memset(c, 0, sizeof(*c));     /* need an uninterrupted still window */
        return;
    }
    for (i = 0; i < 3; i++) c->sum[i] += o->accel_vehicle_filtered[i];
    if (++c->n < (uint32_t)(2.0f * rate_hz)) return;

    c->done = true;
    double f[3] = { c->sum[0] / c->n, c->sum[1] / c->n, c->sum[2] / c->n };
    double g = sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    double roll = atan2(f[1], f[2]) / IMU_DEG2RAD;
    double pitch = atan2(-f[0], sqrt(f[1] * f[1] + f[2] * f[2])) / IMU_DEG2RAD;
    double tilt = acos(f[2] / (g > 1e-6 ? g : 1.0)) / IMU_DEG2RAD;

    fprintf(stderr, "[FLU check] gravity in vehicle frame = (%+.3f, %+.3f, %+.3f) m/s^2  |g|=%.3f\n",
            f[0], f[1], f[2], g);
    if (f[2] < 0.0)
        fprintf(stderr, "[FLU check] FAIL: gravity points to -z — the board is z-down (FRD?). "
                        "Re-run with --mount frd or fix the installation.\n");
    else if (tilt > 30.0)
        fprintf(stderr, "[FLU check] FAIL: gravity is %.0f deg off +z — an axis is mis-assigned; "
                        "check which face of the board points up.\n", tilt);
    else
        fprintf(stderr, "[FLU check] OK: z is up. Residual tilt roll=%+.2f pitch=%+.2f deg "
                        "(mount tilt + ground slope; trim with --mount-rpy after the two-heading test)\n",
                roll, pitch);
    if (fabs(g - IMU_GRAVITY_MPS2) > 0.01 * IMU_GRAVITY_MPS2)
        fprintf(stderr, "[FLU check] WARNING: |g| is %.1f %% off 9.807 — accel scale outside ±1 %% spec "
                        "(see IMU_FIELD_CALIBRATION.md §2)\n", 100.0 * (g - IMU_GRAVITY_MPS2) / IMU_GRAVITY_MPS2);
}

/* ---- Mount persistence + runtime mount calibration ------------------------
 *
 * The dashboard's guided calibration computes a corrective rotation in the
 * current vehicle frame and sends it here ("IMUCTL MOUNT_DELTA <9 floats>").
 * The streamer pre-multiplies it into mount_R_bv, applies it live, saves it
 * to the mount file, and re-stamps the boot-calibration file so the next
 * start reuses both. The file holds the full 3x3 sensor->vehicle matrix. */
#define MOUNT_FILE_TAG "imu_mount_R_bv_v1"

static bool rot_valid(float R[3][3]) {
    int r, c, k;
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++) {
            float dot = 0.0f;
            for (k = 0; k < 3; k++) dot += R[r][k] * R[c][k];
            if (!isfinite(dot) || fabsf(dot - (r == c ? 1.0f : 0.0f)) > 2e-3f) return false;
        }
    return fabsf(R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
               - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
               + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]) - 1.0f) < 2e-3f;
}

static int mount_load(const char *path, float R[3][3]) {
    char tag[32] = "";
    float m[3][3];
    FILE *fp = fopen(path, "r");
    if (!fp) return -errno;
    int n = fscanf(fp, "%31s %f %f %f %f %f %f %f %f %f", tag,
                   &m[0][0], &m[0][1], &m[0][2], &m[1][0], &m[1][1], &m[1][2], &m[2][0], &m[2][1], &m[2][2]);
    fclose(fp);
    if (n != 10 || strcmp(tag, MOUNT_FILE_TAG) != 0 || !rot_valid(m)) return -EINVAL;
    memcpy(R, m, sizeof(m));
    return 0;
}

static int mount_save(const char *path, float R[3][3]) {
    FILE *fp = fopen(path, "w");
    if (!fp) return -errno;
    fprintf(fp, "%s\n%.7f %.7f %.7f\n%.7f %.7f %.7f\n%.7f %.7f %.7f\n", MOUNT_FILE_TAG,
            (double)R[0][0], (double)R[0][1], (double)R[0][2], (double)R[1][0], (double)R[1][1],
            (double)R[1][2], (double)R[2][0], (double)R[2][1], (double)R[2][2]);
    return fclose(fp) == 0 ? 0 : -EIO;
}

static int fmt_mount(char *buf, size_t n, const char *head, float R[3][3]) {
    return snprintf(buf, n, "%s %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f", head,
                    (double)R[0][0], (double)R[0][1], (double)R[0][2], (double)R[1][0], (double)R[1][1],
                    (double)R[1][2], (double)R[2][0], (double)R[2][1], (double)R[2][2]);
}

static void apply_mount(imu_pipeline_t *pl, imu_fusion_t *fu, float R[3][3], const char *mount_path) {
    memcpy(pl->cfg.mount_R_bv, R, sizeof(pl->cfg.mount_R_bv));
    pl->lpf_initialized = false;          /* filter state was in the old frame */
    pl->median_count = 0u;
    imu_fusion_set_mount(fu, R);
    if (pl->cal.valid) {                  /* keep the boot calibration valid for the new config */
        pl->cal.config_crc32 = imu_config_crc32(&pl->cfg);
        if (imu_pipeline_save_calibration(pl) != 0)
            fprintf(stderr, "[mount] warning: could not re-save the calibration file\n");
    }
    if (mount_save(mount_path, R) != 0)
        fprintf(stderr, "[mount] warning: could not write %s\n", mount_path);
}

/* Non-blocking: handle at most one control datagram from the dashboard host.
 *   IMUCTL MOUNT_DELTA r00 .. r22   R_bv <- delta * R_bv (rotation, vehicle frame)
 *   IMUCTL MOUNT_RESET              R_bv <- --mount preset without trim
 *   IMUCTL GET_MOUNT                report R_bv
 *   IMUCTL SFLP_RESET               restart the on-chip SFLP
 * Replies "IMUACK <cmd> OK|ERR ..." to the sender. Packets from any address
 * other than --host are ignored. */
static void poll_control(int sock, const struct sockaddr_in *peer, int imu_fd, imu_pipeline_t *pl,
                         imu_fusion_t *fu, float preset[3][3], const char *mount_path) {
    char buf[512], reply[512];
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, MSG_DONTWAIT, (struct sockaddr *)&from, &fl);
    if (n <= 0 || from.sin_addr.s_addr != peer->sin_addr.s_addr) return;
    buf[n] = '\0';
    if (strncmp(buf, "IMUCTL ", 7) != 0) return;
    const char *cmd = buf + 7;

    if (strncmp(cmd, "MOUNT_DELTA", 11) == 0) {
        float D[3][3], Rn[3][3];
        int r, c, k;
        if (sscanf(cmd + 11, "%f %f %f %f %f %f %f %f %f", &D[0][0], &D[0][1], &D[0][2], &D[1][0],
                   &D[1][1], &D[1][2], &D[2][0], &D[2][1], &D[2][2]) != 9 || !rot_valid(D)) {
            snprintf(reply, sizeof(reply), "IMUACK MOUNT_DELTA ERR not a rotation matrix");
        } else {
            for (r = 0; r < 3; r++)
                for (c = 0; c < 3; c++) {
                    Rn[r][c] = 0.0f;
                    for (k = 0; k < 3; k++) Rn[r][c] += D[r][k] * pl->cfg.mount_R_bv[k][c];
                }
            apply_mount(pl, fu, Rn, mount_path);
            fmt_mount(reply, sizeof(reply), "IMUACK MOUNT_DELTA OK", Rn);
            fprintf(stderr, "[mount] calibration applied and saved to %s\n", mount_path);
        }
    } else if (strncmp(cmd, "MOUNT_RESET", 11) == 0) {
        apply_mount(pl, fu, preset, mount_path);
        fmt_mount(reply, sizeof(reply), "IMUACK MOUNT_RESET OK", preset);
        fprintf(stderr, "[mount] reset to the --mount preset\n");
    } else if (strncmp(cmd, "GET_MOUNT", 9) == 0) {
        fmt_mount(reply, sizeof(reply), "IMUACK GET_MOUNT OK", pl->cfg.mount_R_bv);
    } else if (strncmp(cmd, "SFLP_RESET", 10) == 0) {
        int rc = ioctl(imu_fd, MPU6050_IOC_SFLP_RESET);
        snprintf(reply, sizeof(reply), "IMUACK SFLP_RESET %s", rc == 0 ? "OK" : strerror(errno));
    } else {
        snprintf(reply, sizeof(reply), "IMUACK ERR unknown command");
    }
    sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&from, fl);
}

static int parse_gyro_fs(const char *s, unsigned *out) {
    int v = atoi(s);
    switch (v) {
        case 250:  *out = GYRO_250DPS;  return 0;
        case 500:  *out = GYRO_500DPS;  return 0;
        case 1000: *out = GYRO_1000DPS; return 0;
        case 2000: *out = GYRO_2000DPS; return 0;
        default:   return -1;
    }
}

int main( int argc, char **argv ) {
    /* Guarantee immediate visibility of stderr no matter how the program
     * is launched (ssh session, systemd, redirection). Prints below become
     * the ONLY reliable way to tell whether the binary is even running. */
    setvbuf(stderr, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    fprintf(stderr, "[imu_streamer] main() entered  pid=%d  argc=%d\n",
            (int)getpid(), argc);
    {
        int i;
        for (i = 0; i < argc; i++)
            fprintf(stderr, "[imu_streamer]   argv[%d] = %s\n", i, argv[i]);
    }

    const char *host = NULL;
    const char *device = "/dev/mpu6050-0";
    const char *cal_path = "./imu_calibration.bin";
    int port = 5005;
    bool recalibrate = false;
    bool rate_given = false;
    bool wire_v2 = true;
    float ahrs_kp = 0.5f, ahrs_ki = 0.01f;
    /* Requested driver changes; INT32_MIN = leave as is. */
    int want_gyro_lpf = INT32_MIN, want_accel_lpf = INT32_MIN;
    int want_sflp = INT32_MIN, want_sflp_odr = INT32_MIN, want_sflp_mask = INT32_MIN;
    int want_fifo = INT32_MIN;
    bool want_gyro_fs = false;
    bool mount_frd = false;
    bool mount_cli = false;           /* --mount / --mount-rpy given: ignore the mount file */
    const char *mount_path = "./imu_mount.txt";
    float mount_rpy[3] = { 0.0f, 0.0f, 0.0f };
    unsigned gyro_fs_enum = 0;
    imu_config_t cfg;
    imu_config_set_dr_defaults(&cfg);
    fprintf(stderr, "[imu_streamer] config defaults loaded: sample_rate=%.1f Hz\n",
            (double)cfg.sample_rate_hz);

    static const struct option options[] = {
        {"host",        required_argument, NULL, 'h'},
        {"port",        required_argument, NULL, 'p'},
        {"device",      required_argument, NULL, 'd'},
        {"cal-file",    required_argument, NULL, 'c'},
        {"recalibrate", no_argument,       NULL, 'r'},
        {"rate",        required_argument, NULL, 'f'},   /* was 'r' — collided with --recalibrate */
        {"cutoff",      required_argument, NULL, 'l'},
        {"gyro-fs",     required_argument, NULL, 'G'},
        {"gyro-lpf",    required_argument, NULL, 'g'},
        {"accel-lpf",   required_argument, NULL, 'a'},
        {"sflp-odr",    required_argument, NULL, 's'},
        {"sflp-mask",   required_argument, NULL, 'm'},
        {"no-sflp",     no_argument,       NULL, 'n'},
        {"no-fifo",     no_argument,       NULL, 'N'},
        {"mount",       required_argument, NULL, 'M'},
        {"mount-rpy",   required_argument, NULL, 'R'},
        {"mount-file",  required_argument, NULL, 'F'},
        {"ahrs-kp",     required_argument, NULL, 'k'},
        {"ahrs-ki",     required_argument, NULL, 'i'},
        {"wire",        required_argument, NULL, 'w'},
        {"help",        no_argument,       NULL, '?'},
        {NULL, 0, NULL, 0}
    };

    for(;;) {
        int opt = getopt_long(argc, argv, "h:p:d:c:rf:l:G:g:a:s:m:nNM:R:F:k:i:w:?", options, NULL);
        if( opt == -1 ) break;
        switch(opt) {
            case 'h' : host = optarg; break;
            case 'p' : port = atoi(optarg); break;
            case 'd' : device = optarg; break;
            case 'c' : cal_path = optarg; break;
            case 'r' : recalibrate = true; break;
            case 'f' : cfg.sample_rate_hz = strtof(optarg, NULL); rate_given = true; break;
            case 'l' : cfg.lpf_cutoff_hz = strtof(optarg, NULL); break;
            case 'G' :
                if (parse_gyro_fs(optarg, &gyro_fs_enum) != 0) {
                    fprintf(stderr, "--gyro-fs must be 250/500/1000/2000\n");
                    return 2;
                }
                want_gyro_fs = true;
                break;
            case 'g' : want_gyro_lpf = atoi(optarg); break;
            case 'a' : want_accel_lpf = atoi(optarg); break;
            case 's' : want_sflp_odr = atoi(optarg); break;
            case 'm' : want_sflp_mask = (int)strtol(optarg, NULL, 0); break;
            case 'n' : want_sflp = 0; break;
            case 'N' : want_fifo = 0; break;
            case 'M' :
                if (strcmp(optarg, "flu") == 0) mount_frd = false;
                else if (strcmp(optarg, "frd") == 0) mount_frd = true;
                else { fprintf(stderr, "--mount must be flu or frd\n"); return 2; }
                mount_cli = true;
                break;
            case 'R' :
                if (sscanf(optarg, "%f,%f,%f", &mount_rpy[0], &mount_rpy[1], &mount_rpy[2]) != 3) {
                    fprintf(stderr, "--mount-rpy expects roll,pitch,yaw in degrees\n");
                    return 2;
                }
                mount_cli = true;
                break;
            case 'F' : mount_path = optarg; break;
            case 'k' : ahrs_kp = strtof(optarg, NULL); break;
            case 'i' : ahrs_ki = strtof(optarg, NULL); break;
            case 'w' : wire_v2 = (strcmp(optarg, "v1") != 0); break;
            default: usage(argv[0]); return (opt == '?') ? 0 : 2;
        }
    }
    if( !host || port < 1 || port > 65535 ){
        fprintf(stderr, "[imu_streamer] REQUIRED --host missing (host=%s port=%d) — exiting rc=2\n",
                host ? host : "(null)", port);
        usage(argv[0]);
        return 2;
    }
    fprintf(stderr, "[imu_streamer] parsed args: host=%s port=%d device=%s cal_file=%s recalibrate=%d\n",
            host, port, device, cal_path, (int)recalibrate);

    signal( SIGINT, on_signal );
    signal( SIGTERM, on_signal );

    /* ---- Driver discovery + configuration (before the pipeline, whose
     * config — rate, gyro scale — must match what the chip delivers). ---- */
    fprintf(stderr, "[imu_streamer] opening IMU device %s…\n", device);
    int imu_fd = open(device, O_RDONLY | O_CLOEXEC );
    if( imu_fd < 0 ){
        fprintf(stderr, "[imu_streamer] open(%s) failed: %s (errno=%d) — exiting\n",
                device, strerror(errno), errno);
        return 1;
    }
    fprintf(stderr, "[imu_streamer] IMU fd=%d\n", imu_fd);

    {
        __u32 who = 0, odr = 0;
        struct mpu6050_fs fs;
        memset(&fs, 0, sizeof(fs));
        if (ioctl(imu_fd, MPU6050_IOC_GET_WHOAMI, &who) == 0)
            fprintf(stderr, "[imu_streamer] WHO_AM_I=0x%02X%s\n", (unsigned)who,
                    who == 0x71 ? " (ISM330BX)" : "");
        if (want_gyro_fs && ioctl(imu_fd, MPU6050_IOC_GET_FS, &fs) == 0) {
            fs.gyro = gyro_fs_enum;
            if (ioctl(imu_fd, MPU6050_IOC_SET_FS, &fs) != 0)
                fprintf(stderr, "[imu_streamer] SET_FS failed: %s\n", strerror(errno));
        }
        if (ioctl(imu_fd, MPU6050_IOC_GET_FS, &fs) == 0) {
            float lsb = gyro_lsb_per_dps_for(fs.gyro);
            if (lsb > 0.0f) cfg.gyro_lsb_per_dps = lsb;
            fprintf(stderr, "[imu_streamer] driver FS: accel enum=%u, gyro ±%s dps -> %.4f LSB/dps\n",
                    (unsigned)fs.accel, fs_enum_name_gyro(fs.gyro), (double)cfg.gyro_lsb_per_dps);
            if (fs.gyro == GYRO_2000DPS)
                fprintf(stderr, "[imu_streamer] NOTE: ±2000 dps = 70 mdps/LSB, coarser than the gyro noise; "
                                "--gyro-fs 500 gives 4x finer resolution for vehicle rates\n");
            if (fs.accel != ACCEL_8G && fs.accel <= ACCEL_16G) {
                /* LSQ accel_C was fitted at ±8 g: matrix scales with FS, offset does not. */
                static const float fs_g[] = { 2.0f, 4.0f, 8.0f, 16.0f };
                float k = fs_g[fs.accel] / 8.0f;
                int r, c;
                for (r = 0; r < 3; r++) for (c = 0; c < 3; c++) cfg.accel_C[r][c] *= k;
                fprintf(stderr, "[imu_streamer] accel FS is not ±8 g: rescaled LSQ matrix by %.3f\n", (double)k);
            }
        }
        if (ioctl(imu_fd, MPU6050_IOC_GET_ODR, &odr) == 0 && odr > 0 && !rate_given) {
            cfg.sample_rate_hz = (float)odr;
            fprintf(stderr, "[imu_streamer] driver ODR=%u Hz -> pipeline sample_rate\n", (unsigned)odr);
        }
    }

    bool ext = false;
    {
        struct mpu6050_filter_cfg fc;
        if (ioctl(imu_fd, MPU6050_IOC_GET_FILTER, &fc) == 0) {
            bool change = false;
            if (want_gyro_lpf  != INT32_MIN) { fc.gyro_lpf1_bw   = want_gyro_lpf;  change = true; }
            if (want_accel_lpf != INT32_MIN) { fc.accel_lpf2_bw  = want_accel_lpf; change = true; }
            if (want_sflp      != INT32_MIN) { fc.sflp_enable    = (__u32)want_sflp; change = true; }
            if (want_sflp_odr  != INT32_MIN) { fc.sflp_odr_hz    = (__u32)want_sflp_odr; change = true; }
            if (want_sflp_mask != INT32_MIN) { fc.sflp_fifo_mask = (__u32)want_sflp_mask; change = true; }
            if (want_fifo      != INT32_MIN) { fc.fifo_stream    = (__u32)want_fifo; change = true; }
            if (change && ioctl(imu_fd, MPU6050_IOC_SET_FILTER, &fc) != 0)
                fprintf(stderr, "[imu_streamer] SET_FILTER failed: %s — keeping driver settings\n",
                        strerror(errno));
            if (ioctl(imu_fd, MPU6050_IOC_GET_FILTER, &fc) == 0)
                fprintf(stderr, "[imu_streamer] on-chip chain: gyro LPF1=%d accel LPF2=%d SFLP=%s @ %u Hz "
                                "mask=0x%x read=%s\n",
                        fc.gyro_lpf1_bw, fc.accel_lpf2_bw, fc.sflp_enable ? "on" : "off",
                        fc.sflp_odr_hz, fc.sflp_fifo_mask, fc.fifo_stream ? "HW FIFO" : "register poll");
        } else {
            fprintf(stderr, "[imu_streamer] driver has no filter ioctl (%s) — old mpu6050_char.ko?\n",
                    strerror(errno));
        }
        __u32 fmt = MPU6050_FMT_EXT;
        ext = (ioctl(imu_fd, MPU6050_IOC_SET_READ_FMT, &fmt) == 0);
        fprintf(stderr, "[imu_streamer] read format: %s\n",
                ext ? "extended (raw + SFLP)" : "legacy (no SFLP; host AHRS only)");
    }

    /* Sensor -> vehicle FLU. The config CRC covers the mount, so changing it
     * forces a fresh boot calibration instead of reusing a stale one. */
    float mount_preset[3][3];
    build_mount(mount_frd, 0.0f, 0.0f, 0.0f, mount_preset);
    build_mount(mount_frd, mount_rpy[0], mount_rpy[1], mount_rpy[2], cfg.mount_R_bv);
    if (!mount_cli) {
        int mrc = mount_load(mount_path, cfg.mount_R_bv);
        if (mrc == 0)
            fprintf(stderr, "[imu_streamer] mount loaded from %s (dashboard calibration)\n", mount_path);
        else if (mrc != -ENOENT)
            fprintf(stderr, "[imu_streamer] ignoring %s: %s\n", mount_path, strerror(-mrc));
    }
    fprintf(stderr, "[imu_streamer] mount %s, trim rpy=(%.2f, %.2f, %.2f) deg, R_bv=\n"
                    "    [%+.4f %+.4f %+.4f]\n    [%+.4f %+.4f %+.4f]\n    [%+.4f %+.4f %+.4f]\n",
            mount_frd ? "FRD" : "FLU", (double)mount_rpy[0], (double)mount_rpy[1], (double)mount_rpy[2],
            (double)cfg.mount_R_bv[0][0], (double)cfg.mount_R_bv[0][1], (double)cfg.mount_R_bv[0][2],
            (double)cfg.mount_R_bv[1][0], (double)cfg.mount_R_bv[1][1], (double)cfg.mount_R_bv[1][2],
            (double)cfg.mount_R_bv[2][0], (double)cfg.mount_R_bv[2][1], (double)cfg.mount_R_bv[2][2]);

    fprintf(stderr, "[imu_streamer] calling imu_pipeline_init…\n");
    imu_pipeline_t pipeline;
    int rc = imu_pipeline_init(&pipeline, &cfg, cal_path, recalibrate );
    if( rc != 0 ) {
        fprintf(stderr, "[imu_streamer] imu_pipeline_init failed: %s (%d) — exiting\n",
                strerror(-rc), rc);
        close(imu_fd);
        return 1;
    }
    fprintf(stderr, "[imu_streamer] pipeline OK, cal_state=%s\n",
            imu_cal_state_name(pipeline.cal_state));

    imu_fusion_t fusion;
    imu_fusion_init(&fusion, cfg.mount_R_bv, ahrs_kp, ahrs_ki);

    fprintf(stderr, "[imu_streamer] creating UDP socket…\n");
    int sock = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if( sock < 0 ) {
        fprintf(stderr, "[imu_streamer] socket() failed: %s — exiting\n", strerror(errno));
        close(imu_fd);
        return 1;
    }
    fprintf(stderr, "[imu_streamer] socket fd=%d\n", sock);
    struct sockaddr_in peer;
    memset(&peer, 0, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_port = htons((uint16_t)port);
    if(inet_pton(AF_INET, host, &peer.sin_addr) != 1){
        fprintf(stderr, "Invalid IPv4 address: %s\n", host);
        close(sock); close(imu_fd); return 2;
    }

    fprintf(stderr, "Streaming %s to %s:%d (wire %s); calibration= %s; state = %s\n",
        device, host, port, wire_v2 ? "v2" : "v1", cal_path, imu_cal_state_name(pipeline.cal_state));
    if( pipeline.cal_state != IMU_CAL_VALID ) {
        fprintf(stderr, "Keep the IMU completely stationary until calibration reaches 100%%.\n");
    }

    uint8_t frame[IMU_WIRE_V2_FRAME_SIZE];
    uint32_t last_state = UINT32_MAX;
    uint64_t last_status_ns   = 0;
    uint64_t samples_since    = 0;
    uint64_t packets_sent     = 0;
    uint64_t sendto_errs      = 0;
    uint64_t last_t_ns        = 0;
    uint32_t last_seq         = 0;
    uint32_t seq_gaps         = 0;
    uint32_t sflp_seq_mark    = 0;
    uint32_t last_conv_flags  = 0;
    bool     have_seq         = false;
    flu_check_t flu_check;
    memset(&flu_check, 0, sizeof(flu_check));

    while(!g_stop) {
        struct mpu6050_ext_sample es;

        poll_control(sock, &peer, imu_fd, &pipeline, &fusion, mount_preset, mount_path);
        struct mpu6050_sample ls;
        imu_sflp_input_t sin;
        imu_raw_sample_t raw = { 0 };
        uint64_t t_ns = 0;
        ssize_t n;

        memset(&sin, 0, sizeof(sin));
        if (ext) {
            n = read(imu_fd, &es, sizeof(es));
            if (n == (ssize_t)sizeof(es)) {
                raw = (imu_raw_sample_t){ .ax = es.ax, .ay = es.ay, .az = es.az,
                                          .gx = es.gx, .gy = es.gy, .gz = es.gz };
                t_ns = (uint64_t)es.t_ns;
                sin.have_ext = true;
                sin.flags    = es.flags;
                sin.seq      = es.seq;
                sin.dropped  = es.dropped;
                sin.sflp_seq = es.sflp_seq;
                sin.temp     = es.temp;
                memcpy(sin.quat_h, es.quat_h, sizeof(sin.quat_h));
                memcpy(sin.gravity, es.gravity, sizeof(sin.gravity));
                memcpy(sin.gbias, es.gbias, sizeof(sin.gbias));
                if (have_seq && es.seq != last_seq + 1u) seq_gaps += es.seq - last_seq - 1u;
                last_seq = es.seq;
                have_seq = true;
            }
        } else {
            n = read(imu_fd, &ls, sizeof(ls));
            if (n == (ssize_t)sizeof(ls)) {
                raw = (imu_raw_sample_t){ .ax = ls.ax, .ay = ls.ay, .az = ls.az,
                                          .gx = ls.gx, .gy = ls.gy, .gz = ls.gz };
                t_ns = (uint64_t)ls.t_ns;
                sin.temp = ls.temp;
            }
        }
        if( n < 0 ){
            if( errno == EINTR ) continue;
            perror("read IMU");
            break;
        }
        if( n != (ssize_t)(ext ? sizeof(es) : sizeof(ls))) {
            fprintf(stderr, "Short IMU read: %zd/%zu\n", n, ext ? sizeof(es) : sizeof(ls));
            continue;
        }

        /* Driver time: in FIFO mode it is reconstructed on the chip clock,
         * so dt is regular even when read() returns bursts. */
        if (t_ns <= last_t_ns) t_ns = last_t_ns + 1u;
        last_t_ns = t_ns;

        imu_output_t out;
        uint64_t now_ns = monotonic_ns();
        rc = imu_pipeline_process(&pipeline, &raw, t_ns, &out);
        if( rc != 0 ){
            fprintf(stderr, "pipeline error: %s (%d)\n", strerror(-rc), rc);
            continue;
        }

        /* Gravity reference: the boot-calibration |a| absorbs a uniform accel
         * scale error, so gravity removal stays unbiased until a 6-position
         * LSQ calibration is loaded. */
        float g_ref = (pipeline.cal.valid && pipeline.cal.boot_acc_norm_mean > 1.0f)
                      ? pipeline.cal.boot_acc_norm_mean : cfg.gravity_mps2;
        flu_check_update(&flu_check, &out, cfg.sample_rate_hz);
        imu_fusion_output_t fo;
        imu_fusion_update(&fusion, &sin, &out, g_ref, &fo);

        if ((fo.flags & IMU_FF_CONV_DECIDED) && !(last_conv_flags & IMU_FF_CONV_DECIDED))
            fprintf(stderr, "[fusion] SFLP quaternion convention: %s\n",
                    (fo.flags & IMU_FF_CONV_CONJUGATE) ? "world->sensor (conjugated)" : "sensor->world (direct)");
        last_conv_flags = fo.flags;

        size_t frame_size = 0u;
        rc = wire_v2 ? imu_wire_encode_v2(&out, &pipeline.cal, &fo, frame, sizeof(frame), &frame_size)
                     : imu_wire_encode(&out, &pipeline.cal, frame, sizeof(frame), &frame_size);
        if( rc != 0 ) {
            fprintf(stderr, "wire encode error: %s (%d)\n", strerror(-rc), rc);
            break;
        }
        ssize_t sent = sendto(sock, frame, frame_size, 0, (struct sockaddr *)&peer, sizeof(peer));
        if( sent == (ssize_t)frame_size) {
            ++packets_sent;
        } else if (errno != ENOBUFS) {
            ++sendto_errs;
            if (sendto_errs <= 5 || (sendto_errs % 100) == 0)
                fprintf(stderr, "sendto err #%llu: %s\n",
                        (unsigned long long)sendto_errs, strerror(errno));
        }

        ++samples_since;

        /* State-change print (once per transition — kept from the original). */
        if( (uint32_t)out.cal_state != last_state ) {
            fprintf(stderr, "[state->] %s, progress %.1f%%, quality %.0f\n",
                imu_cal_state_name(out.cal_state),
                100.0f * out.calibration_progress, out.quality_score);
            last_state = (uint32_t)out.cal_state;
        }

        /* Periodic heartbeat — ALWAYS prints once per second, even when
         * the state hasn't changed. Prevents the streamer from looking
         * dead when it's actually spinning in SETTLING or RESTARTING. */
        if (last_status_ns == 0) { last_status_ns = now_ns; sflp_seq_mark = fo.sflp_seq; }
        if (now_ns - last_status_ns >= 1000000000ull) {
            double dt = (now_ns - last_status_ns) * 1e-9;
            double obs_hz = samples_since / dt;
            double sflp_hz = (double)(fo.sflp_seq - sflp_seq_mark) / dt;
            fprintf(stderr,
                "[%.1fs] state=%-11s prog=%5.1f%% qual=%3.0f  obs_rate=%.1f Hz  "
                "sent=%llu err=%llu  raw|a|~=%.2f g|w|~=%.2f dps\n",
                (double)now_ns * 1e-9,
                imu_cal_state_name(out.cal_state),
                100.0f * out.calibration_progress,
                out.quality_score,
                obs_hz,
                (unsigned long long)packets_sent,
                (unsigned long long)sendto_errs,
                (double)out.accel_norm_unfiltered,
                (double)out.gyro_norm_unfiltered_rps / 0.01745329f);
            fprintf(stderr,
                "         SFLP rpy=(%6.1f %6.1f %6.1f) %.0f Hz | AHRS rpy=(%6.1f %6.1f %6.1f) | "
                "lin_lvl=(%5.2f %5.2f %5.2f) | T=%.1fC drv_drop=%u seq_gap=%u%s\n",
                (double)fo.euler_sflp_deg[0], (double)fo.euler_sflp_deg[1], (double)fo.euler_sflp_deg[2],
                sflp_hz,
                (double)fo.euler_ahrs_deg[0], (double)fo.euler_ahrs_deg[1], (double)fo.euler_ahrs_deg[2],
                (double)fo.lin_acc_level_mps2[0], (double)fo.lin_acc_level_mps2[1],
                (double)fo.lin_acc_level_mps2[2],
                (double)fo.temp_c, (unsigned)fo.driver_dropped, (unsigned)seq_gaps,
                (fo.flags & IMU_FF_FIFO_OVERRUN) ? " FIFO-OVERRUN" : "");
            last_status_ns = now_ns;
            samples_since  = 0;
            sflp_seq_mark  = fo.sflp_seq;
        }
    }

    close(sock);
    close(imu_fd);
    return 0;
}
