#include "imu_mount.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define RAD2DEG 57.29577951308232f

static float v_dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

static float v_unit(const float a[3], float out[3]) {
    float n = sqrtf(v_dot(a, a));
    int i;
    for (i = 0; i < 3; i++) out[i] = n > 1e-9f ? a[i] / n : 0.0f;
    return n;
}

static float v_angle_deg(const float a[3], const float b[3]) {
    float ua[3], ub[3], c;
    v_unit(a, ua);
    v_unit(b, ub);
    c = v_dot(ua, ub);
    return acosf(c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c)) * RAD2DEG;
}

void imu_mount_build(bool frd, float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]) {
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

bool imu_mount_rot_valid(float R[3][3]) {
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

int imu_mount_load(const char *path, float R[3][3]) {
    char tag[32] = "";
    float m[3][3];
    FILE *fp = fopen(path, "r");
    int n;
    if (!fp) return -errno;
    n = fscanf(fp, "%31s %f %f %f %f %f %f %f %f %f", tag,
               &m[0][0], &m[0][1], &m[0][2], &m[1][0], &m[1][1], &m[1][2], &m[2][0], &m[2][1], &m[2][2]);
    fclose(fp);
    if (n != 10 || strcmp(tag, IMU_MOUNT_FILE_TAG) != 0 || !imu_mount_rot_valid(m)) return -EINVAL;
    memcpy(R, m, sizeof(m));
    return 0;
}

int imu_mount_save(const char *path, float R[3][3]) {
    char tmp[512];
    FILE *fp;
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    fp = fopen(tmp, "w");
    if (!fp) return -errno;
    fprintf(fp, "%s\n%.7f %.7f %.7f\n%.7f %.7f %.7f\n%.7f %.7f %.7f\n", IMU_MOUNT_FILE_TAG,
            (double)R[0][0], (double)R[0][1], (double)R[0][2], (double)R[1][0], (double)R[1][1],
            (double)R[1][2], (double)R[2][0], (double)R[2][1], (double)R[2][2]);
    if (fclose(fp) != 0) return -EIO;
    return rename(tmp, path) == 0 ? 0 : -errno;        /* atomic: no half-written file on power loss */
}

int imu_mount_format(char *buf, size_t n, const char *head, float R[3][3]) {
    return snprintf(buf, n, "%s %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f", head,
                    (double)R[0][0], (double)R[0][1], (double)R[0][2], (double)R[1][0], (double)R[1][1],
                    (double)R[1][2], (double)R[2][0], (double)R[2][1], (double)R[2][2]);
}

void imu_mount_apply(imu_pipeline_t *pl, imu_fusion_t *fu, float R[3][3], const char *mount_path) {
    memcpy(pl->cfg.mount_R_bv, R, sizeof(pl->cfg.mount_R_bv));
    pl->lpf_initialized = false;          /* filter state was in the old frame */
    pl->median_count = 0u;
    imu_fusion_set_mount(fu, R);
    if (pl->cal.valid) {                  /* keep the boot calibration valid for the new config */
        pl->cal.config_crc32 = imu_config_crc32(&pl->cfg);
        if (imu_pipeline_save_calibration(pl) != 0)
            fprintf(stderr, "[mount] warning: could not re-save the calibration file\n");
    }
    if (mount_path && imu_mount_save(mount_path, R) != 0)
        fprintf(stderr, "[mount] warning: could not write %s\n", mount_path);
}

int imu_mount_solve(const float f1[3], const float *f2, const float fwd[3], bool fwd_is_accel,
                    float D[3][3], imu_mount_cal_info_t *info, char *why, size_t why_len) {
    float u1[3], u2[3], up[3], h[3], x[3], y[3], tmp[3];
    float g = sqrtf(v_dot(f1, f1));
    int i;

    if (why && why_len) why[0] = '\0';
    if (g < 0.5f * IMU_GRAVITY_MPS2 || g > 1.5f * IMU_GRAVITY_MPS2) {
        snprintf(why, why_len, "|g| = %.2f m/s^2 at the level step: not still?", (double)g);
        return -ERANGE;
    }
    v_unit(f1, u1);
    if (f2) {
        v_unit(f2, u2);
        for (i = 0; i < 3; i++) tmp[i] = u1[i] + u2[i];   /* mean of both headings: slope cancels */
        v_unit(tmp, up);
    } else {
        memcpy(up, u1, sizeof(up));
    }

    if (fwd_is_accel) {
        /* Change from the last still capture on the same spot: removes the
         * surface slope's gravity component, which would otherwise leak into
         * the yaw (5 deg for a 2.5 deg slope at 2.5 m/s^2). */
        const float *ref = f2 ? f2 : f1;
        float dv[3], a;
        for (i = 0; i < 3; i++) dv[i] = fwd[i] - ref[i];
        a = v_dot(dv, up);
        for (i = 0; i < 3; i++) h[i] = dv[i] - a * up[i];
        if (sqrtf(v_dot(h, h)) < 0.5f) {
            snprintf(why, why_len, "forward acceleration too small (%.2f m/s^2)", (double)sqrtf(v_dot(h, h)));
            return -ERANGE;
        }
        v_unit(h, x);                                     /* accelerating forward: force points forward */
    } else {
        /* Change from the last still capture on the same spot and heading,
         * so the surface slope does not leak into the yaw. */
        float uf[3], dv[3], a;
        const float *ref = f2 ? u2 : u1;
        v_unit(fwd, uf);
        for (i = 0; i < 3; i++) dv[i] = uf[i] - ref[i];
        a = v_dot(dv, up);
        for (i = 0; i < 3; i++) h[i] = dv[i] - a * up[i];
        if (sqrtf(v_dot(h, h)) < 0.17f) {                 /* ~10 deg tip */
            snprintf(why, why_len, "front end not tipped down far enough");
            return -ERANGE;
        }
        for (i = 0; i < 3; i++) h[i] = -h[i];            /* nose down: gravity reaction moves backwards */
        v_unit(h, x);
    }
    /* y = up x x */
    y[0] = up[1] * x[2] - up[2] * x[1];
    y[1] = up[2] * x[0] - up[0] * x[2];
    y[2] = up[0] * x[1] - up[1] * x[0];
    for (i = 0; i < 3; i++) { D[0][i] = x[i]; D[1][i] = y[i]; D[2][i] = up[i]; }

    if (info) {
        info->g_mps2 = g;
        info->slope_deg = f2 ? 0.5f * v_angle_deg(f1, f2) : NAN;
        info->euler_deg[0] = atan2f(D[2][1], D[2][2]) * RAD2DEG;
        info->euler_deg[1] = asinf(fmaxf(-1.0f, fminf(1.0f, -D[2][0]))) * RAD2DEG;
        info->euler_deg[2] = atan2f(D[1][0], D[0][0]) * RAD2DEG;
    }
    return imu_mount_rot_valid(D) ? 0 : -EDOM;
}

void imu_mount_handle_ctl(const char *cmd, imu_pipeline_t *pl, imu_fusion_t *fu, float preset[3][3],
                          const char *mount_path, imu_mount_sflp_reset_fn sflp_reset, void *sflp_ctx,
                          char *reply, size_t reply_len) {
    if (strncmp(cmd, "MOUNT_DELTA", 11) == 0) {
        float Dm[3][3], Rn[3][3];
        int r, c, k;
        if (sscanf(cmd + 11, "%f %f %f %f %f %f %f %f %f", &Dm[0][0], &Dm[0][1], &Dm[0][2], &Dm[1][0],
                   &Dm[1][1], &Dm[1][2], &Dm[2][0], &Dm[2][1], &Dm[2][2]) != 9 || !imu_mount_rot_valid(Dm)) {
            snprintf(reply, reply_len, "IMUACK MOUNT_DELTA ERR not a rotation matrix");
            return;
        }
        for (r = 0; r < 3; r++)
            for (c = 0; c < 3; c++) {
                Rn[r][c] = 0.0f;
                for (k = 0; k < 3; k++) Rn[r][c] += Dm[r][k] * pl->cfg.mount_R_bv[k][c];
            }
        imu_mount_apply(pl, fu, Rn, mount_path);
        imu_mount_format(reply, reply_len, "IMUACK MOUNT_DELTA OK", Rn);
        fprintf(stderr, "[mount] calibration applied and saved to %s\n", mount_path);
    } else if (strncmp(cmd, "MOUNT_RESET", 11) == 0) {
        imu_mount_apply(pl, fu, preset, mount_path);
        imu_mount_format(reply, reply_len, "IMUACK MOUNT_RESET OK", preset);
        fprintf(stderr, "[mount] reset to the --mount preset\n");
    } else if (strncmp(cmd, "GET_MOUNT", 9) == 0) {
        imu_mount_format(reply, reply_len, "IMUACK GET_MOUNT OK", pl->cfg.mount_R_bv);
    } else if (strncmp(cmd, "SFLP_RESET", 10) == 0) {
        int rc = sflp_reset ? sflp_reset(sflp_ctx) : -ENOSYS;
        snprintf(reply, reply_len, "IMUACK SFLP_RESET %s", rc == 0 ? "OK" : strerror(-rc));
    } else {
        snprintf(reply, reply_len, "IMUACK ERR unknown command");
    }
}
