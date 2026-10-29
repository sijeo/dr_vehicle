#include "imu_fusion.h"

#include <math.h>
#include <string.h>

#define RAD2DEG 57.29577951308232f

/* ST reference-driver scale factors for the SFLP FIFO words. */
#define SFLP_GRAVITY_G_PER_LSB   0.061e-3f   /* 0.061 mg/LSB */
#define SFLP_GBIAS_DPS_PER_LSB   4.375e-3f   /* 4.375 mdps/LSB */

/* Quasi-static window used for the SFLP convention / sign checks. */
#define STATIC_ACC_TOL           0.03f       /* |a| within 3 % of g_ref */
#define STATIC_GYRO_MAX_RPS      (2.0f * IMU_DEG2RAD)
#define CONV_MIN_TILT_DEG        10.0f
#define CONV_MIN_SAMPLES         100u
#define CONV_MAX_SAMPLES         2000u
#define SFLP_MAX_INCONSISTENCY_DEG 10.0f  /* above this the SFLP attitude is not used */
#define SFLP_MIN_CHECKS          50u

/* AHRS accepts the accelerometer as a gravity reference only within this band. */
#define AHRS_ACC_TOL             0.10f

static float vnorm(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float angle_deg(const float a[3], const float b[3]) {
    float na = vnorm(a), nb = vnorm(b);
    if (na < 1e-6f || nb < 1e-6f) return 0.0f;
    return acosf(clampf((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb), -1.0f, 1.0f)) * RAD2DEG;
}

static void quat_normalize(float q[4]) {
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    int i;
    if (n < 1e-9f) { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; return; }
    for (i = 0; i < 4; i++) q[i] /= n;
    if (q[0] < 0.0f) for (i = 0; i < 4; i++) q[i] = -q[i];   /* canonical hemisphere */
}

/* 3x3 matrix parameters are non-const on purpose: C does not convert
 * float (*)[3] to const float (*)[3] implicitly, and GCC warns. */
static void quat_to_mat(const float q[4], float R[3][3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1.0f - 2.0f * (y * y + z * z);
    R[0][1] = 2.0f * (x * y - w * z);
    R[0][2] = 2.0f * (x * z + w * y);
    R[1][0] = 2.0f * (x * y + w * z);
    R[1][1] = 1.0f - 2.0f * (x * x + z * z);
    R[1][2] = 2.0f * (y * z - w * x);
    R[2][0] = 2.0f * (x * z - w * y);
    R[2][1] = 2.0f * (y * z + w * x);
    R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

static void mat_to_quat(float R[3][3], float q[4]) {
    float tr = R[0][0] + R[1][1] + R[2][2], s;
    if (tr > 0.0f) {
        s = sqrtf(tr + 1.0f) * 2.0f;
        q[0] = 0.25f * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        s = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25f * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        s = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25f * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    } else {
        s = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25f * s;
    }
    quat_normalize(q);
}

static void mat_mul(float A[3][3], float B[3][3], float C[3][3]) {
    int r, c, k;
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++) {
            C[r][c] = 0.0f;
            for (k = 0; k < 3; k++) C[r][c] += A[r][k] * B[k][c];
        }
}

static void mat_mul_vec(float A[3][3], const float v[3], float o[3]) {
    int r;
    for (r = 0; r < 3; r++) o[r] = A[r][0] * v[0] + A[r][1] * v[1] + A[r][2] * v[2];
}

static void quat_from_euler(float roll, float pitch, float yaw, float q[4]) {
    float cr = cosf(0.5f * roll),  sr = sinf(0.5f * roll);
    float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
    float cy = cosf(0.5f * yaw),   sy = sinf(0.5f * yaw);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    quat_normalize(q);
}

float imu_half_to_float(uint16_t h) {
    unsigned exp = (h >> 10) & 0x1Fu, mant = h & 0x3FFu;
    float v;
    if (exp == 0u)       v = ldexpf((float)mant, -24);                       /* subnormal */
    else if (exp == 31u) v = mant ? NAN : INFINITY;
    else                 v = ldexpf((float)(mant | 0x400u), (int)exp - 25);   /* (1.m) * 2^(e-15) */
    return (h & 0x8000u) ? -v : v;
}

void imu_quat_to_euler_deg(const float q[4], float e[3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    e[0] = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD2DEG;
    e[1] = asinf(clampf(2.0f * (w * y - z * x), -1.0f, 1.0f)) * RAD2DEG;
    e[2] = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD2DEG;
}

/* SFLP game rotation vector -> unit quaternion (ST sflp2q()). */
static void sflp_to_quat(const uint16_t h[3], float q[4]) {
    float x = imu_half_to_float(h[0]);
    float y = imu_half_to_float(h[1]);
    float z = imu_half_to_float(h[2]);
    float s = x * x + y * y + z * z;
    if (!isfinite(s)) { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; return; }
    if (s > 1.0f) {
        float n = sqrtf(s);
        x /= n; y /= n; z /= n;
        s = 1.0f;
    }
    q[0] = sqrtf(1.0f - s);
    q[1] = x; q[2] = y; q[3] = z;
    quat_normalize(q);
}

/* ---------------- Mahony AHRS (vehicle frame) ---------------- */

static void mahony_init_from_accel(imu_mahony_t *m, const float a[3]) {
    float roll  = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    quat_from_euler(roll, pitch, 0.0f, m->q);
    m->integral[0] = m->integral[1] = m->integral[2] = 0.0f;
    m->initialized = true;
}

static void mahony_update(imu_mahony_t *m, const float gyro[3], const float acc[3],
                          float dt, bool use_acc) {
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float qw = m->q[0], qx = m->q[1], qy = m->q[2], qz = m->q[3];
    float hdt = 0.5f * dt;

    if (use_acc) {
        float n = vnorm(acc);
        float ax = acc[0] / n, ay = acc[1] / n, az = acc[2] / n;
        /* Gravity direction predicted in the body frame: R^T e_z. */
        float vx = 2.0f * (qx * qz - qw * qy);
        float vy = 2.0f * (qw * qx + qy * qz);
        float vz = qw * qw - qx * qx - qy * qy + qz * qz;
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;
        if (m->ki > 0.0f) {
            m->integral[0] += m->ki * ex * dt;
            m->integral[1] += m->ki * ey * dt;
            m->integral[2] += m->ki * ez * dt;
        }
        gx += m->kp * ex;
        gy += m->kp * ey;
        gz += m->kp * ez;
    }
    gx += m->integral[0];
    gy += m->integral[1];
    gz += m->integral[2];

    m->q[0] = qw + (-qx * gx - qy * gy - qz * gz) * hdt;
    m->q[1] = qx + ( qw * gx + qy * gz - qz * gy) * hdt;
    m->q[2] = qy + ( qw * gy - qx * gz + qz * gx) * hdt;
    m->q[3] = qz + ( qw * gz + qx * gy - qy * gx) * hdt;
    quat_normalize(m->q);
}

/* ---------------- public API ---------------- */

void imu_fusion_init(imu_fusion_t *f, float mount_R_bv[3][3], float kp, float ki) {
    memset(f, 0, sizeof(*f));
    memcpy(f->mount_R_bv, mount_R_bv, sizeof(f->mount_R_bv));
    f->ahrs.kp = kp;
    f->ahrs.ki = ki;
    f->ahrs.q[0] = 1.0f;
}

void imu_fusion_set_mount(imu_fusion_t *f, float mount_R_bv[3][3]) {
    imu_fusion_init(f, mount_R_bv, f->ahrs.kp, f->ahrs.ki);
}

/* Learn whether the SFLP quaternion maps sensor->world (direct) or the
 * inverse, by comparing both predictions of gravity against the
 * accelerometer while the IMU is still and visibly tilted. Level poses
 * cannot tell the two apart, so the check waits for tilt. */
static void sflp_convention_check(imu_fusion_t *f, const float q_s[4], const float acc_s[3]) {
    static const float ez[3] = { 0.0f, 0.0f, 1.0f };
    float R[3][3], pred_direct[3], pred_conj[3];
    float ed, ec;

    if (f->conv != 0 || angle_deg(acc_s, ez) < CONV_MIN_TILT_DEG) return;

    quat_to_mat(q_s, R);
    pred_direct[0] = R[2][0]; pred_direct[1] = R[2][1]; pred_direct[2] = R[2][2];  /* R^T e_z */
    pred_conj[0]   = R[0][2]; pred_conj[1]   = R[1][2]; pred_conj[2]   = R[2][2];  /* R e_z */
    ed = angle_deg(acc_s, pred_direct);
    ec = angle_deg(acc_s, pred_conj);
    f->conv_err_direct += ed;
    f->conv_err_conj   += ec;
    if (++f->conv_n < CONV_MIN_SAMPLES) return;

    ed = (float)(f->conv_err_direct / f->conv_n);
    ec = (float)(f->conv_err_conj / f->conv_n);
    if (ed < 0.5f * ec)      f->conv = 1;
    else if (ec < 0.5f * ed) f->conv = 2;
    else if (f->conv_n > CONV_MAX_SAMPLES) {
        f->conv_n = 0;
        f->conv_err_direct = f->conv_err_conj = 0.0;
    }
}

void imu_fusion_update(imu_fusion_t *f, const imu_sflp_input_t *in, const imu_output_t *core,
                       float g_ref, imu_fusion_output_t *out) {
    static const float ez[3] = { 0.0f, 0.0f, 1.0f };
    const float *acc_v = core->accel_vehicle_filtered;
    float acc_n = vnorm(acc_v);
    bool quasi_static = fabsf(acc_n - g_ref) < STATIC_ACC_TOL * g_ref &&
                        core->gyro_norm_filtered_rps < STATIC_GYRO_MAX_RPS;
    bool acc_ok = fabsf(acc_n - g_ref) < AHRS_ACC_TOL * g_ref &&
                  !(core->quality_flags & IMU_QF_BUMP);
    float dt = 0.0f;
    float R_wv[3][3];
    bool have_att = false;
    int i;

    memset(out, 0, sizeof(*out));
    out->q_sflp[0] = out->q_ahrs[0] = 1.0f;
    out->g_ref_mps2 = g_ref;
    out->temp_c = (float)in->temp / 256.0f + 25.0f;

    if (f->last_ns && core->monotonic_ns > f->last_ns)
        dt = (float)((double)(core->monotonic_ns - f->last_ns) * 1e-9);
    f->last_ns = core->monotonic_ns;
    if (dt > 0.05f) dt = 0.0f;    /* gap: do not integrate across it */

    if (in->have_ext) {
        out->flags |= in->flags & (IMU_FF_SFLP_VALID | IMU_FF_GRAVITY_VALID | IMU_FF_GBIAS_VALID |
                                   IMU_FF_FIFO_STREAM | IMU_FF_FIFO_OVERRUN | IMU_FF_SAMPLE_DROPPED);
        out->sflp_seq = in->sflp_seq;
        out->driver_seq = in->seq;
        out->driver_dropped = in->dropped;
    } else {
        out->flags |= IMU_FF_LEGACY_DRIVER;
    }

    /* ---- host AHRS ---- */
    if (!f->ahrs.initialized) {
        if (acc_ok) mahony_init_from_accel(&f->ahrs, acc_v);
    } else if (dt > 0.0f) {
        mahony_update(&f->ahrs, core->gyro_vehicle_unfiltered_rps, acc_v, dt, acc_ok);
        if (acc_ok) out->flags |= IMU_FF_ACCEL_TRUSTED;
    }
    if (f->ahrs.initialized) {
        memcpy(out->q_ahrs, f->ahrs.q, sizeof(out->q_ahrs));
        imu_quat_to_euler_deg(out->q_ahrs, out->euler_ahrs_deg);
        out->flags |= IMU_FF_AHRS_VALID;
    }

    /* ---- SFLP ---- */
    bool have_g = false, sflp_ok = false;
    if (in->have_ext && (in->flags & IMU_FF_GRAVITY_VALID)) {
        /* The gravity word comes in Z, Y, X order (like the accel FIFO words)
         * and is gravity itself (pointing down); negate it to the
         * accelerometer's convention (+g up at rest). Established from the
         * 2026-10-27 logs: matches the accelerometer, the X,Y,Z reading did not. */
        const float k = SFLP_GRAVITY_G_PER_LSB * IMU_GRAVITY_MPS2;
        float g_s[3] = { -(float)in->gravity[2] * k, -(float)in->gravity[1] * k, -(float)in->gravity[0] * k };
        mat_mul_vec(f->mount_R_bv, g_s, out->gravity_sflp_vehicle_mps2);
        have_g = true;
    }
    if (in->have_ext && (in->flags & IMU_FF_GBIAS_VALID)) {
        float b_s[3];
        for (i = 0; i < 3; i++) b_s[i] = (float)in->gbias[i] * SFLP_GBIAS_DPS_PER_LSB;
        mat_mul_vec(f->mount_R_bv, b_s, out->gbias_sflp_dps);
    }
    if (in->have_ext && (in->flags & IMU_FF_SFLP_VALID)) {
        float q_s[4], R_ws[3][3], R_vb[3][3];
        int r, c;

        sflp_to_quat(in->quat_h, q_s);
        if (quasi_static) sflp_convention_check(f, q_s, core->accel_sensor_mps2);
        if (f->conv == 2) { q_s[1] = -q_s[1]; q_s[2] = -q_s[2]; q_s[3] = -q_s[3]; }
        if (f->conv != 0) out->flags |= IMU_FF_CONV_DECIDED;
        if (f->conv == 2) out->flags |= IMU_FF_CONV_CONJUGATE;

        /* v_w = R_ws v_s = R_ws R_bv^T v_v */
        quat_to_mat(q_s, R_ws);
        for (r = 0; r < 3; r++)
            for (c = 0; c < 3; c++) R_vb[r][c] = f->mount_R_bv[c][r];
        mat_mul(R_ws, R_vb, R_wv);
        mat_to_quat(R_wv, out->q_sflp);
        imu_quat_to_euler_deg(out->q_sflp, out->euler_sflp_deg);

        /* SFLP health. On the 2026-10-27 logs the quaternion followed the
         * gyro but its gravity reference was ~90 deg off, so it must earn its
         * place: compare the gravity it implies (R^T e_z) with the SFLP
         * gravity output, or with the accelerometer when at rest. */
        {
            const float *ref = have_g ? out->gravity_sflp_vehicle_mps2 : (quasi_static ? acc_v : NULL);
            if (ref) {
                float pred[3] = { R_wv[2][0], R_wv[2][1], R_wv[2][2] };
                float e = angle_deg(pred, ref);
                f->sflp_err_ema_deg = f->sflp_err_n++ ? 0.98f * f->sflp_err_ema_deg + 0.02f * e : e;
            }
        }
        out->sflp_consistency_deg = f->sflp_err_ema_deg;
        sflp_ok = f->sflp_err_n >= SFLP_MIN_CHECKS && f->sflp_err_ema_deg < SFLP_MAX_INCONSISTENCY_DEG;
        if (!sflp_ok) out->flags |= IMU_FF_SFLP_SUSPECT;
    }

    /* Attitude for gravity removal: SFLP only while it is consistent. */
    if (sflp_ok) {
        have_att = true;
        out->flags |= IMU_FF_LIN_FROM_SFLP;
    } else if (f->ahrs.initialized) {
        quat_to_mat(f->ahrs.q, R_wv);
        have_att = true;
    }

    /* ---- gravity removal ---- */
    if (have_att) {
        float g_body[3], R_level[3][3], Rz_inv[3][3], a_lvl[3];
        float yaw = atan2f(R_wv[1][0], R_wv[0][0]);
        float cy = cosf(yaw), sy = sinf(yaw);

        for (i = 0; i < 3; i++) g_body[i] = g_ref * R_wv[2][i];          /* R^T (g e_z) */
        for (i = 0; i < 3; i++) out->lin_acc_vehicle_mps2[i] = acc_v[i] - g_body[i];
        out->tilt_consistency_deg = angle_deg(acc_v, g_body);

        /* Heading-free level frame: undo yaw only. */
        Rz_inv[0][0] =  cy; Rz_inv[0][1] = sy; Rz_inv[0][2] = 0.0f;
        Rz_inv[1][0] = -sy; Rz_inv[1][1] = cy; Rz_inv[1][2] = 0.0f;
        Rz_inv[2][0] = 0.0f; Rz_inv[2][1] = 0.0f; Rz_inv[2][2] = 1.0f;
        mat_mul(Rz_inv, R_wv, R_level);
        mat_mul_vec(R_level, acc_v, a_lvl);
        for (i = 0; i < 3; i++) out->lin_acc_level_mps2[i] = a_lvl[i] - g_ref * ez[i];
    }
}
