#include "dr_nav.h"

#include <math.h>
#include <string.h>

#define D2R (M_PI / 180.0)

double dr_nav_wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void dr_nav_default_cfg(dr_nav_cfg_t *c) {
    memset(c, 0, sizeof(*c));
    c->sigma_acc  = 0.15;                 /* m/s^2/sqrt(Hz): vibration + road slope error */
    c->sigma_gyro = 0.05 * D2R;           /* rad/s/sqrt(Hz): ARW 0.0038 dps/sqrt(Hz) + tilt error */
    c->sigma_slip = 0.05;                 /* m/s/sqrt(Hz): lateral slip, lever arm */
    c->rw_bg      = 0.0005 * D2R;         /* thermal drift ~0.008 dps/degC (bench, 2026-10-30) */
    c->rw_ba      = 0.002;
    c->rw_sg      = 1e-5;
    c->zupt_sigma = 0.02;
    c->zaru_sigma = 0.01 * D2R;           /* 1 s mean of the yaw rate at standstill */
    c->nis_gate   = 13.8;                 /* chi2, 2 dof, 99.9 % */
    c->reanchor_after = 5;
    c->init_sigma_bg = 0.02 * D2R;        /* SFLP bias already removed: residual is small */
    c->init_sigma_ba = 0.10;
    c->init_sigma_sg = 0.01;
}

void dr_nav_init(dr_nav_t *s, const dr_nav_cfg_t *c) {
    memset(s, 0, sizeof(*s));
    s->cfg = *c;
    s->P[DR_PE][DR_PE] = s->P[DR_PN][DR_PN] = 100.0;
    s->P[DR_V][DR_V]   = 4.0;
    s->P[DR_PSI][DR_PSI] = M_PI * M_PI;
    s->P[DR_BG][DR_BG] = c->init_sigma_bg * c->init_sigma_bg;
    s->P[DR_BA][DR_BA] = c->init_sigma_ba * c->init_sigma_ba;
    s->P[DR_SG][DR_SG] = c->init_sigma_sg * c->init_sigma_sg;
}

static void zero_cross(dr_nav_t *s, int i) {
    int k;
    for (k = 0; k < DR_NX; k++)
        if (k != i) s->P[i][k] = s->P[k][i] = 0.0;
}

void dr_nav_set_position(dr_nav_t *s, double e, double n, double sigma) {
    s->x[DR_PE] = e;
    s->x[DR_PN] = n;
    zero_cross(s, DR_PE);
    zero_cross(s, DR_PN);
    s->P[DR_PE][DR_PE] = s->P[DR_PN][DR_PN] = sigma * sigma;
    s->positioned = 1;
    s->pos_reject_run = 0;
}

void dr_nav_align(dr_nav_t *s, double psi, double v, double sigma_psi, double sigma_v) {
    s->x[DR_PSI] = dr_nav_wrap_pi(psi);
    s->x[DR_V] = v;
    zero_cross(s, DR_PSI);
    zero_cross(s, DR_V);
    s->P[DR_PSI][DR_PSI] = sigma_psi * sigma_psi;
    s->P[DR_V][DR_V] = sigma_v * sigma_v;
    s->aligned = 1;
}

void dr_nav_seed_biases(dr_nav_t *s, double bg, double ba, double sg) {
    s->x[DR_BG] = bg;
    s->x[DR_BA] = ba;
    s->x[DR_SG] = sg;
}

void dr_nav_predict(dr_nav_t *s, double a_fwd, double w_up, double dt) {
    double F[DR_NX][DR_NX], FP[DR_NX][DR_NX];
    double psi = s->x[DR_PSI], v = s->x[DR_V];
    double c = cos(psi), sn = sin(psi);
    double w = (1.0 + s->x[DR_SG]) * w_up - s->x[DR_BG];
    const dr_nav_cfg_t *q = &s->cfg;
    int i, j, k;

    if (dt <= 0.0 || dt > 0.2) return;

    s->x[DR_V] += (a_fwd - s->x[DR_BA]) * dt;
    s->x[DR_PSI] = dr_nav_wrap_pi(psi + w * dt);
    if (s->aligned) {
        s->x[DR_PE] += v * c * dt;
        s->x[DR_PN] += v * sn * dt;
    }

    memset(F, 0, sizeof(F));
    for (i = 0; i < DR_NX; i++) F[i][i] = 1.0;
    if (s->aligned) {
        F[DR_PE][DR_V] = c * dt;   F[DR_PE][DR_PSI] = -v * sn * dt;
        F[DR_PN][DR_V] = sn * dt;  F[DR_PN][DR_PSI] =  v * c * dt;
    }
    F[DR_V][DR_BA] = -dt;
    F[DR_PSI][DR_BG] = -dt;
    F[DR_PSI][DR_SG] = w_up * dt;

    for (i = 0; i < DR_NX; i++)
        for (j = 0; j < DR_NX; j++) {
            double a = 0.0;
            for (k = 0; k < DR_NX; k++) a += F[i][k] * s->P[k][j];
            FP[i][j] = a;
        }
    for (i = 0; i < DR_NX; i++)
        for (j = 0; j < DR_NX; j++) {
            double a = 0.0;
            for (k = 0; k < DR_NX; k++) a += FP[i][k] * F[j][k];
            s->P[i][j] = a;
        }
    s->P[DR_PE][DR_PE]   += q->sigma_slip * q->sigma_slip * dt;
    s->P[DR_PN][DR_PN]   += q->sigma_slip * q->sigma_slip * dt;
    s->P[DR_V][DR_V]     += q->sigma_acc * q->sigma_acc * dt;
    s->P[DR_PSI][DR_PSI] += q->sigma_gyro * q->sigma_gyro * dt;
    s->P[DR_BG][DR_BG]   += q->rw_bg * q->rw_bg * dt;
    s->P[DR_BA][DR_BA]   += q->rw_ba * q->rw_ba * dt;
    s->P[DR_SG][DR_SG]   += q->rw_sg * q->rw_sg * dt;
    if (!s->aligned && s->P[DR_PSI][DR_PSI] > M_PI * M_PI) s->P[DR_PSI][DR_PSI] = M_PI * M_PI;
}

/* Generic EKF update for m <= 2 rows (Joseph form). y = innovation.
 * Returns the NIS; applies only if apply != 0. */
static double kf_update(dr_nav_t *s, int m, double H[2][DR_NX], const double y[2], const double R[2], int apply) {
    double PHt[DR_NX][2], S[2][2], Si[2][2], K[DR_NX][2], nis;
    int i, j, k;

    for (i = 0; i < DR_NX; i++)
        for (j = 0; j < m; j++) {
            double a = 0.0;
            for (k = 0; k < DR_NX; k++) a += s->P[i][k] * H[j][k];
            PHt[i][j] = a;
        }
    for (i = 0; i < m; i++)
        for (j = 0; j < m; j++) {
            double a = (i == j) ? R[i] : 0.0;
            for (k = 0; k < DR_NX; k++) a += H[i][k] * PHt[k][j];
            S[i][j] = a;
        }
    if (m == 1) {
        if (S[0][0] <= 0.0) return INFINITY;
        Si[0][0] = 1.0 / S[0][0];
        nis = y[0] * y[0] * Si[0][0];
    } else {
        double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (!(det > 1e-18)) return INFINITY;
        Si[0][0] = S[1][1] / det; Si[1][1] = S[0][0] / det;
        Si[0][1] = -S[0][1] / det; Si[1][0] = -S[1][0] / det;
        nis = y[0] * (Si[0][0] * y[0] + Si[0][1] * y[1]) + y[1] * (Si[1][0] * y[0] + Si[1][1] * y[1]);
    }
    if (!apply) return nis;

    for (i = 0; i < DR_NX; i++)
        for (j = 0; j < m; j++) {
            double a = 0.0;
            for (k = 0; k < m; k++) a += PHt[i][k] * Si[k][j];
            K[i][j] = a;
        }
    for (i = 0; i < DR_NX; i++) {
        double dx = 0.0;
        for (j = 0; j < m; j++) dx += K[i][j] * y[j];
        s->x[i] += dx;
    }
    s->x[DR_PSI] = dr_nav_wrap_pi(s->x[DR_PSI]);

    {   /* P = (I-KH) P (I-KH)^T + K R K^T */
        double A[DR_NX][DR_NX], AP[DR_NX][DR_NX], Pn[DR_NX][DR_NX];
        for (i = 0; i < DR_NX; i++)
            for (j = 0; j < DR_NX; j++) {
                double a = (i == j) ? 1.0 : 0.0;
                for (k = 0; k < m; k++) a -= K[i][k] * H[k][j];
                A[i][j] = a;
            }
        for (i = 0; i < DR_NX; i++)
            for (j = 0; j < DR_NX; j++) {
                double a = 0.0;
                for (k = 0; k < DR_NX; k++) a += A[i][k] * s->P[k][j];
                AP[i][j] = a;
            }
        for (i = 0; i < DR_NX; i++)
            for (j = 0; j < DR_NX; j++) {
                double a = 0.0;
                for (k = 0; k < DR_NX; k++) a += AP[i][k] * A[j][k];
                for (k = 0; k < m; k++) a += K[i][k] * R[k] * K[j][k];
                Pn[i][j] = a;
            }
        for (i = 0; i < DR_NX; i++)
            for (j = 0; j < DR_NX; j++) s->P[i][j] = 0.5 * (Pn[i][j] + Pn[j][i]);
    }
    return nis;
}

int dr_nav_update_pos(dr_nav_t *s, double e, double n, double sigma, double *nis_out) {
    double H[2][DR_NX] = { { 0 } }, y[2], R[2], nis;
    if (!s->positioned) {
        dr_nav_set_position(s, e, n, sigma);
        if (nis_out) *nis_out = 0.0;
        return DR_UPD_REANCHORED;
    }
    if (!s->aligned) {                 /* without a heading the position cannot be propagated */
        dr_nav_set_position(s, e, n, sigma);
        if (nis_out) *nis_out = 0.0;
        return DR_UPD_SKIPPED;
    }
    H[0][DR_PE] = 1.0; H[1][DR_PN] = 1.0;
    y[0] = e - s->x[DR_PE]; y[1] = n - s->x[DR_PN];
    R[0] = R[1] = sigma * sigma;
    nis = kf_update(s, 2, H, y, R, 0);
    if (nis_out) *nis_out = nis;
    if (nis > s->cfg.nis_gate) {
        if (++s->pos_reject_run >= s->cfg.reanchor_after) {
            /* GNSS has disagreed consistently: the DR solution drifted (long
             * outage) or the GNSS jumped and stayed. Trust GNSS again. */
            dr_nav_set_position(s, e, n, sigma);
            s->n_reanchor++;
            return DR_UPD_REANCHORED;
        }
        return DR_UPD_REJECTED;
    }
    s->pos_reject_run = 0;
    kf_update(s, 2, H, y, R, 1);
    return DR_UPD_APPLIED;
}

int dr_nav_update_vel(dr_nav_t *s, double ve, double vn, double sigma, double *nis_out) {
    double H[2][DR_NX] = { { 0 } }, y[2], R[2], nis;
    double psi = s->x[DR_PSI], v = s->x[DR_V], c = cos(psi), sn = sin(psi);
    if (!s->aligned) return DR_UPD_SKIPPED;
    H[0][DR_V] = c;  H[0][DR_PSI] = -v * sn;
    H[1][DR_V] = sn; H[1][DR_PSI] =  v * c;
    y[0] = ve - v * c; y[1] = vn - v * sn;
    R[0] = R[1] = sigma * sigma;
    nis = kf_update(s, 2, H, y, R, 0);
    if (nis_out) *nis_out = nis;
    if (nis > 4.0 * s->cfg.nis_gate) return DR_UPD_REJECTED;   /* loose: velocity re-aligns heading */
    kf_update(s, 2, H, y, R, 1);
    return DR_UPD_APPLIED;
}

int dr_nav_update_speed(dr_nav_t *s, double speed, double sigma) {
    double H[2][DR_NX] = { { 0 } }, y[2], R[2];
    double sgn = s->x[DR_V] < 0.0 ? -1.0 : 1.0;
    H[0][DR_V] = sgn;
    y[0] = speed - fabs(s->x[DR_V]);
    R[0] = sigma * sigma;
    kf_update(s, 1, H, y, R, 1);
    return DR_UPD_APPLIED;
}

void dr_nav_update_zupt(dr_nav_t *s) {
    double H[2][DR_NX] = { { 0 } }, y[2], R[2];
    H[0][DR_V] = 1.0;
    y[0] = -s->x[DR_V];
    R[0] = s->cfg.zupt_sigma * s->cfg.zupt_sigma;
    kf_update(s, 1, H, y, R, 1);
}

void dr_nav_update_zaru(dr_nav_t *s, double w_up_still) {
    /* At standstill psi' = 0 = (1+sg) w - bg  ->  measurement w = bg/(1+sg) ~ bg */
    double H[2][DR_NX] = { { 0 } }, y[2], R[2];
    H[0][DR_BG] = 1.0;
    y[0] = w_up_still - s->x[DR_BG];
    R[0] = s->cfg.zaru_sigma * s->cfg.zaru_sigma;
    kf_update(s, 1, H, y, R, 1);
}
