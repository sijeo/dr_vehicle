#ifndef DR_NAV_H
#define DR_NAV_H

/*
 * dr_nav — planar dead-reckoning EKF for a road vehicle.
 *
 * Inputs come from the IMU chain (imu_core + imu_fusion):
 *   a_fwd  forward specific force in the heading-free LEVEL frame, gravity
 *          removed with the AHRS attitude (m/s^2)
 *   w_up   rotation rate about the local vertical, gyro bias-corrected with
 *          the SFLP estimate (rad/s, + = counter-clockwise / left turn)
 *
 * State (7):
 *   pE, pN   position in the local ENU plane (m)
 *   v        speed along the heading (m/s, negative when reversing)
 *   psi      heading, ENU convention: 0 = East, +pi/2 = North (rad)
 *   bg       residual yaw-rate bias (rad/s)   -> heading drift correction
 *   ba       forward accel bias (m/s^2)
 *   sg       yaw-rate scale error (-)         -> turn accuracy during outages
 *
 * Motion model: the vehicle moves along its heading (non-holonomic), so no
 * separate NHC update is needed:
 *   pE' = v cos(psi), pN' = v sin(psi), v' = a_fwd - ba,
 *   psi' = (1 + sg) w_up - bg
 *
 * Measurements: GNSS position, GNSS velocity (vE, vN) or speed only at low
 * speed, ZUPT (v = 0) and ZARU (yaw rate = 0) while stationary. ZARU is what
 * removes the residual heading drift (~0.3 deg/min on the bench) at every
 * stop; GNSS velocity does it while driving.
 *
 * Pure C, no OS dependencies (unit-testable on a PC).
 */

#ifdef __cplusplus
extern "C" {
#endif

enum { DR_PE = 0, DR_PN, DR_V, DR_PSI, DR_BG, DR_BA, DR_SG, DR_NX };

typedef struct {
    double sigma_acc;        /* forward accel noise density, m/s^2/sqrt(Hz) */
    double sigma_gyro;       /* yaw-rate noise density, rad/s/sqrt(Hz) */
    double sigma_slip;       /* lateral slip / model error, m/s/sqrt(Hz) */
    double rw_bg;            /* yaw-bias random walk, rad/s/sqrt(s) */
    double rw_ba;            /* accel-bias random walk, m/s^2/sqrt(s) */
    double rw_sg;            /* scale random walk, 1/sqrt(s) */
    double zupt_sigma;       /* m/s */
    double zaru_sigma;       /* rad/s */
    double nis_gate;         /* chi-square gate for 2-D GNSS updates */
    int    reanchor_after;   /* consecutive rejected GNSS positions before re-anchoring */
    double init_sigma_bg, init_sigma_ba, init_sigma_sg;
} dr_nav_cfg_t;

typedef struct {
    dr_nav_cfg_t cfg;
    double x[DR_NX];
    double P[DR_NX][DR_NX];
    int positioned;          /* position initialised from GNSS */
    int aligned;             /* heading initialised */
    int pos_reject_run;
    unsigned n_reanchor;
} dr_nav_t;

/* Return codes of the GNSS updates */
enum { DR_UPD_APPLIED = 0, DR_UPD_REJECTED = 1, DR_UPD_REANCHORED = 2, DR_UPD_SKIPPED = 3 };

void dr_nav_default_cfg(dr_nav_cfg_t *c);
void dr_nav_init(dr_nav_t *s, const dr_nav_cfg_t *c);

/* Position fix before alignment, or a hard reset of the position. */
void dr_nav_set_position(dr_nav_t *s, double e, double n, double sigma);
/* Heading initialisation (GNSS course while driving, or a saved heading). */
void dr_nav_align(dr_nav_t *s, double psi, double v, double sigma_psi, double sigma_v);
/* Seed the slowly varying states (e.g. from the last run). */
void dr_nav_seed_biases(dr_nav_t *s, double bg, double ba, double sg);

void dr_nav_predict(dr_nav_t *s, double a_fwd, double w_up, double dt);

int dr_nav_update_pos(dr_nav_t *s, double e, double n, double sigma, double *nis);
int dr_nav_update_vel(dr_nav_t *s, double ve, double vn, double sigma, double *nis);
int dr_nav_update_speed(dr_nav_t *s, double speed, double sigma);
void dr_nav_update_zupt(dr_nav_t *s);
void dr_nav_update_zaru(dr_nav_t *s, double w_up_still);

double dr_nav_wrap_pi(double a);

#ifdef __cplusplus
}
#endif
#endif
