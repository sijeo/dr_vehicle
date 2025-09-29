#include <string.h> // memcpy
#include "dr_ekf.h"
#include "dr_math.h"


#define IDX(i, j) ((i)*DR_STATE_ERR_DIM + (j))

/** @brief Initialize diagonal covariance P from Initial standard deviation in cfg
 * @param ekf (out) EKF handle (uses ekf->cfg)
 * @return void
 */
static void dr_ekf_init_covariance(dr_ekf_t* ekf)
{
    float *P = ekf->P; 
    int i;
    memset(P, 0, sizeof(float) * DR_STATE_ERR_DIM * DR_STATE_ERR_DIM);
    float sp = ekf->cfg.p0_pos * ekf->cfg.p0_pos;
    float sv = ekf->cfg.p0_vel * ekf->cfg.p0_vel;
    float sa = ekf->cfg.p0_ang * ekf->cfg.p0_ang;
    float sba = ekf->cfg.p0_ba * ekf->cfg.p0_ba;
    float sbg = ekf->cfg.p0_bg * ekf->cfg.p0_bg;
    for( i = 0; i < 3; i++ ) {
        P[IDX(i,i)] = sp;           // pos
        P[IDX(i+3,i+3)] = sv;       // vel
        P[IDX(i+6,i+6)] = sa;       // att
        P[IDX(i+9,i+9)] = sbg;      // gyro bias
        P[IDX(i+12,i+12)] = sba;    // acc bias
    } 
}