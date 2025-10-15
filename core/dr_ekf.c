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
    float sp = ekf->cfg.p0_pos * ekf->cfg.p0_pos;
    float sv = ekf->cfg.p0_vel * ekf->cfg.p0_vel;
    float sa = ekf->cfg.p0_ang * ekf->cfg.p0_ang;
    float sba = ekf->cfg.p0_ba * ekf->cfg.p0_ba;
    float sbg = ekf->cfg.p0_bg * ekf->cfg.p0_bg;
    memset(P, 0, sizeof(float) * DR_STATE_ERR_DIM * DR_STATE_ERR_DIM);
    for( i = 0; i < 3; i++ ) {
        P[IDX(i,i)] = sp;           // pos
        P[IDX(i+3,i+3)] = sv;       // vel
        P[IDX(i+6,i+6)] = sa;       // att
        P[IDX(i+9,i+9)] = sbg;      // gyro bias
        P[IDX(i+12,i+12)] = sba;    // acc bias
    } 
}

/** @brief Initialize EKF with config and optional initial state
 * @param ekf (out) EKF handle to initialize.
 * @param cfg (in) EKF configuration (noise PSDs, gravity, intial std devs)
 * @param X0 optional initial nominal state (if NULL, indetity quaternion and zeros)
 * @return void
 */

 void dr_ekf_init(dr_ekf_t *ekf, const dr_ekf_config_t *cfg, const dr_nominal_state_t *x0)
 {
    memset(ekf, 0, sizeof(dr_ekf_t));
    ekf->cfg = *cfg;
   if ( x0 ) {
    ekf->x = *x0;
   }
   else {
    ekf->x.q = dr_quat_identity();
    if( ekf->cfg.gravity[0] == 0 && ekf->cfg.gravity[1] == 0 && ekf->cfg.gravity[2] == 0 ) {
        ekf->cfg.gravity[2] = -DR_GRAVITY_F; // default gravity down
    }
   }
   dr_ekf_init_covariance(ekf);
 }

 /** @brief EKF predict step (nominal mechanization + covariance propagation).
  *  - integrates quaternion from gyro, velocity position from accel (in world gravity frame)
  * - Builds F and G, computes P = FPF' + GQG' then P <- P + P'dt
  * @param ekf EKF handle (in/out)
  * @param imu IMU sample in SI units (rad/s, m/s^2 )
  * @param dt timestep (s)
  * @return void
  */

  void dr_ekf_predict(dr_ekf_t *ekf, const dr_imu_sample_t *imu, float dt)
  {
    dr_nominal_state_t *x = &ekf->x;
    int i;
    dr_vec3f_t aw;
    // Nominal state mechanization
    dr_vec3f_t omega = dr_v3(imu->gyro[0] - x->bg[0], imu->gyro[1] - x->bg[1], imu->gyro[2] - x->bg[2]);
    dr_vec3f_t acc_b = dr_v3(imu->accel[0] - x->ba[0], imu->accel[1] - x->ba[1], imu->accel[2] - x->ba[2]);
    float F[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM] = { 0 };
    float G[DR_STATE_ERR_DIM * 12] = { 0 };
    float Rwb[9]; // Rotation body to world
    float accx[9]; // Skew symmetric acc
    float omegax[9]; // Skew symmetric omega
    float Raccx[9] = {0}; // Rwb * accx
    float Qc[12 * 12] = { 0 };
    float sg2, sa2, sbg2, sba2;
    int r, c;
    float *P = ekf->P;
    float FP[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float Ft[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float PFt[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float GQc[DR_STATE_ERR_DIM * 12];
    float Gt[12 * DR_STATE_ERR_DIM];
    float GQcGt[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float Pdot[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];

    /* Nominal Propogation */
    x->q = dr_q_integrate_gyro(x->q, omega, dt);
    aw = dr_v3_add(dr_q_rotate(x->q, acc_b), dr_v3(ekf->cfg.gravity[0], ekf->cfg.gravity[1], ekf->cfg.gravity[2]));
    x->v[0] += aw.x * dt; x->v[1] += aw.y * dt; x->v[2] += aw.z * dt;
    x->p[0] += x->v[0] * dt; x->p[1] += x->v[1] * dt; x->p[2] += x->v[2] * dt;

    /* Build F and G */
    dr_R_from_q(x->q, Rwb);
    dr_skew3(acc_b, accx);
    dr_skew3(omega, omegax);
    for (i = 0; i < 3; i++) {
        // F matrix
        F[IDX(i, i + 3)] = 1.0f; // dp/dv
    }
    dr_mat_mul(Raccx, Rwb, accx, 3, 3, 3); 
    for ( r = 0; r < 3; r++ )
        for ( c = 0; c < 3; c++ )
            {
                F[IDX(r + 3, c + 6)] = -Raccx[r*3 + c]; // dv/dtheta
            }
    for (i = 0; i < 3; i++) {
        F[IDX(i + 3, i + 12)] = -1.0f; // dv/dba
    }
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++)
        {
            F[IDX(r + 6, c + 6)] = -omegax[r * 3 + c]; // dtheta/dtheta
        }
    for (i = 0; i < 3; i++) {
        F[IDX(i + 6, i + 9)] = -1.0f; // dtheta/dbg
    }

    /* Noise Mapping G: Columns [ n_g, n_a, n_bg, n_ba]*/
    for ( i = 0; i < 3; i++ )
        G[(6+i)*12 + (0 + i)] = 1.0f;   /* n_g -> dtheta */
    for ( r = 0; r < 3; r++ )
        for ( c = 0; c < 3; c++ )
            {
                G[(3 + r)*12 + (3 + c)] = Rwb[r*3 + c]; // n_a -> dv
            }
    for ( i = 0; i < 3; i++ )
        G[(9 + i)*12 + (6 + i)] = 1.0f;   /* n_bg -> dbg */
    for ( i = 0; i < 3; i++ )
        G[(12 + i)*12 + (9 + i)] = 1.0f;   /* n_ba -> dba */

    /* Continuous noise Covariance Qc (diags of PSDs)*/
    
    sg2 = ekf->cfg.sigma_g * ekf->cfg.sigma_g;
    sa2 = ekf->cfg.sigma_a * ekf->cfg.sigma_a;
    sbg2 = ekf->cfg.sigma_bg * ekf->cfg.sigma_bg;
    sba2 = ekf->cfg.sigma_ba * ekf->cfg.sigma_ba;

    for( i = 0; i < 3; i++ ) {
        Qc[i*12 + i] = sg2;
        Qc[(i+3)*12 + (i+3)] = sa2;
        Qc[(i+6)*12 + (i+6)] = sbg2;
        Qc[(i+9)*12 + (i+9)] = sba2;
    }

    /* Covariance derivative and Euler step */
    dr_mat_mul(FP, F, P, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);
    dr_mat_transpose(Ft, F, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);
    dr_mat_mul(PFt, P, Ft, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);
    dr_mat_mul(GQc, G, Qc, DR_STATE_ERR_DIM, 12, 12);
    dr_mat_transpose(Gt, G, DR_STATE_ERR_DIM, 12);
    dr_mat_mul(GQcGt, GQc, Gt, DR_STATE_ERR_DIM, 12, DR_STATE_ERR_DIM);

    dr_mat_add(Pdot, FP, PFt, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);
    dr_mat_add(Pdot, Pdot, GQcGt, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);

    for( i = 0; i < DR_STATE_ERR_DIM * DR_STATE_ERR_DIM; i++ ) {
        P[i] += Pdot[i] * dt;
    }


}

/** @brief EKF Measurement update using GPS position in ENU (3D).
 *  H = [1, 0, 0, 0, 0], updates p, v, q , bg, ba via error state injection and
 * Joseph form covariance updata. 
 * @param ekf EKF handle (in/out)
 * @param z GPS position measurement in ENU (m)
 * @param Rpos 3x3 GPS position measurement covariance (row major, m^2)
 * @return 0 on success, -1 if innovation covariance is singular (no update performed)
 */

 int dr_ekf_update_gps_pos(dr_ekf_t *ekf, const dr_gps_pos_t *z, const float Rpos[9])
 {
    float H[3 * DR_STATE_ERR_DIM] = { 0 };
    float y[3]; // innovation
    float HP[3 * DR_STATE_ERR_DIM];
    float Ht[DR_STATE_ERR_DIM * 3];
    float HPHt[3 * 3];
    float S[3 * 3];
    float Sinv[3 * 3];
    float PHt[DR_STATE_ERR_DIM * 3];
    float K[DR_STATE_ERR_DIM * 3]; // Kalman Gain
    float dx[DR_STATE_ERR_DIM] = {0}; // error state
    float I15[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM] = { 0 };
    float KH[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float I_KH[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float tmp[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float I_KH_T[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float Pnew[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    float KR[DR_STATE_ERR_DIM * 3];
    float KT[3 * DR_STATE_ERR_DIM];
    float KRT[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];
    dr_quatf_t dq;
    int i;

    /* Build H matrix */
    for (i = 0; i < 3; i++) {
        H[i * DR_STATE_ERR_DIM + i] = 1.0f; // pos
    }

    /* Innovation y = z - Hx */
    y[0] = z->pos[0] - ekf->x.p[0];
    y[1] = z->pos[1] - ekf->x.p[1];
    y[2] = z->pos[2] - ekf->x.p[2];

    /* Innovation covariance S = HPH' + R */
    dr_mat_mul(HP, H, ekf->P, 3, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);
    dr_mat_transpose(Ht, H, 3, DR_STATE_ERR_DIM);
    dr_mat_mul(HPHt, HP, Ht, 3, DR_STATE_ERR_DIM, 3);
    for ( i = 0; i < 9; i++ ) {
        S[i] = HPHt[i] + Rpos[i];
    }
    if( dr_mat3_inv(Sinv, S) != 0 ) {
        return -1; // singular innovation covariance
    }
    dr_mat_mul(PHt, ekf->P, Ht, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM , 3);
    /* Kalman Gain K = PH'S^-1 */
    dr_mat_mul(K, PHt, Sinv, DR_STATE_ERR_DIM, 3, 3);

    for( i = 0; i < DR_STATE_ERR_DIM; i++ ) {
        dx[i] = K[i*3 + 0] * y[0] + K[i*3 + 1] * y[1] + K[i*3 + 2] * y[2];
    }
    /* Inject error state */
    for ( i = 0; i < 3; i++ ) {
        ekf->x.p[i] += dx[i];
        ekf->x.v[i] += dx[i + 3];
    }
    dq = dr_q_from_rotvec(dr_v3(dx[6], dx[7], dx[8]));
    ekf->x.q = dr_q_normalize(dr_q_mult(ekf->x.q, dq));
    for( i = 0; i < 3; i++) ekf->x.bg[i] += dx[i + 9];
    for( i = 0; i < 3; i++) ekf->x.ba[i] += dx[i + 12];

    /* Joseph-stable covariance update */
    dr_mat_set_identity(I15, DR_STATE_ERR_DIM);
    dr_mat_mul(KH, K, H, DR_STATE_ERR_DIM, 3, DR_STATE_ERR_DIM);
    for( i = 0; i < DR_STATE_ERR_DIM * DR_STATE_ERR_DIM; i++ ) {
        I_KH[i] = I15[i] - KH[i];
    }
    dr_mat_mul(tmp, I_KH, ekf->P, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);
    dr_mat_transpose(I_KH_T, I_KH, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);
    dr_mat_mul(Pnew, tmp, I_KH_T, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM, DR_STATE_ERR_DIM);
    dr_mat_mul(KR, K, (float *)Rpos, DR_STATE_ERR_DIM, 3, 3);
    dr_mat_transpose(KT, K, DR_STATE_ERR_DIM, 3);
    dr_mat_mul(KRT, KR, KT, DR_STATE_ERR_DIM, 3, DR_STATE_ERR_DIM);
    for( i = 0; i < DR_STATE_ERR_DIM * DR_STATE_ERR_DIM; i++ ) {
        ekf->P[i] = Pnew[i] + KRT[i];
    }
    return 0;
    
 }


