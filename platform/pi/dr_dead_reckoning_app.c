// dr_dead_reckoning_app.c
//
// RPi3B + MPU6050 + Neo6M dead-reckoning navigation app
// - Waits for first GNSS fix -> sets ENU origin
// - Attitude EKF (quaternion) from IMU
// - Position/velocity EKF (6-State) in ENU
// - GNSS position + velocity + yaw (course over ground )
// - HDOP-adaptive GNSS covariance
// - NHC, ZUPT, velocity decay
// - Q inflation during GNSS outrage
// - Fade-in of GNSS after outage
//
// NOTE: This file expects these headers to exist and be implemented:
// - mpu6050_ioctl.h    (with struct mpu6050_sample, device path, IOCTLs)
// - neo6m_gnss_ioctl.h (with struct neo6m_gnss_fix, IOCTL NEO6M_GNSS_IOC_GET_FIX)
// - dr_math.h / dr_types.h (quaternions, matrices, vector helpers )
//
// 

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>

// ================Project Headers (adjust path as needed )============== //
#include "mpu6050_ioctl.h"
#include "neo6m_gnss_ioctl.h"
#include "../core/dr_math.h"
#include "../core/dr_types.h"

//==========Device Paths (adjust if needed )=======//
#define IMU_DEVICE_PATH     "/dev/mpu6050-0"
#define NEO6M_DEVICE_PATH   "/dev/neo6m-0"

//====== Constant & tuning =======//

#define IMU_HZ          100.0f
#define DT_IMU          (1.0/IMU_HZ)
#define GNSS_TIMEOUT_S  2.0f

#define GRAVITY         9.80665f
/**
 * @brief Value of PI
 */
#define M_PI 3.14159265358979323846
#define DEG2RAD        (M_PI/180.0) 

// Yaw dead-band (when nearly still )
#define YAW_DEADBAND_RAD    (0.03f)          // ~1.7 deg/s
#define ACC_STILL_TOL       (0.1f*GRAVITY)
#define GYRO_STILL_TOL_RAD  (3.0f * DEG2RAD)

// ZUPT (Zero-Velocity update) detection
#define ZUPT_ACC_THR        (0.4f)
#define ZUPT_GYRO_THR       (5.0f*DEG2RAD)
#define ZUPT_COUNT_REQUIRED  5
#define VEL_DECAY            0.98f
#define VEL_EPS              1e-3f

// Adaptive Q Scaling (tiers by outage length)
#define TIER_A      2.0f
#define TIER_B      10.0f
#define TIER_C      60.0f
#define QSCL_A      2.0f
#define QSCL_B      5.0f
#define QSCL_C      10.0f
#define QSCL_D      20.0f

/* GNSS gating and Fade in */
#define CHI2_3DOF_GATE      16.2f       // ~99.5% for 3 DOF
#define R_GNSS_POS_VAR      4.0f        // base position var (m^2) before HDOP scaling
#define FADE_IN_FACT_INIT   4.0f        // R = FADE_IN_FACT_INIT * R_base when GNSS just returns
#define FADE_IN_STEPS       3           // Number of GNSS updates to Fade in

// NHC Measurement Noise (m/s^2)
#define R_NHC_VY    0.01f
#define R_NHC_VZ    0.04f

// ZUPT measurement noise (m/s)^2
#define R_ZUPT_V    (0.0004f)

// GNSS Velocity measurement Noise (m/s)^2
#define RV_VEL_E    (0.09f)     // (0.3 m/s)^2
#define RV_VEL_N    (0.09f)     //
#define RV_VEL_U    (0.36f)     // (0.6 m/s)^2

/* Yaw fusion from GNSS Heading */
#define YAW_FUSION_SPEED_MIN    (1.0f)  // m/s; only fuse heading above this 
#define YAW_FUSION_HDOP_MAX     (4.0f)  // only fuse heading when HDOP <= this
#define YAW_FUSION_ALPHA        (0.05f) // 0..1 fraction of yaw error corrected per update



/** Time Utilities  */
static inline uint64_t monotonic_ns( void ) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;   
}

static inline double now_sec( void ){
    return monotonic_ns() * 1e-9;
}

/* Attitude EKF (4-state quaternion )*/
typedef struct {
    dr_quatf_t q; /*< body->world*/
    float P[16];  /*< 4x4 Covariance */
} ekf4_t;

static void ekf4_init(ekf4_t *e) {
    e->q = dr_quat_identity();
    memset(e->P, 0, sizeof(e->P));
    /* Small initial uncertainity */
    e->P[0] = e->P[5] = e->P[10] = e->P[15] = 1e-3f;
}

static void quat_deriv( const dr_quatf_t *q, const float w[3], dr_quatf_t *dqdt ){
    float wx = w[0], wy = w[1], wz = w[2];
    dqdt->w = -0.5f * (wx*q->x + wy*q->y + wz*q->z);
    dqdt->x = 0.5f * (wx*q->w + wz*q->y - wy*q->z);
    dqdt->y = 0.5f * (wy*q->w - wz*q->x + wx*q->z);
    dqdt->z = 0.5f * (wz*q->w + wy*q->x - wx*q->y);
}

static void gravity_body_from_q( const dr_quatf_t *q, float gb[3]) {
    dr_quatf_t qc = (dr_quatf_t){q->w, q->x, q->y, q->z};
    dr_vec3f_t gw = dr_v3(0, 0, 1);
    dr_vec3f_t gbv = dr_q_rotate(qc, gw);
    float n = sqrtf(gbv.x*gbv.x + gbv.y*gbv.y + gbv.z*gbv.z);
    gb[0] = gbv.x / n;
    gb[1] = gbv.y / n;
    gb[2] = gbv.z / n;
}

static ekf4_predict( ekf4_t *e, const float gyro[3], float dt ){
    const float q_proc = 1e-6f;
    dr_quatf_t dqdt;
    quat_deriv(&e->q, gyro, &dqdt);

    dr_quatf_t qp = {
        e->q.w + dqdt.w*dt,
        e->q.x + dqdt.x*dt,
        e->q.y + dqdt.y*dt,
        e->q.z + dqdt.z*dt
    };

    e->q = dr_q_normalize(qp);
    e->P[0] += q_proc;
    e->P[5] += q_proc;
    e->P[10] += q_proc;
    e->P[15] += q_proc;

}

static void ekf4_update_accel(ekf4_t *e, const float acc[3]) {
    float an = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if ( an < 0.5f * GRAVITY || an > 1.5f*GRAVITY ) {
        return;
    }

    float z[3] = { acc[0]/an, acc[1]/an, acc[2]/an };
    float h[3];
    gravity_body_from_q(&e->q, h);

    flaot y[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };

    /* Numerical Jacobian H(3x4) */
    float H[12];
    float base[4] = { e->q.w, e->q.x, e->q.y, e->q.z };
    const float eps = 1e-6f;
    int r, c, k;

    for( c=0; c<4; c++ ){
        float qp[4] = { base[0], base[1], base[2], base[3] };
        float qm[4] = { base[0], base[1], base[2], base[3] };

        qp[c] += eps;
        qm[c] -= eps;
        dr_quatf_t qpp = dr_q_normalize((dr_quatf_t){qp[0], qp[1], qp[2], qp[3]});
        dr_quatf_t qmm = dr_q_normalize((dr_quatf_t){qm[0], qm[1], qm[2], qm[3]});

        float hp[3], hm[3];
        gravity_body_from_q(&qpp, hp);
        gravity_body_from_q(&qmm, hm);
        H[0*4 + c] = (hp[0] - hm[0])/(2*eps);
        H[1*4 + c] = (hp[1] - hm[1])/(2*eps);
        H[2*4 + c] = (hp[2] - hm[2])/(2*eps);
    }

    const float r_meas = 0.0004f;
    float *p = e->P;
    float HP[12];
    for( r=0; r<3; r++) {
        for(c=0; c<4; c++){
            float s = 0.0f;
            for( k=0; k<4; k++){
                s += H[r*4 + k] * p[k*4 + c];

            }
            HP[r*4 + c] = s;
        }
    }

    float S[9];
    for( r=0; r<3; r++){
        for( c=0; c<3; c++){
            float s=0.0f;
            for( k=0; k<4; k++){
                s += HP[r*4 + k] * H[c*4 + k];

            }
            S[r*3 + c] = s + (r == c ? r_meas : 0);
        }
    }
    float Sinv[9];
    if( dr_mat3_inv(Sinv, S)){
        return;
    }

    float PHt[12];
    for( r=0; r<4; r++) {
        for( c=0; c<3; c++) {
            float s = 0.0f;
            for( k=0; k<4; k++){
                s += p[r*4 + k]*H[c*4 + k];
            }
            PHt[r*3 + c] = s;
        }
    }

    float K[12];
    for( r=0; r<4; r++){
        for( c=0; c<3; c++){
            float s = 0.0f;
            for( k=0; k<3; k++){
                s += PHt[r*3 + k] * Sinv[k*3 + c];
            }
            K[r*3 + c] = s;
        }
    }

    float Ky[4] = {0};
    for( r=0; r<4; r++){
        for( c=0; c<3; c++){
            Ky[r] += K[r*3 + c]*y[c];
        }
    }

    dr_quatf_t qn = {
        e->q.w + Ky[0],
        e->q.x + Ky[1],
        e->q.y + Ky[2],
        e->q.z + Ky[3]
    };

    e->q = dr_q_normalize(qn);

    float KH[16];
    for( r=0; r<4; r++ ){
        for( c=0; c<4; c++ ){
            float s = 0.0f;
            for( k=0; k<3; k++ ){
                s += K[r*3 + k]*H[k*4 + c];

            }
            KH[r*4 + c] = s;
        }
    }

    float I_KH[16];
    for( r=0; r<4; r++ ){
        for( c=0; c<4; c++ ){
            I_KH[r*4 + c] = ( r==c ? 1.0f : 0.0f) - KH[r*4 + c];
        }
    }

    float Pn[16];
    for( r=0; r<4; r++){
        for( c=0; c<4; c++){
            float s = 0.0f;
            for( k=0; k<4; k++ ){
                s += I_KH[r*4 + k]*p[k*4 + c];
            }
            Pn[r*4 + c] = s;
        }
    }
    memcpy(p, Pn, sizeof(Pn));

}

/*===============Translational (p, v) state with 6x6 covariance =================*/
typedef struct {
    float p[3];      /**< ENU Position  */
    float v[3];     /**< ENU Velocity  */
    float P[36];    /**< 6x6 Covariance */
} pv6_t;

static void pv6_init(pv6_t *s) {
    memset(s, 0, sizeof(*s));
    int i;
    for( i=0; i<3; i++ ){
        s->P[i*6 + i] = 1e-2f;
    }
    for( i=0; i<3; i++) {
        s->P[(i+3)*6 + (i+3)] = 1e-2f;
    }
}

static void mat6_mul(const float *A, const float *B, float *C) {
    int r, c, k;
    for( r=0; r<6; r++) {
        for( c=0; c<6; c++) {
            float s = 0.0f;
            for( k=0; k<6; k++ ){
                s += A[r*6 + k]*B[k*6 + c];

            }
            C[r*6 + c] = s;
        }
    }


}

static void mat6_T(const float *A, float *AT ){
    int r, c, k;
    for( r=0; r<6; r++ ){
        for( c=0; c<6; c++ ){
            AT[c*6 + r] = A[r*6 + c];
        }
    }
}

static void mat6_add(float *A, const float *B) {
    int i;
    for( i=0; i<36; i++ ){
        A[i] += B[i];
    }
}

static void pv6_predict( pv6_t *s, const float a_w[3], float dt, float q_scale ) {
    // Integrate Kinematics
    s->p[0] += s->v[0]*dt + 0.5f*a_w[0]*dt*dt;
    s->p[1] += s->v[1]*dt + 0.5f*a_w[1]*dt*dt;
    s->p[2] += s->v[2]*dt + 0.5f*a_w[2]*dt*dt;
    s->v[0] += a_w[0]*dt;
    s->v[1] += a_w[1]*dt;
    s->v[2] += a_w[2]*dt;
    
    // F constant acceleration model
    float F[36] = {0};
    int i;
    for( i=0; i<3; i++) {
        F[i*6 + i] = 1.0f;
        F[i*6 + (i+3)] = dt;
        F[(i+3)*6 + (i+3)] = 1.0f;
    }

    /* Process Noise */
    const float sa2 = (0.15f*0.15f) * q_scale;
    float Qp = (1.0f/3.0f)*sa2*dt*dt*dt*dt;
    float Qpv = 0.5f*sa2*dt*dt*dt;
    float Qv = sa2*dt*dt;

    float Q[36] = {0};
    for(i=0; i<3; i++ ) {
        Q[i*6 + i]          = Qp;
        Q[i*6 + (i+3)]      = Qpv;
        Q[(i+3)*6 + i]      = Qpv;
        Q[(i+3)*6 + (i+3)]  = Qv;
    }

    float FP[36], FT[36], FPFT[36];
    mat6_mul(F, s->P, FP);
    mat6_T(F, FT);
    mat6_mul(FP, FT, FPFT);

    memcpy(s->P, FPFT, sizeof(FPFT));
    mat6_add(s->P, Q);
}

/* Position update: z_pos (ENU), H =[I 0], scalar Rpos (same for all )*/
static bool pv6_update_pos( pv6_t *s, const float z[3], float Rpos, float *out_NIS){
    float H[18] = {0};
    int i, j, r, c, k;
    for( i=0; i<3; i++ ){
        H[i*6 + i] = 1.0f;
    }

    float HP[18];
    for( r=0; r<3; r++ ){
        for( c=0; c<6; c++ ){
            float x = 0.0f;
            for( k=0; k<6; k++ ){
                x += H[r*6 + k] * s->P[k*6 + c];

            }
            H[r*6 + c] = x;
        }
    }

    float S[9];
    for( r=0; r<3; r++ ){
        for( c=0; c<3; c++ ){
            float x = 0.0f;
            for(k=0; k<6; k++){
                x += HP[r*6 + k]*H[c*6 + k];
            }
            S[r*3 + c] = x + (r == c ? Rpos : 0.0f);
        }
    }

    float Sinv[9];
    if (dr_mat3_inv(Sinv, S)){
        return false;
    }

    float h[3] = { s->p[0], s->p[1], s->p[2] };
    float nu[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };

    if( out_NIS ){
        float tmp[3] = {0};
        for( i=0; i<3; i++ )
        {
            for( j=0; j<3; j++){
                tmp[i] += nu[j] * Sinv[j*3 + i];
            }
        }
        *out_NIS = nu[0]*tmp[0] + nu[1]*tmp[1] + nu[2]*tmp[2];
    }

    float PHt[18];
    for(r=0; r<6; r++){
        for(c=0; c<3; c++){
            PHt[r*3 + c] = s->P[r*6 + c];
        }
    }

    float K[18];
    for( r=0; r<6; r++ ){
        for( c=0; c<3; c++ ){
           float x=0.0f;
           for( k=0; k<3; k++ ){
            x += PHt[r*3 + k]*Sinv[k*3 + c];
           }
           K[r*3 + c] = x;
        }
    }

    /* State Update */
    for( i=0; i<3; i++ ){
        s->p[i] += K[i*3 + 0] * nu[0] + K[i*3 + 1]*nu[1] + K[i*3 + 2]*nu[2];
        s->v[i] += K[(i+3)*3 + 0]*nu[0] + K[(i+3)*3 + 1]*nu[1] + K[(i+3)*3 + 2]*nu[2];
    }

    /* Covariance Update */
    float Hfull[18];
    memcpy(Hfull, H, sizeof(Hfull));
    float KH[36] = {0};
    for( r=0; r<6; r++ ){
        for( c=0; c<6; c++){
            float x = 0.0f;
            for( k=0; k<3; k++ ){
                x += K[r*3 + k]*Hfull[k*6 + c];
            }
            KH[r*6 + c] = x;
        }
    }

    float I_KH[36];
    for( r=0; r<6; r++ ){
        for( c=0; c<6; c++){
            I_KH[r*6 + c] = (r==c ? 1.0f : 0.0f) - KH[r*6 + c];
        }
    }

    float Pn[36];
    mat6_mul(I_KH, s->P, Pn);
    memcpy(s->P, Pn, sizeof(Pn));

    return true;

}

/* Velocity update ENU: z_vel, H = [0 I], Rv diag */
static bool pv6_update_vel( pv6_t *s, const float z[3], const float Rv[3], float *out_NIS){
    float H[18] = {0};
    int i, j, k, r, c;

    for( i=0; i<3; i++){
        H[i*6 + (i+3)] = 1.0f;
    }

    float HP[18];
    for( r=0; r<3; r++ ){
        for( c=0; c<6; c++){
            float x = 0.0f;
            for( k=0; k<6; k++){
                x += H[r*6 + k]*s->P[k*6 + c];
            }
            HP[r*6 + c] = x;
        }
    }

    float S[9];
    for( r=0; r<3; r++){
        for( c=0; c<3; c++ ){
            float x = 0.0f;
            for( k=0; k<6; k++){
                x += HP[r*6 + k]*H[c*6 + k];
            }
            if( r == c){
                x += Rv[r];
            }
            S[r*3 + c] = x;
        }
    }

    float Sinv[9];
    if( dr_mat3_inv( Sinv, S))
    {
        return false;
    }

    float h[3] = { s->v[0], s->v[1], s->v[2] };
    float nu[3] = { z[0]-h[0], z[1]-h[1], z[2]-h[2] };

    if( out_NIS ){
        float tmp[3] = {0};
        for( i=0; i<3; i++ ){
            for( j=0; j<3; j++ ){
                tmp[i] += nu[j]*Sinv[j*3 + i];
            }
        }
        *out_NIS = nu[0]*tmp[0] + nu[1]*tmp[1] + nu[2]*tmp[2];
    }

    float PHt[18];
    for( r=0; r<6; r++ ){
        for( c=0; c<3; c++ ){
            float x = 0.0f;
            for( k=0; k<6; k++ ){
                x += s->P[r*6 + k]*H[c*6 + k];
            }
            PHt[r*3 + c] = x;
        }
    }

    float K[18];
    for( r=0; r<6; r++ ){
        for( c=0; c<3; c++ ){
            float x = 0.0f;
            for( k=0; k<3; k++ ){
                x += PHt[r*3 + k]*Sinv[k*3 + c];
            }
            K[r*3 + c] = x;
        }
    }

    /* State Update */
    for( i=0; i<3; i++ ){
        s->p[i] += K[i*3 + 0]*nu[0] + K[i*3 + 1]*nu[1] + K[i*3 + 2]*nu[2];
        s->v[i] += K[(i+3)*3 + 0]*nu[0] + K[(i+3)*3 + 1]*nu[1] + K[(i+3)*3 + 2]*nu[2];
    }

    float KH[36] = {0};
    for( r=0; r<6; r++ ){
        for( c=0; c<6; c++ ){
            float x = 0.0f;
            for( k=0; k<3; k++ ){
                x += K[r*3 + k] * H[k*6 + c];
            }
            KH[r*6 + c] = x;
        }
    }

    float I_KH[36];
    for( r=0; r<6; r++ ){
        for( c=0; c<6; c++ ){
            I_KH[r*6 + c] = (r==c ? 1.0f : 0.0f ) - KH[r*6 + c];
        }
    }

    float Pn[36];
    mat6_mul(I_KH, s->P, Pn);
    memcpy(s->P, Pn, sizeof(Pn));

    return true;
}

/* NHC: enforce v_body, y~ 0, v_body, z ~ 0 */
static void pv6_update_nhc(pv6_t *s, const dr_quatf_t *q){
    float R[9];
    int i, j, k, r, c;
    dr_R_from_q(*q, R);
    float vbx = R[0]*s->v[0] + R[3]*s->v[1] + R[6]*s->v[2];
    float vby = R[1]*s->v[0] + R[4]*s->v[1] + R[7]*s->v[2];
    float vbz = R[2]*s->v[0] + R[5]*s->v[1] + R[8]*s->v[2];

    (void)vbx;  /* not used in NHC, but here if needed later*/

    float z[2] ={0, 0};
    float h[2] = {vby, vbz};

    /*R^T = body axes; we need y, z rows for velocity w.r.t world */
    float RT[9] = {R[0],R[3],R[6],R[1],R[4],R[7],R[2],R[5],R[8]};
    float Hv[6] = {
        RT[3], RT[4], RT[5], /*<< Y row */
        RT[6], RT[7], RT[8] /*<< Z row */
    };
    
    float H[12] = {0}; /*<< 2x6 */
    for( c=0; c<3; c++ ){
        H[0*6 + (c+3)] = Hv[0 + c];
        H[1*6 + (c+3)] = Hv[3 + c];
    }

    float Rm[4] = { R_NHC_VY, 0, 0, R_NHC_VZ };
    float HP[12];
    for( r=0; r<2; r++ ){
        for( c=0; c<6; c++ ) {
            float x=0.0f;
            for( k=0; k<6; k++ ){
                x += H[r*6 + k] * s->[k*6 + c];
            }
            HP[r*6 + c] = x;
        }
    }

    float S[4];
    for( r=0; r<2; r++ ){
        for( c=0; c<2; c++ ){
            float x=0.0f;
            for( k=0; k<6; k++ ){
                x += HP[r*6 + k]*H[c*6 + k];
            }
            S[r*2 + c] = x + (r == c ? Rm[r*2 + c] : 0.0f);
        }
    }

    float det = S[0] * S[3] - S[1]*S[2];
    if( fabsf(det) < 1e-12f ){
        return;
    }
    float Sinv[4] = {
        S[3]/det, -S[1]/det,
        -S[2]/det, S[0]/det
    };

    float nu[2] = { z[0]-h[0], z[1]-h[1] };

    float PHt[12];
    for( r=0; r<6; r++ ){
        for( c=0; c<2; c++ ){
            float x = 0.0f;
            for( k=0; k<6; k++ ){
                x += s->P[r*6 + k]*H[c*6 + k];
            }
            PHt[r*2 + c] = x;
        }
    }
    

    float K[12];
    for( r=0; r<6; r++ ){
        for( c=0; c<2; c++ ){
            K[r*2 + c] = PHt[r*2 + 0]*Sinv[0*2 + c] +
                         PHt[r*2 +1]*Sinv[1*2 + c];
        }
    }

    /* x = x + K nu */
    for( i=0; i<3; i++ ){
        s->p[i] += K[i*2 + 0]*nu[0] + K[i*2 + 1]*nu[1];
        s->v[i] += K[(i+3)*2 + 0]*nu[0] + K[(i+3)*2 + 1]*nu[1];
    }

    float KH[36] = {0};
    for( r=0; r<6; r++ ){
        for ( c=0; c<6; c++ ){
            float x = 0.0f;
            for( k=0; k<2; k++ ){
                x += K[r*2 + k]*H[k*6 + c];
            }
            KH[r*6 + c] = x;
        }
    }

    float I_KH[36];
    for( r=0; r<6; r++ ){
        for( c=0; c<6; c++ ){
            I_KH[r*6 + c] = (r == c ? 1.0f : 0.0f) - KH[r*6 + c];
        }
    }

    float Pn[36];
    mat6_mul(I_KH, s->P, Pn);
    memcpy(s->P, Pn, sizeof(Pn));
}

/* ZUPT: v^w ~ d */

static void pv6_update_zupt(pv6_t *s){
    int i;
    for( i=0; i<3; i++ ){
        float zi = 0.0f;
        float hi = s->v[i];
        float Pi = s->P[(i+3)*6 + (i + 3)];
        float S = Pi + R_ZUPT_V;
        if( S <= 0 ){
            continue;
        }

        float Kp0 = s->P[0*6 + (i+3)] / S;
        float Kp1 = s->P[1*6 + (i+3)] / S;
        float Kp2 = s->P[2*6 + (i+3)] / S;
        float Kv0 = s->P[3*6 + (i+3)] / S;
        float Kv1 = s->P[4*6 + (i+3)] / S;
        float Kv2 = s->P[5*6 + (i+3)] / S;
        flaot nu = zi - hi;

        s->p[0] += Kp0*nu;
        s->p[1] += Kp1*nu;
        s->p[2] += Kp2*nu;
        s->v[0] += Kv0*nu;
        s->v[1] += Kv1*nu;
        s->v[2] += Kv2*nu;
    }
}

/** ========================= Calibration ( replace with actual LSQ/bias valuse post calibration )===================== */
static const float ACCEL_C[3][3] = {
    {0.0005964462462844185f, -9.211739001666678e-07f, 1.5763305507490624e-05f},
    {-1.1573398643476584e-06f, 0.0006037351040586055f, 3.881537441769146e-07f},
    {-3.851697134466662e-05f, -3.2356391081574615e-05f, 0.0005895175306304627f}
 };

static const float ACCEL_O[3] = { -0.1329121010477303f, -0.047222673271787766f, 1.257425727446983f}; 

/* Gyro biass (raw ADC coounts) - can be learned dynamically */
static const float GYRO_B[3] = { 153.461f, 69.446f, 992.782f };

#define GYRO_LSB_PER_DPS    65.5f

static void calib_accel(const struct mpu6050_sample *raw, float acc_mps2[3]) {
    const float a[3] = { raw->ax, raw->ay, raw->az };
    int i;
    for( i=0; i<3; i++ ){
        acc_mps2[i] = ACCEL_C[i][0]*a[0] +
                      ACCEL_C[i][1]*a[1] +
                      ACCEL_C[i][2]*a[2] +
                      ACCEL_O[i];
    }
}

static void calib_gyro(const struct mpu6050_sample *raw, float gyro_rad[3]) {
    float g[3] = {
        raw->gx - GYRO_B[0],
        raw->gy - GYRO_B[1],
        raw->gz - GYRO_B[2]
    };

    float dps[3] = {
        g[0]/GYRO_LSB_PER_DPS,
        g[1]/GYRO_LSB_PER_DPS,
        g[2]/GYRO_LSB_PER_DPS
    };
    gyro_rad[0] = dps[0]*DEG2RAD;
    gyro_rad[1] = dps[1]*DEG2RAD;
    gyro_rad[2] = dps[2]*DEG2RAD;
}

/**========================LLA/ECEF/ENU helpers (WGS-84)====================== */

typedef struct { double lat, lon, h;} lla_t;
typedef struct { double x, y, z } ecef_t;

static const double aWGS = 6378137.0;
static const double fWGS = 1.0/298.257223563;
static const double bWGS = 6378137.0 * (1 - fWGS);
static const double e2 = (aWGS*aWGS - bWGS*bWGS) / (aWGS*aWGS);

static ecef_t lla2ecef(lla_t L) {
    double s = sin(L.lat), c = cos(L.lat);
    double N = aWGS / sqrt(1 - e2*s*s);
    ecef_t e;
    e.x = (N + L.h)*c*cos(L.lon);
    e.y = (N + L.h)*c*sin(L.lon);
    e.z = (N*(1-e2) + L.h)*s;

    return e;
}

static void ecef2enu( ecef_t e, lla_t ref, ecef_t e0, double out[3] ) {
    double s1 = sin(ref.lat), c1 = cos(ref.lat);
    double s0 = sin(ref.lon), c0 = cos(ref.lon);
    double dx = e.x - e0.x;
    double dy = e.y - e0.y;
    double dz = e.z - e0.z;

    out[0] = -s0*dx + c0*dy;
    out[1] = -c1*c0*dx - c1*s0*dy + s1*dz;
    out[2] = s1*c0*dx + s1*s0*dy + c1*dz;
}

/**==============Utility for yaw fusion================ */
static float wrap_pi(float a ){
    while ( a > M_PI ) a -= -2.0f * M_PI;
    while ( a <= -M_PI) a += 2.0f * M_PI;
    return a;
}

static float yaw_from_q( const dr_quatf_t *q ){
    /* ENU convention,  yaw about + Z*/
    float siny_cosp = 2.0f * (q->w*q->z + q->x*q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y*q->y + q->z*q->z);
    return atan2f(siny_cosp, cosy_cosp);
}

static dr_quatf_t q_from_yaw_delta( float dpsi ) {
    float half = 0.5f * dpsi;
    float s = sinf(half), c = cosf(half);
    dr_quatf_t dq = { c, 0.0f, 0.0f, s }; /*<< rotation about Z */
    return dq;
}

/**====================================== Context and Threading ==================== */
typedef struct {
    /* Latest IMU Sample */
    struct mpu6050_sample imu_raw;
    bool have_imu;

    /* Data Logging */
    FILE *logf;
    char log_filename[256];
    double last_log_sec;        /*< for 1 second logging */

    /* Storage for raw data */
    struct mpu6050_sampel imu_last_raw;
    bool imu_last_valid;

    double gnss_last_lat_deg;
    double gnss_last_lon_deg;
    double gnss_last_alt_m;
    float gnss_last_speed_mps;
    float gnss_last_heading_deg;
    float gnss_last_hdop;
    int gnss_last_fix;
    bool gnss_last_valid;

    /* Latest GNSS Info */
    bool    have_gnss;
    double  gnss_lat_rad;
    double  gnss_lon_rad;
    double  gnss_alt_m;
    float   gnss_speed_mps;
    float   gnss_vel_enu[3];
    bool    gnss_vel_valid;
    float   gnss_heading_rad;
    bool    gnss_heading_valid;
    float   gnss_hdop;
    bool    gnss_hdop_valid;

    /* EKF States */
    ekf4_t  ekf;
    pv6_t   pv;

    /* ENU Reference and Nav State */
    bool    enu_ref_set;
    lla_t   enu_ref_lla;
    ecef_t  enu_ref_ecef;
    bool    nav_ready;

    /* GNSS health and outage tracking */
    double  last_gnss_s;
    bool    gnss_valid;
    bool    gnss_just_returned;
    int     fade_in_left;
    double  outage_s;
    float   qscale;

    /* Misc*/
    uint64_t    last_tick_ns;
    int         zupt_count;

    /* Threading */
    pthread_mutex_t mtx;
    pthread_cond_t  cv;
    bool running;

} ctx_t;

static float compute_qscale(double outage_s ){
    if( outage_s < TIER_A ) return QSCL_A;
    if( outage_s < TIER_B ) return QSCL_B;
    if( outage_s < TIER_C ) return QSCL_C;
    return QSCL_D;

}

/*=====================Device Open Helpers ================== */
static int open_imu( void ){
    int fd = open(IMU_DEVICE_PATH, O_RDONLY | O_CLOEXEC );
    return (fd < 0) ? -errno : fd;
}

static int open_gnss( void ){
    int fd = open(NEO6M_DEVICE_PATH, O_RDONLY  | O_CLOEXEC );
    return (fd < 0) ? -errno : fd;
}

/*==========================LOG FILE HELPERS ====================*/
static void make_log_file( ctx_t *C ){
    const char *log_dir = "home/sijeo/nav_logs";

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "mkdir -p %s", log_dir);
    system(cmd);

    time_t t = time(NULL);
    struct tm *tmv = localtime(&t);

    snprintf(C->log_filename, sizeof(C->log_filename), "%s/navlog_%04d-%02d-%02d_%02d-%02d-%02d.csv", log_dir, 
            tmv->tm_year + 1900, tmv->tm_mon + 1, tmv->tm_mday, tmv->tm_hour, tmv->tm_min, tmv->tm_sec);
    C->logf = fopen(C->log_filename, "w");
    if( !C->logf ){
        fprintf(stderr, "ERROR: unable to create log file\n");
        exit(1);
    }

    fprintf(C->logf,
        "time_s,E,N,U,Vx,Vy,Vz,Ax,Ay,Az,imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,gnss_fix,lat,lon,alt,speed,heading,hdop,outage_s");

    fflush(C->logf);
    C->last_log_sec = 0;
    fprintf(stderr, "logging to %s\n",C->log_filename);

}

/* ===================IMU THREAD ============================== */
static void* imu_thread( void *arg ){
    ctx_t *C = (ctx_t*)arg;
    int fd = open_imu();
    if ( fd < 0 ) {
        fprintf(stderr, "open_imu failed: %d\n", fd);
        return NULL;
    }

    while( __atomic_load_n(&C->running, __ATOMIC_ACQUIRE)) {
        struct mpu6050_sample s;
        ssize_t n = read(fd, &s, sizeof(s));
        if( n == (ssize_t)sizeof(s) ){
            pthread_mutex_lock(&C->mtx);
            C->imu_raw = s;
            C->have_imu = true;
            C->imu_last_raw = s;
            C->imu_last_valid = true;
            pthread_cond_signal(&C->cv);
            pthread_mutex_unlock(&C->mtx);
        } else {
            usleep(1000);
        }
    }

    close(fd);
    return NULL;
}


/*===================GNSS THREAD ======================== */
static void* gnss_thread( void* arg){
    ctx_t *C = (ctx_t*)arg;
    int fd = open_gnss();
    if( fd < 0 ){
        fprintf(stderr, "open_gnss failed: %d\n", fd);
        return NULL
    }

    while( __atomic_load_n(&C->running, __ATOMIC_ACQUIRE)) {
        struct neo6m_gnss_fix f;
        if( ioctl(fd, NEO6M_GNSS_IOC_FIX, &f) != 0){
            usleep(10000);
            continue;
        }

        pthread_mutex_lock(&C->mtx);
        if( f.have_fix ){
            /** read values */
            double lat_deg = f.lat_e7/1e7;
            double lon_deg = f.lon_e7/1e7;
            double alt_m = f.alt_mm/1000.0f;

            C->gnss_lat_rad = lat_deg * (M_PI/180.0);
            C->gnss_lon_rad = lon_deg * (M_PI/180.0);
            C->gnss_alt_m = alt_m;
            C->gnss_speed_mps = f.speed_mmps/1000.0f;

            /** HDOP */
            if( f.hdop_valid ) {
                C->gnss_hdop = (float)f.hdop_x10 * 0.1f;
                C->gnss_hdop_valid = true;

            }
            else {
                C->gnss_hdop_valid = false;

            }

            /* Heading / Course over ground (deg * 1e5)*/
            if (f.heading_valid && C->gnss_speed_mps > 0.5f ) {
                float heading_deg = f.heading_deg_e5 / 1e5f;
                C->gnss_heading_rad = heading_deg * (M_PI/180.0f);
                C->gnss_heading_valid = true;

                float spd =  C->gnss_speed_mps;
                float psi = C->gnss_heading_rad;

                C->gnss_vel_enu[0] = spd * sinf(psi); /**< EAST */
                C->gnss_vel_enu[1] = spd * cosf(psi); /**< NORTH */
                C->gnss_vel_enu[2] = 0.0f;
                C->gnss_vel_valid = true;
            } else {
                C->gnss_heading_valid = false;
                C->gnss_vel_valid = false;
            }

            C->have_gnss = true;

            /* Save for Logging */
            C->gnss_last_lat_deg    = lat_deg;
            C->gnss_last_lon_deg    = lon_deg;
            C->gnss_last_alt_m      = alt_m;
            C->gnss_last_speed_mps  = C->gnss_speed_mps;
            C->gnss_last_heading_deg = C->gnss_heading_valid ? (C->gnss_heading_rad * 180.0f/M_PI) : 0.0f;
            C->gnss_last_hdop        = C->gnss_hdop_valid ? C->gnss_hdop : -1.0f;
            C->gnss_last_fix        = f.have_fix ? 1 : 0;
            C->gnss_last_valid      = true;

            /**< ENU Origin on First-Ever Fix */
            if( !C->enu_ref_set ){
                C->enu_ref_lla = (lla_t){
                    C->gnss_lat_rad,
                    C->gnss_lon_rad,
                    C->gnss_alt_m
                };
                C->enu_ref_ecef = lla2ecef(C->enu_ref_lla);
                C->enu_ref_set = true;
                fprintf(stederr, "ENU Origin set (First GNSS Fix)\n");
            }

        }
        pthread_conf_signal(&C->cv);
        pthread_mutex_unlock(&C->mtx);

        usleep(100000); /**< ~10Hz poll (GNSS typically 1 Hz) */

    }

    close(fd);
    return NULL;
}

/*=====================FUSION THREAD =========================== */

static void* fusion_thread( void* arg ) {
    ctx_t *C = (ctx_t*)arg;
    C->last_tick_ns = monotonic_ns();

    while(__atomic_load_n(&C->running, __ATOMIC_ACQUIRE )) {
        pthread_mutex_lock(&C->mtx);

        /* Wait for the IMU Sample */
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 5*1000*1000;
        if( ts.tv_nsec >= 1000000000 ) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }
        pthread_cond_timedwait(&C->cv, &C->mtx, &ts);

        uint64_t now_ns = monotonic_ns();
        float dt = (now_ns - C->last_tick_ns)*1e-9f;
        if( dt <= 0 || dt > 1.0f ) dt = DT_IMU;
        C->last_tick_ns = now_ns;

        if( !C->have_imu ){
            pthread_mutex_unlock(&C->mtx);
            continue;
        }

        /* Apply Calibration to IMU */
        float acc_b[3], gyro_b[3];
        calib_accel(&C->imu_raw, acc_b);
        calib_gyro(&C->imu_raw, gyro_b);

        float acc_norm = sqrtf( acc_b[0]*acc_b[0] + acc_b[1]*acc_b[1] + acc_b[2]*acc_b[2]);
        bool near_still = fabsf(acc_norm - GRAVITY ) < ACC_STILL_TOL;

        /* Yaw dead band at low motion  */
        if( near_still && fabsf(gyro_b[2]) < YAW_DEADBAND_RAD ){
            gyro_b[2] = 0.0f;

        }
        /* Attitude EKF always runs */
        ekf4_predict(&C->ekf, gyro_b, dt);
        ekf4_update_accel(&C->ekf, acc_b);

        /* If nav not ready yet, try to initialize from First GNSS fix */
        if( !C->nav_ready ){
            if( C->enu_ref_set && C->have_gnss ){
                lla_t L = { C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m };
                ecef_t E = lla2ecef(L);
                double z_enu_d[3];
                ecef2enu(E, C->enu_ref_lla, C->enu_ref_ecef, z_enu_d);

                pv6_init(&C->pv);
                C->pv.p[0] = (float)z_enu_d[0];
                C->pv.p[1] = (float)z_enu_d[1];
                C->pv.p[2] = (float)z_enu_d[2];
                C->pv.v[0] = 0.0f;
                C->pv.v[1] = 0.0f;
                C->pv.v[2] = 0.0f;

                C->nav_ready    = true;
                C->gnss_valid   = true;
                C->gnss_just_returned = false;
                C->fade_in_left = 0;
                C->outage_s     = 0.0f;
                C->qscale       = 1.0f;
                C->last_gnss_s  = now_sec();
                C->have_gnss    = false;

                fprintf(stderr, "Nav Initialized: ENU (%.2f, %.2f, %.2f)\n", C->pv.p[0], C->pv.p[1], C->pv.p[2]);
            }

            pthread_mutex_unlock(&C->mtx);
            continue;
        }

        /* From Here: nav_active == true */

        /** Outage Tracking  */
        double tnow = now_sec();
        if( !C->gnss_valid || (tnow - C->last_gnss_s ) > GNSS_TIMEOUT_S ){
            if( C->gnss_valid ){
                C->gnss_valid = false;
                C->gnss_just_returned = false;
            }
            C->outage_s += dt;
        } else {
            C->outage_s = 0.0f;

        }
        C->qscale = C->gnss_valid ? 1.0f : compute_qscale(C->outage_s);

        /* World frame accel */
        float Rm[9];
        dr_R_from_q(C->ekf.q, Rm);
        float aw[3] = {
            Rm[0]*acc_b[0] + Rm[1]*acc_b[1] + Rm[2]*acc_b[2],
            Rm[3]*acc_b[0] + Rm[4]*acc_b[1] + Rm[5]*acc_b[2],
            Rm[6]*acc_b[0] + Rm[7]*acc_b[1] + Rm[8]*acc_b[2] - GRAVITY
        };

        pv6_predict(&C->pv, aw, dt, C->qscale);
        /*NHC*/
        pv6_update_nhc(&C->pv, &C->ekf.q);

        /*ZUPT detection */
        bool zupt_cond = 
                (fabsf(acc_norm - GRAVITY ) < ZUPT_ACC_THR ) &&
                (fabsf(gyro_b[0]) < ZUPT_GYRO_THR  &&
                 fabsf(gyro_b[1]) < ZUPT_GYRO_THR  &&
                 fabsf(gyro_b[2]) < ZUPT_GYRO_THR);

        if ( zupt_cond ) {
            C->zupt_count++;
        } else {
            C->zupt_count = 0;

        }

        if( C->zupt_count >= ZUPT_COUNT_REQUIRED ){
            pv6_update_zupt(&C->pv);
            C->zupt_count = 0;
        }

        /** Velocity Decay  */
        float v_norm = sqrtf(C->pv.v[0]*C->pv.v[0] + C->pv.v[1]*C->pv.v[1] + C->pv.v[2]*C->pv.v[2] );
        if( vnorm < VEL_EPS ) {
            C->pv.v[0] *= VEL_DECAY;
            C->pv.v[1] *= VEL_DECAY;
            C->pv.v[2] *= VEL_DECAY;
        }

        /** GNSS Fusion (pos + vel + yaw ) */
        if( C->have_gnss ){
            /** POSITION UPDATE WITH HDOP ADAPTIVE R */
            lla_t L = { C->gnss_lat_rad, C->gnss_lon_rad, C->gnss_alt_m };
            ecef_t E = lla2ecef(L);
            double z_enu_d[3];
            ecef2enu(E, C->enu_ref_lla, C->enu_ref_ecef, z_enu_d);

            float z_pos[3] ={
                (float)z_enu_d[0], 
                (float)z_enu_d[1],
                (float)z_enu_d[2]
            };

            float hdop = (C->gnss_hdop_valid ? C->gnss_hdop : 1.0f );
            float Rpos = R_GNSS_POS_VAR * hdop * hdop;

            if( !C->gnss_valid ){
                C->gnss_just_returned = true;
                C->fade_in_left = FADE_IN_STEPS;

            }
            if( C->gnss_just_returned ){
                Rpos *= FADE_IN_FACT_INIT;
            }

            float NIS_pos = 0.0f;
            bool ok_pos = pv6_update_pos(&C->pv, z_pos, Rpos, &NIS_pos);
            bool gate_pos = ok_pos && (NIS_pos < CHI2_3DOF_GATE);

            if(gate_pos){
                double tfix = now_sec();
                C->last_gnss_s = tfix;
                C->gnss_valid = true;
                C->outage_s = 0.0f;
                C->qscale = 1.0f;
            }

            /** Velocity Update  */
            if (C->gnss_vel_valid){
                float z_vel[3] = {
                    C->gnss_vel_enu[0],
                    C->gnss_vel_enu[1],
                    C->gnss_vel_enu[2]
                };

                float Rv[3] = { RV_VEL_E, RV_VEL_N, RV_VEL_U };

                float NIS_vel = 0.0f;
                bool ok_vel = pv6_update_vel(&C->pv, z_vel, Rv, &NIS_vel);
                bool gate_vel = ok_vel && (NIS_vel < CHI2_3DOF_GATE);

                fprintf(stderr, "GNSS Vel upd: NIS=%.2f gate=%s vel=(%.2f, %.2f, %.2f)\n", NIS_vel, gate_vel ? "OK":"REJ", 
                            z_vel[0], z_vel[1], z_vel[2]);

            }
            /* Yaw Fusion Complementary */
            if( C->gnss_heading_valid && C->gnss_speed_mps > YAW_FUSION_SPEED_MIN && hdop <= YAW_FUSION_HDOP_MAX){
                float yaw_est = yaw_from_q(&C->ekf.q);
                float yaw_gnss = C->gnss_heading_rad;
                float dpsi = wrap_pi( yaw_gnss - yaw_est );

                float dpsi_corr = YAW_FUSION_ALPHA * dpsi;
                dr_quatf_t dq = q_from_yaw_delta(dpsi_corr);
                C->ekf.q = dr_q_normalize( dr_q_mult(C->ekf.q, dq));
            }

            /** Fade in Bookkeeping  */
            if( C->gnss_just_returned && gate_pos ){
                C->fade_in_left--;
                if(C->fade_in_left <= 0){
                    C->gnss_just_returned = false;
                }
            }
            fprintf(stderr, "GNSS pos upd: NIS=%.2f gate=%s, Rpos=%.2f, HDOP=%.2f outage=%.1f qS=%.1f\n", NIS_pos, 
                    gate_pos ? "OK":"REJ", Rpos, hdop, C->outage_s, C->qscale);

            C->have_gnss = false; /*<< Consumed */
        } else {
            //DR - Only Logging 
            static double t_last_log = 0;
            double t = now_sec();
            if( t - t_last_log > 0.5f ){
                fprintf(stderr, "DR: outage=%.1fs qS=%.1f p=(%.1f,%.1f,%.1f) v=(%.2f, %.2f, %.2f)\n", C->outage_s, C->qscale,
                        C->pv.p[0], C->pv.p[1], C->pv.p[2], C->pv.v[0], C->pv.v[1], C->pv.v[2]);
                t_last_log = t;
            }
        }

        /* ENU output (only after nav initialized )*/
        if(C->nav_ready ) {
            printf("%.3f, %.3f, %.3f\n", C->pv.p[0], C->pv.p[1], C->pv.p[2]);
            fflush(stdout);
        }

        /**1Hz Combined Logging  */
        double tnow_log = now_sec();
        if( tnow_log - C->last_log_sec >= 1.0) {
            if(C->logf && C->imu_last_valid ){
                fprintf(C->logf, 
                "%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%d,%d,%d",
                "%d,%d,%d,"
                "%d,%.7f,%.7f,%.2f,"
                "%.3f,%.2f,%.2f,"
                "%.3f\n",
                tnow_log,
                C->pv.p[0],C->pv.p[1],C->pv.p[2],
                C->pv.v[0],C->pv.v[1],C->pv.v[2],
                aw[0],aw[1],aw[2],
                C->imu_last_raw.ax,
                C->imu_last_raw.ay,
                C->imu_last_raw.az,
                C->imu_last_raw.gx,
                C->imu_last_raw.gy,
                C->imu_last_raw.gz,
                C->gnss_last_fix,
                C->gnss_last_lat_deg,
                C->gnss_last_lon_deg,
                C->gnss_last_alt_m,
                C->gnss_last_speed_mps,
                C->gnss_last_heading_deg,
                C->gnss_last_hdop,
                C->outage_s
              );
              fflush(C->logf);
            }
            C->last_log_sec = tnow_log;
        }

        pthread_mutex_unlock( &C->mtx );
    }

    return NULL;
}

/*===================MAIN APPLICATION============================== */
int main( void ){
    ctx_t C;
    memset(&C, 0, sizeof(C));
    ekf4_init(&C.ekf);
    pv6_init(&C.pv);

    C.enu_ref_set   = false;
    C.nav_ready     = false;
    C.gnss_valid    = false;
    C.gnss_just_returned = false;
    C.fade_in_left  = 0;
    C.outage_s      = 0.0f;
    C.qscale        = 1.0f;
    C.zupt_count    = 0;
    C.gnss_hdop_valid   = false;
    C.gnss_vel_valid    = false;
    C.gnss_heading_valid = false;

    pthread_mutex_init(&C.mtx, NULL);
    pthread_cond_init(&C.cv, NULL);
    C.running       = true;

    make_log_file(&C);

    pthread_t th_imu, th_gnss, th_fuse;
    if( pthread_create(&th_imu, NULL, imu_thread, &C ) != 0 ) { perror("imu_thread"); return 1; }
    if( pthread_create(&th_gnss, NULL, gnss_thread, &C ) != 0 ) { perror("gnss thread "); return 1; }
    if( pthread_create(&th_fuse, NULL, fusion_thread, &C ) != 0 ){ perror("fusion thread "); return 1; }

    /* Simple blocking run ( Ctrl + C to kill )*/
    pause();

    C.running=false;
    pthread_join(th_imu, NULL);
    pthread_join(th_gnss, NULL);
    pthread_join(th_fuse, NULL);

    pthread_mutex_destroy(&C.mtx);
    pthread_cond_destroy(&C.cv);

    return 0;

}