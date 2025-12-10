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
#define DEG2RAD        (3.14159265358979323846/180.0) 

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
    return (uint64_t)ts.tv_sec * 1000000000ULL + tv.tv_nsec;   
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

