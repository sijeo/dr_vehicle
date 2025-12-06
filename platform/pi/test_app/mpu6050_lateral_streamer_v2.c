/**
 * mpu6050_lateral_streamer_v2.c
 * --------------------------------
 * 
 * Streams calibrated IMU data + EKF quaternion + integrated velocity + position over TCP to imu_lateral_viewer.py
 * 
 * Features:
 * - Accel is calibrated (ACCEL_C, ACCEL_O)
 * - Gyro bias + scale -> rad/s
 * - 4 - state quaternion EKF(PREDICT: GYRO, UPDATE: ACCEL, GRAVITY)
 * - Yaw drift compensation (deadband + optional bias learning )
 * - ZUPT ( Zero velocity update) + hysteresis
 * - Velocity decay near zero
 * - JSON streaming at SAMPLE_HZ
 * 
 */

 #define _GNU_SOURCE
 #include <stdio.h>
 #include <stdlib.h>
 #include <stdbool.h>
 #include <math.h>
 #include <string.h>
 #include <unistd.h>
 #include <fcntl.h>
 #include <errno.h>
 #include <time.h>
 #include <arpa/inet.h>
 #include <sys/socket.h>
 #include <sys/types.h>

 #include "../mpu6050_ioctl.h"
 #include "../core/dr_math.h"
 #include "../core/dr_types.h"

 #define IMU_DEV_PATH "/dev/mpu6050-0"
 #define SAMPLE_HZ 100
 #define SEVER_PORT 9010

 #define GRAVITY_CONST 9.80665f
 #define DEG2RAD (3.14159265f/180.0f)
 #define GYRO_SCALE_LBS_PER_DPS 65.5f // MPU6050 gyro LSB per degree-per-second (for 500 dps full-scale)


 /*======================Calibration Constants ============================*/
 /* Replace these with the values in the imu_calibration.json file in /tools folder */
 static const float ACCEL_C[3][3] = {
    {1.0025f, 0.0021f, -0.0015f},
    {0.0021f, 0.9987f, 0.0030f},
    {-0.0015f, 0.0030f, 1.0050f}
 };

static const float ACCEL_O[3] = { -0.12f, 0.05f, 0.98f }; 

/* Gyro biass (raw ADC coounts) - can be learned dynamically */
static const float GYRO_B[3] = { 1.5f, -2.3f, 0.7f };

/*=================================ZUPT Parameters ===========================*/
#define ACC_ZUPT_THRESH           0.25f  /* m/s^2 */
#define GYRO_ZUPT_THRESH         (5.0f * DEG2RAD)    /* deg/s */
#define ZUPT_COUNT_REQUIRED       5      /* number of consecutive samples below threshold to trigger ZUPT */

#define LIN_ACC_ALPHA             0.90f 
#define VEL_DECAY_NEAR_ZERO       0.98f
#define VEL_EPSILON               1e-3f
#define POS_CLAMP_MAX             5.0f  /* meters */

#define YAW_DRIFT_DEADBAND        0.03f
#define ACC_STILL_TOL             (0.1f * GRAVITY_CONST) /* m/s^2 */


// Default parameters
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*======================== Helper Functions ================================*/

static uint64_t monotonic_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static float vec3_norm(const float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void euler_deg_from_q( dr_quatf_t q, float* yaw_deg, float* pitch_deg, float* roll_deg){
    float R[9];
    dr_R_from_q(q, R);
    *roll_deg = atan2f(R[5], R[8]) * (180.0f / (float)M_PI);
    *pitch_deg = -asinf(R[2]) * (180.0f / (float)M_PI);
    *yaw_deg = atan2f(R[1], R[0]) * (180.0f / (float)M_PI);
}

/*=========================Calibration Functions=============================*/
static void apply_accel_calib( const struct mpu6050_sample* raw, float accel_mps2[3]) {
    float a_raw[3] = { raw->ax, raw->ay, raw->az };
    int i;
    for( i = 0; i < 3; i++ ) {
        accel_mps2[i] = ACCEL_C[i][0] * a_raw[0] + ACCEL_C[i][1] * a_raw[1] + ACCEL_C[i][2] * a_raw[2] + ACCEL_O[i];
    }  


}

static void apply_gyro_calib( const struct mpu6050_sample* raw, float gyro_radps[3]) {
    float g_raw[3] = { raw->gx, raw->gy, raw->gz };

    g_raw[0] -= GYRO_B[0];
    g_raw[1] -= GYRO_B[1];
    g_raw[2] -= GYRO_B[2];

    float dps[3] = {
        g_raw[0] / GYRO_SCALE_LBS_PER_DPS,
        g_raw[1] / GYRO_SCALE_LBS_PER_DPS,
        g_raw[2] / GYRO_SCALE_LBS_PER_DPS
    };

    gyro_radps[0] = dps[0] * DEG2RAD;
    gyro_radps[1] = dps[1] * DEG2RAD;
    gyro_radps[2] = dps[2] * DEG2RAD;
}


/*========================= 4 State Quaternion EKF =============================== */
typedef struct {
    dr_quatf_t q;  /* orientation body->world */
    float P[16]; /* covariance matrix (4x4) row-major */
} ekf4_t;

static void ekf4_init( ekf4_t* ekf, dr_quatf_t q0) {
    ekf->q = dr_q_normalize(q0);
    memset(ekf->P, 0, sizeof(ekf->P));

    float p0 = 1e-3f;
    ekf->P[0] = p0; ekf->P[5] = p0; ekf->P[10] = p0; ekf->P[15] = p0;
}

static void quat_derivative_c( const dr_quatf_t *q, const float w[3], dr_quatf_t *dqdt) {
    float wx = w[0];
    float wy = w[1];
    float wz = w[2];
    float wq = q->w;
    float xq = q->x;
    float yq = q->y;
    float zq = q->z;

    dqdt->w = -0.5f * (wx * xq + wy * yq + wz * zq);
    dqdt->x =  0.5f * (wx * wq + wz * yq - wy * zq);
    dqdt->y =  0.5f * (wy * wq - wz * xq + wx * zq);
    dqdt->z =  0.5f * (wz * wq + wy * xq - wx * yq);
}

static void gravity_body_from_quat( const dr_quatf_t* q, float g_b[3]) {
    dr_quatf_t qc = { q->w, -q->x, -q->y, -q->z };
    dr_vec3f_t g_w = dr_v3(0.0f, 0.0f, 1.0f);
    dr_vec3f_t g_bv = dr_q_rotate(qc, g_w);
    float n = sqrtf(g_bv.x * g_bv.x + g_bv.y * g_bv.y + g_bv.z * g_bv.z);
    g_b[0] = g_bv.x / n;
    g_b[1] = g_bv.y / n;
    g_b[2] = g_bv.z / n;

}

static void ekf4_predict(ekf4_t *e, const float gyro[3], float dt) {
    const float q_proc = 1e-6f;

    dr_quatf_t dqdt;
    quat_derivative_c(&e->q, gyro, &dqdt);

    dr_quatf_t q_pred;
    q_pred.w = e->q.w + dqdt.w * dt;
    q_pred.x = e->q.x + dqdt.x * dt;
    q_pred.y = e->q.y + dqdt.y * dt;
    q_pred.z = e->q.z + dqdt.z * dt;
    e->q = dr_q_normalize(q_pred);

    // Covariance prediction
    e->P[0] += q_proc;
    e->P[5] += q_proc;
    e->P[10] += q_proc;
    e->P[15] += q_proc;

}

/**
 * EKF Update: accel-> gravity direction
 * Uses Numerical Jacobian, Identical to imu_calib_gui.py logic.
 */
static void ekf4_update( ekf4_t *e, const float accel[3]) {
    float acc_norm = vec3_norm(accel);
    if ( acc_norm  < 0.5f*GRAVITY_CONST || acc_norm > 1.5f*GRAVITY_CONST )
        return; /* invalid accel measurement */

    float z[3] = {
        accel[0] / acc_norm,
        accel[1] / acc_norm,
        accel[2] / acc_norm
    };

    float h[3];
    gravity_body_from_quat(&e->q, h);

    float y[3] = {
        z[0] - h[0],
        z[1] - h[1],
        z[2] - h[2]
    };

    /* Numerical Jacobian H (3x4)*/
    float H[12];
    const float eps = 1e-6f;
    int cols, rows;
    float base[4] = { e->q.w, e->q.x, e->q.y, e->q.z };
    for( cols = 0; cols < 4; cols++ ) {
        float q_plus[4] = { base[0], base[1], base[2], base[3] };
        float q_minus[4] = { base[0], base[1], base[2], base[3] };

        q_plus[cols] += eps;
        q_minus[cols] -= eps;

        dr_quatf_t qp = dr_q_normalize((dr_quatf_t){ .w = q_plus[0], .x = q_plus[1], .y = q_plus[2], .z = q_plus[3] });
        dr_quatf_t qm = dr_q_normalize((dr_quatf_t){ .w = q_minus[0], .x = q_minus[1], .y = q_minus[2], .z = q_minus[3] });
        float hp[3], hm[3];
        gravity_body_from_quat(&qp, hp);
        gravity_body_from_quat(&qm, hm);

        H[0*4 + cols] = (hp[0] - hm[0]) / (2.0f * eps);
        H[1*4 + cols] = (hp[1] - hm[1]) / (2.0f * eps);
        H[2*4 + cols] = (hp[2] - hm[2]) / (2.0f * eps);
    }

    const float r = 0.0004f;

    /* S = HPH^T + R */
    float HP[12];
    float *P = e->P;

    for( rows = 0; rows < 3; rows++ ) {
        for( cols = 0; cols < 4; cols++ ) {
            float sum = 0;
            int k;
            for( k = 0; k < 4; k++ ) {
                sum += H[rows*4 + k] * P[k*4 + cols];
            }
            HP[rows*4 + cols] = sum;
        }
    }

    float S[9];
    for( rows = 0; rows < 3; rows++ ) {
        for( cols = 0; cols < 3; cols++ ) {
            float sum = 0;
            int k;
            for( k = 0; k < 4; k++ ) {
                sum += HP[rows*4 + k] * H[cols*4 + k];
            }
            S[rows*3 + cols] = sum + ( (rows == cols) ? r : 0.0f );
        }
    }

    /* S^-1 (3x3) */
    float S_inv[9];
    if( dr_mat3_inv(S_inv, S) ) {
        return; /* singular S */
    }

    /* K = P H^T S^-1 (4x3) */
    float PHt[12];
    for( rows = 0; rows < 4; rows++ ) {
        for( cols = 0; cols < 3; cols++ ) {
            float sum = 0;
            int k;
            for( k = 0; k < 4; k++ ) {
                sum += P[rows*4 + k] * H[cols*4 + k];
            }
            PHt[rows*3 + cols] = sum;
        }
    }
    float K[12];
    for( rows = 0; rows < 4; rows++ ) {
        for( cols = 0; cols < 3; cols++ ) {
            float sum = 0;
            int k;
            for( k = 0; k < 3; k++ ) {
                sum += PHt[rows*3 + k] * S_inv[k*3 + cols];
            }
            K[rows*3 + cols] = sum;
        }
    }

    /* Update Quaternions */
    float Ky[4] = {0};
    for( rows = 0; rows < 4; rows++ ) {
        for( cols = 0; cols < 3; cols++ ) {
            Ky[rows] += K[rows*3 + cols] * y[cols];
        }
    }

    float q_new[4];
    q_new[0] = e->q.w + Ky[0];
    q_new[1] = e->q.x + Ky[1];
    q_new[2] = e->q.y + Ky[2];
    q_new[3] = e->q.z + Ky[3];

    e->q = dr_q_normalize((dr_quatf_t){ .w = q_new[0], .x = q_new[1], .y = q_new[2], .z = q_new[3] });

    /* Update Covariance: P = (I - K H) P */
    float KH[16];
    for( rows = 0; rows < 4; rows++ ) {
        for( cols = 0; cols < 4; cols++ ) {
            float sum = 0;
            int k;
            for( k = 0; k < 3; k++ ) {
                sum += K[rows*3 + k] * H[k*4 + cols];
            }
            KH[rows*4 + cols] = sum;
        }
    }

    float I_KH[16];
    for( rows = 0; rows < 4; rows++ ) {
        for( cols = 0; cols < 4; cols++ ) {
            I_KH[rows*4 + cols] = (rows == cols ? 1.0f : 0.0f) - KH[rows*4 + cols];
        }
    }

    float Pn[16];
    for( rows = 0; rows < 4; rows++ ) {
        for( cols = 0; cols < 4; cols++ ) {
            float sum = 0;
            int k;
            for( k = 0; k < 4; k++ ) {
                sum += I_KH[rows*4 + k] * P[k*4 + cols];
            }
            Pn[rows*4 + cols] = sum;
        }
    }

    memcpy(P, Pn, sizeof(Pn));

}

/* ================================= MAIN LOOP ==================================*/
int main() {
    int imu = open(IMU_DEV_PATH, O_RDONLY);
    if ( imu < 0 ) {
        perror("open imu"); 
        return 1;
    }

    /* Setup TCP server */
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if ( srv < 0 ) {
        perror("socket");
        close(imu);
        return 1;
    }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(SEVER_PORT);

    if ( bind(srv, (void*)&addr, sizeof(addr) ) < 0 ) {
        perror("bind");
        close(srv);
        close(imu);
        return 1;
    }

    if ( listen(srv, 1) < 0 ) {
        perror("listen");
        close(srv);
        close(imu);
        return 1;
    }

    fprintf(stderr, "[INFO] Waiting for viewer on port %d...\n", SEVER_PORT);

    struct sockaddr_in cli;
    socklen_t cli_len = sizeof(cli);
    int cli_fd = accept(srv, (struct sockaddr*)&cli, &cli_len);
    if ( cli_fd < 0 ) {
        perror("accept");
        close(srv);
        close(imu);
        return 1;
    }

    /* State */
    float pos[3] = {0};
    float vel[3] = {0};
    float lin_filt[3] = {0};
    bool lin_init = false;

    ekf4_t ekf;
    ekf4_init( &ekf, dr_quat_identity());

    uint64_t t_prev = monotonic_time_ns();
    int zupt_cnt = 0;

    struct mpu6050_sample s;

    /*=======================Main Stream Loop =======================*/

    while(1) {
        int i;
     if( read(imu, &s, sizeof(s)) != sizeof(s)) {
        continue;
     }   

     uint64_t now = monotonic_time_ns();
     float dt = (now - t_prev) * 1e-9f;
     if( dt < 0.0f || dt > 0.1f ) {
        dt = 1.0f / SAMPLE_HZ;
     }
     t_prev = now;

     float acc_b[3], gyro[3];
     apply_accel_calib( &s, acc_b );
     apply_gyro_calib( &s, gyro );   

     float acc_norm_body = vec3_norm(acc_b);
     bool near_still = fabsf(acc_norm_body - GRAVITY_CONST) < ACC_STILL_TOL;
     
     if ( near_still && fabsf(gyro[2]) < YAW_DRIFT_DEADBAND ){
        /* Yaw drift compensation */
        gyro[2] = 0.0f;
     }

     /* Attitude EKF update */
     ekf4_predict( &ekf, gyro, dt );
     ekf4_update( &ekf, acc_b );

     dr_quatf_t q = ekf.q;

     /* World-frame linear accel */
     dr_vec3f_t aB = dr_v3( acc_b[0], acc_b[1], acc_b[2] );
     dr_vec3f_t aW = dr_q_rotate( q, aB );

     float lin_raw[3] = {
        aW.x, 
        aW.y,
        aW.z + GRAVITY_CONST
   };

   float lin_norm = vec3_norm(lin_raw);
   float gyro_norm = vec3_norm(gyro);

   bool acc_ok = fabsf(lin_norm - GRAVITY_CONST) < ACC_ZUPT_THRESH;
   bool gyro_ok = gyro_norm < GYRO_ZUPT_THRESH;
   bool still = acc_ok && gyro_ok;

   if ( still ) {
        zupt_cnt++;
         /* Zero out linear accel */
            lin_raw[0] = 0.0f;
            lin_raw[1] = 0.0f;
            lin_raw[2] = 0.0f;
        if ( zupt_cnt >= ZUPT_COUNT_REQUIRED ) {
            /* Zero Velocity Update */
            vel[0] = 0.0f;
            vel[1] = 0.0f;
            vel[2] = 0.0f;
        }
   } else {
        zupt_cnt = 0;
   }

   if (!lin_init ) {
    memcpy(lin_filt, lin_raw, sizeof(lin_raw));
    lin_init = true;
   } else {
    lin_filt[0] = LIN_ACC_ALPHA * lin_filt[0] + (1.0f - LIN_ACC_ALPHA) * lin_raw[0];
    lin_filt[1] = LIN_ACC_ALPHA * lin_filt[1] + (1.0f - LIN_ACC_ALPHA) * lin_raw[1];
    lin_filt[2] = LIN_ACC_ALPHA * lin_filt[2] + (1.0f - LIN_ACC_ALPHA) * lin_raw[2];
   }

   float lin[3] = {
    lin_filt[0],
    lin_filt[1],
    lin_filt[2]
   };

    /* Integrate velocity */
    vel[0] += lin[0] * dt;
    vel[1] += lin[1] * dt;
    vel[2] += lin[2] * dt;

    /* Velocity decay near zero */
    for ( i = 0; i < 3; i++ ) {
        if ( fabsf(lin[i]) < ACC_ZUPT_THRESH ) {
            vel[i] *= VEL_DECAY_NEAR_ZERO;
            if ( fabsf(vel[i]) < VEL_EPSILON ) {
                vel[i] = 0.0f;
            }
        }
    }

    for( i = 0; i < 3; i++ ) {
        pos[i] += vel[i] * dt;
        if ( pos[i] > POS_CLAMP_MAX ) pos[i] = POS_CLAMP_MAX;
        if ( pos[i] < -POS_CLAMP_MAX ) pos[i] = -POS_CLAMP_MAX;
    }

    float yaw_deg, pitch_deg, roll_deg;
    euler_deg_from_q( q, &yaw_deg, &pitch_deg, &roll_deg);
    /* Send JSON packet */
    char json_buf[512];
    int n = snprintf( json_buf, sizeof(json_buf),
        "{ \"t_ns\": %llu, \"pos_m\": [%.4f, %.4f, %.4f], \"vel_mps\": [%.4f, %.4f, %.4f], "
        "\"q\": [%.6f, %.6f, %.6f, %.6f], \"euler_deg\": [%.2f, %.2f, %.2f] ,
        \"lin_acc_mps2\": [%.4f, %.4f, %.4f] }\n", (unsigned long long)now, 
        pos[0], pos[1], pos[2],
        vel[0], vel[1], vel[2],
        q.w, q.x, q.y, q.z,
        yaw_deg, pitch_deg, roll_deg,
        lin[0], lin[1], lin[2]
    );

    if( send(cli_fd, json_buf, n, MSG_NOSIGNAL) < 0) {
        perror("send");
        break;
    }
}
    close(cli_fd);
    close(srv);
    close(imu);
    return 0;
}



