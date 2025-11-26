/**
 * mpu6050_lateral_streamer.c
 * @brief lateral translation and orientation streamer using EKF fusion
 * (accel + gyro ) with LS Calibration (A, b, C, o).
 * 
 * RunTime:
 * - Opens /dev/mpu6050-0 (mpu6050_iio_driver must be loaded and device created).
 * - Uses LS Calibration matrices (A, b, C, o) from the imu_calibration.json.
 * - Uses 15-state EKF to fuse accel and gyro:
 *  States: position(3), velocity(3), attitude quaternion(4), gyro bias(3), accel bias(3)
 * - On each sample, it:
 *    does ekf predict
 *    computes linear acceleration from EKF velocities
 *    applies ZUPT + velocity decay
 *    streams JSON to imu_lateral_viewer.py over TCP 
 *
 */

 #define _GNU_SOURCE
 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include <string.h>
 #include <math.h>
 #include <errno.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <time.h>
 #include <stdbool.h>
 #include <sys/socket.h>
 #include <sys/types.h>
 #include <sys/ioctl.h>
 #include <arpa/inet.h>
 #include <netinet/in.h>

 #include "../mpu6050_ioctl.h"
 #include "../core/dr_math.h"
 #include "../core/dr_types.h"
 #include "../core/dr_ekf.h"


 #define IMU_DEV_PATH        "/dev/mpu6050-0"
 #define SERVER_PORT         9010
 #define SAMPLE_HZ         100

 #define G_CONST           9.80665f
 #define DEG2RAD           (3.14159265f / 180.0f)

 #define GYRO_SCALE_LSB_PER_DPS   65.5f   /* for +/- 500 dps full scale */

 /* -----------Paste form the imu_calibration.json here ---------------*/
 
 static const float ACCEL_C[3][3] = {
     { 0.0005961934151525817, -1.4056802633620443e-05,1.6514162578776096e-05},
     {1.3947209675238649e-05, 0.0006017838672845617, 1.1787053553186505e-05},
     {-3.239315574068853e-05, -1.6415215206900584e-05, 0.0005899858658002145}
 };

 static const float ACCEL_O[3] = {-0.1145544034597488, -0.08974878378291522, 0.993766742347543};

 static const float GYRO_A[3][3] = {    
     { 1673.8370901378141, 37.799809313068124, -47.60718492043665},
     { -40.571550937374134,1659.9048604773304,-32.02683893072529},
     { 90.77314883267991, 48.25903850958298, 1691.4509032136361}
 };   

 /* Gyro bias in raw counts */
    static const float GYRO_B[3] = { -163.114, 80.188, 699.349 };

/* -------------------ZUPT / DECAY TUNING ----------------------*/

    /* Attitude fusion gains (Mahony / Complementary style )*/
    #define ATT_KP  0.8f    /* Accel correction propotional gain */
    #define ATT_KI  0.05f    /* Accel correction integral gain */

    /* Linear acceleration Low Pass filter coefficient (0 .. 1)*/
    #define LIN_ACC_ALPHA   0.90f    /* 0.90 @ 100Hz ~ 1-2Hz corner */


    #define ACC_ZUPT_THRESH   0.08f    /* m/s^2 */
    #define GYRO_ZUPT_THRESH  (1.5f * DEG2RAD)   /* rad/s */
    #define ZUPT_COUNT_REQUIRED   10 /* Consecutive samples and qualified */


    #define VEL_DECAY_NEAR_ZERO   0.98f
    #define VEL_EPSILON           1e-3f
    #define POS_CLAMP_MAX         5.0f  /* m */

    #define YAW_FREEZE_GYRO_Z_THRESH   (0.3f * DEG2RAD) /* rad/s */



/* --------------------------------------------------------------*/

static uint64_t monotonic_time_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static float vec3_norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
#if 0
static void vec3_add( float out[3], const float a[3], const float b[3] )
{
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

static void vec3_scale( float out[3], const float v[3], float s )
{
    out[0] = v[0] * s;
    out[1] = v[1] * s;
    out[2] = v[2] * s;
}
#endif

/* quaternion from two unit vectors (v_from -> v_to )*/
static dr_quatf_t q_from_two_unit_vecs( dr_vec3f_t v_from, dr_vec3f_t v_to )
{
    dr_vec3f_t c = {
        v_from.y * v_to.z - v_from.z * v_to.y,
        v_from.z * v_to.x - v_from.x * v_to.z,
        v_from.x * v_to.y - v_from.y * v_to.x
    };
    float d = v_from.x * v_to.x + v_from.y * v_to.y + v_to.z;
    dr_quatf_t q = {1.0f + d, c.x, c.y, c.z };
    return dr_q_normalize(q);
}

/* Euler (deg) from quaternion, ZYX order: yaw(Z)-pitch(Y)-roll(X) */
static void euler_deg_from_q( dr_quatf_t q, float *yaw, float *pitch, float *roll)
{
    float R[9];
    dr_R_from_q(q, R); /* row-major 3x3 */

    float r = atan2f( R[5], R[8] ); /* roll */
    float p = asinf( -R[2] );        /* pitch */
    float y = atan2f( R[1], R[0] ); /* yaw */

    *roll = r * (180.0f)/3.14159265f ;
    *pitch = p * (180.0f)/3.14159265f ;
    *yaw = y * (180.0f)/3.14159265f ;
}

/* Apply LS calibration to accel raw counts -> m/s^2*/
static void apply_accel_calib( const struct mpu6050_sample *s, float accel_mps2[3] )
{
    int i;
    float r[3] = {
        (float)s->ax, (float)s->ay, (float)s->az
    };

    for( i = 0; i < 3; i++ )
    {
        accel_mps2[i] = ACCEL_C[i][0] * r[0] + ACCEL_C[i][1] * r[1] + ACCEL_C[i][2] * r[2] + ACCEL_O[i];
    }


}

/* Apply gyro bias + scale -> rad/s */
static void apply_gyro_calib( const struct mpu6050_sample *s, float gyro_rads[3] )
{
    
    float r[3] = {
        (float)s->gx - GYRO_B[0],
        (float)s->gy - GYRO_B[1],
        (float)s->gz - GYRO_B[2]
    };
    float gyro_dps[3];
    gyro_dps[0] = r[0] / GYRO_SCALE_LSB_PER_DPS;
    gyro_dps[1] = r[1] / GYRO_SCALE_LSB_PER_DPS;
    gyro_dps[2] = r[2] / GYRO_SCALE_LSB_PER_DPS;


    /* Convert to rad/s */
    gyro_rads[0] = gyro_dps[0] * DEG2RAD;
    gyro_rads[1] = gyro_dps[1] * DEG2RAD;
    gyro_rads[2] = gyro_dps[2] * DEG2RAD;
}

/**
 * Estimate gravity vector in IMU frame from a few seconds of still 
 * calibrated accel samples
 * 
 */
#if 0
static void estimate_gravity_vector( int imu_fd, float g_body[3] )
{
    const int N = 500;
    float g_sum[3] = {0};
    struct mpu6050_sample sample;
    int i;

    fprintf(stderr, "[CAL] Estimating gravity vector... Keep IMU still.\n");

    for( i = 0; i < N; i++ )
    {
        if( read( imu_fd, &sample, sizeof(sample) ) != sizeof(sample) )
        {
            perror("Failed to read mpu6050 sample for gravity estimation");
            exit(EXIT_FAILURE);
        }

        float accel_mps2[3];
        apply_accel_calib( sample.ax, sample.ay, sample.az, accel_mps2 );

        g_sum[0] += accel_mps2[0];
        g_sum[1] += accel_mps2[1];
        g_sum[2] += accel_mps2[2];
        usleep(1000000 / SAMPLE_HZ);
    }

    g_body[0] = g_sum[0] / (float)N;
    g_body[1] = g_sum[1] / (float)N;
    g_body[2] = g_sum[2] / (float)N;

    fprintf(stderr, "[CAL] Estimated gravity vector (m/s^2): [%.4f, %.4f, %.4f] (norm=%.4f)\n",
        g_body[0], g_body[1], g_body[2], vec3_norm(g_body) );
}


/* Complementary attitude update: integrate gyro, correct with accel tilt */

static dr_quatf_t attitude_update( dr_quatf_t q, const float gyro_rad[3], const float accel_mps2[3], float dt, float accel_gain )
{
    /* Integrate gyro */
    dr_quatf_t qg = dr_q_integrate_gyro( q, (dr_vec3f_t){ gyro_rad[0], gyro_rad[1], gyro_rad[2] }, dt);

    /* 2) Accel tilt correction only if accel mangitude ~ 1g */
    float ax = accel_mps2[0], ay = accel_mps2[1], az = accel_mps2[2];
    float anorm = sqrtf(ax*ax + ay*ay + az*az);
    if ( anorm < 1e-4f ) {
        return dr_q_normalize(qg);
    }
    
    /* normailize accel -> approximate gravity direction (body frame ) */
    dr_vec3f_t a_unit = { ax/anorm, ay/anorm, az/anorm };

    /* World gravity is [0, 0, -1 ]*/
    dr_vec3f_t g_world = {0.0f , 0.0f, -1.0f};

    /* predicted gravity in body frame: q^{-1} * g_world * q */
    dr_quatf_t qi = (dr_quatf_t){qg.w, -qg.x, -qg.y, -qg.z };
    dr_vec3f_t g_b = dr_q_rotate(qi, g_world);

    /* error axis = g_b x a_unit */
    dr_vec3f_t err = {
        g_b.y * a_unit.z - g_b.z * a_unit.y,
        g_b.z * a_unit.x - g_b.x * a_unit.z,
        g_b.x * a_unit.y - g_b.y * a_unit.x
    };

    dr_vec3f_t dtheta = { accel_gain * err.x,
        accel_gain * err.y,
        accel_gain * err.z
    };

    dr_quatf_t qcorr = dr_q_from_rotvec(dtheta);
    dr_quatf_t q_new = dr_q_mult( qg, qcorr);
    return dr_q_normalize(q_new);
}
#endif 

/* Estimate initial orientation from average accel (gravity )*/

static void estimate_initial_orientation( int fd , dr_nominal_state_t *x0, const dr_ekf_config_t *cfg)
{
    const int N = 500;
    float sum[3] = {0};
    struct mpu6050_sample s;
    int i;

    fprintf(stderr, "[CAL] Estimating initial orientation from accel (keep still )... \n");
    for( i = 0; i < N; i++ ){
        if ( read(fd, &s, sizeof(s)) != sizeof(s))
        {
            perror("read");
            break;
        }
        float a[3];
        apply_accel_calib(&s, a);
        sum[0] += a[0];
        sum[1] += a[1];
        sum[2] += a[2];
        usleep((useconds_t)(1e6f / SAMPLE_HZ));
    } 
    float mean[3] = {sum[0]/N, sum[1]/N, sum[2]/N};
    dr_vec3f_t f_body =  dr_v3( mean[0], mean[1], mean[2] ); /* Specific force in body frame */
    float norm = vec3_norm( mean );

   /* direction of specific force in body frame */
   dr_vec3f_t f_body = dr_v3( mean[0]/norm, mean[1]/norm, mean[2]/norm );

   /* At rest: f_world = -gravity direction */
    dr_vec3f_t g_world = dr_v3(cfg->gravity[0], cfg->gravity[1], cfg->gravity[2] );
    float g_norm = sqrtf(g_world.x * g_world.x + g_world.y * g_world.y + g_world.z * g_world.z );
    dr_vec3f_t f_world = dr_v3( -g_world.x / g_norm, -g_world.y / g_norm, -g_world.z / g_norm );

    dr_quatf_t q0 = q_from_two_unit_vecs( f_body, f_world ); /* body f -> worold f*/
    q0 = dr_q_normalize(q0);

    memset(x0, 0, sizeof(*x0));
    x0->q = q0;
    x0->p[0] = 0.0f;
    x0->p[1] = 0.0f;
    x0->p[2] = 0.0f;
    x0->v[0] = 0.0f;
    x0->v[1] = 0.0f;
    x0->v[2] = 0.0f;
    x0->ba[0] = 0.0f;
    x0->ba[1] = 0.0f;
    x0->ba[2] = 0.0f;
    x0->bg[0] = 0.0f;
    x0->bg[1] = 0.0f;
    x0->bg[2] = 0.0f;

    fprintf(stderr, "[CAL] Mean Accel: [%.4f, %.4f, %.4f]m/s^2 (norm=%.4f)\n",
        mean[0], mean[1], mean[2], norm );

    fprintf(stderr, "[CAL] Initial Orientation Quaternion: [%.6f, %.6f, %.6f, %.6f]\n",
        q0.w, q0.x, q0.y, q0.z );
}


/* Mahony / Complementry-style attitude update 
* Integrate gyro
* Uses accel to correct tilt
* can freeze yaw when stationary.
*/

int main( void )
{
    int i;
    /* Open IMU */
    int imu_fd = open( IMU_DEV_PATH, O_RDONLY );
    if ( imu_fd < 0 )
    {
        perror("Failed to open IMU device");
        return 1;
    }

    /* ---------------- Setup EKF Config ---------------- */
    dr_ekf_t ekf;
    memset( &ekf, 0, sizeof(ekf) );

    dr_ekf_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.gravity[0] = 0.0f;
    cfg.gravity[1] = 0.0f;
    cfg.gravity[2] = -G_CONST;

    /* Noise densities /RW (tune as needed )*/
    cfg.sigma_g = 0.02f;        /* rad/s/rtHz */
    cfg.sigma_a = 0.01f;         /* m/s^2/rtHz */
    cfg.sigma_bg = 0.005f;     /* rad/s^2/rtHz */
    cfg.sigma_ba = 0.005f;     /* m/s^3/rtHz */

    /* Initial uncertainties */
    cfg.p0_pos = 0.1f;        /* m */
    cfg.p0_vel = 0.1f;        /* m/s */
    cfg.p0_ang = 5.0f * DEG2RAD;   /* rad */
    cfg.p0_ba = 0.5f;         /* m/s^2 */
    cfg.p0_bg = 0.05f;        /* rad/s */

    ekf.cfg = cfg;

    /* Estimate Initial orientation from accel (gravity )*/
    dr_nominal_state_t x0;
    memset( &x0, 0, sizeof(x0) );
    estimate_initial_orientation( imu_fd, &x0, &ekf.cfg );

    dr_ekf_init( &ekf, &ekf.cfg, &x0 );


    /* Open TCP Server */
    int sock_listen =  socket(AF_INET, SOCK_STREAM, 0);
    if ( sock_listen < 0 )
    {
        perror("socket");
        close( imu_fd );
        return 1;
    }

    int opt = 1;
    setsockopt( sock_listen, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt) );

    struct sockaddr_in server_addr;
    memset (&server_addr, 0, sizeof(server_addr) );
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons( SERVER_PORT );

    if ( bind( sock_listen, (struct sockaddr *)&server_addr, sizeof(server_addr) ) < 0 )
    {
        perror("bind");
        close( imu_fd );
        close( sock_listen );
        return 1;
    }

    if ( listen( sock_listen, 1 ) < 0 )
    {
        perror("listen");
        close( imu_fd );
        close( sock_listen );
        return 1;
    }

    fprintf(stderr, "[INFO] Waiting for Viewer on port %d...\n", SERVER_PORT );

    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    int sock_client = accept( sock_listen, (struct sockaddr *)&client_addr, &client_addr_len );

    if ( sock_client < 0 )
    {
        perror("accept");
        close( imu_fd );
        close( sock_listen );
        return 1;
    }

    fprintf(stderr, "[INFO] Client Connected.\n");

    /*----------------------------------------- Visual Integration State ------------------------ */
    float pos_vis[3] = {0.0f, 0.0f, 0.0f};
    float vel_vis[3] = {0.0f, 0.0f, 0.0f};

    float lin_acc_filt[3] = {0.0f, 0.0f, 0.0f};
    float lin_acc_init = false;

    int zupt_count = 0;


    float yaw_display = 0.0f;
    bool yaw_display_init = false;

    uint64_t t_prev = monotonic_time_ns();

    struct mpu6050_sample s;

    while(1)
    {
       ssize_t r = read( imu_fd, &s, sizeof(s) );
       if( r != (ssize_t)sizeof(s))
       {
        if ( r < 0 && errno == EINTR )
            continue;
        perror("read IMU ");
        break;
       }

       uint64_t t_now_ns = monotonic_time_ns();
       float dt = (float)( t_now_ns - t_prev ) * 1e-9f;
       if( dt <= 0.0f || dt > 0.1f)
       {
            dt = 1.0f / SAMPLE_HZ;
       }
       t_prev = t_now_ns;

       /* Calibrate IMU Sample */
       float acc_b[3], gyro_rad[3];
        apply_accel_calib( &s, acc_b );
        apply_gyro_calib( &s, gyro_rad );

        /* 2. EKF Predict using calibrated accel + gyro */
        dr_imu_sample_t imu;
        imu.accel[0] = acc_b[0];
        imu.accel[1] = acc_b[1];
        imu.accel[2] = acc_b[2];
        imu.gyro[0] = gyro_rad[0];
        imu.gyro[1] = gyro_rad[1];
        imu.gyro[2] = gyro_rad[2];

        dr_ekf_predict( &ekf, &imu, dt );

        /* 3. World frame linear acceleration from EKF state. */

        dr_vec3f_t acc_b_unbiased = dr_v3(
            acc_b[0] - ekf.x.ba[0],
            acc_b[1] - ekf.x.ba[1],
            acc_b[2] - ekf.x.ba[2]
        );

        dr_vec3f_t acc_w = dr_q_rotate( ekf.x.q, acc_b_unbiased ); /* body -> world */

        float lin_acc_raw[3];
        lin_acc_raw[0] = acc_w.x - ekf.cfg.gravity[0];
        lin_acc_raw[1] = acc_w.y - ekf.cfg.gravity[1];
        lin_acc_raw[2] = acc_w.z - ekf.cfg.gravity[2];

        /* 4. Stationary Detection ( for ZUPT and yaw freeze )*/
        float acc_norm = vec3_norm(acc_b);
        float gyro_norm = vec3_norm(gyro_rad);
        bool accel_near_1g = fabsf(acc_norm - G_CONST) < 0.15f * G_CONST; /* 15% tolerance */
        bool gyro_small = gyro_norm < GYRO_ZUPT_THRESH;
        bool still = accel_near_1g && gyro_small;

        if ( still )
        {
            /* if at rest, we *expect* no linear acceleration */
            lin_acc_raw[0] = 0.0f;
            lin_acc_raw[1] = 0.0f;
            lin_acc_raw[2] = 0.0f;

        }
        /* 5. Low Pass filter Linear acceleration */
        if ( !lin_acc_init )
        {
            lin_acc_filt[0] = lin_acc_raw[0];
            lin_acc_filt[1] = lin_acc_raw[1];
            lin_acc_filt[2] = lin_acc_raw[2];
            lin_acc_init = true;
        }
        else
        {
            lin_acc_filt[0] = LIN_ACC_ALPHA * lin_acc_filt[0] + (1.0f - LIN_ACC_ALPHA) * lin_acc_raw[0];
            lin_acc_filt[1] = LIN_ACC_ALPHA * lin_acc_filt[1] + (1.0f - LIN_ACC_ALPHA) * lin_acc_raw[1];
            lin_acc_filt[2] = LIN_ACC_ALPHA * lin_acc_filt[2] + (1.0f - LIN_ACC_ALPHA) * lin_acc_raw[2];
        }

        float lin_acc[3] = {
            lin_acc_filt[0],
            lin_acc_filt[1],
            lin_acc_filt[2]
        };

        /* 6. ZUPT detection using filtered linear acceleration + gyro */
        float lin_norm = vec3_norm(lin_acc);
        bool zupt = (lin_norn < ACC_ZUPT_THRESH) && gyro_small;

        if (zupt)
        {
            if ( ++zupt_count >= ZUPT_COUNT_REQUIRED )
            {
                vel_vis[0] = 0.0f;
                vel_vis[1] = 0.0f;  
                vel_vis[2] = 0.0f; /* reset velocity */ 

            } 
        } else
        {
                zupt_count = 0; /* reset count */
        }

        /*7. Integrate Velocity (visual) when not firmly in ZUPT */
        if (!zupt) {
            vel_vis[0] += lin_acc[0] * dt;
            vel_vis[1] += lin_acc[1] * dt;
            vel_vis[2] += lin_acc[2] * dt;
        }

            /* Apply velocity decay */
        for(i = 0; i < 3; i++)
        {
          if( fabsf(lin_acc[i]) < ACC_ZUPT_THRESH )
          {
            vel_vis[i] *= VEL_DECAY_NEAR_ZERO;
            if( fabsf(vel_vis[i]) < VEL_EPSILON )
            {
                vel_vis[i] = 0.0f; /* clamp near-zero velocities */
            }
          }

        }

        /* 8. Integrate Position (visual) */
        for( i = 0; i < 3; i++ )
        {
            pos_vis[i] += vel_vis[i] * dt;
            if ( pos_vis[i] > POS_CLAMP_MAX )
            {
                pos_vis[i] = POS_CLAMP_MAX; /* clamp position */
            }
            else if ( pos_vis[i] < -POS_CLAMP_MAX )
            {
                pos_vis[i] = -POS_CLAMP_MAX; /* clamp position */
            }
        }

        /* 9. Euler angles from EKF quaternion */
        float yaw_ekf, pitch_ekf, roll_ekf;
        euler_deg_from_q(ekf.x.q, &yaw_ekf, &pitch_ekf, &roll_ekf);

        /* Yaw display stabilization: freeze yaw when still and gyro_z small */
        if (!yaw_display_init) {
            yaw_display = yaw_ekf; /* initial value */
            yaw_display_init = true;
        } else {
            if ( still && fabsf(gyro_rad[2]) < YAW_FREEZE_GYRO_Z_THRESH ) {
                /* keep previous yaw_display */
            } else {
                yaw_display = yaw_ekf;
            }
        }

        /* 10. Send JSON over TCP */
        char buf[512];
        int n = snprintf( buf, sizeof(buf),
            "{ \"t_ns\": %llu, \"pos_m\": [%.4f, %.4f, %.4f], \"vel_mps\": [%.4f, %.4f, %.4f], "
            "\"euler_deg\": [%.2f, %.2f, %.2f], \"lin_acc_mps2\": [%.4f, %.4f, %.4f], \"q\": [%.6f, %.6f, %.6f, %.6f] }\n",
            (unsigned long long)t_now_ns, pos_vis[0], pos_vis[1], pos_vis[2],
            vel_vis[0], vel_vis[1], vel_vis[2],
            yaw_display, pitch_ekf, roll_ekf,
            lin_acc[0], lin_acc[1], lin_acc[2],
            ekf.x.q.w, ekf.x.q.x, ekf.x.q.y, ekf.x.q.z
        );

        if( n  < 0)
        {
            perror("snprintf");
            break;
        }
        ssize_t sent = send( sock_client, buf, (size_t)n, MSG_NOSIGNAL );
        if ( sent < 0 )
        {
            perror("send");
            break;
        }
            
         /* Optional: Periodically print raw EKF p/v for testing core */
         // static int dbg_counter = 0;
         // if ( ++dbg_counter >= SAMPLE_HZ )
            // { 
            // dbg_counter = 0;
            // fprintf(stderr, "[EKF] p: [%.4f, %.4f, %.4f] m, v: [%.4f, %.4f, %.4f] m/s\n",
            //     ekf.x.p[0], ekf.x.p[1], ekf.x.p[2],
            //     ekf.x.v[0], ekf.x.v[1], ekf.x.v[2] );
            //}

            
    }
    close( sock_client );
    close( sock_listen );
    close( imu_fd );
    return 0;
}
