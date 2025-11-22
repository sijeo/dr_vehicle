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
     { 0.0005961972665110525f, -3.3994858694753906e-06f, -3.142231005362935e-05f},
     {-1.8945179220887655e-05f, 0.0006019028574102943f, -1.7336208684040234e-05f},
     { -2.3401281900608534e-05f, -7.32518405425386e-06f, -0.0005912131487684689f}
 };

 static const float ACCEL_O[3] = {-0.20401249607212701f, -0.135733929694587f, -1.084172271336748f};

 static const float GYRO_A[3][3] = {    
     { 1674.061172775616f, 8.369116874774079f, -89.21986611126125f},
     { 50.7652460320293f, 1661.0587713439343f,-51.40552584215798f},
     { -66.89134413892583f, -20.911932209265842f, -1687.268945052593f}
 };   

 /* Gyro bias in raw counts */
    static const float GYRO_B[3] = { 245.93566666666536f, 180.08633333333228f, -1845.775333333301f };

/* -------------------ZUPT / DECAY TUNING ----------------------*/

    #define ACC_ZUPT_THRESH   0.05f    /* m/s^2 */
    #define GYRO_ZUPT_THRESH  (1.5f * DEG2RAD)   /* rad/s */

    #define ZUPT_COUNT_REQUIRED   10 /* Consecutive samples and qualified */
    #define VEL_DECAY_NEAR_ZERO   0.90f
    #define VEL_EPSILON           1e-3f
    #define POS_CLAMP_MAX         5.0f  /* m */

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
static void apply_accel_calib( int16_t ax, int16_t ay, int16_t az, float accel_mps2[3] )
{
    int i;
    float r[3] = {
        (float)ax,
        (float)ay,
        (float)az
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
        (float)s->gx,
        (float)s->gy,
        (float)s->gz
    };
    float gyro_dps[3];
    int i;
    for( i = 0; i < 3; i++ )
    {
        gyro_dps[i] = GYRO_A[i][0] * r[0] + GYRO_A[i][1] * r[1] + GYRO_A[i][2] * r[2] + GYRO_B[i];
    }

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
#endif


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


/* Estimate initial orientation from average accel (gravity )*/

static dr_quatf_t estimate_initial_orientation( int fd , const dr_ekf_config_t *cfg)
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
        apply_accel_calib(s.ax, s.ay, s.az, a);
        sum[0] += a[0];
        sum[1] += a[1];
        sum[2] += a[2];
        usleep((useconds_t)(1e6f / SAMPLE_HZ));
    } 
    float mean[3] = {sum[0]/N, sum[1]/N, sum[2]/N};
    dr_vec3f_t f_body =  dr_v3( mean[0], mean[1], mean[2] ); /* Specific force in body frame */
    dr_vec3f_t f_body_u = dr_v3_normalize( f_body );

    /* At rest, specific force direction = - gravity direction */
    dr_vec3f_t g_world = dr_v3( cfg->gravity[0], cfg->gravity[1], cfg->gravity[2] );\
    dr_vec3f_t f_world = dr_v3( -g_world.x, -g_world.y, -g_world.z );
    dr_vec3f_t f_world_u = dr_v3_normalize( f_world );
    
    dr_quatf_t q0 = q_from_two_unit_vecs( f_body_u, f_world_u );

    fprintf(stderr, "[CAL] Mean accel (m/s^2): [%.4f, %.4f, %.4f], norm=%.4f\n",
        mean[0], mean[1], mean[2], vec3_norm(mean) );
    return q0;
}

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
    cfg.gravity[2] = G_CONST;

    /* Noise densities /RW (tune as needed )*/
    cfg.sigma_g = 0.02f;        /* rad/s/rtHz */
    cfg.sigma_a = 0.1f;         /* m/s^2/rtHz */
    cfg.sigma_bg = 0.0005f;     /* rad/s^2/rtHz */
    cfg.sigma_ba = 0.005f;     /* m/s^3/rtHz */

    /* Initial uncertainties */
    cfg.p0_pos = 0.1f;        /* m */
    cfg.p0_vel = 0.1f;        /* m/s */
    cfg.p0_ang = 5.0f * DEG2RAD;   /* rad */
    cfg.p0_ba = 0.5f;         /* m/s^2 */
    cfg.p0_bg = 0.05f;        /* rad/s */

    /* Estimate Initial orientation from accel (gravity )*/
    dr_nominal_state_t x0;
    memset( &x0, 0, sizeof(x0) );
    x0.p[0] = 0.0f;
    x0.p[1] = 0.0f;
    x0.p[2] = 0.0f;
    x0.v[0] = 0.0f;
    x0.v[1] = 0.0f;
    x0.v[2] = 0.0f;
    x0.ba[0] = 0.0f;
    x0.ba[1] = 0.0f;
    x0.ba[2] = 0.0f;
    x0.bg[0] = 0.0f;
    x0.bg[1] = 0.0f;
    x0.bg[2] = 0.0f;
    x0.q = estimate_initial_orientation( imu_fd , &cfg);

    dr_ekf_init( &ekf, &cfg, &x0 );




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

    /* Main Streaming Loop */
    uint64_t t_prev = monotonic_time_ns();
    int zupt_counter = 0;

    while(1)
    {
       struct mpu6050_sample sample;
       if ( read( imu_fd, &sample, sizeof(sample) ) != sizeof(sample))
       {
          if( errno == EINTR )
              continue;
            perror("read");
            break;
       }

       uint64_t t_now = monotonic_time_ns();
       float dt = (float)(t_now - t_prev) * 1e-9f;
       if( dt <= 0.0f || dt > 1.0f )
              dt = 1.0f / SAMPLE_HZ;    
              
         t_prev = t_now;

         /* Apply calibration */
         float acc[3], gyro[3];
         apply_accel_calib( sample.ax, sample.ay, sample.az, acc );
         apply_gyro_calib( &sample, gyro );

         dr_imu_sample_t imu_s;
         imu_s.accel[0] = acc[0];
         imu_s.accel[1] = acc[1];
         imu_s.accel[2] = acc[2];
         imu_s.gyro[0] = gyro[0];
         imu_s.gyro[1] = gyro[1];
         imu_s.gyro[2] = gyro[2];

         /* Save previous velocity to derive linear accel later */
         float v_prev[3] = {
             ekf.x.v[0],
             ekf.x.v[1],
             ekf.x.v[2]
         };

            /* EKF Predict */
            dr_ekf_predict( &ekf, &imu_s, dt );

        /* Estimate linear acceleration in world frame from delta-v  */
        float lin_acc[3];
        lin_acc[0] = (ekf.x.v[0] - v_prev[0]) / dt;
        lin_acc[1] = (ekf.x.v[1] - v_prev[1]) / dt;
        lin_acc[2] = (ekf.x.v[2] - v_prev[2]) / dt;

        /* ZUPT check */
        float lin_norm = vec3_norm( lin_acc );
        float gyro_norm = vec3_norm( gyro );

        bool zupt = (lin_norm < ACC_ZUPT_THRESH ) && ( gyro_norm < GYRO_ZUPT_THRESH );

        if( zupt )
        {
            if( ++zupt_counter >= ZUPT_COUNT_REQUIRED ) {
                /* Apply ZUPT: zero velocity */
                ekf.x.v[0] = 0.0f;
                ekf.x.v[1] = 0.0f;
                ekf.x.v[2] = 0.0f;
                zupt_counter = ZUPT_COUNT_REQUIRED; /* cap */
            }
            
        } else {
                /* Not enough consecutive samples yet */
                zupt_counter = 0;
        }

        /* Velocity decay near zero to reduce drift */
        for ( i = 0; i < 3; i++ ){
            if (fabsf(lin_acc[i]) < ACC_ZUPT_THRESH ) {
                ekf.x.v[i] *= VEL_DECAY_NEAR_ZERO;
                if ( fabsf( ekf.x_est.v[i] ) < VEL_EPSILON )
                    ekf.x.v[i] = 0.0f;
            }

        }
        /* Position integration is already done in EKF predict 
        * but we can clamp for visualization 
        */
        for ( i = 0; i < 3; i++ ){
            if ( ekf.x.p[i] > POS_CLAMP_MAX )
                ekf.x.p[i] = POS_CLAMP_MAX;
            else if ( ekf.x.p[i] < -POS_CLAMP_MAX )
                ekf.x.p[i] = -POS_CLAMP_MAX;
        }

        float yaw, pitch, roll;
        euler_deg_from_q( ekf.x.q, &yaw, &pitch, &roll );
        /* Send JSON packet */
        char buf[512];
        int n  = snprintf( buf, sizeof(buf),
            "{ \"t_ns\": %llu, \"pos_m\": [%.4f, %.4f, %.4f], \"vel_mps\": [%.4f, %.4f, %.4f],"
            "\"lin_acc_mps2\": [%.4f, %.4f, %.4f], \"euler_deg\": [%.2f, %.2f, %.2f] , \"q\": [%.6f, %.6f, %.6f, %.6f] }\n", 
            (unsigned long long)t_now, ekf.x.p[0], ekf.x.p[1], ekf.x.p[2], 
            ekf.x.v[0], ekf.x.v[1], ekf.x.v[2],
            lin_acc[0], lin_acc[1], lin_acc[2],
            yaw, pitch, roll,
            ekf.x.q.w, ekf.x.q.x, ekf.x.q.y, ekf.x.q.z );

            if (n > 0 )
            {
                perror("snprintf");
                break;
            }

            ssize_t sw = send( sock_client, buf, (size_t)n, MSG_NOSIGNAL );
            if ( sw < 0 )
            {
                perror("send");
                break;
            }

            usleep( (useconds_t)(1e6f / SAMPLE_HZ) );
            

    }
    close( sock_client );
    close( sock_listen );
    close( imu_fd );
    return 0;
}
