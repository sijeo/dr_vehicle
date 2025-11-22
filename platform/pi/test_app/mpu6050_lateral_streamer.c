/**
 * mpu6050_lateral_streamer.c
 * 
 * - Uses mpu6050_char driver (/dev/mpu6050-0)
 * - Applies  LS calibration (C, o, gyro_bias, gyro_scale) from imu_calib_gui.py
 * - Estimates linear acceleration (gravity removed)
 * - Integrates to velocity and position
 * - Applies ZUPT + velocity decay so motion stops when IMU stops
 * - Streams JSON over TCP to imu_lateral_streamer.py
 * - Streams JSON over TCP to imu_lateral_streamer.py
 * { "t_ns": ..., "pos_m": [...], "vel_mps": [...], "lin_acc_mps2": [...] }
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


 #define IMU_DEV_PATH        "/dev/mpu6050-0"
 #define SERVER_PORT         9010
 #define SAMPLE_HZ         100

 #define G_CONST           9.80665f
 #define DEG2RAD           (3.14159265f / 180.0f)

 #define GYRO_SCALE_LSB_PER_DPS   65.5f   /* for +/- 500 dps full scale */

 /* -----------Paste form the imu_calibration.json here ---------------*/
 
 static const float C[3][3] = {
     { 0.0005963711958255211f,  1.158486442783522e-05f, 2.0885245451422986e-06f},
     {-1.0070047127218174e-05f,   0.000602066470161879f, 9.559588107960574e-06f},
     { -1.2489036924665308e-05f,  1.0884932229524887e-05f,  0.0005925365798407456f}
 };

 static const float accel_o[3] = {-0.24456310935638237f, -0.12142627167583814f, 0.7449054548921515f };

 /* Gyro bias in raw counts */
    static const float gyro_bias[3] = {  -143.799f, 48.377f, 850.153f };

/* -------------------ZUPT / DECAY TUNING ----------------------*/

    #define ACC_ZUPT_THRESH   0.05f    /* m/s^2 */
    #define GYRO_ZUPT_THRESH  (1.5f * DEG2RAD)   /* rad/s */

    #define ZUPT_COUNT_REQUIRED   10 /* Consecutive samples and qualified */
    #define VEL_DECAY_NEAR_ZERO   0.90f
    #define VEL_EPSILON           1e-3f

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
        accel_mps2[i] = C[i][0] * r[0] + C[i][1] * r[1] + C[i][2] * r[2] + accel_o[i];
    }


}

/* Apply gyro bias + scale -> rad/s */
static void apply_gyro_calib( int16_t gx, int16_t gy, int16_t gz, float gyro_rads[3] )
{
    float gx_corr = (float)gx - gyro_bias[0];
    float gy_corr = (float)gy - gyro_bias[1];
    float gz_corr = (float)gz - gyro_bias[2];

    float gx_dps = gx_corr / GYRO_SCALE_LSB_PER_DPS;
    float gy_dps = gy_corr / GYRO_SCALE_LSB_PER_DPS;
    float gz_dps = gz_corr / GYRO_SCALE_LSB_PER_DPS;

    gyro_rads[0] = gx_dps * DEG2RAD;
    gyro_rads[1] = gy_dps * DEG2RAD;
    gyro_rads[2] = gz_dps * DEG2RAD;
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

static dr_quatf_t estimate_initial_orientation( int fd )
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
    float n = vec3_norm(mean);
    dr_vec3f_t g_body = { mean[0]/n, mean[1]/n, mean[2]/n };
    dr_vec3f_t g_world = { 0.0f, 0.0f, -1.0f };

    fprintf(stderr, "[CAL] Mean accel: [%.4f, %.4f, %.4f] m/s^2, |a|=%.4f\n", mean[0], mean[1], mean[2], n);

    dr_quatf_t q0 = q_from_two_unit_vecs(g_body, g_world);
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

    /* Initial orientation from IMU */
    dr_quatf_t q0 = estimate_initial_orientation( imu_fd );

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

    /* State */
    dr_quatf_t q = q0;
    float pos[3] = {0};
    float vel[3] = {0};
    uint64_t t_prev_ns = monotonic_time_ns();
    int zupt_counter = 0;


    while(1)
    {
        struct mpu6050_sample sample;
        ssize_t r = read( imu_fd, &sample, sizeof(sample) );
        if ( r != sizeof(sample) )
        {
            perror("Failed to read mpu6050 sample");
            break;
        }

        uint64_t t_ns = monotonic_time_ns();
        float dt = (float)(t_ns - t_prev_ns) * 1e-9f;
        if( dt <= 0.0f || dt > 0.1f )
        {
            dt = 1.0f / (float)SAMPLE_HZ;
        }
        t_prev_ns = t_ns;

        /* Calibrated accel and gyro */
        float acc[3];
        float gyro[3];
        apply_accel_calib( sample.ax, sample.ay, sample.az, acc );
        apply_gyro_calib( sample.gx, sample.gy, sample.gz, gyro );

        /* Attitude : Complementary Filter */
        float acc_norm = vec3_norm( acc );
        float accel_gain = 0.0f;
        if( acc_norm > 0.7f * G_CONST && acc_norm < 1.3f * G_CONST )
        {
            accel_gain = 0.02f; /* Tune : 0.005 - 0.05 */
        }
        q = attitude_update( q, gyro, acc, dt, accel_gain );

        /* Rotation Matrix: body -> world   */
        float R[9];
        dr_R_from_q( q, R ); /* row-major 3x3 */

        /* Linear Acceleration: a_world - g_world */
        float ax_b = acc[0];
        float ay_b = acc[1];
        float az_b = acc[2];
        float ax_w = R[0]*ax_b + R[1]*ay_b + R[2]*az_b;
        float ay_w = R[3]*ax_b + R[4]*ay_b + R[5]*az_b;
        float az_w = R[6]*ax_b + R[7]*ay_b + R[8]*az_b;

        float lin_acc[3];
        lin_acc[0] = ax_w;
        lin_acc[1] = ay_w;
        lin_acc[2] = az_w + G_CONST; /* world gravity is [0,0,-G_CONST]*/

        /* ZUPT detection */
        float lin_norm = vec3_norm( lin_acc );  
        float gyro_norm = vec3_norm( gyro );
        bool zupt = (lin_norm < ACC_ZUPT_THRESH ) && ( gyro_norm < GYRO_ZUPT_THRESH );

        if( zupt )
        {
            if ( ++zupt_counter >= ZUPT_COUNT_REQUIRED )
            {
                /* Apply ZUPT */
                vel[0] = 0.0f;
                vel[1] = 0.0f;
                vel[2] = 0.0f;
            }
        }else {
            zupt_counter = 0;
        }

        /* Integrate to velocity and position */
        if( !zupt )
        {
            vel[0] += lin_acc[0] * dt;
            vel[1] += lin_acc[1] * dt;
            vel[2] += lin_acc[2] * dt;
        }

        /* Velocity decay near zero to bleed drift */
        for( i = 0; i < 3; i++ )
        {
            if( fabsf( lin_acc[i] ) < ACC_ZUPT_THRESH )
            {
                vel[i] *= VEL_DECAY_NEAR_ZERO;
                if( fabsf( vel[i] ) < VEL_EPSILON )
                {
                    vel[i] = 0.0f;
                }
            }
        }

        /* Orientation as Euler (deg) */
        float yaw, pitch, roll;
        euler_deg_from_q( q, &yaw, &pitch, &roll );

        /* JSON packet */
        char buf[512];
        int n = snprintf( buf, sizeof(buf),
            "{ \"t_ns\": %llu, \"pos_m\": [%.4f, %.4f, %.4f], \"vel_mps\": [%.4f, %.4f, %.4f], "
            "\"lin_acc_mps2\": [%.4f, %.4f, %.4f], \"euler_deg\": [%.2f, %.2f, %.2f] ,\"q\": [%.6f, %.6f, %.6f, %.6f] }\n",
            (unsigned long long)t_ns, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], lin_acc[0], lin_acc[1], lin_acc[2],
            yaw, pitch, roll, q.w, q.x, q.y, q.z);

        if (n < 0) {
            perror("snprintf");
            break;
        }

        /* Send to client */
        if ( send(sock_client, buf, (size_t)n, MSG_NOSIGNAL) < 0 )
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
