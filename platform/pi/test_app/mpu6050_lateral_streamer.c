/**
 * mpu6050_lateral_streamer.c
 * Build:
 * gcc -std=c11 -o mpu6050_lateral_streamer mpu6050_lateral_streamer.c -lm
 * Description:
 * - Calibrates MPU6050 accel + gyro ( assumes flat, Z+ up during calibration )
 * - Estimates orientation via complementary filter.
 * - Rotates accel to world frame, subtracts gravity, integrates to get lateral position.
 * - Uses simple ZUPT to reduce drift when stationary.
 * - Streams JSON over TCP to a viewer.
 */

 #define _GNU_SOURCE
    #include <stdio.h>
    #include <stdlib.h>
    #include <stdint.h>
    #include <stdbool.h>
    #include <string.h>
    #include <unistd.h>
    #include <errno.h>
    #include <fcntl.h>
    #include <glob.h>
    #include <math.h>
    #include <time.h>
    #include <sys/ioctl.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>


    #include "../mpu6050_ioctl.h"
    #include "../core/dr_math.h"
    
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif

    /*--------------------Helper Functions--------------------*/
    static void msleep(unsigned int ms)
{
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

static int64_t mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
}

static int open_mpu6050(void) {
    glob_t g;
    if(glob("/dev/mpu6050-*", 0, NULL, &g) !=0 || ( g.gl_pathc == 0)){
        globfree(&g);
        return -ENOENT;
    }
    int fd = open(g.gl_pathv[0], O_RDONLY | O_CLOEXEC);
    int err = (fd <0) ? -errno : 0;
    globfree(&g);
    return (fd < 0) ? err : fd;
}

static int read_sample(int fd, struct mpu6050_sample* out)
{
    ssize_t r = read(fd, out, sizeof(struct mpu6050_sample));
    if ( r == (ssize_t)sizeof(struct mpu6050_sample)) {
        return 0;
    }
    return (r <0) ? -errno : -EIO;
}

static void raw_to_si(const struct mpu6050_fs* fs, const struct mpu6050_sample* s,float gyro_radps[3], float accel_mps2[3]) 
{
    const float g_range = (fs->accel == ACCEL_2G) ? 2.0f :
                          (fs->accel == ACCEL_4G) ? 4.0f :
                          (fs->accel == ACCEL_8G) ? 8.0f : 16.0f;
    const float dps_range = (fs->gyro == GYRO_250DPS) ? 250.0f :
                            (fs->gyro == GYRO_500DPS) ? 500.0f :
                            (fs->gyro == GYRO_1000DPS) ? 1000.0f : 2000.0f;
    const float a_lsb_to_mps2 = (g_range * 9.860665f) / 32768.0f; // m/s^2 per LSB
    const float g_lsb_to_rad = ((dps_range * (float)M_PI / 180.0f) / 32768.0f); // rad/s per LSB
    accel_mps2[0] = s->ax * a_lsb_to_mps2;
    accel_mps2[1] = s->ay * a_lsb_to_mps2;
    accel_mps2[2] = s->az * a_lsb_to_mps2;
    gyro_radps[0] = s->gx * g_lsb_to_rad;
    gyro_radps[1] = s->gy * g_lsb_to_rad;
    gyro_radps[2] = s->gz * g_lsb_to_rad;
}

/*--------------------Complementary Attitude Filter -------------------- */
static dr_quatf_t attitude_update(dr_quatf_t q, const float gyro[3], const float accel[3], float dt, float accel_gain)
{
    //1. Integrate gyro
    dr_quatf_t qg = dr_q_integrate_gyro(q, (dr_vec3f_t){ .x = gyro[0], .y = gyro[1], .z = gyro[2] }, dt);
    //2. Accel tilt correction (only direction used)
    dr_vec3f_t a_b = {.x = accel[0], .y = accel[1], .z = accel[2]};
    float an = sqrtf(a_b.x*a_b.x + a_b.y*a_b.y + a_b.z*a_b.z);
    if ( an < 1e-6f ) {
        return dr_q_normalize(qg);
    }
    a_b.x /= an; a_b.y /= an; a_b.z /= an;

    // Expected gravity in world frame: [0, 0, -1]
    dr_vec3f_t g_world = {0.0f, 0.0f, -1.0f};

    // gravity predicted in body frame: R^T * g_world
    // which is q^-1 * g_world * q

    dr_quatf_t qi = (dr_quatf_t){ qg.w, -qg.y, -qg.z};
    dr_vec3f_t g_b = dr_q_rotate(qi, g_world);


    // error = g_b x a_b ( axis that rotates g_b toward a_b )
    dr_vec3f_t err = {
        g_b.y*a_b.z - g_b.z*a_b*y,
        g_b.z*a_b.x - g_b.x*a_b.z,
        g_b.x*a_b.y - g_b.y*a_b.x
    };

    // small rotation propotional to error
    dr_vec3f_t dtheta = {
        accel_gain * err.x,
        accel_gain * err.y,
        accel_gain * err.z
    };

    dr_quatf_t qcorr = dr_q_from_rotvec(dtheta);
    dr_quatf_t q_new = dr_q_mult(qg, qcorr);
    return dr_q_normalize(q_new);
}

/* ----------------------Config ------------------*/
typedef struct {
    int port;
    int rate_hz;
    int calib_seconds;
    float accel_gain;
}cfg_t;

static void parse_args(int argc, char **argv, cfg_t *c)
{
    c->port = 9010;
    c->rate_hz = 100;
    c->calib_seconds = 10;
    c->accel_gain = 0.02f;  //0.005-0.05 is typical

    for(int i=1; i < argc; i++) {
        if(!strcmp(argv[i], "--port") && (i+1 < argc)) {
            c->port = atoi(argv[++i]);
        } else if(!strcmp(argv[i], "--rate_hz") && (i+1 < argc)) {
            c->rate_hz = atoi(argv[++i]);
        } else if(!strcmp(argv[i], "--calib_s") && (i+1 < argc)) {
            c->calib_seconds = atoi(argv[++i]);
        } else if(!strcmp(argv[i], "--accel_gain") && (i+1 < argc)) {
            c->accel_gain = atof(argv[++i]);
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            exit(1);
        }
    }
}

/* --------------Main --------------*/
int main(int argc, char** argv)
{
    cfg_t c;
    parse_args(argc, argv, &c);
    // Open MPU6050
    int fd = open_mpu6050();
    if ( fd < 0 ) {
        fprintf(stderr, "Failed to open MPU6050: %s\n", strerror(-fd));
        return 1;
    }
    struct mpu6050_fs fs;
    memset(&fs, 0, sizeof(fs));
    if ( ioctl(fd, MPU6050_IOC_GET_FS, &fs) != 0 ) {
        fprintf(stderr, "Failed to get MPU6050 full-scale: %s\n", strerror(errno));
        close(fd);
        return 1;
    }

    fprintf(stderr, "[INFO] Lateral Streamer @%d Hz, accel_gain=%.4f\n", c.rate_hz, c.accel_gain);
    fprintf(stderr, "[INFO] Place IMU Flat, Z+ up for %d seconds ...\n", c.calib_seconds);

    /* -------------Calibration-------------*/
    double sum_g[3] = {0}, sum_a[3] = {0};
    int N = c.calib_seconds * c.rate_hz;
    struct mpu6050_sample sample;
    int i;

    for(i = 0; i < N; i++) {
        if ( read_sample(fd, &sample) == 0 ) {
            float g[3], a[3];
            raw_to_si(&fs, &sample, g, a);
            sum_g[0] += g[0]; sum_g[1] += g[1]; sum_g[2] += g[2];
            sum_a[0] += a[0]; sum_a[1] += a[1]; sum_a[2] += a[2];
        }
        msleep(1000 / c.rate_hz);   
        }

    float bias_g[3] = { (float)(sum_g[0]/N), (float)(sum_g[1]/N), (float)(sum_g[2]/N) };

    // Accel bias: assume during calib gravity is +Z in sensor frame.
    // So expected: ax= 0, ay=0, az=+9.81
    float mean_ax = (float)(sum_a[0]/N);
    float mean_ay = (float)(sum_a[1]/N);
    float mean_az = (float)(sum_a[2]/N);

    float bias_a[3] = { mean_ax, mean_ay, mean_az - 9.860665f };

    fprintf(stderr, "[CAL] Gyro bias: [%.6f, %.6f, %.6f] rad/s\n", bias_g[0], bias_g[1], bias_g[2]);
    fprintf(stderr, "[CAL] Accel bias: [%.6f, %.6f, %.6f] m/s^2\n", bias_a[0], bias_a[1], bias_a[2]);
    fprintf(stderr, "[INFO] Starting TCP Server on port %d\n", c.port);

    // Setup TCP server
    int server = socket(AF_INET, SOCK_STREAM, 0);
    if ( server < 0 ) {
        fprintf(stderr, "Failed to create socket: %s\n", strerror(errno));
        close(fd);
        return 1;
    }
    int yes = 1; 
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)c.port);
    if ( bind(server, (struct sockaddr*)&addr, sizeof(addr)) < 0 || listen(server, 1) < 0 ) {
        fprintf(stderr, "Failed to bind/listen: %s\n", strerror(errno));
        close(server);
        close(fd);
        return 1;
    }

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(server, (struct sockaddr*)&client_addr, &client_len);
        if ( client < 0 ) {
            fprintf(stderr, "Failed to accept client: %s\n", strerror(errno));
            if(errno == EINTR)
                continue;
        }
        fprintf(stderr, "[INFO] Client connected: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
        
        // State
        dr_quatf_t q = dr_quat_identity();
        float vel[3] = {0}, pos[3] = {0};
        float g_bias[3] = {bias_g[0], bias_g[1], bias_g[2]};

        bool lp_init = false;
        float g_lp[3] = {0};
        float a_lp[3] = {0};

        int64_t t_prev = mono_ns();

        for(;;)
        {
            if( read_sample(fd, &sample) != 0 ) {
                msleep(1);
                continue;
            }
            float g_raw[3], a_raw[3];
            raw_to_si(&fs, &sample, g_raw, a_raw);
        
            // Remove bias
            float g0[3] = { g_raw[0] - g_bias[0], g_raw[1] - g_bias[1], g_raw[2] - g_bias[2] };
            float a0[3] = { a_raw[0] - bias_a[0], a_raw[1] - bias_a[1], a_raw[2] - bias_a[2] };

            // IIR Low-pass filter on both (cheap de-noise)
            float alpha = 0.1f;
            if ( !lp_init ) {
             memcpy(g_lp, g0, sizeof(g_lp));
             memcpy(a_lp, a0, sizeof(a_lp));
                lp_init = true;

            } else {
                for( i = 0; i < 3; i++ ){
                    g_lp[i] = alpha * g0[i] + (1.0f - alpha) * g_lp[i];
                    a_lp[i] = alpha * a0[i] + (1.0f - alpha) * a_lp[i];
                }
            }
            int64_t t_now = mono_ns();
            float dt = (float)(t_now - t_prev) * 1e-9f;
            if ( dt < 1e-5f ) dt = 1.0f / c.rate_hz;
            t_prev = t_now;

            // Adaptive gyro bias: refinement when still
            float g_norm = sqrtf(g_raw[0]*g_raw[0] + g_raw[1]*g_raw[1] + g_raw[2]*g_raw[2]);
            float a_norm = sqrtf(a_raw[0]*a_raw[0] + a_raw[1]*a_raw[1] + a_raw[2]*a_raw[2]);
            const float gyro_thresh = 0.02f; // rad/s
            const float g_tol = 0.15f * 9.800665f; // m/s^2
            const float k_bias - 0.001f; // bias correction rate
            if ( (g_norm < gyro_thresh) && (fabsf(a_norm - 9.860665f) < g_tol) ) {
                for(i =0; i <3; i++) {
                    g_bias[i] = (1.0f - k_bias) * g_bias[i] + k_bias * g_raw[i];
                }
            }

            //Complementary filter: attitude from gyro + accel
            // only trust accel if magnitude is near 1g
            float accel_gain = c.accel_gain;
            if( a_norm < 7.0f || a_norm > 12.0f ){
                accel_gain = 0.0f; // ignore accel
            }
            q = attitude_update(q, g_lp, a_raw, dt, accel_gain); // use raw accel dir for tilt

            // Rotate accel to world frame
            float R[9];
            dr_R_from_q(q, R);
            // R: row major, v_world = R * v_body

            // Linear accleration in world frame
            float ax_b = a_lp[0];
            float ay_b = a_lp[1];
            float az_b = a_lp[2];

            float ax_w = R[0]*ax_b + R[1]*ay_b + R[2]*az_b;
            float ay_w = R[3]*ax_b + R[4]*ay_b + R[5]*az_b;
            float az_w = R[6]*ax_b + R[7]*ay_b + R[8]*az_b;

            // Gravity ( world ) is [0,0,-9.81], subtract
            float lin_ax = ax_w;
            float lin_ay = ay_w;
            float lin_az = az_w + 9.860665f; // cause g_world is -9.81 in Z

            // Small accel noise gate
            const float a_eps = 0.03f; // m/s^2
            if ( fabsf(lin_ax) < a_eps ) lin_ax = 0.0f;
            if ( fabsf(lin_ay) < a_eps ) lin_ay = 0.0f;
            if ( fabsf(lin_az) < a_eps ) lin_az = 0.0f;

            // Zero velocity update (ZUPT) if still
            bool stationary = (g_norm < gyro_thresh) && (fabsf(a_norm - 9.860665f) < g_tol);
            if ( stationary ) {
                vel[0] = 0.0f;
                vel[1] = 0.0f;
                vel[2] = 0.0f;
            } else {
                // Integrate to get velocity + position
                vel[0] += lin_ax * dt;
                vel[1] += lin_ay * dt;
                vel[2] += lin_az * dt;

                 // clamp tiny velocities
            const float v_eps = 0.002f; // m/s
            for(i =0; i <3; i++) {
                if ( fabsf(vel[i]) < v_eps ) vel[i] = 0.0f;
            }

            }

            pos[0] += vel[0] * dt;
            pos[1] += vel[1] * dt;
            pos[2] += vel[2] * dt;

            // Optional soft bound to avoid runaway during experiments
            const float max_range = 5.0f;
            for(i =0; i <3; i++) {
                if ( pos[i] < -max_range ) pos[i] = -max_range;
                if ( pos[i] > max_range ) pos[i] = max_range;
            }

            char line[320];
            int n = snprintf(line, sizeof(line),
                "{ \"t_ns\":%lld,\"pos_m\":[%.4f,%.4f,%.4f],\"vel_mps\":[%.4f,%.4f,%.4f],\"lin_acc_mps2\":[%.4f,%.4f,%.4f]\n",
                (long long)t_now, pos[0], pos[1], pos[2],
                vel[0], vel[1], vel[2],
                lin_ax, lin_ay, lin_az);

            if ( send(client, line, (size_t)n, 0) <= 0 ) {
                fprintf(stderr, "[INFO] Client disconnected\n");
                break;
            }
            // Sleep to maintain rate
            msleep(1000 / c.rate_hz);

        
        }
        close(client);

    }
close(fd);
close(server);
return 0;
}
