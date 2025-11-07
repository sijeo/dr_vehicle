// mpu6050_streamer.c
// Run : ./mpu6050_streamer [--port 9009] [--rate_hz 100] [--calib_s 5] [--avg 5] [--accel_gain 0.02]

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
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "../mpu6050_ioctl.h"
#include "../core/dr_math.h"

// Default parameters
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ----------------Small Helper Functions ------------------*/
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

static void euler_deg_from_q(dr_quatf_t q, float* yaw_deg, float* pitch_deg, float* roll_deg){
    float R[9]; 
    dr_R_from_q(q, R);
    *roll_deg = (float)atan2f(R[5], R[8]) * (180.0f / (float)M_PI);
    *pitch_deg = (float)asinf(-R[2]) * (180.0f / (float)M_PI);
    *yaw_deg = (float)atan2f(R[1], R[0]) * (180.0f / (float)M_PI);
}

/*-------------Config-------------*/
typedef struct {
    int port;
    unsigned rate_hz;
    unsigned calib_s;
    unsigned avg;
    float accel_gain;  /* complementary gain for accel correction (0 .. 0.1)*/
} cfg_t;

/*------------ Moving Average Filter -------------*/
typedef struct {
    int n, idx, cap;
    float *buf; /* [cap][6]: gx, gy, gz, ax, ay, az */
}ma_t;

static void ma_init(ma_t* ma, int capacity){
    ma->n = 0;
    ma->idx = 0;
    ma->cap = capacity;
    ma->buf = (float*)calloc(capacity * 6, sizeof(float));
}

static void ma_free(ma_t* ma){
    free(ma->buf);
    ma->buf = NULL;
}

static void ma_push(ma_t* ma, const float g[3], const float a[3]){
    float *row = &ma->buf[ma->idx * 6];
    row[0] = g[0]; row[1] = g[1]; row[2] = g[2];
    row[3] = a[0]; row[4] = a[1]; row[5] = a[2];
    ma->idx = (ma->idx + 1) % ma->cap;
    if ( ma->n < ma->cap ) ma->n++;
}

static void ma_get(const ma_t* ma, float g[3], float a[3]){
    float s[6] = {0};
    int N = ma->n ? ma->n : 1;
    int i, k;
    for(i = 0; i< ma->n; i++) {
        for( k = 0; k < 6; k++) {
            s[k] += ma->buf[i * 6 + k];
        }
    }
    for( k = 0; k <3; k++) {
        g[k] = s[k] / N;
        a[k] = s[k + 3] / N;
    }
}

/*-------------------------attitude filter(complementary)-------------------------*/
static dr_quatf_t attitude_update(dr_quatf_t q, const float gyro[3], const float accel[3], float dt, float accel_gain)
{
    //1. Integrate gyro
    dr_quatf_t qg; dr_q_integrate_gyro(q, (dr_vec3f_t){ .x = gyro[0], .y = gyro[1], .z = gyro[2] }, dt);

    //2. Accel tilt correction (assume accel = gravity in body; normalize)
    dr_vec3f_t a_b = dr_v3_normalize((dr_vec3f_t){ .x = accel[0], .y = accel[1], .z = accel[2] });
    // expected gravity in body from current attitude R^T * [ 0, 0, 1 ]world
    dr_vec3f_t g_world = (dr_vec3f_t){ .x = 0.0f, .y = 0.0f, .z = -1.0f };
    // rotate world -> body = {q^-1} * g * q
    dr_quatf_t qc = (dr_quatf_t){ .w = qg.w, .x = -qg.x, .y = -qg.y, .z = -qg.z };
    dr_vec3f_t g_b = dr_q_rotate(qc, (dr_vec3f_t){ .x = g_world.x, .y = g_world.y, .z = g_world.z });

    // error = a_b x g_b (axis bringing g_b to a_b); small angle correction
    dr_vec3f_t err = dr_v3_cross(g_b, a_b);
    // apply small corrective rotation
    dr_vec3f_t dtheta = dr_v3_scale(accel_gain, err);
    dr_quatf_t qcorr = dr_q_from_rotvec(dtheta);
    dr_quatf_t q_new = dr_q_mult(qg, qcorr);
    return dr_q_normalize(q_new);
}

/* ----------------Networking------------------------- */
static int tcp_listen( int port )
{
    int s = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if ( s < 0 ) {
        return -errno;
    }
    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in a;
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons((uint16_t)port);
    if ( bind(s, (struct sockaddr*)&a, sizeof(a)) < 0 ) {
        int err = -errno;
        close(s);
        return err;
    }
    if ( listen(s, 1) < 0 ) {
        int err = -errno;
        close(s);
        return err;
    }
    return s; // listen socket fd
}

static void parse_args(int argc, char* argv[], cfg_t* cfg)
{
    int i;
    cfg->port = 9009;
    cfg->rate_hz = 100;
    cfg->calib_s = 5;
    cfg->avg = 5;
    cfg->accel_gain = 0.02f;
    for(i = 1; i < argc; i++) {
        if ( !strcmp(argv[i], "--port") && (i + 1) < argc ) {
            cfg->port = atoi(argv[++i]);
        } else if ( !strcmp(argv[i], "--rate_hz") && (i + 1) < argc ) {
            cfg->rate_hz = (unsigned)atoi(argv[++i]);
        } else if ( !strcmp(argv[i], "--calib_s") && (i + 1) < argc ) {
            cfg->calib_s = (unsigned)atoi(argv[++i]);
        } else if ( !strcmp(argv[i], "--avg") && (i + 1) < argc ) {
            cfg->avg = (unsigned)atoi(argv[++i]);
        } else if ( !strcmp(argv[i], "--accel_gain") && (i + 1) < argc ) {
            cfg->accel_gain = strtof(argv[++i], NULL);
        }
    }
}

int main(int argc, char* argv[])
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

    fprintf(stderr, "MPU6050 opened. FS: Accel=%u g, Gyro=%u dps | rate = %d Hz calib = %d s accel_gain=%.3f\n",
            (unsigned)((fs.accel==ACCEL_2G)?2:(fs.accel==ACCEL_4G)?4:(fs.accel==ACCEL_8G)?8:16),
            (unsigned)((fs.gyro==GYRO_250DPS)?250:(fs.gyro==GYRO_500DPS)?500:(fs.gyro==GYRO_1000DPS)?1000:2000),
            c.rate_hz, c.calib_s, c.accel_gain);

    // Calibration (stationary)
    fprintf(stderr, "Calibrating for %u s. Keep device still...\n", c.calib_s);
    double sumg[3] = {0}, suma[3] = {0};
    int N = c.calib_s * (int)c.rate_hz;
    struct mpu6050_sample sample;
    int i;
    for(i = 0; i < N; i++) {
        if ( read_sample(fd, &sample) != 0 ) {
            fprintf(stderr, "Failed to read sample during calibration\n");
            close(fd);
            return 1;
        }
        float g[3], a[3];
        raw_to_si(&fs, &sample, g, a);
        sumg[0] += g[0]; sumg[1] += g[1]; sumg[2] += g[2];
        suma[0] += a[0]; suma[1] += a[1]; suma[2] += a[2];
        msleep(1000 / c.rate_hz);
    }
    float bias_g[3] = { (float)(sumg[0]/N), (float)(sumg[1]/N), (float)(sumg[2]/N) };
    float bias_a[3] = { (float)(suma[0]/N), (float)(suma[1]/N), (float)(suma[2]/N) };
    fprintf(stderr, "Calibration complete. Gyro bias (rad/s): [%.5f %.5f %.5f], Accel bias (m/s^2): [%.5f %.5f %.5f]\n",
            bias_g[0], bias_g[1], bias_g[2], bias_a[0], bias_a[1], bias_a[2]);

    // Moving average and attitude state.
    ma_t ma;
    ma_init(&ma, (int)c.avg);
    dr_quatf_t q = dr_quat_identity();
    int server = tcp_listen(c.port);
    if ( server < 0 ) {
        fprintf(stderr, "Failed to start TCP server: %s\n", strerror(-server));
        return 1;
    }
    fprintf(stderr, "Listening on port %d\n", c.port);
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(server, (struct sockaddr*)&client_addr, &client_len);
        if ( client < 0 ) {
            fprintf(stderr, "Failed to accept client: %s\n", strerror(errno));
            if(errno == EINTR)
                continue;
        }
        fprintf(stderr, "Client connected: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
        
        // Stream loop
        int64_t t_prev = mono_ns();
        while (1) {
            if( read_sample(fd, &sample) != 0 ) {
                fprintf(stderr, "Failed to read sample\n");
                break;
            }
            float g_raw[3], a_raw[3];
            raw_to_si(&fs, &sample, g_raw, a_raw);
            // Remove bias
            float g0[3] = { g_raw[0] - bias_g[0], g_raw[1] - bias_g[1], g_raw[2] - bias_g[2] };
            float a0[3] = { a_raw[0] - bias_a[0], a_raw[1] - bias_a[1], a_raw[2] - bias_a[2] };

            // Moving average
            ma_push(&ma, g0, a0);
            float g[3], a[3];
            ma_get(&ma, g, a);

            int64_t t_now = mono_ns();
            float dt = (t_now - t_prev) * 1e-9f;
            if ( dt < 1e-5f ) dt = 1/c.rate_hz;
            t_prev = t_now;

            q = attitude_update(q, g0, a0, dt, c.accel_gain);
            float yaw_deg, pitch_deg, roll_deg;
            euler_deg_from_q(q, &yaw_deg, &pitch_deg, &roll_deg);
            // Send data to client
            char line[512];
            memset(line, 0, sizeof(line));
            int n = snprintf(line, sizeof(line),
                             "{\"t_ns\":%lld,\"q\":[%.7f,%.7f,%.7f,%.7f],\"euler_deg\":[%.2f,%.2f,%.2f]}\n",
                             (long long)t_now, q.w, q.x, q.y, q.z,
                             yaw_deg, pitch_deg, roll_deg);
            printf("%s", line);
            if ( send( client, line, (size_t)n, 0) < 0 ) {
                fprintf(stderr, "Client disconnected: %s\n", strerror(errno));
                break;
            }
            // Sleep to maintain rate
            msleep(1000 / c.rate_hz);
        }
        close(client);
    }
    ma_free(&ma);
    close(fd);
    close(server);
    return 0;
}