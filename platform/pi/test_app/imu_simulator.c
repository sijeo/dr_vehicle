#define _POSIX_C_SOURCE 200809L
#include "imu_core.h"
#include "imu_wire.h"

#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif 

static volatile sig_atomic_t g_stop = 0;
static void stop_handler(int sig) { (void)sig; g_stop = 1; }

static uint64_t mono_ns( void ){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static bool inv3( const float A[3][3], float B[3][3]) {
    float det = A[0][0]*(A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1]*(A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2]*(A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (fabsf(det) < 1e-12f) return false;
    float d = 1.0f/det;
    B[0][0] = (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * d;
    B[0][1] = (A[0][2]*A[2][1] - A[0][1]*A[2][2]) * d;
    B[0][2] = (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * d;
    B[1][0] = (A[1][2]*A[2][0] - A[1][0]*A[2][2]) * d;
    B[1][1] = (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * d;
    B[1][2] = (A[0][2]*A[1][0] - A[0][0]*A[1][2]) * d;
    B[2][0] = (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * d;
    B[2][1] = (A[0][1]*A[2][0] - A[0][0]*A[2][1]) * d;
    B[2][2] = (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * d;
    return true;
}

static float guassian(void) {
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
    return sqrtf(-2.0f*logf(u1)) * cosf(2.0f*(float)M_PI*u2);
}

static void accel_to_raw(const imu_config_t *cfg, const float target_sensor[3], int32_t raw[3])
{
    float inv[3][3];
    if(!inv3(cfg->accel_C, inv)) {
        raw[0]=raw[1]=raw[2] = 0;
        return;
    }
    float y[3] = {target_sensor[0] - cfg->accel_O[0], target_sensor[1]-cfg->accel_O[1], target_sensor[2]-cfg->accel_O[2]};
    int r;
    for( r = 0; r< 3; r++) {
        float v = inv[r][0] * y[0] + inv[r][1]*y[1] + inv[r][2]*y[2];
        raw[r] = (int32_t)lroundf(v);
    }
}

int main(int argc, char **argv ) {
    const char *host = "127.0.0.1";
    int port = 5005;
    bool fast_cal = false;

    static const struct option opts[] = {
        {"host", required_argument, NULL, 'h'},
        {"port", required_argument, NULL, 'p'},
        {"fast-cal", no_argument, NULL, 'f'},
        {NULL, 0, NULL, 0}
    };

    for(;;){
        int o = getopt_long(argc, argv, "h:p:f", opts, NULL);
        if( o == -1) break;
        if( o == 'h') host = optarg;
        else if( o == 'p') port = atoi(optarg);
    }

    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);
    srand((unsigned)time(NULL));

    imu_config_t cfg;
    imu_config_set_dr_defaults(&cfg);
    if(fast_cal) { cfg.boot_settle_s = 0.5f; cfg.boot_collect_s = 3.0f;}
    imu_pipeline_t pipe;
    int rc = imu_pipeline_init( &pipe, &cfg, "/tmp/imu_sim_cal.bin", true);
    if( rc != 0 ) {fprintf(stderr, "pipeline init failed %d\n", rc); return 1; }

    int sock = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if( sock < 0 ) { perror("socket"); return 1; }
    struct sockaddr_in peer = {0};
    peer.sin_family = AF_INET;
    peer.sin_port = htons((uint16_t)port);
    if( inet_pton(AF_INET, host, &peer.sin_addr ) != 1) {fprintf(stderr, "bad host\n"); return 2; }

    /* Simulated zero-rate offset in raw ADC counts. These were originally
     * captured at ±500 dps (65.536 LSB/dps); the ISM330 driver default has
     * since been raised to ±2000 dps (16.384 LSB/dps → ¼ the counts for the
     * same physical bias in °/s), so divide by 4 to keep the simulated
     * physical bias (~0.59, 0.88, -0.25 °/s) unchanged. Update if you
     * change gyro_lsb_per_dps in imu_config_set_dr_defaults(). */
    const float gyro_bias_counts[3] = { -9.685f, 14.410f, -4.062f };
    const long period_ns = 10000000L;
    struct timespec sleep_ts = {.tv_sec = 0, .tv_nsec=period_ns };
    uint64_t start = mono_ns();

    while(!g_stop ){
        float elapsed = (float)((double)(mono_ns() - start)* 1.0e-9);
        bool calibrated = pipe.cal_state == IMU_CAL_VALID;
        float vehicle_acc[3] = {0.0f, 0.0f, cfg.gravity_mps2};
        float gyro_vehicle[3] = {0.0f, 0.0f, 0.0f};
        if( calibrated ){
            vehicle_acc[0] = 0.8f * sinf(2.0f * (float)M_PI * 0.35f * elapsed);
            vehicle_acc[1] = 0.25f * sinf(2.0f * (float)M_PI * 0.15f * elapsed);
            gyro_vehicle[2] = 8.0f * IMU_DEG2RAD * sinf(2.0f * (float)M_PI * 0.10f * elapsed);
            if (fmodf(elapsed, 12.0f) < 0.15f) vehicle_acc[2] += 7.0f;
        }

        /* R is diag(1, -1, -1), so inverse equals itself. */
        float sensor_acc[3] = { vehicle_acc[0], -vehicle_acc[1], -vehicle_acc[2]};
        int32_t ar[3];
        accel_to_raw(&cfg, sensor_acc, ar);
        imu_raw_sample_t raw = {
            .ax = ar[0] + (int32_t)lroundf(guassian() * 2.0f),
            .ay = ar[1] + (int32_t)lroundf(guassian() * 2.0f),
            .az = ar[2] + (int32_t)lroundf(guassian() * 2.0f),
            .gx = (int32_t)lroundf(gyro_bias_counts[0] + gyro_vehicle[0]/IMU_DEG2RAD*cfg.gyro_lsb_per_dps + guassian()*0.8f),
            .gy = (int32_t)lroundf(gyro_bias_counts[1] - gyro_vehicle[1]/IMU_DEG2RAD*cfg.gyro_lsb_per_dps + guassian()*0.8f),
            .gz = (int32_t)lroundf(gyro_bias_counts[2] - gyro_vehicle[2]/IMU_DEG2RAD*cfg.gyro_lsb_per_dps + guassian()*0.8f),
        };
        imu_output_t out;
        rc = imu_pipeline_process(&pipe, &raw, mono_ns(), &out);
        if( rc == 0 ) {
            uint8_t frame[IMU_WIRE_FRAME_SIZE]; size_t n = 0;
            if (imu_wire_encode(&out, &pipe.cal, frame, sizeof(frame), &n) == 0)
                (void)sendto(sock, frame, n, 0, (struct sockaddr*)&peer, sizeof(peer));
        }
        nanosleep(&sleep_ts, NULL);

    }
    close(sock);
    return 0;
}