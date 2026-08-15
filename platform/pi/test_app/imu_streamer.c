#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <fnctl.h>
#include <getopt.h>
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
#include "imu_core.h"
#include "imu_wire.h"


/* This is the existing kernel-driver ABI used by the DR application. */
#include "../mpu6050_ioctl.h"

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

static uint64_t monotonic_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void usage(const char *prog){
    fprintf(stderr, 
        "Usage: %s --host RECEIVER IP [options]\n "
    " --host IP         UDP receiver address (required)\n "
    " --port N          UDP port, default 5005\n "
    " --device PATH     IMU device, default /dev/mpu6050-0\n"
    " --cal-file PATH   calibration blob, default ./imu_calibration.bin\n"
    " --recalibrate     ignore saved calibration and run stationary boot calibration\n"
    " --rate HZ         expected IMU rate, default 100\n"
    " --cutoff HZ       LPF cutoff, default 6Hz\n",
    prog);
}

int main( int argc, char **argv ) {
    const char *host = NULL;
    const char *device = "/dev/mpu6050-0";
    const char *cal_path = "./imu_calibration.bin";
    int port = 5005;
    bool recalibrate = false;
    imu_config_t cfg;
    imu_config_set_dr_defaults(&cfg);

    static const struct option options[] = {
        {"host", required_argument, NULL, 'h'},
        {"port", required_argument, NULL, 'p'}, 
        {"device", required_argument, NULL, 'd'},
        {"cal-file", required_argument, NULL, 'c'},
        {"recalibrate", no_argument, NULL, 'r'},
        {"rate", required_argument, NULL, 'r'},
        {"cutoff", required_argument, NULL, 'l'},
        {"help", no_argument, NULL, '?'},
        {NULL, 0, NULL, 0}
    };

    for(;;) {
        int opt = getopt_long(argc, argv, "h:p:d:c:rf:l:?", options, NULL);
        if( opt == -1 ) break;
        switch(opt) {
            case 'h' : host = optarg; break;
            case 'p' : port = atoi(optarg); break;
            case 'd' : device = optarg; break;
            case 'c' : cal_path = optarg; break;
            case 'r' : recalibrate = true; break;
            case 'f' : cfg.sample_rate_hz = strtof(optarg, NULL); break;
            case 'l' : cfg.lpf_cutoff_hz = strtof(optarg, NULL); break;
            default: usage(argv[0]); return (opt == '?') : ? : 2;
        }
    }
    if( !host || port < 1 || port > 65535 ){
        usage(argv[0]);
        return 2;
    }

    signal( SIGINT, on_signal );
    signal( SIGTERM, on_signal );

    imu_pipeline_t pipeline;
    int rc = imu_pipeline_init(&pipeline, &cfg, cal_path, recalibrate );
    if( rc != 0 ) {
        fprintf(stderr, "imu_pipeline_init failed: %s (%d)\n", strerror(-rc), rc);
        return 1;
    }

    int imu_fd = open(device, O_RDONLY | O_CLOEXEC );
    if( imu_fd < 0 ){
        perror("open IMU device");
        return 1;
    }
    
    int sock = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if( sock < 0 ) {
        perror("socket");
        close(imu_fd);
        return 1;
    }
    struct sockaddr_in peer;
    memset(&peer, 0, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_port = htons((uint16_t)port);
    if(inet_pton(AF_INET, host, &peer.sin_addr) != 1){
        fprintf(stderr, "Invalid IPv4 address: %s\n", host);
        close(sock); close(imu_fd); return 2;
    }

    fprintf(stderr, "Streaming %s to %s:%d; calibration= %s; state = %s\n", 
        device, host, port, cal_path, imu_cal_state_name(pipeline.cal_state));
    if( pipeline.cal_state != IMU_CAL_VALID ) {
        fprintf(stderr, "Keep the IMU completely stationary until calibration reaches 100%%.\n");
    }

    uint8_t frame[IMU_WIRE_FRAME_SIZE];
    uint32_t last_state = UINT32_MAX;
    while(!g_stop) {
        struct mpu6050_sample s;
        ssize_t n = read(imu_fd, &s, sizeof(s));
        if( n < 0 ){
            if( errono == EINTR ) continue;
            perror("read IMU");
            break;
        }
        if( n != (ssize_t)sizeof(s)) {
            fprintf(stderr, "Short IMU read: %zd/%zu\n", n, sizeof(s));
            continue;
        }

        imu_raw_sample_t raw = {
            .ax = s.ax_corr, .ay = s.ay_corr, .az = s.az_corr, 
            .gx = s.gx_corr, .gy = s.gy_corr, .gz = s.gz_corr
        };
        imu_output_t out;
        rc = imu_pipeline_process(&pipeline, &raw, monotonic_ns(), &out);
        if( rc != 0 ){
            fprintf(stderr, "pipeline error: %s (%d)\n", strerror(-rc), rc);
            continue;
        }

        size_t frame_size = 0u;
        rc = imu_wire_encode(&out, &pipeline.cal, frame, sizeof(frame), &frame_size);
        if( rc != 0 ) {
            fprintf(stderr, "wire encode error: %s (%d)\n", strerror(-rc), rc);
            break;
        }
        ssize_t sent = sendto(sock, frame, frame_size, 0, (struct sockaddr *)&peer, sizeof(peer));
        if( sent != (ssize_t)frame_size && errno != ENOBUFS) perror("sendto");

        if( (uint32_t)out.cal_state != last_state ) {
            fprintf(stderr, "Calibration state: %s, progress %.1f%%, quality %.0f\n", 
            imu_cal_state_name(out.cal_state), 100.0f*out.calibration_progress, out.quality_score);
            last_state = (uint32_t)out.cal_state;
        }
    }

    close(sock);
    close(imu_fd);
    return 0;
}