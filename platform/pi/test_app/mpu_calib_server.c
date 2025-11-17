/**
 * mpu_calib_server.c
 * 
 * TCP Server running on the Raspberry Pi ( or other Linux platform ) to talk to
 * mpu6050_char.ko driver and provide data for calibration and streaming.
 * 
 * Protocol (Line-based ASCII):
 * 1) SAMPLE N\n
 *   - Pi takes N samples from /dev/mpu6050_char 
 *   - Averages them
 *   - Send one line:
 *     - AX AY AZ GX GY GZ\n  (raw values, space-separated)
 *     where values are floats in raw counts (averaged)
 * 
 * 2) STREAM 1\n
 *  - Pi enters streaming mode and sends one line per sample:
 *   - AX AY AZ GX GY GZ\n
 *   continuously at ~50Hz until client disconnects.
 * 
 * NOTE: In this simple implementation, the only way to stop streaming is to close the TCP 
 * connection from the client side.
 * 
 * The Python GUI above uses only SAMPLE N and polls SAMPLE 1 for streaming. STREAM 1 is provided
 * as an extra feature. (e.g. for a custom client).
 * 
 * Run: sudo ./mpu_calib_server <port>
 * 
 * Dependencies:
 * - mpu6050_char.ko loaded and /dev/mpu6050_char accessible
 * - The driver must return a struct:
 * int 16_t ax, ay, az, gx, gy, gz; // raw values
 * per read() call.
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define IMU_DEV_PATH "/dev/mpu6050-0"

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} imu_sample_t;


/**
 * OPEN IMU DEVICE
 * 
 */
static int open_imu_device() {
    int fd = open(IMU_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open IMU device");
        exit(1);
    }
    return fd;
}

/**
 * READ IMU SAMPLE (blocking read of struct )
 * 
 */
static int read_imu_sample(int imu_fd, imu_sample_t *sample) {
    ssize_t ret = read(imu_fd, sample, sizeof(imu_sample_t));
    if ( ret < 0 )
    {
        perror("Read IMU");
        return -1;
    }
    if ( ret != sizeof(imu_sample_t) )
    {
        fprintf(stderr, "Incomplete read from IMU device\n");
        return -1;
    }
    return 0;
}

/**
 * Capture N samples, returns averages in double
 */
static int capture_average(int imu_fd, int N, double *ax, double *ay, double *az, double *gx, double *gy, double *gz)
{
    int i;
    long sum_ax = 0, sum_ay = 0, sum_az = 0;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (i = 0; i < N; i++) {
        imu_sample_t sample;
        if ( read_imu_sample(imu_fd, &sample) != 0 ) {
            return -1;
        }
        sum_ax += sample.ax;
        sum_ay += sample.ay;
        sum_az += sample.az;
        sum_gx += sample.gx;
        sum_gy += sample.gy;
        sum_gz += sample.gz;

        /* Sleep 10ms between samples ( adjust to  your ODR if needed )*/
        usleep(10000);
    }

    *ax = sum_ax / (double)N;
    *ay = sum_ay / (double)N;
    *az = sum_az / (double)N;
    *gx = sum_gx / (double)N;
    *gy = sum_gy / (double)N;
    *gz = sum_gz / (double)N;
    return 0;
}

/**
 * Stream Mode: Send continuous raw samples until client disconnects.
 * 
 * NOTE:
 * We read one sample per iteration and immediately send it.
 * To stop stream, the client must close the connection. (or you can kill this server )
 */

 static void stream_continuous( int client_fd, int imu_fd )
 {
    char out[256];

    fprintf(stderr, "Entering the Stream Mode (Continuous Samples)...\n");
    while(1) {
        imu_sample_t sample;
        if ( read_imu_sample(imu_fd, &sample) != 0 ) {
            fprintf(stderr, "Error reading IMU sample in stream mode\n");
            break;
        }

        /* Send raw sample as ASCII line AX AY AZ GX GY GZ\n */
        int len  = snprintf(out, sizeof(out), "%d %d %d %d %d %d\n",
            sample.ax, sample.ay, sample.az,
            sample.gx, sample.gy, sample.gz);

        if ( len <= 0 ) {
            fprintf(stderr, "Error formatting output line\n");
            break;
        }

        ssize_t n = send(client_fd, out, len, 0);
        if ( n <= 0 ) {
            fprintf(stderr, "STREAM: Client disconnected or send error\n");
            break;
        }

        /* Adjust delay for desired streaming rate; 20ms -> ~50Hz */
       usleep(20000);
    }
   fprintf(stderr, "Leaving STREAM mode.\n");

}

/**
 *  Handle a single client: parse commands and respnd
 */
static void handle_client(int client_fd, int imu_fd) {
    char buffer[256];

    while(1) {
        memset(buffer, 0, sizeof(buffer));
        ssize_t n = recv(client_fd, buffer, sizeof(buffer)-1, 0);
        if ( n <= 0 ) {
            /* Client disconnected or error */
                perror("recv");
                break;
            }
            buffer[n] = '\0';
            printf("Received command: %s", buffer);

            /* Very simple line based command parsing; expect one command per recv */
            int N = 0;
            int stream_on = 0;

            if ( sscanf(buffer, "SAMPLE %d\n", &N) == 1 && N > 0 ) {
                /* Average N samples and send one line of floats */
                double ax, ay, az, gx, gy, gz;
                if ( capture_average(imu_fd, N, &ax, &ay, &az, &gx, &gy, &gz) == 0 ) {
                    char out[256];
                    int len = snprintf(out, sizeof(out), "%.3f %.3f %.3f %.3f %.3f %.3f\n",
                        ax, ay, az, gx, gy, gz);
                    if ( len > 0 ) {
                        send(client_fd, out, len, 0);
                    }
                } else {
                    const char *err_msg = "ERR IMU\n";
                    send(client_fd, err_msg, strlen(err_msg), 0);
                }
            } else if ( sscanf(buffer, "STREAM %d", &stream_on) == 1 && stream_on == 1 ) {
                /**
                 * STREAM 1:
                 * - Enter continuous streaming mode
                 * - We stay in stream_continuous() until the client disconnects
                 * - After that, we break out of handle_client() and close.
                 * 
                 * If you want to support STREAM 0 to stop without closing, you
                 * would need a more complex multiplexed I/O design (select/poll)
                 */
                stream_continuous(client_fd, imu_fd);
                break; /* Exit handle_client after streaming ends */
            } else {
                const char *err_msg = "ERR CMD\n";
                send(client_fd, err_msg, strlen(err_msg), 0);
            }
    }
        
}

/**
 * Main: Listen for incoming TCP connections and serve one client at a time.
 * 
 */

 int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <port>\n", argv[0]);
        exit(1);

    }

    int port = atoi(argv[1]);
    int imu_fd = open_imu_device();

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        exit(1);
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        close(server_fd);
        exit(1);
    }

    if (listen(server_fd, 1) < 0) {
        perror("listen");
        close(server_fd);
        exit(1);
    }

    printf("Calibration server listening on port %d\n", port);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            perror("accept");
            continue;
        }

        printf("Client connected\n");
        handle_client(client_fd, imu_fd);
        close(client_fd);
        printf("Client disconnected\n");
    }

    close(imu_fd);
    close(server_fd);
    return 0;
}





