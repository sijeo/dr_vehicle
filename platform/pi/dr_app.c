/**
 * @file dr_app.c
 * @brief Dead-Reckoning application for ground vehicles (MPU6050 + NEO-6M + EKF)
 * 
 * @details
 * Pipeline:
 * 1. Discover MPU6050 IIO device and NEO-6M char device. 
 * 2. Calibrate IMU when stationary: Estimate gyro bias and accel bias so that a_b = [0,0,g].
 * 3. Block until first GNSS fix; use as ENU reference (lat0, lon0, alt0) and seed EKF.
 * 4. Loop at configured rate (IMU_LOOP_HZ): read IMU -> moving average -> EKF predict.
 * 5. When a fresh GNSS fix arrives: convert LLA -> ECEF -> ENU, EKF update with position.
 * 6. Emit CSV: UTC, t_mono_ns, lat, lon, h, ENU(x,y,z), vel( x, y, z), yaw, pitch, roll.\
 * 
 * Build:
 * gcc -O2 -Wall -Wextra -o dr_app dr_app.c -I./core -I./platform/pi -lm
 */

#define GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>

#include "../../core/dr_types.h"
#include "../../core/dr_ekf.h"
#include "../../core/dr_math.h"
#include "../../core/dr_cal.h"

#include "dr_app_config.h"
#include "neo6m_gnss_ioctl.h"

/*----------------Small Helpers ------------------ */

/**
 * @brief Sleep for specified milliseconds.
 */
static void msleep(unsigned ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

/**
 * @brief Read a single integer from a sysfs file. (e.g. IIO *_raw)
 * 
 * @return 0 on success and errno on failuare.
 */
static int read_int_from_file(const char *path, int *out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -errno;
    }
    int v = 0;
    int n = fscanf(f, "%d", &v);
    fclose(f);
    if (n != 1) {
        return -EINVAL;
    }
    *out = v;
    return 0;
}

/**
 * @brief Read a double from a sysfs file (e.g. IIO scale)
 */
static int read_double_from_file(const char *path, double *out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -errno;
    }
    double v = 0;
    int n = fscanf(f, "%lf", &v);
    fclose(f);
    if (n != 1) {
        return -EINVAL;
    }
    *out = v;
    return 0;
}

/**
 * @brief Find the IIO device directory for the MPU6050 (by name)
 * @param out_path Buffer to receive director path (e.g. /sys/bus/iio/devices/iio:deviceX)
 * @param cap size of out_path buffer
 * @return 0 on success and -errno on failure.
 */
static int find_mpu6050_iio(char *out_path, size_t cap)
{
    const char *base = "/sys/bus/iio/devices";
    DIR *d = opendir(base);
    if (!d) {
        return -errno;
    }
    struct dirent *e;
    int ret = -ENOENT;
    while ((e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, "iio:device", 10) == 0) {
            char p[512];
            snprintf(p, sizeof(p), "%s/%s/name", base, e->d_name);
            FILE *f = fopen(p, "r");
            if (!f) continue;
            char name[128] = {0};
            if (fgets(name, sizeof(name), f) != NULL) {
                // Remove newline
                name[strcspn(name, "\n")] = 0;
                if (strcmp(name, MPU6050_IIO_NAME) == 0) {
                    // Found it
                    snprintf(out_path, cap, "%s/%s", base, e->d_name);
                    ret = 0;
                    fclose(f);
                    break;
                }
            }
            fclose(f);
        }
    }
    closedir(d);
    return ret;
}

/**
 * @brief Read one IMU sample (raw->SI) from IIO sysfs
 * @param devpath IIO device directory
 * @param[out] accel_mps2 body specific force (m/s^2)
 * @param[out] gyro_radps body angular rate (rad/s)
 * @return 0 on success and -errno on failure.
 */
static int read_imu_once(const char *devpath, float gyro_radps[3], float accel_mps2[3])
{
    char path[512];
    int raw;
    double scale;

    /* Accelerometer X Y Z*/
    const char *acc_nodes[3] = {"in_accel_x", "in_accel_y", "in_accel_z"};
    for (int i = 0; i < 3; i++) {
        snprintf(path, sizeof(path), "%s/%s_raw", devpath, acc_nodes[i]);
        if (read_int_from_file(path, &raw))
        {
            return -EIO;
        }
        snprintf(path, sizeof(path), "%s/%s_scale", devpath, acc_nodes[i]);
        if (read_double_from_file(path, &scale))
        {
            return -EIO;
        }
        accel_mps2[i] = (float)(raw * scale);
    }

    /* Gyro X Y Z*/
    const char *gyro_nodes[3] = {"in_gyro_x", "in_gyro_y", "in_gyro_z"};
    for (int i = 0; i < 3; i++) {
        snprintf(path, sizeof(path), "%s/%s_raw", devpath, gyro_nodes[i]);
        if (read_int_from_file(path, &raw))
        {
            return -EIO;
        }
        snprintf(path, sizeof(path), "%s/%s_scale", devpath, gyro_nodes[i]);
        if (read_double_from_file(path, &scale))
        {
            return -EIO;
        }
        gyro_radps[i] = (float)(raw * scale);
    }

    return 0;

}


   