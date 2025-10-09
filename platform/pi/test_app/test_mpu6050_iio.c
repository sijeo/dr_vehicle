// Simple IIO Userspace poller for MPU6050 6-axis IMU
// Reads accel, gyro and temperature  every 100ms and prints to stdout

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <dirent.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <limits.h>
#include <fcntl.h>
#include <time.h>

static volatile int g_stop = 0;
static void on_sigint(int signum) {
    (void)signum;
    g_stop = 1;
}

static int read_string( const char* path, char *buf, size_t sz ){
    FILE *f = fopen(path, "r");
    if( !f ) return -errno;
    if( !fgets(buf, sz, f) ) {
        int err = ferror(f) ? -errno : -EIO;
        fclose(f);
        return err;
    }
    fclose(f);
    //trim newline
    size_t n = strlen(buf);
    if ( n && (buf[n-1] == '\n') || (buf[n-1] == '\r') )
        buf[n-1] = '\0';
    return 0;
}

static int read_long( const char* path, long *out ){
    FILE *f = fopen(path, "r");
    if( !f ) return -errno;
    long v;
    if( fscanf(f, "%ld", &v) != 1 ) {
        int err = ferror(f) ? -errno : -EIO;
        fclose(f);
        return err;
    }
    fclose(f);
    *out = v;
    return 0;
}

static int read_double( const char* path, double *out )
{
    FILE *f = fopen(path, "r");
    if( !f ) return -errno;
    double v;
    if( fscanf(f, "%lf", &v) != 1 ) {
        int err = ferror(f) ? -errno : -EIO;
        fclose(f);
        return err;
    }
    fclose(f);
    *out = v;
    return 0;
}

// Find iio:deviceX whose name equals "mpu6050"
static int find_mpu6050_device(char *devpath, size_t devpath_sz)
{
    const char *iio_base = "/sys/bus/iio/devices";
    DIR *d = opendir(iio_base);
    if( !d ) {
        perror("opendir");
        return -1;
    }
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if( strncmp(de->d_name, "iio:device", 10 != 0)) continue;

        char path[PATH_MAX], namebuf[256];
        snprintf(path, sizeof(path), "%s/%s/name", iio_base, de->d_name);
        if( read_string(path, namebuf, sizeof(namebuf) ) == 0 ) {
            if( strcmp(namebuf, "mpu6050") == 0 ) {
                snprintf(devpath, devpath_sz, "%s/%s", iio_base, de->d_name);
                closedir(d);
                return 0;
            }
        }

    }
    closedir(d);
    return -ENOENT;
}

static void build_path( char *dst, size_t sz, const char *devpath, const char *fname) {
    snprintf(dst, sz, "%s/%s", devpath, fname);
}

int main( void )
{
    signal(SIGINT, on_sigint);

    char devpath[PATH_MAX];
    if( find_mpu6050_device(devpath, sizeof(devpath)) != 0 ) {
        fprintf(stderr, "MPU6050 device not found, Is the Driver loaded and DT correct?\n");
        return 1;
    }

    // Paths for raw and scale/offset
    char p_ax[PATH_MAX], p_ay[PATH_MAX], p_az[PATH_MAX];
    char p_gx[PATH_MAX], p_gy[PATH_MAX], p_gz[PATH_MAX];
    char p_temp[PATH_MAX];
    char p_ascl[PATH_MAX], p_gscl[PATH_MAX], p_toff[PATH_MAX], p_tscl[PATH_MAX];

    build_path(p_ax, sizeof(p_ax), devpath, "in_accel_x_raw");
    build_path(p_ay, sizeof(p_ay), devpath, "in_accel_y_raw");
    build_path(p_az, sizeof(p_az), devpath, "in_accel_z_raw");

    build_path(p_gx, sizeof(p_gx), devpath, "in_anglvel_x_raw");
    build_path(p_gy, sizeof(p_gy), devpath, "in_anglvel_y_raw");
    build_path(p_gz, sizeof(p_gz), devpath, "in_anglvel_z_raw");

    build_path(p_temp, sizeof(p_temp), devpath, "in_temp_raw");

    // Scales are typically common per sensor type (no _x/_y/_z suffix)
    build_path(p_ascl, sizeof(p_ascl), devpath, "in_accel_scale");
    build_path(p_gscl, sizeof(p_gscl), devpath, "in_anglvel_scale");
    build_path(p_toff, sizeof(p_toff), devpath, "in_temp_offset");
    build_path(p_tscl, sizeof(p_tscl), devpath, "in_temp_scale");

    // Read scales and offset once (they rarely change unless you reconfigure the sensor)
    double a_scale = 0.0, g_scale = 0.0, t_offset = 0.0, t_scale = 0.0;
    if( read_double(p_ascl, &a_scale) != 0 ||
        read_double(p_gscl, &g_scale) != 0 ||
        read_double(p_toff, &t_offset) != 0 ||
        read_double(p_tscl, &t_scale) != 0 ) 
    {
        fprintf(stderr, "Failed to read scale/offset values\n");
        return 1;
    }

    printf("Using Device: %s\n", devpath);
    printf(" Scales: Accel=%g g/LSB, Gyro=%g dps/LSB, Temp=%g degC/LSB + %g degC\n",
           a_scale, g_scale, t_scale, t_offset);
    printf(" Press Ctrl-C to stop\n");

    while( !g_stop ) {
        long ax=0, ay=0, az=0;
        long gx=0, gy=0, gz=0;
        long temp=0;

        if( read_long(p_ax, &ax) != 0 ||
            read_long(p_ay, &ay) != 0 ||
            read_long(p_az, &az) != 0 ||
            read_long(p_gx, &gx) != 0 ||
            read_long(p_gy, &gy) != 0 ||
            read_long(p_gz, &gz) != 0 ||
            read_long(p_temp, &temp) != 0 )
        {
            fprintf(stderr, "Failed to read sensor values\n");
            return 1;
        }

        // Convert to physical units
        double ax_g = ax * a_scale;
        double ay_g = ay * a_scale;
        double az_g = az * a_scale;
        double gx_dps = gx * g_scale;
        double gy_dps = gy * g_scale;
        double gz_dps = gz * g_scale;
        double temp_c = temp * t_scale + t_offset;

        // Timestamp (host time ) for reference
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        double tsec = ts.tv_sec + ts.tv_nsec / 1e9;

        printf("[%.3f] ACC (m/s^2): X=%+8.4f, Y=%+8.4f, Z=%+8.4f | GYRO (rad/s): X=%+8.4f, Y=%+8.4f, Z=%+8.4f | TEMP (degC): %+8.4f\n",
                tsec, ax_g*9.80665, ay_g*9.80665, az_g*9.80665,
                gx_dps*3.141592653589793/180.0, gy_dps*3.141592653589793/180.0, gz_dps*3.141592653589793/180.0,
                temp_c);
        fflush(stdout);
        // Sleep 100ms
        usleep(100000); // 100ms
    }
    printf("Exiting...\n");
    return 0;
}