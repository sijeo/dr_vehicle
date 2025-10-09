// Buffered IIO reader for MPU6050 6-axis IMU
// Reads from /dev/iio:deviceX after enabling the scan elements + buffer
// Prints accel(m/s^2), gyro(rad/s), temp(degC) with timestamp (ns)

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
#include <sys/stat.h>
#include <sys/types.h>
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

static int write_str( const char *path, const char *s)
{
    int fd = open(path, O_WRONLY);
    if( fd < 0 ) return -errno;
    ssize_t nw = write(fd, s, strlen(s));
    if( nw < 0 ) {
        int err = -errno;
        close(fd);
        return err;
    }
}

static int exists( const char *path )
{
    struct stat st;
    return (stat(path, &st) == 0);
}

// Find iio:deviceX with name "mpu6050"; return base sysfs dir and chardev path.
static int find_mpu6050( char *sysfs_dev, size_t ssz, char *chardev, size_t csz )
{
    const char *base = "/sys/bus/iio/devices";
    DIR *d = opendir(base);
    if( !d ) {
        perror("opendir");
        return -errno;
    }
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if( strncmp(de->d_name, "iio:device", 10) != 0) continue;
        char path[PATH_MAX];
        char namebuf[128];
        char devnum[32];
        snprintf(path, sizeof(path), "%s/%s/name", base, de->d_name);
        if( read_string(path, namebuf, sizeof(namebuf) ) == 0 ) {
            if( strcmp(namebuf, "mpu6050") == 0 ) {
                snprintf(sysfs_dev, ssz, "%s/%s", base, de->d_name);
                // Get char device number
                snprintf(path, sizeof(path), "%s/%s/dev", base, de->d_name);
                if( read_string(path, devnum, sizeof(devnum)) != 0 ) {
                    closedir(d);
                    return -ENOENT;
                }
                snprintf(chardev, csz, "/dev/%s", de->d_name);
                closedir(d);
                return 0;
            }
        }
    }
    closedir(d);
    return -ENOENT;
}

#pragma pack(push, 1)
struct mpu6050_sample {
    int16_t ax, ay, az;   // Accel raw
    int16_t temp;         // Temp raw
    int16_t gx, gy, gz;   // Gyro raw
    int64_t ts;           // Timestamp ns
};
#pragma pack(pop)

static void build( char *out, size_t sz, const char *base, const char *rel) {
    snprintf(out, sz, "%s/%s", base, rel);
}

int main( void )
{
    signal(SIGINT, on_sigint);

    char sysfs[PATH_MAX], chardev[PATH_MAX];
    if( find_mpu6050(sysfs, sizeof(sysfs), chardev, sizeof(chardev)) != 0 ) {
        fprintf(stderr, "MPU6050 device not found, Is the Driver loaded and DT correct?\n");
        return 1;
    }

    printf("Using Device: %s (sysfs:%s)\n", chardev, sysfs);

    //Path to scan elements
    char scan_dir[PATH_MAX];
    build(scan_dir, sizeof(scan_dir), sysfs, "scan_elements");

    // Enable the channels we expect (indices/order from mpu6050_iio_driver.c)
    char p_en[8][PATH_MAX];
    build(p_en[0], sizeof(p_en[0]), scan_dir, "in_accel_x_en");
    build(p_en[1], sizeof(p_en[1]), scan_dir, "in_accel_y_en");
    build(p_en[2], sizeof(p_en[2]), scan_dir, "in_accel_z_en");
    build(p_en[3], sizeof(p_en[3]), scan_dir, "in_temp_en");
    build(p_en[4], sizeof(p_en[4]), scan_dir, "in_anglvel_x_en");
    build(p_en[5], sizeof(p_en[5]), scan_dir, "in_anglvel_y_en");
    build(p_en[6], sizeof(p_en[6]), scan_dir, "in_anglvel_z_en");
    build(p_en[7], sizeof(p_en[7]), scan_dir, "timestamp_en");

    for( int i=0; i<8; i++) {
        if( !exists(p_en[i]) ) {
            fprintf(stderr, "Expected scan element not found: %s\n", p_en[i]);
            return 1;
        }
        if( write_str(p_en[i], "1") != 0 ) {
            perror(p_en[i]);
            return 1;
        }
    }

    // Read scales/offest (for printing in physical units)
    char p_ascl[PATH_MAX], p_gscl[PATH_MAX], p_toff[PATH_MAX], p_tscl[PATH_MAX];
    build(p_ascl, sizeof(p_ascl), sysfs, "in_accel_scale");
    build(p_gscl, sizeof(p_gscl), sysfs, "in_anglvel_scale");
    build(p_toff, sizeof(p_toff), sysfs, "in_temp_offset");
    build(p_tscl, sizeof(p_tscl), sysfs, "in_temp_scale");
    double a_scale = 0.0, g_scale = 0.0, t_offset = 0.0, t_scale = 0.0;
    if( read_double(p_ascl, &a_scale) != 0 ||
        read_double(p_gscl, &g_scale) != 0 ||
        read_double(p_toff, &t_offset) != 0 ||
        read_double(p_tscl, &t_scale) != 0 ) 
    {
        fprintf(stderr, "Failed to read scale/offset values\n");
        return 1;
    }   

    // Configure buffer length (number for samples the kernel can queue)
    char p_blen[PATH_MAX], p_ben[PATH_MAX];
    build(p_blen, sizeof(p_blen), sysfs, "buffer/length");
    build(p_ben, sizeof(p_ben), sysfs, "buffer/enable");
    if( write_str(p_blen, "256") != 0 ) {
        perror(p_blen);
        // Not fatal, continue
    }
    // Enable the buffer
    if( write_str(p_ben, "1") != 0 ) {
        perror(p_ben);
        for (int i=0; i<8; i++)   // Clean up the scan enables before exit
            write_str(p_en[i], "0");
        return 1;
    }

    // Open the char device for reading packed samples
    int fd = open(chardev, O_RDONLY);
    if( fd < 0 ) {
        perror(chardev);
        write_str(p_ben, "0");
        for (int i=0; i<8; i++)
            write_str(p_en[i], "0");
        return 1;
    }

    printf(" Reading the Buffered Samples... Ctrl-C to stop\n");
    while( !g_stop ) {
        struct mpu6050_sample s;
        ssize_t nr = read(fd, &s, sizeof(s));
        if( nr < 0 ) {
            if (errno == EINTR) continue;
            perror("read");
            break;
        }
        if( nr != sizeof(s) ) {
            fprintf(stderr, "Short read %zd\n", nr);
            break;
        }

        // Convert to physical units
        double ax_g = s.ax * a_scale;
        double ay_g = s.ay * a_scale;
        double az_g = s.az * a_scale;
        double temp_c = s.temp * t_scale + t_offset;
        double gx_dps = s.gx * g_scale;
        double gy_dps = s.gy * g_scale;
        double gz_dps = s.gz * g_scale;
        double gx_rads = gx_dps * (3.141592653589793/180.0);
        double gy_rads = gy_dps * (3.141592653589793/180.0);
        double gz_rads = gz_dps * (3.141592653589793/180.0);
        printf("TS: %lld ns, Accel: [%+8.4f %+8.4f %+8.4f] g, Gyro: [%+8.4f %+8.4f %+8.4f] rad/s, Temp: %6.2f C\n",
               (long long)s.ts, ax_g, ay_g, az_g, gx_rads, gy_rads, gz_rads, temp_c);
        fflush(stdout);
    }

    printf("Exiting...\n");
    close(fd);
    write_str(p_ben, "0");
    for (int i=0; i<8; i++)
        write_str(p_en[i], "0");
    return 0;
}   


/**
 * @brief Initialize the MPU6050 hardware with default settings.
 * Resets the device, configures basic LPF, ODR, and full-scale ranges.
 * Verifies the WHO_AM_I register.
 * @param st Driver instance.
 * @see mpu6050_set_odr(), mpu6050_config_ranges()
 * @note Assumes regmap is already initialized.
 * @note Called with lock held.
 * @note Uses usleep_range() for delays.
 * @note Uses default ODR and ranges if not configured via DT.
 * @note Enables only accelerometer and gyroscope; temp is always on.
 * @note Does not enable interrupts or FIFO; done in buffer_postenable().
 * @note Puts device in active mode (not sleep).
 * @note Caller should handle errors appropriately.
 * @note This function is called during probe after parsing DT properties.
 * @note Returns -ENODEV if WHO_AM_I does not match expected value.
 * @note Returns other negative errno on I2C/regmap errors.
 * @note Returns 0 on success.
 * @see mpu6050_iio_driver.c for register definitions and constants.
 * @see mpu6050_iio_driver.c for DT properties for ODR and ranges.
 */ 


 