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

#include "core/dr_types.h"
#include "core/dr_ekf.h"
#include "core/dr_math.h"
#include "core/dr_cal.h"

#include "dr_app_config.h"
#include "neo6m_gnss_ioctl.h"

/*----------------Small Helpers ------------------ */
