#ifndef IMU_MOUNT_H
#define IMU_MOUNT_H

/*
 * Sensor -> vehicle (FLU) mount rotation: presets, persistence, live apply,
 * calibration from captures, and the UDP control commands shared by
 * imu_streamer and the DR application.
 *
 * The mount file holds the full 3x3 matrix:
 *     imu_mount_R_bv_v1
 *     r00 r01 r02
 *     r10 r11 r12
 *     r20 r21 r22
 * imu_rotation_dashboard.py (bench wizard) and the DR app's in-vehicle
 * procedure both write it; both programs load it at start.
 */

#include "imu_core.h"
#include "imu_fusion.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_MOUNT_FILE_TAG "imu_mount_R_bv_v1"

/* R = Rz(yaw) Ry(pitch) Rx(roll) * preset; preset = identity (FLU) or FRD. */
void imu_mount_build(bool frd, float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]);
bool imu_mount_rot_valid(float R[3][3]);
int  imu_mount_load(const char *path, float R[3][3]);       /* 0, -ENOENT, -EINVAL */
int  imu_mount_save(const char *path, float R[3][3]);
int  imu_mount_format(char *buf, size_t n, const char *head, float R[3][3]);

/* Replace the live mount in the pipeline + fusion, keep the boot gyro
 * calibration valid (re-stamp its config CRC and re-save), write the
 * mount file. */
void imu_mount_apply(imu_pipeline_t *pl, imu_fusion_t *fu, float R[3][3], const char *mount_path);

/* Calibration from captures, all in the CURRENT vehicle frame:
 *   f1      mean specific force while still (required)
 *   f2      mean specific force still after a 180 deg turn on the same spot,
 *           or NULL (single position: surface slope stays in roll/pitch)
 *   fwd     forward evidence, captured on the same spot and heading as the
 *           last still capture: if fwd_is_accel, the mean specific force
 *           while pulling away straight ahead; otherwise the mean specific
 *           force while the front end is tipped DOWN
 * Output D: correction, R_bv_new = D * R_bv_old. Returns 0 or a negative
 * error with a reason in why. */
typedef struct {
    float euler_deg[3];      /* roll, pitch, yaw of the correction */
    float slope_deg;         /* surface slope (NaN without f2) */
    float g_mps2;            /* |f1| */
} imu_mount_cal_info_t;

int imu_mount_solve(const float f1[3], const float *f2, const float fwd[3], bool fwd_is_accel,
                    float D[3][3], imu_mount_cal_info_t *info, char *why, size_t why_len);

/* Handle one "IMUCTL ..." command (without the prefix). Writes the reply
 * ("IMUACK ...") into reply. sflp_reset may be NULL. */
typedef int (*imu_mount_sflp_reset_fn)(void *ctx);
void imu_mount_handle_ctl(const char *cmd, imu_pipeline_t *pl, imu_fusion_t *fu, float preset[3][3],
                          const char *mount_path, imu_mount_sflp_reset_fn sflp_reset, void *sflp_ctx,
                          char *reply, size_t reply_len);

#ifdef __cplusplus
}
#endif
#endif
