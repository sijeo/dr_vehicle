#ifndef DR_TYPES_H
#define DR_TYPES_H

#include <stdint.h>
#include <stddef.h>


#ifdef __cplusplus
extern "C" {
#endif  

#ifndef DR_INLINE
#  if defined(_MSC_VER)
#    define DR_INLINE __forceinline
#  else 
#    define DR_INLINE inline __attribute__((always_inline))
#  endif
#endif

/** @file dr_types.h
 * @brief Core project-wide types, constants, and configuration structs
 */

 #define DR_STATE_ERR_DIM       15
 #define DR_GRAVITY_F           9.80665f

 /** @brief 3D Vector float */
 typedef struct dr_vec3f {
        float x; /**< X component */
        float y; /**< Y component */
        float z; /**< Z component */
 } dr_vec3f_t;

 /** @brief Quaternion (w, x, y, z) mapping body->world */
    typedef struct dr_quatf {
            float w; /**< W component */
            float x; /**< X component */
            float y; /**< Y component */
            float z; /**< Z component */
    } dr_quatf_t;

/** @brief Nominal (nonlinear) state for DR EKF */
typedef struct  {
    float p[3];      /**< Position in world ENU frame (m) */
    float v[3];      /**< Velocity in world ENU frame (m/s) */
    dr_quatf_t q;  /**< Orientation body->world (unit norm) */
    float ba[3];     /**< Accelerometer bias in body frame (m/s^2) */
    float bg[3];     /**< Gyroscope bias in body frame (rad/s) */
} dr_nominal_state_t;

/** @brief IMU sample in SI units  */
typedef struct {
    float gyro[3]; /**< Angular rate in body frame (rad/s) */
    float accel[3]; /**< Specific force in body frame (m/s^2) */
} dr_imu_sample_t;

/** @brief GPS Position Measurement (ENU) */
typedef struct {
    float pos[3];  /**< Position in world ENU frame (m) */
} dr_gps_pos_t;

/** @brief Affine calibration (bias + per axis scale ) */
typedef struct {
    float bias[3];  /**< Bias term */
    float scale[3]; /**< Scale term */
} dr_cal3_t;

/** @brief EKF Configuration Parameters */
typedef struct {
    float gravity[3];        /**< Gravity vector in world frame (m/s^2) default {0, 0, -9,80665 }   */
    float sigma_g;         /**< Gyroscope noise density (rad/s/rtHz) */
    float sigma_a;         /**< Accelerometer noise density (m/s^2/rtHz) */
    float sigma_bg;        /**< Gyroscope bias random walk (rad/s^2/rtHz) */
    float sigma_ba;        /**< Accelerometer bias random walk (m/s^2/rtHz) */
    float p0_pos;        /**< Initial position stddev [m] */
    float p0_vel;        /**< Initial velocity stddev [m/s] */
    float p0_ang;        /**< Initial attitude stddev [rad] */
    float p0_ba;         /**< Initial accelerometer bias stddev [m/s^2] */
    float p0_bg;         /**< Initial gyro bias stddev [rad/s] */
} dr_ekf_config_t;

/** @brief EKF handle bundling nominal state, covariance and config */
typedef struct {
    dr_nominal_state_t x;          /**< Nominal state */
    float P[DR_STATE_ERR_DIM * DR_STATE_ERR_DIM];  /**< State covariance matrix (row major) */
    dr_ekf_config_t cfg;           /**< EKF configuration parameters */
} dr_ekf_t;

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // DR_TYPES_H