#ifndef DR_EKF_H
#define DR_EKF_H

#include "dr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @file dr_ekf.h
 * @brief Error-state EKF API for dead reckoning (15-state IMU+GPS filter)
 */

 /** @brief Initialize EKF config/state and set initial covariance
  * @param ekf (out) EKF handle
  * @param cfg EKF configuration (noise PSDs, gravity, initial stddevs )
  * @param x0 optional initial nominal state (NULL -> zeros & identity quaternion)
  */
 void dr_ekf_init(dr_ekf_t* ekf, const dr_ekf_config_t* cfg, const dr_nominal_state_t* x0);

 /** @brief Predict step: mechanize nominal state and propagate covariance
  * @param ekf EKF handle (in/out)
  * @param imu IMU sample in SI units (rad/s, m/s^2 )
  * @param dt timestep (s)
  */
 void dr_ekf_predict(dr_ekf_t* ekf, const dr_imu_sample_t* imu, float dt);

 /** @brief GPS position update (ENU), 3D.
  * @param ekf EKF handle (in/out)
  * @param z measured position in ENU frame (m)
  * @param Rpos 3x3 measurement covariance (row-major) (m^2)
  * @return 0 on success, -1 if innovation covariance is singular
  */
 int dr_ekf_update_gps_pos(dr_ekf_t* ekf, const dr_gps_pos_t* z, const float Rpos[9]);

#ifdef __cplusplus
}
#endif  
#endif // DR_EKF_H

