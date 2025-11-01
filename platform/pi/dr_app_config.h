/**
 * @file dr_app_config.h
 * @brief Configuation constants for the Dead Reckoning application.
 * 
 * @details
 * This header file centralizes all tunable parameters and constants
 * used in dr_app.c so that the sensor, timing and the EKF behavior can be
 * easily modified without touching the core logic or main application code. 
 * 
 * @note All units are in SI unless otherwise specified.
 */

#ifndef DR_APP_CONFIG_H
#define DR_APP_CONFIG_H

/*-----------------------------------------------------------------------------------
*Hardware device configuration
*----------------------------------------------------------------------------------*/

/**
 * @brief IIO device name for the MPU-9250 IMU sensor.
 * 
 * The discovery function in dr_app.c searches for this device name
 * under /sys/bus/iio/devices/iio:deviceX/name.
 */
#define MPU6050_IIO_NAME "mpu6050"

/**
 * @brief Path to the NEO-6M GNSS character device .
 * 
 * This is created by the neo6m_gnss_serdev kernel driver. 
 * 
 */
#define NEO6M_DEVICE_PATH "/dev/neo6m0"

/**
 * @brief Value of PI
 */
#define M_PI 3.14159265358979323846

/* ----------------------------------------------------------------
* IMU sampling and calibration parameters.
*------------------------------------------------------------------*/
/**
 * @brief IMU Loop rate (Hz)
 * 
 * IMU data are read and processed every 1/IMU_LOOP_HZ seconds.
 * 
 */
#define IMU_LOOP_HZ 10      /** < 10 Hz => 100 ms period */

/**
 * @brief Moving average window size for IMU data.
 */
#define IMU_AVG_WINDOW      5

/**
 * @brief Number of IMU samples to collect for stationary calibration.
 */
#define CAL_SAMPLES        1000 /** < 1000 samples @ 10 Hz = 100 seconds ~10 Seconds at 100 Hz */

/**
 * @brief Standard gravity in m/s^2.
 */
#define GRAVITY_CONST      9.80665f

/* ----------------------------------------------------------------
* EKF initial covariance and process noise parameters.
*------------------------------------------------------------------*/
/**
 * @brief Initial EKF uncertainity settings
 */
#define P0_POS_STD_M      5.0f    /** < Initial position std in meters */
#define P0_VEL_STD_MPS    1.0f    /** < Initial velocity std in m/s */
#define P0_ANG_STD_RAD   (5.0f * (float)M_PI / 180.0f)  /** < Initial angle std in radians */
#define P0_BA_STD_MPS2   0.5f
#define P0_BG_STD_RADPS  0.01f

/**
 * @brief Continuous noise standard deviations for IMU process model
 */
#define SIGMA_G_RADPS   0.01f  /** < Gyro noise density (rad/s) */
#define SIGMA_A_MPS2    0.5f   /**< accel noise density (m/s^2) */
#define SIGMA_BG_RADPS  0.005f /**< gyro bias random walk (rad/sqrt(s)) */
#define SIGMA_BA_MPS2   0.01f  /**< accel bias random walk (m/s^2/sqrt(s)) */

/* ----------------------------------------------------------------
* GPS Measurement Noise
*------------------------------------------------------------------*/
/**
 * @brief Default position variance (m^2) for GPS update.
 * (e.g., 5 meters std => 25 m^2 variance)
 */
#define GPS_POS_VAR_M2   25.0f

/*-----------------------------------------------------------------
* Output formatting 
*-------------------------------------------------------------------*/
/**
 * @brief CSV header printed at start up.
 */
#define DR_CSV_HEADER "utc,mono_ns,lat_deg,lon_deg,h_m,x_e,y_n,z_u,v_e,v_n,v_u,yaw_deg,pitch_deg,roll_deg\n"

/**
 * @brief Format string for one CSV line of EKF output.
 */
#define DR_CSV_FORMAT "%s,%lld,%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n"

/*---------------------------------------------------------------------
* Miscellaneous
*---------------------------------------------------------------------*/

/**
 * @brief Default sleep resolution for main loop (ms)
 */
#define MAIN_LOOP_DELAY_MS   ((int)(1000.0f/IMU_LOOP_HZ + 0.5f))
    


#endif // DR_APP_CONFIG_H  
/* End of dr_app_config.h */