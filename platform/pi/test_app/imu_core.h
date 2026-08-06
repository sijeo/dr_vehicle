#ifndef IMU_CORE_H
#define IMU_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_MEDIAN_LEN      5u
#define IMU_STATS_MAX_SAMPLES   512u
#define IMU_CAL_PATH_MAX        256u

#define IMU_GRAVITY_MPS2        9.80665f
#define IMU_DEG2RAD             0.01745329251994329577f

typedef enum {
    IMU_CAL_UNINITIALIZED = 0,
    IMU_CAL_LOADING       = 1,
    IMU_CAL_SETTLING      = 2, 
    IMU_CAL_COLLECTING    = 3,
    IMU_CAL_RESTARTING    = 4,
    IMU_CAL_VALID         = 5,
    IMU_CAL_FAILED        = 6
} imu_cal_state_t;

enum {
    IMU_QF_CAL_VALID        = 1U << 0,
    IMU_QF_STATIONARY       = 1U << 1,
    IMU_QF_ACCEL_NORM_OK    = 1U << 2,
    IMU_QF_GYRO_STILL       = 1U << 3,
    IMU_QF_BUMP             = 1U << 4,
    IMU_QF_RAW_SATURATION   = 1U << 5,
    IMU_QF_SAMPLE_GAP       = 1U << 6,
    IMU_QF_ACCEL_SCALE_BAD  = 1U << 7,
    IMU_QF_VARIANCE_BAD     = 1U << 8,
    IMU_QF_RATE_BAD         = 1U << 9,
    IMU_QF_CONFIG_MISMATCH  = 1U << 10,
    IMU_QF_FILTER_WARMUP    = 1U << 11,
    IMU_QF_QUALITY_GOOD     = 1U << 12,
    IMU_QF_QUALITY_MARGINAL = 1U << 13,
    IMU_QF_QUALITY_BAD      = 1U << 14,
    IMU_QF_CAL_SAVE_FAILED  = 1U << 15
};

typedef struct {
    int32_t ax;
    int32_t ay;
    int32_t az;
    int32_t gx;
    int32_t gy;
    int32_t gz;
} imu_raw_sample_t;

typedef struct {
    float sample_rate_hz;
    float lpf_cutoff_hz;
    float stats_window_s;

    float gyro_lsb_per_dps;
    float gravity_mps2;

    float boot_settle_s;
    float boot_collect_s;
    float boot_ema_alpha;
    float boot_gyro_ac_threshold_rps;
    float boot_acc_norm_ac_threshold_msp2;
    float boot_gyro_gross_threshold_rps;
    float boot_acc_norm_gross_threshold_mps2;

    float cal_gyro_bias_abs_max_rps;
    float cal_acc_norm_min_mps2;
    float cal_acc_norm_max_mps2;
    float cal_acc_axis_var_max;
    float cal_gyro_axis_var_max;

    float stationary_acc_norm_tol_mps2;
    float stationary_gyro_axis_max_rps;
    float bump_vertical_threshold_mps2;
    float raw_saturation_fraction;

    /*
     Exact six position LSQ model extracted from the DR Application;
     accel_sensor_mps2 = accel_C * raw_counts + accel_O.
    */
   float accel_C[3][3];
   float accel_O[3];

   /* Sensor IMU frame -> vehicle FLU Frame*/
   float mount_R_bv[3][3];
    
} imu_config_t;

typedef struct {
    float gyro_bias_counts[3];
    float gyro_scale[3];

    uint32_t created_unix_s;
    uint32_t boot_sample_count;
    uint32_t config_crc32;

    float boot_accel_mean[3];
    float boot_accel_var[3];
    float boot_gyro_mean_rps[3];
    float boot_gyro_var[3];
    float boot_acc_norm_mean;
    float boot_acc_norm_std;
    float observed_rate_hz;

    uint32_t valid;
} imu_calibration_t;

typedef struct {
    uint64_t monotonic_ns;
    uint32_t sequence;
    imu_raw_sample_t raw;

    float accel_sensor_mps2[3];     /* LSQ output before mount rotation */
    float gyro_sensor_raw_rps[3];   /* raw counts converted, before bias */
    float accel_vehicle_unfiltered[3];
    float gyro_vehicle_unfiltered_rps[3];
    float accel_vehicle_filtered[3];
    float gyro_vehicle_filtered_rps[3];

    float accel_mean[3];
    float accel_std[3];
    float gyro_mean_rps[3];
    float gyro_std_rps[3];

    float accel_norm_unfiltered;
    float accel_norm_filtered;
    float gyro_norm_unfiltered_rps;
    float gyro_norm_filtered_rps;
    float calibration_progress;
    float observed_sample_rate_hz;
    float bump_duty_percent;
    float lpf_cutoff_hz;
    float quality_score;
    float stationary_score;

    uint32_t quality_flags;
    imu_cal_state_t cal_state;
    uint32_t boot_sample_count;
} imu_output_t;

typedef struct {
    bool ema_initialized;
    uint32_t settle_count;
    uint32_t stable_count;
    uint64_t stable_start_ns;

    float gyro_ema_rps[3];
    float acc_norm_ema;

    double gyro_sum_counts[3];
    double gyro_sum_sq_counts[3];
    double accel_sum[3];
    double accel_sum_sq[3];
    double acc_norm_sum;
    double acc_norm_sum_sq;
    uint32_t n;
} imu_boot_accum_t;

typedef struct {
    imu_config_t cfg;
    imu_calibration_t cal;
    imu_cal_state_t cal_state;
    uint32_t quality_flags_latched;

    uint32_t sequence;
    uint64_t last_sample_ns;
    double rate_ema_hz;

    float median_ring[6][IMU_MEDIAN_LEN];
    uint32_t median_index;
    uint32_t median_count;

    float lpf[6];
    bool lpf_initialized;

    float stats_ring[6][IMU_STATS_MAX_SAMPLES];
    double stats_sum[6];
    double stats_sum_sq[6];
    uint32_t stats_index;
    uint32_t stats_count;
    uint32_t stats_capacity;

    uint32_t bump_ring[IMU_STATS_MAX_SAMPLES];
    uint32_t bump_sum;

    imu_boot_accum_t boot;
    char calibration_path[IMU_CAL_PATH_MAX];
} imu_pipeline_t;

void imu_config_set_dr_defaults( imu_config_t *cfg);
int imu_config_validate(const imu_config_t *cfg);
uint32_t imu_config_crc32( const imu_config_t *cfg);

int imu_pipeline_init(imu_pipeline_t *p,
                    const imu_config_t *cfg,
                    const char *calibration_path,
                    bool force_recalibration);

int imu_pipeline_process( imu_pipeline_t *p, 
                         const imu_raw_sample_t *raw,
                        uint64_t monotonic_ns,
                        imu_output_t *out);
int imu_pipeline_save_calibration(imu_pipeline_t *p);
int imu_pipeline_load_calibration(imu_pipeline_t *p);
void imu_pipeline_force_recalibration(imu_pipeline_t *p);

const char *imu_cal_state_name(imu_cal_state_t state);

#ifdef __cplusplus
}
#endif
#endif
