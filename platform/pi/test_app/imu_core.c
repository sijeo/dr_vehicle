#define _POSIX_C_SOURCE_200809L
#include "imu_core.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define IMU_CAL_FILE_MAGIC  0x494D4333u /* IMC3 */
#define IMU_CAL_FILE_VERSION    1u

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif 

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t payload_size;
    uint32_t checksum;
    imu_calibration_t calibration;
} imu_cal_file_t;

static float clampf_local( float x, float lo, float hi){
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static bool finite_float( float x ){
    return isfinite(x) != 0;
}

static float vec_norm3( const float v[3]){
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void mat3_mul_vec( const float R[3][3], const float v[3], float out[3]) {
    out[0] = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
    out[1] = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
    out[2] = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
}

static float determinants3( const float R[3][3]) {
    return R[0][0] *(R[1][1]*R[2][2] - R[1][2]*R[2][1])
        - R[0][1] * (R[1][0]*R[2][2] - R[1][2]*R[2][0])
        + R[0][2] * (R[1][0]*R[2][1] - R[1][1]*R[2][0]);
}

static uint32_t crc32_bytes( const void *data, size_t nbytes ){
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    size_t i, b;
    for( i = 0; i < nbytes; ++i) {
        crc ^= p[i];
        for( b=0; b < 8; ++b) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

uint32_t imu_config_crc32(const imu_config_t *cfg) {
    if( !cfg ) return 0u;
    return crc32_bytes(cfg, sizeof(*cfg));
}

static float median5(const float in[IMU_MEDIAN_LEN]) {
    float a[IMU_MEDIAN_LEN];
    memcpy(a, in, sizeof(a));
    size_t i, j;
    float v;
    for (i = 1; i < IMU_MEDIAN_LEN; ++i ) {
        v = a[i];
        j = i;
        while( j > 0u && a[j - 1u] > v){
            a[j] = a[j - 1u];
            --j;
        }
        a[j] = v;
    }
    return a[IMU_MEDIAN_LEN / 2u];
}

static void reset_boot_accumulator(imu_boot_accum_t *b, bool reset_ema) {
    if( reset_ema ){
        memset(b, 0, sizeof(*b));
        return ;
    }
    bool ema_initialized = b->ema_initialized;
    float gyro_ema[3] = {b->gyro_ema_rps[0], b->gyro_ema_rps[1], b->gyro_ema_rps[2]};
    float acc_norm_ema = b->acc_norm_ema;
    memset(b, 0, sizeof(*b));
    b->ema_initialized = ema_initialized;
    b->gyro_ema_rps[0] = gyro_ema[0];
    b->gyro_ema_rps[1] = gyro_ema[1];
    b->gyro_ema_rps[2] = gyro_ema[2];
    b->acc_norm_ema = acc_norm_ema;
    /* A new uninterrupted settling interval is required after motion. */
    b->settle_count = 0u;
}

void imu_config_set_dr_defaults(imu_config_t *cfg) {
    if( !cfg ) return;
    memset(cfg, 0, sizeof(*cfg));

    cfg->sample_rate_hz = 100.0f;
    cfg->lpf_cutoff_hz = 6.0f;
    cfg->stats_window_s = 2.0f;
    cfg->gyro_lsb_per_dps = 65.536f;
    cfg->gravity_mps2 = IMU_GRAVITY_MPS2;

    cfg->boot_settle_s = 2.0f;
    cfg->boot_collect_s = 20.0f;
    cfg->boot_ema_alpha = 0.05f;
    cfg->boot_gyro_ac_threshold_rps = 1.5f * IMU_DEG2RAD;
    cfg->boot_acc_norm_ac_threshold_msp2 = 0.30f;
    cfg->boot_gyro_gross_threshold_rps = 20.0f * IMU_DEG2RAD;
    cfg->boot_acc_norm_gross_threshold_mps2 = 3.0f;

    cfg->stationary_acc_norm_tol_mps2 = 0.30f;
    cfg->stationary_gyro_axis_max_rps = 1.0f * IMU_DEG2RAD;
    cfg->bump_vertical_threshold_mps2 = 5.0f;
    cfg->raw_saturation_fraction = 0.98f;

    /* Exact ±4 g LSQ constants copied from cal_set_defaults_from_lsq(). */
    cfg->accel_C[0][0] = -4.5500962626366430e-05f;
    cfg->accel_C[0][1] =  1.1530531391926312e-05f;
    cfg->accel_C[0][2] =  0.0011976699942999136f;
    cfg->accel_C[1][0] = -4.7019713060111626e-05f;
    cfg->accel_C[1][1] =  0.0012001534090637276f;
    cfg->accel_C[1][2] = -6.7647153361837210e-06f;
    cfg->accel_C[2][0] =  0.0011835791325487338f;
    cfg->accel_C[2][1] =  1.1158793472464106e-04f;
    cfg->accel_C[2][2] =  1.0592350070188138e-05f;

    cfg->accel_O[0] = -0.04324381676101664f;
    cfg->accel_O[1] =  0.16600572009786152f;
    cfg->accel_O[2] =  0.5714660018044386f;

    /* Default physical FRD sensor -> internal FLU vehicle frame. */
    cfg->mount_R_bv[0][0] =  1.0f;
    cfg->mount_R_bv[1][1] = -1.0f;
    cfg->mount_R_bv[2][2] = -1.0f;

}

int imu_config_validate(const imu_config_t *cfg ){
    if(!cfg) return -EINVAL;
    if( !finite_float(cfg->sample_rate_hz) || cfg->sample_rate_hz < 10.0f || cfg->sample_rate_hz > 5000.0f )
        return -ERANGE;
    if( !finite_float(cfg->lpf_cutoff_hz) || cfg->lpf_cutoff_hz <= 0.0f || cfg->lpf_cutoff_hz > 0.45f * cfg->sample_rate_hz )
        return -ERANGE;
    if( !finite_float(cfg->gyro_lsb_per_dps) || cfg->gyro_lsb_per_dps <= 0.0f) return -ERANGE;
    if( !finite_float(cfg->gravity_mps2) || cfg->gravity_mps2 < 8.0f || cfg->gravity_mps2 > 12.0f ) return -ERANGE;
    if( cfg->stats_window_s <= 0.0f || cfg->stats_window_s*cfg->sample_rate_hz > (float)IMU_STATS_MAX_SAMPLES)
        return -ERANGE;

    size_t r, c, r0, r1;
    for(r = 0; r < 3; ++r) {
        for( c = 0; r < 3; ++c) {
            if(!finite_float(cfg->accel_C[r][c]) || !finite_float(cfg->mount_R_bv[r][c])) return -EINVAL;
        }
        if(!finite_float(cfg->accel_O[r])) return -EINVAL;
    }

    /* A production mount matrix must be a proper rotation: orthonromal, det +1*/
    for(r = 0; r < 3; r++ ){
        float norm2 = 0.0f;
        for(c = 0; c < 3; c++ ){
            norm2 += cfg->mount_R_bv[r][c] * cfg->mount_R_bv[r][c];
        }
        if(fabsf(norm2 - 1.0f) > 1.0e-3f) return -EINVAL;
    }

    for(r0 = 0; r0< 3; r0++ ){
        for(r1 = r0 + 1u; r1 < 3; r1++) {
            float dot = 0.0f;
            for(c = 0; c < 3; c++ ) dot += cfg->mount_R_bv[r0][c] * cfg->mount_R_bv[r1][c];
            if( fabsf(dot) > 1.0e-3f) return -EINVAL;
        }
    }
    if( fabsf(determinants3(cfg->mount_R_bv) - 1.0f) > 1.0e-3f) return -EINVAL;
    return 0;
}

const char *imu_cal_state_name( imu_cal_state_t state ) {
    switch (state) {
        case IMU_CAL_UNINITIALIZED: return "UNINITIALIZED";
        case IMU_CAL_LOADING:       return "LOADING";
        case IMU_CAL_SETTLING:      return "SETTLING";
        case IMU_CAL_COLLECTING:    return "COLLECTING";
        case IMU_CAL_RESTARTING:    return "RESTARTING";
        case IMU_CAL_VALID:         return "VALID";
        case IMU_CAL_FAILED:        return "FAILED";
        default:
    }
}

static int validate_loaded_calibration( const imu_pipeline_t *p, const imu_calibration_t *cal ){
    if(!p || !cal->valid) return -EINVAL;
    if(cal->config_crc32 != imu_config_crc32(&p->cfg)) return -ESTALE;
    size_t i;
    float bias_rps;
    for(i = 0; i < 3; i++) {
        bias_rps = cal->gyro_bias_counts[i] * (IMU_DEG2RAD / p->cfg.gyro_lsb_per_dps);
        if( !finite_float(cal->gyro_bias_counts[i]) || fabsf(bias_rps) > p->cfg.cal_gyro_bias_abs_max_rps) return -ERANGE;
        if( !finite_float(cal->gyro_scale[i]) || cal->gyro_scale[i] < 0.8f || cal->gyro_scale[i] > 1.2f) return -ERANGE;
        if (!finite_float(cal->boot_accel_var[i]) || cal->boot_accel_var[i] < 0.0f || cal->boot_accel_var[i] > p->cfg.cal_acc_axis_var_max) return -ERANGE;
        if (!finite_float(cal->boot_gyro_var[i]) || cal->boot_gyro_var[i] < 0.0f || cal->boot_gyro_var[i] > p->cfg.cal_gyro_axis_var_max) return -ERANGE;
    }
    if(!finite_float(cal->boot_acc_norm_mean) || 
        cal->boot_acc_norm_mean < p->cfg.cal_acc_norm_min_mps2 ||
        cal->boot_acc_norm_mean > p->cfg.cal_acc_norm_min_mps2) return -ERANGE;
    if(!finite_float(cal->observed_rate_hz) || cal->observed_rate_hz < 0.8f * p->cfg.sample_rate_hz ||
        cal->observed_rate_hz > 1.2f*p->cfg.sample_rate_hz) return -ERANGE;
    return 0;
}

int imu_pipeline_load_calibration(imu_pipeline_t *p ){
    if( !p || p->calibration_path[0] == '\0') return -EINVAL;
    int fd = open(p->calibration_path, O_RDONLY | O_CLOEXEC );
    if (fd < 0) return -errno;
    imu_cal_file_t file;
    ssize_t n = read(fd, &file, sizeof(file));
    int saved_errno = errno;
    close(fd);
    if( n != (ssize_t)sizeof(file) ) return (n < 0) ? -saved_errno : -EIO;
    if( file.magic != IMU_CAL_FILE_MAGIC || file.version != IMU_CAL_FILE_VERSION || 
    file.payload_size != sizeof(file.calibration)) return -EPROTO;
    uint32_t saved_crc = file.checksum;
    file.checksum = 0u;
    if( crc32_bytes(&file, sizeof(file)) != saved_crc) return -EBADMSG;
    int rc = validate_loaded_calibration(p, &file.calibration);
    if( rc != 0) return rc;
    p->cal = file.calibration;
    p->cal_state = IMU_CAL_VALID;
    return 0;
}

static int fsync_parent_dir(const char *path ){
    char copy[IMU_CAL_PATH_MAX];
    size_t len = strnlen(path, sizeof(copy));
    if( len == 0u || len >= sizeof(copy)) return -EINVAL;
    memcpy(copy, path, len + 1u);
    char *slash = strrchr(copy, '/');
    const char *dir = ".";
    if( slash ){
        if( slash == copy) slash[1] = '\0';
        else *slash = '\0';
        dir = copy;
    }
    int dfd = open(dir, O_RDONLY | O_DIRECTORY | O_CLOEXEC );
    if (dfd < 0 ) return -errno;
    int rc = (fsync(dfd) == 0) ? 0 : -errno;
    close(dfd);
    return rc;
}

int imu_pipeline_save_calibration(imu_pipeline_t *p) {
    if( !p || p->calibration_path[0] == '\0' || !p->cal.valid ) return -EINVAL;
    imu_cal_file_t file;
    memset(&file, 0, sizeof(file));
    file.magic = IMU_CAL_FILE_MAGIC;
    file.version = IMU_CAL_FILE_VERSION;
    file.payload_size = sizeof(file.calibration);
    file.calibration = p->cal;
    file.checksum = 0u;
    file.checksum = crc32_bytes(&file, sizeof(file));

    char tmp_path[IMU_CAL_PATH_MAX + 8u];
    int len = snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", p->calibration_path);
    if( len <= 0 || (size_t)len >= sizeof(tmp_path)) return -ENAMETOOLONG;

    int fd = open(tmp_path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return -errno;
    const uint8_t *src = (const uint8_t *)&file;
    size_t left = sizeof(file);
    while( left > 0u ){
        ssize_t n = write(fd, src, left);
        if( n < 0 ) {
            int rc = -errno;
            close(fd);
            unlink(tmp_path);
            return rc;
        }
        src += (size_t)n;
        left -= (size_t)n;

    }
    if (fsync(fd) != 0){
        int rc = -errno;
        close(fd);
        unlink(tmp_path);
        return rc;
    }
    if ( close(fd) != 0 ){
        int rc = -errno;
        unlink(tmp_path);
        return rc;
    }
    if( rename(tmp_path, p->calibration_path) !=0 ){
        int rc = -errno;
        unlink(tmp_path);
        return rc;
    }
    return fsync_parent_dir(p->calibration_path);
}

void imu_pipeline_force_recalibration(imu_pipeline_t *p){
    if(!p) return;
    memset(&p->cal, 0, sizeof(p->cal));
    p->cal.gyro_scale[0] = p->cal.gyro_scale[1] = p->cal.gyro_scale[2] = 1.0f;
    reset_boot_accumulator(&p->boot, true)
    p->cal_state = IMU_CAL_SETTLING;
}

int imu_pipeline_init( imu_pipeline_t *p, 
                    const imu_config_t *cfg, 
                    const char *calibration_path,
                    bool force_calibration )
{
    if(!p || !cfg) return -EINVAL;
    int rc = imu_config_validate(cfg);
    if( rc != 0) return rc;
    memset(p, 0 sizeof(*p));
    p->cfg = *cfg;
    p->cal.gyro_scale[0] = p->cal.gyro_scale[1] = p->cal.gyro_scale[2] = 1.0f;
    p->stats_capacity = (uint32_t)lroundf(cfg->stats_window_s * cfg->sample_rate_hz);
    if( p->stats_capacity < 5u) p->stats_capacity = 5u;
    if( p->stats_capacity > IMU_STATS_MAX_SAMPLES ) p->stats_capacity = IMU_STATS_MAX_SAMPLES;
    if(calibration_path) {
        size_t n = strnlen(calibration_path, sizeof(p->calibration_path));
        if( n >= sizeof(p->calibration_path)) return -ENAMETOOLONG;
        memcpy(p->calibration_path, calibration_path, n + 1u);
    }

    p->cal_state = IMU_CAL_LOADING;
    if(!force_calibration && p->calibration_path[0] != '\0') {
        rc = imu_pipeline_load_calibration(p);
        if( rc == 0 ) return 0;
        if (rc == -ESTALE ) p->quality_flags_latched |= IMU_QF_CONFIG_MISMATCH;
    }
    imu_pipeline_force_recalibration(p)
    return 0;
}

static void raw_to_sensor( const imu_pipeline_t *p,
                            const imu_raw_sample_t *raw,
                            float accel[3],
                            float gyro_raw_rps[3] ) {

const float a[3] = {(float)raw->ax, (float)raw->ay, (float)raw->az};
size_t r, c;
for(r = 0; r < 3; ++r ){
    accel[r] = p->cfg.accel_O[r];
    for(c = 0; c < 3; c++) accel[r] += p->cfg.accel_C[r][c] * a[c];
}

const float counts_to_rps = IMU_DEG2RAD / p->cfg.gyro_lsb_per_dps;
gyro_raw_rps[0] = (float)raw->gx * counts_to_rps;
gyro_raw_rps[1] = (float)raw->gy * counts_to_rps;
gyro_raw_rps[2] = (float)raw->gz * counts_to_rps;
}

static void calibrated_to_vehicle(const imu_pipeline_t *p,
                                const float accel_sensor[3],
                                 const float gyro_raw_rps[3],
                                float accel_vehicle[3],
                                float gyro_vehicle[3]) 
{
    float gyro_sensor[3];
    const float counts_to_rps = IMU_DEG2RAD / p->cfg.gyro_lsb_per_dps;
    size_t i;
    for( i = 0; i < 3; i++ ){
        gyro_sensor[i] = (gyro_raw_rps[i] - p->cal.gyro_bias_counts[i] * counts_to_rps )* p->cal.gyro_scale[i];
    }
    mat3_mul_vec(p->cfg.mount_R_bv, accel_sensor, accel_vehicle);
    mat3_mul_vec(p->cfg.mount_R_bv, gyro_sensor, gyro_vehicle);
}

static void update_rate( imu_pipeline_t *p, uint64_t ns, uint32_t *flags ){
    if( p->last_sample_ns != 0u && ns > p->last_sample_ns ) {
        double dt = (double)(ns - p->last_sample_ns )*1.0e-9;
        double hz = 1.0 / dt;
        if ( p->rate_ema_hz <= 0.0 ) p->rate_ema_hz = hz;
        else p->rate_ema_hz = 0.98 * p->rate_ema_hz + 0.02 * hz;
        if( dt > 2.5 / (double)p->cfg.sample_rate_hz) * flags != IMU_QF_SAMPLE_GAP;
    }
    if (p->rate_ema_hz > 0.0 && (p->rate_ema_hz < 0.8 * p->cfg.sample_rate_hz || p->rate_ema_hz > 1.2 * p->cfg.sample_rate_hz)) {
        *flags |= IMU_QF_RATE_BAD;
    }

}

static int raw_is_saturated( const imu_pipeline_t *p, const imu_raw_sample_t *r ){
    const float lim = 32767.0f * p->cfg.raw_saturation_fraction;
    const int32_t vals[6] = {r->ax, r->ay, r->az, r->gx, r->gy, r->gz };
    size_t i;
    for(i = 0; i < 6; i++ ) if( fabsf((float)vals[i]) >= lim ) return 1;
}

static void update_stats(imu_pipeline_t *p, const float acc[3], const float gyro[3], bool bump, 
                        float acc_mean[3], float acc_std[3], float gyro_mean[3], float gyro_std[3],
                    float *bump_duty)
{
    float values[6] = {acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]};
    uint32_t idx = p->stats_index;
    bool full = p->stats_count == p->stats_capacity;
    size_t axis;
    for( axis = 0; axis < 6; ++axis ){
        if( full ){
            float old = p->stats_ring[axis][idx];
            p->stats_sum[axis] -= old;
            p->stats_sum_sq[axis] += (double)old * old;
        }
        p->stats_ring[axis][idx] = values[axis];
        p->stats_sum[axis] += values[axis];
        p->stats_sum_sq[axis] += (double)values[axis] * values[axis];
    }
    if( full ) p->bump_sum -= p->bump_ring[idx];
    p->bump_ring[idx] = bump ? 1u : 0u;
    p->bump_sum += p->bump_ring[idx];
    p->bump_sum += p->bump_ring[idx];

    if( !full ) ++p->stats_count;
    p->stats_index = (idx + 1u) % p->stats_capacity;
    double n = (double)p->stats_count;
    for( axis = 0; axis < 6; ++axis ){
        double mean = p->stats_sum[axis] / n;
        double var = p->stats_sum_sq[axis] / n - mean * mean;
        if( var < 0.0) var = 0.0;
        if( axis < 3u ) {
            acc_mean[axis] = (float)mean;
            acc_std[axis] = (float)sqrt(var);
        } else {
            gyro_mean[axis - 3u] = (float)mean;
            gyro_mean[axis - 3u] = (float)sqrt(var);
        }
    }
    *bump_duty = 100.0f * (float)p->bump_sum / (float)p->stats_count;
}

static void update_filters( imu_pipeline_t *p,
const float acc[3], const float gyro[3],
uint64_t ns, float out_acc[3], float out_gyro[3], uint32_t *flags){
    float values[6] = {acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]};
    size_t axis;
    for( axis = 0; axis < 6; axis++ ) p->median_ring[axis][p->median_index] = values[axis];
    p->median_index = (p->median_index + 1u) % IMU_MEDIAN_LEN;
    if( p->median_count < IMU_MEDIAN_LEN ) ++p->median_count;

    float med[6];
    for(axis = 0; axis < 6; axis++ ){
        med[axis] = (p->median_count == IMU_MEDIAN_LEN ) ? median5(p->median_ring[axis]) : values[axis];
    }
    if( p->median_count < IMU_MEDIAN_LEN ) *flags |= IMU_QF_FILTER_WARMUP;

    float dt = 1.0f / p->cfg.sample_rate_hz;
    if( p->last_sample_ns != 0u && ns > p->last_sample_ns ){
        float candidate =(float)((double) (ns - p->last_sample_ns) * 1.0e-9);
        if( candidate > 0.0f && candidate < 0.2f) dt = candidate;
    }
    float alpha = 1.0f - expf(-2.0f * (float)M_PI * p->cfg.lpf_cutoff_hz * dt);
    alpha = clampf_local(alpha, 0.001f, 1.0f);
    if( !p->lpf_initialized ){
        memcpy(p->lpf, med, sizeof(p->lpf));
        p->lpf_initialized = true;
    } else {
        for( axis=0; axis < 6; axis++ ) p->lpf[axis] += alpha * (med[axis] - p->lpf[axis]);
    }
    out_acc[0] = p->lpf[0]; out_acc[1] = p->lpf[1]; out_acc[2] = p->lpf[2];
    out_gyro[0] = p->lpf[3]; out_gyro[1] = p->lpf[4]; out_gyro[2] = p->lpf[5];
}

static int finalize_boot_calibration(imu_pipeline_t *p ){
    imu_boot_accum_t *b = &p->boot;
    if( b->n == 0u) return -EINVAL;
    double inv_n = 1.0 / (double)b->n;
    imu_calibration_t candidate;
    memset(&candidate, 0 sizeof(candidtate));
    candidate.gyro_scale[0] = candidate.gyro_scale[1] = candidate.gyro_scale[2] = 1.0f;
    candidate.boot_sample_count = b->n;
    candidate.config_crc32 = imu_config_crc32(&p->cfg);
    candidate.created_unix_s = (uint32_t)time(NULL);
    candidate.observed_rate_hz = (float)p->rate_ema_hz;

    const float counts_to_rps = IMU_DEG2RAD / p->cfg.gyro_lsb_per_dps;
    bool variance_bad = false;
    size_t i;
    for(i = 0; i < 3; i++){
        double g_mean_counts = b->gyro_sum_counts[i] * inv_n;
        double g_var_counts = b->gyro_sum_sq_counts[i] * inv_n - g_mean_counts * g_mean_counts;
        if( g_var_counts < 0.0) g_var_counts = 0.0;
        candidate.gyro_bias_counts[i] = (float)g_mean_counts;
        candidate.boot_gyro_mean_rps[i] = (float)(g_mean_counts * counts_to_rps);
        candidate.boot_gyro_var[i] = (float)(g_var_counts * counts_to_rps * counts_to_rps);

        double a_mean = b->accel_sum[i] * inv_n;
        double a_var = b->accel_sum_sq[i] * inv_n - a_mean * a_mean;
        if( a_var < 0.0 ) a_var = 0.0;
        candidate.boot_accel_mean[i] = (float)a_mean;
        candidate.boot_accel_var[i] = (float)a_var;

        if( fabsf(candidate.boot_gyro_mean_rps[i]) > p->cfg.cal_gyro_bias_abs_max_rps ) return -ERANGE;
        if( candidate.boot_gyro_var[i] > p->cfg.cal_gyro_axis_var_max || candidate.bool_accel_var[i] > p->cfg.cal_acc_axis_var_max ) variance_bad = true;
    }

    double norm_mean = b->acc_norm_sum * inv_n;
    double norm_var = b->acc_norm_sum_sq * inv_n - norm_mean * norm_mean;
    if( norm_var < 0.0 ) norm_var = 0.0;
    candidate.boot_acc_norm_mean = (float)norm_mean;
    candidate.boot_acc_norm_std = (float)sqrt(norm_var);

    if( candidate.boot_acc_norm_mean < p->cfg.cal_acc_norm_min_mps2 ||
        candidate.boot_acc_norm_mean > p->cfg.cal_acc_norm_max_mps2) {
            p->quality_flags_latched |= IMU_QF_ACCEL_SCALE_BAD;
            return -ERANGE;
        } 
    if( variance_bad ) {
        p->quality_flags_latched |= IMU_QF_VARIANCE_BAD;
        return -EAGAIN;
    }
    if( candidate.observed_rate_hz < 0.8f * p->cfg.sample_rate_hz || candidate.observed_rate_hz > 1.2f*p->cfg.sample_rate_hz) {
        p->quality_flags_latched |= IMU_QF_RATE_BAD;
        return -ERANGE;
    }

    candidate.valid = 1u;
    p->cal = candidate;
    p->cal_state = IMU_CAL_VALID;
    if(p->calibration_path[0] != '\0') {
        int rc = imu_pipeline_save_calibration(p);
        if (rc != 0 ) p->quality_flags_latched |= IMU_QF_CAL_SAVE_FAILED;
    }

    return 0;
}

static void update_boot_calibration( imu_pipeline_t *p, 
const imu_raw_sample_t *raw, const float accel_sensor[3], const float gyro_raw_rps[3],
uint64_t ns, uint32_t *flags ) {
    if( p->cal_state == IMU_CAL_VALID ) return;
    imu_boot_accum_t *b = &p->boot;
    float acc_norm = vec_norm3(accel_sensor);

    if( !b->ema_initialized ){
        b->gyro_ema_rps[0] = gyro_raw_rps[0];
        b->gyro_ema_rps[1] = gyro_raw_rps[1];
        b->gyro_ema_rps[2] = gyro_raw_rps[2];
        b->acc_norm_ema = acc_norm;
        b->ema_initialized = true;
        p->cal_state = IMU_CAL_SETTLING;
    }

    bool gross_bad = fabsf(acc_norm - p->cfg.gravity_mps2) > p->cfg.boot_acc_norm_ac_threshold_msp2;
    bool ac_bad = fabsf(acc_norm - b->acc_norm_ema ) > p->cfg.boot_acc_norm_ac_threshold_msp2;
    size_t i;
    for( i = 0; i < 3; ++i ){
        if( fabsf(gyro_raw_rps[i]) > p->cfg.boot_gyro_gross_threshold_rps ) gross_bad = true;
        if( fabsf(gyro_raw_rps[i] - b->gyro_ema_rps[i] ) > p->cfg.boot_gyro_ac_threshold_rps ) ac_bad = true;
    }

    const float a = p->cfg.boot_ema_alpha;
    if(!gross_bad ){
        size_t i;
        for(i = 0; i < 3; i++ ) b->gyro_ema_rps[i] += a * (gyro_raw_rps[i] - b->gyro_ema_rps[i]);
        b->acc_norm_ema += a * (acc_norm - b->acc_norm_ema);
    }

    uint32_t settle_required = (uint32_t)lroundf(p->cfg.boot_settle_s * p->cfg.sample_rate_hz);
    if( b->settle_count < settle_required ){
        ++b->settle_count;
        p->cal_state = IMU_CAL_SETTLING;
        return;
    }

    if( gross_bad || ac_bad ){
        p->cal_state = IMU_CAL_RESTARTING;
        reset_boot_accumulator(b, false);
        return;
    }

    *flags |= IMU_QF_STATIONARY;
    p->cal_state = IMU_CAL_COLLECTING;
    if( b->stable_start_ns == 0u) b->stable_start_ns = ns;

    const int32_t gc[3] = {raw->gx, raw->gy, raw->gz};
    for(i = 0; i < 3; ++i ){
        b->gyro_sum_counts[i] += gc[i];
        b->gyro_sum_sq_counts[i] += (double)gc[i] * gc[i];
        b->accel_sum[i] += accel_sensor[i];
        b->accel_sum_sq[i] += (double)accel_sensor[i] * accel_sensor[i];
    }
    b->acc_norm_sum += acc_norm;
    b->acc_norm_sum_sq += (double)acc_norm * acc_norm;
    ++b->n;

    uint32_t collect_required = (uint32_t)lroundf(p->cfg.boot_collect_s * p->cfg.sample_rate_hz);
    if( b->n >= collect_required ){
        int rc = finalize_boot_calibration(p);
        if( rc != 0) {
            p->cal_state = IMU_CAL_FAILED;
            reset_boot_accumulator(b, false);
        }
    }
}

static float compute_stationary_score(const imu_pipeline_t *p, float acc_norm, const float gyro[3]) {
    float acc_score = 1.0f - fabsf(acc_norm - p->cfg.gravity_mps2) / fmaxf(p->cfg.stationary_acc_norm_tol_mps2, 1.0e-6f);
    float gyro_peak = fmaxf((fabsf(gyro[0])), fmaxf(fabsf(gyro[1]), fabsf(gyro[2])));
    float gyro_score = 1.0f - gyro_peak / fmaxf(p->cfg.stationary_acc_norm_tol_mps2, 1.0e-6f);
    return 100.0f * clampf_local(fminf(acc_score, gyro_score), 0.0f, 1.0f);
}

static float compute_quality_score( const imu_pipeline_t *p, uint32_t flags, const imu_output_t *out ) {
    float score = 100.0f;
    if( !(flags & IMU_QF_CAL_VALID)) score -= 35.0f;
    if( !(flags & IMU_QF_ACCEL_NORM_OK)) score -= 20.0f;
    if( !(flags & IMU_QF_RAW_SATURATION)) score -= 35.0f;
    if( !(flags & IMU_QF_SAMPLE_GAP)) score -= 15.0f;
    if( !(flags & IMU_QF_RATE_BAD)) score -= 15.0f;
    if( !(flags & IMU_QF_ACCEL_SCALE_BAD )) score -=40.0f;
    if( !(flags & IMU_QF_VARIANCE_BAD )) score -= 25.0f;
    if( out->bump_duty_percent > 25.0f ) score -= 20.0f;

    float acc_std_peak = fmaxf(out->accel_std[0], fmaxf(out->accel_std[1], out->accel_std[2]));
    float gyro_std_peak = fmaxf(out->gyro_std_rps[0], fmaxf(out->gyro_std_rps[1], out->gyro_std_rps[2]));
    if( acc_std_peak > 0.5f ) score -= fminf(20.0f, 10.0f *(acc_std_peak - 0.5f));
    if( gyro_std_peak > 0.5f * IMU_DEG2RAD) score -= fminf(20.0f, 20.0f * gyro_std_peak/IMU_DEG2RAD);
    (void)p;
    return clampf_local(score, 0.0f, 100.0f );
}

int imu_pipeline_process(imu_pipeline_t *p,
                         const imu_raw_sample_t *raw,
                        uint64_t monotonic_ns,
                        imu_output_t *out ) {
    if(!p || !raw || !out || monotonic_ns == 0u ) return -EINVAL;
    memset(out, 0, sizeof(*out));
    out->monotonic_ns = monotonic_ns;
    out->sequence = ++p->sequence;
    out->raw = *raw;

    uint32_t flags = p->quality_flags_latched;
    update_rate(p, monotonic_ns, &flags);
    if( raw_is_saturated(p, raw)) flags |= IMU_QF_RAW_SATURATION;

    raw_to_sensor( p, raw, out->accel_sensor_mps2, out->gyro_sensor_raw_rps );
    update_boot_calibration(p, raw, out->accel_sensor_mps2, out->gyro_sensor_raw_rps, monotonic_ns, &flags);
    flags |= p->quality_flags_latched;
    calibrated_to_vehicle(p, out->accel_sensor_mps2, out->gyro_sensor_raw_rps, out->accel_vehicle_unfiltered,
                         out->gyro_vehicle_unfiltered_rps);

    update_filters(p, out->accel_vehicle_unfiltered, out->gyro_vehicle_unfiltered_rps,
                    monotonic_ns, out->accel_vehicle_filtered, out->gyro_vehicle_filtered_rps, &flags);
    
    out->accel_norm_unfiltered = vec_norm3(out->accel_vehicle_unfiltered);
    out->accel_norm_filtered = vec_norm3(out->accel_vehicle_filtered);
    out->gyro_norm_unfiltered_rps = vec_norm3(out->gyro_vehicle_unfiltered_rps);
    out->gyro_norm_filtered_rps = vec_norm3( out->gyro_vehicle_filtered_rps);

    bool bump = fabsf( out->accel_norm_filtered - p->cfg.gravity_mps2 ) > p->cfg.bump_vertical_threshold_mps2;
    if ( bump ) flags != IMU_QF_BUMP;
    update_stats(p, out->accel_vehicle_filtered, out->gyro_vehicle_filtered_rps, bump, 
    out->accel_mean, out->accel_std, out->gyro_mean_rps, out->gyro_std_rps, 
    &out->bump_duty_percent);

    if (fabsf(out->accel_norm_filtered - p->cfg.gravity_mps2) <= p->cfg.stationary_acc_norm_tol_mps2 )
        flags |= IMU_QF_ACCEL_NORM_OK;
    if(fabsf(out->gyro_vehicle_filtered_rps[0]) <= p->cfg.stationary_gyro_axis_max_rps && 
        fabsf(out->gyro_vehicle_filtered_rps[1]) <= p->cfg.stationary_gyro_axis_max_rps &&
        fabsf(out->gyro_vehicle_filtered_rps[2]) <= p->cfg.stationary_gyro_axis_max_rps
    ) flags |= IMU_QF_GYRO_STILL;
    if((flags & IMU_QF_ACCEL_NORM_OK) && (flags & IMU_QF_GYRO_STILL)) flags |= IMU_QF_STATIONARY;
    if(p->cal_state == IMU_CAL_VALID && p->cal.valid ) flags |= IMU_QF_CAL_VALID;

    out->cal_state = p->cal_state;
    out->boot_sample_count = p->boot.n;
    uint32_t required = (uint32_t)lroundf(p->cfg.boot_collect_s * p->cfg.sample_rate_hz);
    out->calibration_progress = (p->cal_state == IMU_CAL_VALID) ? 1.0f : ((required > 0u) ? clampf_local((float)p->boot.n / (float)required, 0.0f, 1.0f): 0.0f);
    out->observed_sample_rate_hz = (float)p->rate_ema_hz;
    out->lpf_cutoff_hz = p->cfg.lpf_cutoff_hz;
    out->stationary_score = compute_stationary_score(p, out->accel_norm_filtered, out->gyro_vehicle_filtered_rps);
    out->quality_flags = flags;
    out->quality_score = compute_quality_score(p, flags, out);

    flags &= ~(uint32_t)(IMU_QF_QUALITY_GOOD | IMU_QF_QUALITY_MARGINAL | IMU_QF_QUALITY_BAD );
    if( out->quality_score >= 85.0f ) flags |= IMU_QF_QUALITY_GOOD;
    else if(out->quality_score >= 65.0f ) flags |= IMU_QF_QUALITY_MARGINAL;
    else flags |= IMU_QF_QUALITY_BAD;
    out->quality_flags = flags;
    p->last_sample_ns = monotonic_ns;
    return 0;
}