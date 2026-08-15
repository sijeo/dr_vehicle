#include "imu_core.h"
#include "imu_wire.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int inv3(const float A[3][3], float B[3][3]) {
    float det = A[0][0] * (A[1][1]*A[2][2]-A[1][2]*A[2][1])
               -A[0][1] * (A[1][0]*A[2][2]-A[1][2]*A[2][0])
               +A[0][2] * (A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (fabsf(det) < 1e-12f) return 0;
    float d = 1.0f/det;
    B[0][0] = (A[1][1]*A[2][2]-A[1][2]*A[2][1]) * d; 
    B[0][1] = (A[0][2]*A[2][1]-A[0][1]*A[2][2]) * d;
    B[0][2] = (A[0][1]*A[1][2]-A[0][2]*A[1][1]) * d;
    B[1][0] = (A[1][2]*A[2][0]-A[1][0]*A[2][2]) * d;
    B[1][1] = (A[0][0]*A[2][2]-A[0][2]*A[2][0]) * d;
    B[1][2] = (A[0][2]*A[1][0]-A[0][0]*A[1][2]) * d;
    B[2][0] = (A[1][0]*A[2][1]-A[1][1]*A[2][1]) * d;
    B[2][1] = (A[0][1]*A[2][0]-A[0][0]*A[2][1]) * d;
    B[2][2] = (A[0][0]*A[1][1]-A[0][1]*A[1][0]) * d;
    return 1; 
}

static imu_raw_sample_t make_level_sample(const imu_config_t *cfg ){
    float inv[3][3]; assert(inv3(cfg->accel_C, inv));
    float target[3] = {0.0f, 0.0f, -cfg->gravity_mps2};
    float y[3] = {target[0]-cfg->accel_O[0], target[1]-cfg->accel_O[1], target[2]-cfg->accel_O[2]};
    int32_t a[3];
    int r;
    for( r=0; r<3; r++ ) a[r]=(int32_t)lroundf(inv[r][0]*y[0] + inv[r][1]*y[1] + inv[r][2]*y[2]);
    imu_raw_sample_t s = { .ax = a[0], .ay=a[1], .az=a[2], .gx=-39, .gy=58, .gz=-16};
    return s;
}

int main( void )
{
    assert(IMU_WIRE_FRAME_SIZE == 236u);
    imu_config_t cfg; imu_config_set_dr_defaults(&cfg);
    cfg.boot_settle_s = 0.1f;
    cfg.boot_collect_s = 1.0f;
    assert(imu_config_validate(&cfg) == 0);
    unlink("/tmp/test_imu_cal.bin");
    imu_pipeline_t p;
    assert(imu_pipeline_init(&p, &cfg, "/tmp/test_imu_cal.bin", true) == 0);
    imu_raw_sample_t raw = make_level_sample(&cfg);
    imu_output_t out;
    uint64_t ns = 1000000000ull;
    int i;
    for(i = 0; i < 150; i++, ns+=10000000ull){
        assert(imu_pipeline_process(&p, &raw, ns, &out) == 0);
    }
    assert(p.cal_state == IMU_CAL_VALID );
    assert(p.cal.valid == 1u);
    assert(fabsf(out.accel_norm_filtered - cfg.gravity_mps2) < 0.05f);
    assert(out.quality_flags & IMU_QF_CAL_VALID);
    uint8_t frame[IMU_WIRE_FRAME_SIZE]; size_t encoded = 0;
    assert(imu_wire_encode(&out, &p.cal, frame, sizeof(frame), &encoded) == 0);
    assert(encoded == IMU_WIRE_FRAME_SIZE);
    printf("PASS: cal=%s rate=%.2fHz acc_norm=%.4f quality=%.1f\n",
            imu_cal_state_name(p.cal_state), out.observed_sample_rate_hz, 
        out.accel_norm_filtered, out.quality_score);
    unlink("/tmp/test_imu_cal.bin");
    return 0;
}