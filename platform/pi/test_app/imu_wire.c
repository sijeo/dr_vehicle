#include "imu_wire.h"

#include <arpa/inet.h>
#include <errno.h>
#include <string.h>

static void put_u16( uint8_t **p, uint16_t v) {
    uint16_t n = htons(v);
    memcpy(*p, &n, sizeof(n));
    *p += sizeof(n);
}

static void put_u32( uint8_t **p, uint32_t v) {
    uint32_t n = htonl(v);
    memcpy(*p, &n, sizeof(n));
    *p += sizeof(n);
}

static void put_u64( uint8_t **p, uint64_t v) {
    put_u32(p, (uint32_t)( v >> 32));
    put_u32(p, (uint32_t)(v & 0xFFFFFFFFu));
}

static void put_i32( uint8_t **p, int32_t v) {
    put_u32(p , (uint32_t)v);
}

static void put_f32(uint8_t **p, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    put_u32(p, bits);
}

uint32_t imu_wire_crc32( const void *data, size_t size) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    size_t i;
    int b;
    for( i = 0; i < size; ++i ) {
        crc ^= p[i];
        for( b = 0; b < 8; ++b ) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

int imu_wire_encode( const imu_output_t *s,
const imu_calibration_t *cal,
uint8_t *dst, 
size_t dst_size,
size_t *encoded_size ){
    if( !s || !cal || !dst || !encoded_size) return -EINVAL;
    if(dst_size < IMU_WIRE_FRAME_SIZE ) return -ENOSPC;
    uint8_t *p = dst;
    size_t g, i;

    put_u32(&p, IMU_WIRE_MAGIC);
    put_u16(&p, IMU_WIRE_VERSION);
    put_u16(&p, IMU_WIRE_FRAME_SIZE);
    put_u32(&p, s->sequence);
    put_u64(&p, s->monotonic_ns);

    put_i32(&p, s->raw.ax); put_i32(&p, s->raw.ay); put_i32(&p, s->raw.az);
    put_i32(&p, s->raw.gx); put_i32(&p, s->raw.gy); put_i32(&p, s->raw.gz);

    put_u32(&p, (uint32_t)s->cal_state);
    put_u32(&p, s->quality_flags);
    put_u32(&p, s->boot_sample_count);
    put_u32(&p, cal->created_unix_s);

    const float *groups[] = {
        s->accel_sensor_mps2,
        s->gyro_sensor_raw_rps,
        s->accel_vehicle_unfiltered,
        s->gyro_vehicle_unfiltered_rps,
        s->accel_vehicle_filtered,
        s->gyro_vehicle_filtered_rps,
        s->accel_mean,
        s->accel_std,
        s->gyro_mean_rps,
        s->gyro_std_rps,
        cal->gyro_bias_counts
    };
    for( g = 0; g < sizeof(groups)/sizeof(groups[0]); ++g)
        for( i = 0; i < 3; ++i ) put_f32(&p, groups[g][i]);

    put_f32(&p, s->accel_norm_unfiltered);
    put_f32(&p, s->accel_norm_filtered);
    put_f32(&p, s->gyro_norm_unfiltered_rps);
    put_f32(&p, s->gyro_norm_filtered_rps);
    put_f32(&p, s->calibration_progress);
    put_f32(&p, s->observed_sample_rate_hz);
    put_f32(&p, s->bump_duty_percent);
    put_f32(&p, s->lpf_cutoff_hz);
    put_f32(&p, s->quality_score);
    put_f32(&p, s->stationary_score);

    size_t without_crc = (size_t)(p - dst);
    if ( without_crc != IMU_WIRE_FRAME_SIZE - sizeof(uint32_t)) return -EPROTO;
    put_u32(&p, imu_wire_crc32(dst, without_crc));
    *encoded_size = (size_t)(p - dst);
    return (*encoded_size == IMU_WIRE_FRAME_SIZE) ? 0 : -EPROTO;
}

