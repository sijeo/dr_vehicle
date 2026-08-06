#ifndef IMU_WIRE_H
#define IMU_WIRE_H

#include "imu_core.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_WIRE_MAGIC          0x494D5531u     /*IMU1*/
#define IMU_WIRE_VERSION        1u
#define IMU_WIRE_FLOAT_COUNT    43u
#define IMU_WIRE_FRAME_SIZE     236u

int imu_wire_encode( const imu_output_t *sample,
                    const imu_calibration_t *cal,
                    uint8_t *dst,
                    size_t dst_size,
                    size_t *encoded_size);

uint32_t imu_wire_crc32( const void *data, size_t size );


#ifdef __cplusplus
}
#endif
#endif



