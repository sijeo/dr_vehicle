#ifndef IMU_WIRE_H
#define IMU_WIRE_H

#include "imu_core.h"
#include "imu_fusion.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_WIRE_MAGIC          0x494D5531u     /*IMU1*/
#define IMU_WIRE_VERSION        1u
#define IMU_WIRE_FLOAT_COUNT    43u
#define IMU_WIRE_FRAME_SIZE     236u

/* v2 = v1 body + fusion block (4 x u32, IMU_FUSION_FLOAT_COUNT floats) + CRC.
 * Network byte order; Python: struct "!IHHIQ6i4I43f4I30fI" (372 bytes).
 * Version 3 added sflp_consistency_deg (version 2 had 29 floats). */
#define IMU_WIRE_MAGIC_V2       0x494D5532u     /*IMU2*/
#define IMU_WIRE_VERSION_V2     3u
#define IMU_WIRE_V2_FRAME_SIZE  (IMU_WIRE_FRAME_SIZE + 16u + 4u * IMU_FUSION_FLOAT_COUNT)

int imu_wire_encode( const imu_output_t *sample,
                    const imu_calibration_t *cal,
                    uint8_t *dst,
                    size_t dst_size,
                    size_t *encoded_size);

int imu_wire_encode_v2( const imu_output_t *sample,
                       const imu_calibration_t *cal,
                       const imu_fusion_output_t *fusion,
                       uint8_t *dst,
                       size_t dst_size,
                       size_t *encoded_size);

uint32_t imu_wire_crc32( const void *data, size_t size );


#ifdef __cplusplus
}
#endif
#endif



