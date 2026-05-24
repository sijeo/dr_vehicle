/**
 * @file neo6m_gnss_ioctl.h
 * @brief Userspace ABI for NEO-6M GNSS module char device
 * 
 * This header can be copied to userspace. It defines the ioctl numbers and 
 * packed structures returned by the NEO-6M GNSS module char device.
 * 
 */

#ifndef NEO6M_GNSS_IOCTL_H
#define NEO6M_GNSS_IOCTL_H  

#include <linux/ioctl.h>
#include <linux/types.h>

#define NEO6M_GNSS_CHARDEV_NAME "neo6m_gnss"

/** v1 ABI — preserved for backwards compatibility. Do not modify. */
struct neo6m_gnss_fix {
    __u32 monotonic_ms;       /**< boottime ms of last parsed sentence (jiffies_to_msecs) */
    __s8  have_fix;           /**< 1 if fix, 0 if no fix */

    __s32 lat_e7;             /**< Latitude in degrees * 10^7 (WGS-84) */
    __s32 lon_e7;             /**< Longitude in degrees * 10^7 (WGS-84) */
    __s32 alt_mm;             /**< Altitude above mean sea level in mm */

    __s32 speed_mmps;         /**< Ground speed in mm/s. Userspace: speed_mps = speed_mmps / 1000.0 */
    __s32 course_deg_e5;      /**< Course over ground in degrees * 10^5 */

    __u16 hdop_x100;          /**< Horizontal DOP * 100 (unitless) */

    __u16 utc_year;
    __u8  utc_mon;
    __u8  utc_day;
    __u8  utc_hour;
    __u8  utc_min;
    __u8  utc_sec;
    __u16 utc_millis;

    bool  heading_valid;
    bool  hdop_valid;
} __attribute__((packed));


/**
 * v2 ABI — extends v1 with sequence counters for reliable fresh-data detection.
 *
 * Userspace poll pattern (10 Hz):
 *   save last_pos_seq = fix_v2.pos_seq;
 *   ioctl(fd, NEO6M_GNSS_IOC_GET_FIX_V2, &fix_v2);
 *   if (fix_v2.pos_seq != last_pos_seq) → fresh position arrived.
 *
 * Counters increment monotonically and never wrap in normal operation
 * (u32 at 1 Hz would wrap after ~136 years).
 */
struct neo6m_gnss_fix_v2 {
    /* v1-compatible fields — same layout and semantics as struct neo6m_gnss_fix */
    __u32 monotonic_ms;       /**< boottime ms of last recognised sentence (jiffies_to_msecs) */
    __s8  have_fix;
    __s32 lat_e7;
    __s32 lon_e7;
    __s32 alt_mm;
    __s32 speed_mmps;
    __s32 course_deg_e5;
    __u16 hdop_x100;
    __u16 utc_year;
    __u8  utc_mon;
    __u8  utc_day;
    __u8  utc_hour;
    __u8  utc_min;
    __u8  utc_sec;
    __u16 utc_millis;
    bool  heading_valid;
    bool  hdop_valid;

    /* v2 sequence counters */
    __u32 fix_seq;            /**< increments on each accepted valid GNSS epoch (new RMC-A UTC second) */
    __u32 pos_seq;            /**< increments only when fresh lat/lon is stored */
    __u32 vel_seq;            /**< increments only when fresh speed/course is stored */
    __u32 nmea_seq;           /**< increments for every checksum-valid recognised sentence */

    /* v2 per-quantity validity and timestamps */
    __u8  pos_valid;          /**< 1 if current position is valid (have_fix and pos parsed) */
    __u8  vel_valid;          /**< 1 if current velocity is valid (have_fix and vel parsed) */
    __u8  stale;              /**< 1 if last sentence provided neither fresh pos nor fresh vel */
    __u8  reserved0;          /**< pad — must be zero */
    __u32 pos_monotonic_ms;   /**< boottime ms when pos_seq last incremented (jiffies_to_msecs) */
    __u32 vel_monotonic_ms;   /**< boottime ms when vel_seq last incremented (jiffies_to_msecs) */
} __attribute__((packed));


#define NEO6M_GNSS_IOC_MAGIC   'N'
#define NEO6M_GNSS_IOC_GET_FIX    _IOR(NEO6M_GNSS_IOC_MAGIC, 1, struct neo6m_gnss_fix)
#define NEO6M_GNSS_IOC_GET_FIX_V2 _IOR(NEO6M_GNSS_IOC_MAGIC, 2, struct neo6m_gnss_fix_v2)



#endif // NEO6M_GNSS_IOCTL_H