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

/** Monotonic snapshot of  parsed fields  (SI Units) */
struct neo6m_gnss_fix {
    __s64 monotonic_ns;       /**< ktime_get_clock boottime in ns when parsed */
    __s8 have_fix;         /**< 1 if fix, 0 if no fix (RMC = 'A' and/or GGA fix > 0) */

    /** Position */
    __s32 lat_e7;        /**< Latitude in degrees * 10^7 (WGS-84)*/
    __s32 lon_e7;        /**< Longitude in degrees * 10^7 (WGS-84)*/
    __s32 alt_mm;        /**< Altitude above mean sea level in mm */

    /** Velocity */
    __s32 speed_mmps;        /**< Ground Speed in mm/s * 10^7 (from RMC/VTG)*/

    /** Heading / Course Over Ground  */
    __s32 course_deg_e5;     /**< Course/heading over ground in degrees x 10^5  */

    /** Dilution of precision */
    __u16 hdop_x100;           /**< Horizontal DOP x100 (unitless) */

    /** UTC time (if available) */
    __u16 utc_year;     /**< Year (4 digit)  e.g., 2025*/
    __u8 utc_mon;    /**< Month (1-12) */
    __u8 utc_day;    /**< Day (1-31) */
    __u8 utc_hour;   /**< Hour (0-23) */
    __u8 utc_min;    /**< Minute (0-59) */
    __u8 utc_sec;    /**< Second (0-60) */
    __u16 utc_millis;  /**< Milliseconds (0-999) */

    /** Validity Flags */
    bool heading_valid;     /**< True if course_deg_e5 is valid  */
    bool hdop_valid;        /**< True if hdop_x100 is valid  */
}__attribute__((packed));


#define NEO6M_GNSS_IOC_MAGIC 'N'
#define NEO6M_GNSS_IOC_GET_FIX _IOR(NEO6M_GNSS_IOC_MAGIC, 1, struct neo6m_gnss_fix)



#endif // NEO6M_GNSS_IOCTL_H