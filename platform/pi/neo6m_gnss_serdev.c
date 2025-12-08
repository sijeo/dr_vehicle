// SPDX-License-Identifier: GPL-2.0
/**
 * @file neo6m_gnss_serdev.c
 * @brief ublox NEO-6M GNSS NMEA driver (UART via serdev)
 * 
 * This driver receives variable-length, comma-seperated NMEA sentences over 
 * UART, tolerates missing fields (pre-fix), parses (RMC/GGA/VTG), and exposes
 * parsed values through sysfs and a character device.
 * 
 * Design Notes:
 * - Uses serdev receive_buf() with a line-accumlator for CR/LF-terminated NMEA.
 * - Robust field parsing: empty fields are allowed, numerical coversions checks.
 * - Locking: spinlock protects the latest fix snapshot; parsing happens in a 
 * worer to keep RX fast.
 * - Exposes /sys/class/neo6m_gnss/neo6m0/{lat, lon, alt_mm, speed_mmps, have_fix, utc, hdop_x100, course_deg_e5}
 * and /dev/neo6m_gnss for ioclt(N...GET_FIX).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/cdev.h>        // added
#include <linux/string.h>      // added

#include "neo6m_gnss_ioctl.h"

#define DRV_NAME "neo6m_gnss_serdev"
#define NMEA_MAX_LINE 128
#define RX_RING_SIZE 2048

/*******************Utilities ********************/

/**
 * @brief Verify the NMEA sentence checksum.
 *
 * NMEA sentences begin with '$', contain a payload, and end with '*XX'
 * where XX is the hexadecimal XOR of all characters between '$' and '*'.
 *
 * @param s   Pointer to the NMEA sentence (starting at '$').
 * @param len Length of the sentence (without CR/LF terminators).
 *
 * @return true if checksum is valid, false otherwise.
 */


static inline bool nmea_checksum_ok(const char *s, size_t len)
{
    /* s point to '$', len includes full line without CRLF. Expect *XX at end. */
    const char *star = memchr(s, '*', len);
    unsigned int sum = 0, got;
    size_t i;

    if (!star || s[0] != '$')
    {
        return false; // no star or not starting with '$'
    }

    /* Compute checksum  by XORing all the characters between '$' and '*' */
    for (i = 1; &s[i] < star; i++)
    {
        sum ^= (unsigned char)s[i];
    }

    /* Ensure there are atleast 2 hex digits after '*' 
    * (star + 3) points just past the checksum (2 digits + null terminator)
    * If the string ends before that, the sentence is malformed.
    */
   if (star + 3 > s + len)
    {
         return false;
    }

    /* Parse the two hex digits after '*' as an unsigned int */
    if (sscanf(star + 1, "%2x", &got ) != 1)
    {
        return false; // failed to parse
    }

    /* Return true  if computer XOR checksum matches the transmitted one */
    return (sum & 0xFF) == (got & 0xFF);

}

/**
 * @brief Safe string-to-long conversion with default.
 *
 * @param p   Input string (may be NULL or empty).
 * @param def Default value if conversion fails.
 *
 * @return Converted value or def on failure.
 */

static inline long strtol_safe(const char *p, long def)
{
    char *e;
    long v;

    if ( !p || !*p)
    {
        return def; // empty field
    }
    v = simple_strtol(p, &e, 10);
    if (e == p)
    {
        return def; // no conversion
    }
    return v;   
}

/**
 * @brief Convert decimal string to milli-units (×1000) with rounding.
 *
 * Example: "12.3456" -> 12346 (approx).
 *
 * @param p   Input decimal ASCII string.
 * @param def Default value if input is invalid.
 *
 * @return Value in milli-units or def on failure.
 */

static inline long strtod_milli(const char *p, long def )
{
    /* Convert decimal string to milli-units (x1000) with rounding */
    long s = 1, whole = 0, frac = 0, frac_digits = 0;
    const char *c = p;

    if (!p || !*p)
    {
        return def; // empty field
    }

    if ( *c == '-')
    {
        s = -1;
        c++;
    }
    else if (*c == '+')
    {
        c++;
    }

    while (isdigit(*c)) {
        whole = whole * 10 + (*c - '0');
        c++;
    }
    if (*c == '.')
    {
        c++;
        while (isdigit(*c) && frac_digits < 3) {
            frac = frac * 10 + (*c - '0');
            frac_digits++;
            c++;
        }
        while (isdigit(*c)) { // skip remaining digits
            c++;
        }
    }
    while (frac_digits++ < 3) { // scale to milli
        frac *= 10;
    }
    return s * (whole * 1000 + frac);
}

/**
 * @brief Convert NMEA latitude/longitude to degrees × 1e7.
 *
 * NMEA format:
 *   - Latitude:  ddmm.mmmm
 *   - Longitude: dddmm.mmmm
 *
 * @param val     NMEA ddmm.mmmm or dddmm.mmmm field.
 * @param hem     Hemisphere indicator ('N','S','E','W').
 * @param out_e7  Pointer to store result (degrees ×1e7).
 *
 * @return true on success, false on failure.
 */

static bool nmea_latlon_to_e7(const char *val, const char *hem, s32 *out_e7)
{
    long sign = 1;
    long milli = 0;
    long deg;
    size_t len;
    const char *dot;
    size_t mm_start;
    char deg_str[4] = {0};
    char mm_str[16] = {0};
    long deg_e7;
    long minutes_deg_milli;
    long minutes_e7;
    long total;

    if (!val || !*val || !hem || !*hem) {
        return false; // empty field
    }
    if (hem[0] == 'S' || hem[0] == 'W') {
        sign = -1;
    }

    /* Split degrees vs minutes: degrees are first 2 (lat) or 3 (lon) chars */
    len = strlen(val);
    if (len < 4)
        return false; // too short

    /* Find the decimal point to decide */
    dot = strchr(val, '.');
    mm_start = (len > 5) ? (len - 7) : 2; /* heuristic */
    /* Better: degrees are all chars before the last 7 mm.mmmm */
    if (dot && (dot - val >= 3)) {
        /* for lon: 3 deg digits */
        mm_start = (dot - val) - 2; /* xxmm.mmmm => mm start 2 before dot */
    }

    /* Copy degrees part */
    if (mm_start > sizeof(deg_str)) {
        if (mm_start > 3)
            return false; // too long
        strncpy(deg_str, val, mm_start);
    } else {
        strncpy(deg_str, val, mm_start);
    }

    /* Copy minutes part */
    strlcpy(mm_str, val + mm_start, sizeof(mm_str));

    deg = strtol_safe(deg_str, 0);
    milli = strtod_milli(mm_str, 0); /* minutes in milli-units */

    /* Convert ddmm.mmmm to degrees*1e7 */
    deg_e7 = deg * 10000000L;
    minutes_deg_milli = (milli + 30) / 60; /* rounded */
    minutes_e7 = minutes_deg_milli * 10000L;

    total = deg_e7 + minutes_e7;
    *out_e7 = (s32)(sign * total);
    return true;
}

/**
 * @brief Convert speed in knots to mm/s.
 *
 * 1 knot = 0.514444 m/s.
 *
 * @param knots Speed over ground in knots (ASCII).
 *
 * @return Speed in mm/s.
 */

static u32 knots_to_mmps_maybe(const char *knots)
{
    /* Speed over ground in knots => m/s * 1000*/
    /* 1 knot = 0.514444 m/s */
    long mps_milli = (strtod_milli(knots, 0) * 514444 + 500000) / 1000;
    if ( mps_milli < 0)
    {
        mps_milli = 0;
    }
    return (u32)mps_milli;
}

/******************* Driver Core ***************************/
struct neo6m_priv {
    struct device *dev;
    struct serdev_device *serdev;

    /* Rx line assembly */
    char line[NMEA_MAX_LINE];
    size_t line_len;

    /* Workqueue to parse completed lines */
    struct work_struct parse_work;
    /* Simple buffer to hand one line to  work context */
    char parse_buf[NMEA_MAX_LINE];
    size_t parse_len;

    /* Latest parses fix (protected )*/
    spinlock_t lock;
    struct neo6m_gnss_fix fix;

    /* Character device */
    dev_t devt;
    struct cdev cdev;       // embedded cdev
    struct class *cls;
    struct device *chardev; // created with device_create()

    /* sysfs kobj */
    struct kobject *kobj;
};

static struct of_device_id neo6m_of_match[] = {
    { .compatible = "ublox,neo-6m", },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, neo6m_of_match);

/**
 * @brief Hand a completed NMEA line to the worker for parsing.
 *
 * @param priv Driver private data.
 * @param s    Pointer to NMEA buffer.
 * @param len  Length of buffer.
 */
 static void neo6m_push_line_for_parse(struct neo6m_priv *priv, const char *s, size_t len)
 {
    if(len >= sizeof(priv->parse_buf))
    {
        len = sizeof(priv->parse_buf) - 1;
    }
    memcpy(priv->parse_buf, s, len);
    priv->parse_buf[len] = '\0';
    priv->parse_len = len;
    schedule_work(&priv->parse_work);
 }

 /**
 * @brief Parse UTC date/time from RMC fields.
 *
 * RMC fields:
 *   [1] = time hhmmss.sss
 *   [9] = date ddmmyy
 *
 * @param f      Pointer to fix structure to update.
 * @param fields Token array.
 * @param n      Number of tokens.
 */


  static void neo6m_parse_utc( struct neo6m_gnss_fix *f, char *const *fields, int n)
  {
    /* RMC: hhmmss.sss at fields[1], date ddmmyy at fields[9] */
    const char *t = (n > 1) ? fields[1] : NULL;
    const char *d = (n > 9) ? fields[9] : NULL;

    if ( t && *t ){
        long hh = strtol_safe(t, 0) / 10000;
        long mm = (strtol_safe(t, 0) / 100) % 100;
        long ss = strtol_safe(t, 0) % 100;
        const char *dot = strchr(t, '.');
        long ms = 0;
        if ( dot ){
            ms = strtol_safe(dot + 1, 0);
            while ( ms > 999) ms /= 10;
        }
        f->utc_hour = clamp_t(int, hh, 0, 23);
        f->utc_min = clamp_t(int, mm, 0, 59);
        f->utc_sec = clamp_t(int, ss, 0, 60);
        f->utc_millis = clamp_t(int, ms, 0, 999);
    }
    if ( d && *d) {
        long dd = strtol_safe(d, 0) / 10000;
        long mo = (strtol_safe(d, 0) / 100) % 100;
        long yy = strtol_safe(d, 0) % 100;
        f->utc_day = clamp_t(int, dd, 1, 31);
        f->utc_mon = clamp_t(int, mo, 1, 12);
        /* NMEA two digit year: 80..99 => 1980..1999, 00..79 => 2000..2079 */
        f->utc_year = (yy >= 80) ? (1900 + yy) : (2000 + yy);
    }
  }

  /**
 * @brief Parse a single NMEA line and update the fix.
 *
 * Handles:
 *   - $GPRMC / $GNRMC
 *   - $GPGGA / $GNGGA
 *   - $GPVTG / $GNVTG
 *
 * Empty tokens are tolerated. Checksum is verified before parsing.
 *
 * @param priv Driver private data.
 * @param s    Pointer to NMEA sentence buffer.
 * @param len  Length of sentence.
 */

   static void neo6m_parse_line(struct neo6m_priv *priv, const char *s, size_t len)
   {
    char *tmp, *p;
    char *tok[32];
    int ntok = 0;
    struct neo6m_gnss_fix newfix;

    if (!nmea_checksum_ok(s, len)) {
        dev_warn(priv->dev, "Bad checksum: %.*s\n", (int)len, s);
        return;
    }

    tmp = kstrdup(s, GFP_KERNEL);
    if (!tmp)
        return;

    /* Tokenize on ',' and strip trailing *cs */
    p = tmp;
    while ((ntok < ARRAY_SIZE(tok)) && (tok[ntok] = strsep(&p, ",")) != NULL)
        ntok++;

    /* Remove trailing *XX from last token if present */
    if (ntok > 0) {
        char *star = strchr(tok[ntok - 1], '*');
        if (star)
            *star = '\0';
    }

    /* Initialize from last fix */
    spin_lock_bh(&priv->lock);
    newfix = priv->fix;
    spin_unlock_bh(&priv->lock);

    if ((ntok > 0)&& (tok[0][0] == '$')){
        /* Identify sentence type */
        if ((!strncmp(tok[0]+1, "GPRMC", 5)) || (!strncmp(tok[0]+1, "GNRMC", 5)))
        {
            /**
             * RMC fields:
             * 1 = time, 2 = status A/V, 3=lat, 4=N/S, 5=lon, 6=E/W,
             * 7 = sog(knots), 8 = cog, 9 = date, ...
             */
            const char *status = (ntok > 2) ? tok[2] : "";
            bool active = (status && *status == 'A');
            if( active )
            {
                newfix.have_fix = 1;
            } else {
                newfix.have_fix = 0;
            }
            neo6m_parse_utc(&newfix, tok, ntok);

            /*Lat/Lon*/
            if( ntok > 5 )
            {
                s32 lat_e7, lon_e7;
                if ( nmea_latlon_to_e7(tok[3], tok[4], &lat_e7))
                {
                    newfix.lat_e7 = lat_e7;
                }
                if ( nmea_latlon_to_e7(tok[5], tok[6], &lon_e7))
                {
                    newfix.lon_e7 = lon_e7;
                }
            }

            /*Speed (Knots)*/
            if( ntok > 7 )
            {
                newfix.speed_mmps = knots_to_mmps_maybe(tok[7]);
            }

            if( ntok > 8 && tok[8] && *tok[8] ){
                long hdg_milli = strtod_milli(tok[8], 0); /* deg x1000 */
                newfix.course_deg_e5 = (s32)(hdg_milli * 100);
                newfix.heading_valid = true;
            } else {
                newfix.heading_valid = false;
            }
        }
        else if( (!strncmp(tok[0] + 1, "GPGGA", 5)) || (!strncmp(tok[0] + 1, "GNGGA", 5)))
        {
            /**
             * GGA fields:
             * 1= time, 2= lat, 3 = N/S, 4 = lon, 5 = E/W, 6=fix(0 = no),
             * 7 = numstats, 8 = hdop, 9 = alt(MSL), 10 = M, 11= geoid sep...
             */
            long fixq = ((ntok > 6) && (*tok[6])) ? strtol_safe(tok[6], 0) : 0;
            if ( fixq > 0 ) newfix.have_fix = 1;
            /* lat/lon */
            if( ntok > 5 ){
                s32 lat_e7, lon_e7;
                if( nmea_latlon_to_e7(tok[2], tok[3], &lat_e7))
                { newfix.lat_e7 = lat_e7;}
                if( nmea_latlon_to_e7(tok[4], tok[5], &lon_e7))
                { newfix.lon_e7 = lon_e7;}
            }
            /* altitude in meters -> mm*/
            if( (ntok > 9) && (tok[9]) && (*tok[9]))
            {
                long alt_milli = strtod_milli(tok[9], 0);
                newfix.alt_mm = (s32)(alt_milli * 1); /* milli-meters already */
            }

            /* HDOP (field 8) as x100 */
            if( ntok > 8 && tok[8] && *tok[8]) {
                long hdop_milli = strtod_milli(tok[8], 0); /* x1000 */
                newfix.hdop_x100 = (u16)((hdop_milli + 5) / 10); /* x100 */
                newfix.hdop_valid = true;
            }

            /* UTC from field[1] if not set by RMC*/
            if ((ntok > 1) && (tok[1]) && (*tok[1])) {
                const char *t = tok[1];
                long hh = strtol_safe(t, 0) / 10000;
                long mm = (strtol_safe(t, 0) / 100) % 100;
                long ss = strtol_safe(t, 0) % 100;
                const char *dot = strchr(t, '.');
                long ms = 0;

                if(dot) {
                    ms = strtol_safe(dot + 1, 0);
                    while (ms > 999) ms /= 10;
                }
                newfix.utc_hour = clamp_t(int, hh, 0, 23);
                newfix.utc_min = clamp_t(int, mm, 0, 59);
                newfix.utc_sec = clamp_t(int, ss, 0, 60);
                newfix.utc_millis = clamp_t(int, ms, 0, 999);
            }
        } 
        else if ( (!strncmp(tok[0] + 1, "GPVTG", 5)) || (!strncmp(tok[0] + 1, "GNVTG", 5))) {
            /**
             * VTG fields:
             * 5 = speed(knots), 7= speed(km/h)- we prefer knots (field 5)
             */
            if(!newfix.heading_valid) {
                if( ntok >1 && tok[1] && *tok[1]) {
                long hdg_milli = strtod_milli(tok[1], 0);
                newfix.course_deg_e5 = (s32)(hdg_milli * 100);
                newfix.heading_valid = true;
               } else {
                newfix.heading_valid = false;
               }

            }
            

            if( (ntok > 5) &&( tok[5]) && (*tok[5])){
                newfix.speed_mmps = knots_to_mmps_maybe(tok[5]);
            }
        }        
        
    }
    newfix.monotonic_ns = ktime_get_boottime_ns();

    spin_lock_bh(&priv->lock);
    priv->fix = newfix;
    spin_unlock_bh(&priv->lock);

    kfree(tmp);
}

static void neo6m_parse_workfn( struct work_struct *w)
{
    struct neo6m_priv *priv = container_of(w, struct neo6m_priv, parse_work);
    neo6m_parse_line(priv, priv->parse_buf, priv->parse_len);
}

/****************************serdev ops*************************/

static int neo6m_receive( struct serdev_device *serdev, const unsigned char *buf, size_t count)
{
    struct neo6m_priv *priv = serdev_device_get_drvdata(serdev);
    size_t i;

    for (i = 0; i < count; i++)
    {
        char c = buf[i];
        if ( c == '\r')
        {
            continue; // ignore
        }
        if ( c == '\n')
        {
            /* Completed line */
            if ((priv->line_len >= 1) && (priv->line[0] == '$'))
            {
                /* Valid line, push for parsing */
                neo6m_push_line_for_parse(priv, priv->line, priv->line_len);
            }
            priv->line_len = 0;
            continue;
        }
        if( priv->line_len < (NMEA_MAX_LINE - 1))
        {
            priv->line[priv->line_len++] = c;
        }
        else
        {
            /* Line too long, drop i.e. overflow -> reset*/
            priv->line_len = 0;
        }
        
    }
    return count;
}

static void neo6m_write_wakeup(struct serdev_device *serdev)
{
    /* We do not use write */
}

static const struct serdev_device_ops neo6m_serdev_ops = {
    .receive_buf = neo6m_receive,
    .write_wakeup = neo6m_write_wakeup,
};

/**************sysfs******************/

static ssize_t show_lat(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    s32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix.lat_e7;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(lat, 0444, show_lat, NULL);

static ssize_t show_lon(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    s32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix.lon_e7;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(lon, 0444, show_lon, NULL);

static ssize_t show_alt( struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    s32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix.alt_mm;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(alt_mm, 0444, show_alt, NULL);

static ssize_t show_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix.speed_mmps;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(speed_mmps, 0444, show_speed, NULL);

static ssize_t show_have_fix(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    s8 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix.have_fix;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(have_fix, 0444, show_have_fix, NULL);

static ssize_t show_utc(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    struct neo6m_gnss_fix f;
    spin_lock_irqsave(&p->lock, flags);
    f = p->fix;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ\n",
        f.utc_year, f.utc_mon, f.utc_day,
        f.utc_hour, f.utc_min, f.utc_sec, f.utc_millis);
}
static DEVICE_ATTR(utc, 0444, show_utc, NULL);

static ssize_t show_hdop(struct device *dev, struct device_attribute *attr, char *buf){
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u16 v;

    spin_lock_irqsave(&p->lock, flags);
    v = p->fix.hdop_x100;
    spin_unlock_irqrestore(&p->lock, flags);

    return scnprintf(buf, PAGE_SIZE, "%u\n", v);

}
static DEVICE_ATTR(hdop_x100, 0444, show_hdop, NULL);

static ssize_t show_course(struct device *dev, struct device_attribute *attr, char *buf){
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;

    s32 v;

    spin_lock_irqsave(&p->lock, flags);
    v = p->fix.course_deg_e5;
    spin_unlock_irqrestore(&p->lock, flags);

    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(course_deg_e5, 0444, show_course, NULL);

static struct attribute *neo6m_attrs[] = {
    &dev_attr_lat.attr,
    &dev_attr_lon.attr,
    &dev_attr_alt_mm.attr,
    &dev_attr_speed_mmps.attr,
    &dev_attr_have_fix.attr,
    &dev_attr_utc.attr,
    &dev_attr_hdop_x100.attr,
    &dev_attr_course_deg_e5.attr,
    NULL,
};

static const struct attribute_group neo6m_attr_group = {
    .attrs = neo6m_attrs,
};

/******************** Char device ********************/
static long neo6m_chr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct neo6m_priv *p = file->private_data;
    struct neo6m_gnss_fix f;
    unsigned long flags;

    if ( cmd != NEO6M_GNSS_IOC_GET_FIX)
    {
        return -ENOTTY;
    }

    spin_lock_irqsave(&p->lock, flags);
    f = p->fix;
    spin_unlock_irqrestore(&p->lock, flags);

    if (copy_to_user((void __user *)arg, &f, sizeof(f)))
        return -EFAULT;

    return 0;
}

static int neo6m_chr_open(struct inode *inode, struct file *file)
{
    struct neo6m_priv *p = container_of(inode->i_cdev, struct neo6m_priv, cdev);
    file->private_data = p;
    return 0;
}

static int neo6m_chr_release(struct inode *inode, struct file *file)
{
    return 0;
}

static const struct file_operations neo6m_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = neo6m_chr_ioctl,
    .open = neo6m_chr_open,
    .release = neo6m_chr_release,
    #ifdef CONFIG_COMPAT
    .compat_ioctl = neo6m_chr_ioctl,
    #endif
};

/**********************Probe/Remove***********************/
/**
 * neo6m_probe() - serdev probe
 * @serdev: serdev device
 * 
 * Configures baudrate/termios, register sysfs + chrdev and primes RX Path.
 * 
 * Returns 0 on success, negative errno on failure.
 */
static int neo6m_probe(struct serdev_device *serdev)
{
    struct device *dev = &serdev->dev;
    struct neo6m_priv *p;
    int ret;

    p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
    if (!p)
    {
        return -ENOMEM;
    }

    p->dev = dev;
    p->serdev = serdev;
    memset(&p->fix, 0, sizeof(p->fix));
    p->fix.heading_valid = false;
    p->fix.hdop_valid = false;
    spin_lock_init(&p->lock);
    INIT_WORK(&p->parse_work, neo6m_parse_workfn);

    serdev_device_set_drvdata(serdev, p);
    serdev_device_set_client_ops(serdev, &neo6m_serdev_ops);

    /* Configure UART (typical NEO-6M default 9600-8N1)*/
    ret = serdev_device_open(serdev);
    if (ret)
    {
        dev_err(dev, "Failed to open serdev device\n");
        return ret; 
    }
    ret = serdev_device_set_baudrate(serdev, 9600);
    if (ret == 0)
    {               
        dev_err(dev, "Failed to set baudrate\n");
        goto err_close;
    }
    serdev_device_set_flow_control(serdev, false);
    /* 8N1 is default */

    /* Create class */
    p->cls = class_create(THIS_MODULE, "neo6m_gnss");
    if (IS_ERR(p->cls))
    {
        dev_err(dev, "Failed to create class\n");
        ret = PTR_ERR(p->cls);
        goto err_close;
    }

    /* Allocate devt, register cdev, create device node */
    ret = alloc_chrdev_region(&p->devt, 0, 1, NEO6M_GNSS_CHARDEV_NAME);
    if (ret) {
        dev_err(dev, "alloc_chrdev_region failed: %d\n", ret);
        goto err_class;
    }
    cdev_init(&p->cdev, &neo6m_fops);
    p->cdev.owner = THIS_MODULE;
    ret = cdev_add(&p->cdev, p->devt, 1);
    if (ret) {
        dev_err(dev, "cdev_add failed: %d\n", ret);
        goto err_unreg_devt;
    }
    p->chardev = device_create(p->cls, dev, p->devt, p, "neo6m0");
    if (IS_ERR(p->chardev)) {
        ret = PTR_ERR(p->chardev);
        dev_err(dev, "device_create failed: %d\n", ret);
        goto err_cdev_del;
    }
    dev_set_drvdata(p->chardev, p);

    ret = sysfs_create_group(&p->chardev->kobj, &neo6m_attr_group);
    if (ret)
    {
        dev_err(dev, "Failed to create sysfs group\n");
        goto err_dev_destroy;
    }
    dev_info(dev, "NEO-6M GNSS driver probed and ready @9600 8N1\n");
    return 0;

err_dev_destroy:
    device_destroy(p->cls, p->devt);
err_cdev_del:
    cdev_del(&p->cdev);
err_unreg_devt:
    unregister_chrdev_region(p->devt, 1);
 err_class:
     class_destroy(p->cls);
 err_close:
     serdev_device_close(serdev);
     return ret;
}


/**
 * neo6m_remove() - serdev remove
 */
static void neo6m_remove(struct serdev_device *serdev)
{
    struct neo6m_priv *p = serdev_device_get_drvdata(serdev);

    sysfs_remove_group(&p->chardev->kobj, &neo6m_attr_group);
    device_destroy(p->cls, p->devt);
    cdev_del(&p->cdev);
    unregister_chrdev_region(p->devt, 1);
    class_destroy(p->cls);
    serdev_device_close(serdev);
    cancel_work_sync(&p->parse_work);
    serdev_device_close(serdev);
    dev_info(p->dev, "NEO-6M GNSS driver removed\n");
}

static struct serdev_device_driver neo6m_driver = {
    .driver = {
        .name = DRV_NAME,
        .of_match_table = neo6m_of_match,
    },
    .probe = neo6m_probe,
    .remove = neo6m_remove,
};
module_serdev_device_driver(neo6m_driver);

MODULE_DESCRIPTION("ublox NEO-6M GNSS NMEA driver (UART via serdev)");
MODULE_AUTHOR("Sijeo Philip <sijeo80@gmail.com>");
MODULE_LICENSE("GPL v2");