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
#define NMEA_PARSE_QUEUE_MAX 32

/**
 * struct nmea_line - one completed NMEA sentence waiting to be parsed.
 *
 * Allocated with GFP_ATOMIC in the serdev receive callback, freed by the
 * worker after parsing.  Kept on parse_queue in FIFO order.
 */
struct nmea_line {
    struct list_head node;
    size_t           len;
    char             buf[NMEA_MAX_LINE];
};

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
    long sign;
    int  exp_deg_digits;          /* 2 for N/S (latitude), 3 for E/W (longitude) */
    int  dot_pos;
    const char *p;
    const char *dot;
    char deg_str[4] = {0};        /* holds up to 3 degree digits + NUL */
    long deg;
    long mm_whole = 0;            /* integer minutes */
    long mm_frac  = 0;            /* fractional minutes scaled to 1e-7 */
    int  frac_dig = 0;
    long minutes_e7_raw;
    long degrees_e7_from_min;
    long total;

    /* 1. Reject NULL / empty inputs */
    if (!val || !*val || !hem || !*hem || !out_e7)
        return false;

    /* 2. Validate hemisphere — derive sign and expected degree-digit count.
     *    N/S → latitude → 2 degree digits (ddmm.mmmm).
     *    E/W → longitude → 3 degree digits (dddmm.mmmm). */
    switch (hem[0]) {
    case 'N': sign =  1; exp_deg_digits = 2; break;
    case 'S': sign = -1; exp_deg_digits = 2; break;
    case 'E': sign =  1; exp_deg_digits = 3; break;
    case 'W': sign = -1; exp_deg_digits = 3; break;
    default:
        return false;
    }

    /* 3. Validate all characters in val: only ASCII digits and at most one '.'.
     *    Reject empty string, sign characters, letters, or multiple dots. */
    dot = NULL;
    for (p = val; *p; p++) {
        if (*p == '.') {
            if (dot)
                return false;   /* second decimal point */
            dot = p;
        } else if (!isdigit((unsigned char)*p)) {
            return false;       /* non-digit, non-dot character */
        }
    }

    /* 4. Determine position of decimal point (or end-of-string if absent).
     *    NMEA format: <deg_digits><mm>[.<frac>]
     *    The two characters immediately before the decimal (or end) are the
     *    integer minute digits. Everything before them is the degree field.
     *
     *    Strict requirement: dot_pos must equal exp_deg_digits + 2.
     *    Examples: "4807.038" N  → dot_pos=4, exp=2+2=4 ✓
     *              "01230.456" E → dot_pos=5, exp=3+2=5 ✓
     *              "1260.000" N  → dot_pos=4, deg="12", min=60 → range fail
     *              "18100.00" E  → dot_pos=5, deg="181" → range fail          */
    dot_pos = (int)(dot ? (dot - val) : (long)(p - val));
    if (dot_pos != exp_deg_digits + 2)
        return false;

    /* 5. Parse degree portion (first exp_deg_digits characters) — safe: buffer
     *    is 4 bytes and exp_deg_digits is at most 3. */
    memcpy(deg_str, val, (size_t)exp_deg_digits);
    deg_str[exp_deg_digits] = '\0';
    deg = simple_strtol(deg_str, NULL, 10);

    /* 6. Validate degree range */
    if (exp_deg_digits == 2) {
        if (deg < 0 || deg > 90)       /* latitude  0..90  */
            return false;
    } else {
        if (deg < 0 || deg > 180)      /* longitude 0..180 */
            return false;
    }

    /* 7. Parse integer minutes: the two digits at [exp_deg_digits .. dot_pos-1] */
    for (p = val + exp_deg_digits; p < val + dot_pos; p++)
        mm_whole = mm_whole * 10 + (*p - '0');

    /* 8. Validate integer minutes: 0 ≤ mm_whole < 60 */
    if (mm_whole < 0 || mm_whole >= 60)
        return false;

    /* 9. Parse fractional minutes — up to 7 digits after the decimal point */
    if (dot) {
        for (p = dot + 1; isdigit((unsigned char)*p) && frac_dig < 7; p++, frac_dig++)
            mm_frac = mm_frac * 10 + (*p - '0');
    }
    /* Pad to exactly 7 fractional digits (units of 1e-7 minutes) */
    while (frac_dig < 7) {
        mm_frac *= 10;
        frac_dig++;
    }

    /* 10. Convert minutes → degrees × 1e7.  All values fit in 32-bit long on
     *     ARM32 (no 64-bit intermediates):
     *       minutes_e7_raw  max = 59*10_000_000 + 9_999_999 = 599_999_999 < 2^31
     *       deg*10_000_000  max = 180*10_000_000 = 1_800_000_000           < 2^31
     *       total           max ≈ 1_810_000_000                            < 2^31 */
    minutes_e7_raw      = mm_whole * 10000000L + mm_frac;
    degrees_e7_from_min = (minutes_e7_raw + 30L) / 60L;
    total               = deg * 10000000L + degrees_e7_from_min;

    *out_e7 = (s32)(sign * total);
    return true;
}

/**
 * @brief Convert NMEA ground-speed-in-knots ASCII to millimetres per second.
 *
 * Unit chain (kept explicit to prevent factor-of-1000 errors):
 *   1 knot = 0.514444 m/s = 514.444 mm/s
 *   strtod_milli() returns the input in milli-knots (knots x 1000), so:
 *
 *       speed_mm_s = milli_knots * 514444 / 1_000_000
 *
 * Verified against the GNSS spec examples:
 *   "0.50"  knots ->  ~257 mm/s
 *   "2.744" knots -> ~1412 mm/s
 *   "10.0"  knots -> ~5144 mm/s
 *   "29.05" knots -> ~14945 mm/s
 *
 * @param knots Speed over ground in knots (NMEA ASCII field).
 *
 * @return Speed in millimetres per second. Returns 0 for empty, invalid,
 *         negative, or non-finite input. Clamped at U32_MAX (an impossibly
 *         high speed) to avoid wraparound.
 */
static u32 knots_to_mmps_maybe(const char *knots)
{
    /* Pure 32-bit implementation — avoids 64-bit divides that would
     * require libgcc helpers (__aeabi_ldivmod / __udivdi3) which the
     * kernel does not link on 32-bit ARM.
     *
     * Unit chain:
     *   strtod_milli() returns the input in milli-knots (knots × 1000).
     *   1 knot = 0.514444 m/s = 514.444 mm/s.
     *   mm/s = mk × 514444 / 1_000_000.
     *
     * Overflow avoidance:
     *   mk × 514444 overflows a u32 for mk > ~8348 (≈ 8.3 knots).
     *   Split mk = 1000·q + r with q = mk/1000, r = mk%1000, then:
     *       mm/s = q × 514444 / 1000   +   r × 514444 / 1_000_000
     *   The two intermediate products fit in u32 for any plausible q
     *   (we clamp to 1000 knots beforehand). Both divisions are u32 /
     *   u32 and compile to a single native ARM instruction — no helper.
     */
    long parsed;
    u32  mk, q, r, a, b;

    parsed = strtod_milli(knots, 0);
    if (parsed <= 0)
        return 0;                                     /* empty / negative / NaN */

    /* Saturation safety: 1000 knots = ~514 m/s, well beyond any vehicle. */
    if (parsed > 1000000L)
        return U32_MAX;

    mk = (u32)parsed;
    q  = mk / 1000U;                                  /* integer-knot part */
    r  = mk % 1000U;                                  /* milli-fractional  */

    a = (q * 514444U + 500U)    / 1000U;              /* major: ~514 mm/s/knot   */
    b = (r * 514444U + 500000U) / 1000000U;           /* minor: < 1 mm/s correction */

    return a + b;
}
/******************* Driver Core ***************************/
struct neo6m_priv {
    struct device *dev;
    struct serdev_device *serdev;

    /* Rx line assembly */
    char line[NMEA_MAX_LINE];
    size_t line_len;

    /* Workqueue + FIFO queue of completed NMEA lines */
    struct work_struct parse_work;
    struct list_head   parse_queue;       /* protected by qlock */
    spinlock_t         qlock;
    unsigned int       parse_queue_depth;   /* current depth */
    unsigned int       parse_queue_dropped; /* lines lost due to full queue or OOM */

    /* Parser diagnostics — read from sysfs for field health checks.
     * All fields below are protected by lock except queue_drop_count (qlock). */
    u32 nmea_rx_lines;              /* sentences reaching the parser */
    u32 nmea_bad_checksum;          /* sentences rejected by checksum */
    u32 nmea_parse_ok;              /* recognised sentences (RMC/GGA/VTG) with good checksum */
    u32 rmc_count;                  /* RMC sentences seen */
    u32 gga_count;                  /* GGA sentences seen */
    u32 vtg_count;                  /* VTG sentences seen */
    u32 rmc_invalid_count;          /* RMC sentences with status != 'A' */
    u32 gga_no_fix_count;           /* GGA sentences with fixq == 0 */
    u32 vtg_ignored_no_fix_count;   /* VTG sentences skipped due to no fix / mode N */
    u32 latlon_parse_fail_count;    /* lat or lon field failed nmea_latlon_to_e7() */
    u32 speed_parse_fail_count;     /* speed field present but failed to parse */
    u32 queue_drop_count;           /* queue overflow / OOM drops (mirrors parse_queue_dropped, protected by qlock) */
    u32 pos_update_count;           /* times pos_seq was incremented */
    u32 vel_update_count;           /* times vel_seq was incremented */

    /* Latest fix snapshot (v2, protected by lock) */
    spinlock_t lock;
    struct neo6m_gnss_fix_v2 fix_v2;
    /* Last accepted RMC-A UTC epoch (hour*3600+min*60+sec).
     * Initialised to U32_MAX so the first valid RMC always triggers epoch_changed. */
    u32 last_utc_epoch;

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
    struct nmea_line *nl;
    unsigned long flags;

    if (len >= NMEA_MAX_LINE)
        len = NMEA_MAX_LINE - 1;

    /* Allocate outside the lock — GFP_ATOMIC is safe in any callback context */
    nl = kmalloc(sizeof(*nl), GFP_ATOMIC);
    if (!nl) {
        spin_lock_irqsave(&priv->qlock, flags);
        priv->parse_queue_dropped++;
        priv->queue_drop_count++;
        spin_unlock_irqrestore(&priv->qlock, flags);
        dev_warn_ratelimited(priv->dev, "NMEA: kmalloc failed, line dropped\n");
        return;
    }

    memcpy(nl->buf, s, len);
    nl->buf[len] = '\0';
    nl->len = len;

    spin_lock_irqsave(&priv->qlock, flags);
    if (priv->parse_queue_depth >= NMEA_PARSE_QUEUE_MAX) {
        priv->parse_queue_dropped++;
        priv->queue_drop_count++;
        spin_unlock_irqrestore(&priv->qlock, flags);
        kfree(nl);
        dev_warn_ratelimited(priv->dev,
            "NMEA: parse queue full, line dropped (total dropped: %u)\n",
            priv->parse_queue_dropped);
        return;
    }
    list_add_tail(&nl->node, &priv->parse_queue);
    priv->parse_queue_depth++;
    spin_unlock_irqrestore(&priv->qlock, flags);

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


  static void neo6m_parse_utc( struct neo6m_gnss_fix_v2 *f, char *const *fields, int n)
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

/**
 * nmea_type_is() - match sentence type by the 3-character message code.
 *
 * Accepts any talker ID (GP, GN, BD, GB, GA, GL, …) by comparing only
 * characters [3..5] of the sentence ID field (tok[0]).
 *
 * @sentence_id: tok[0] including the leading '$', e.g. "$GPRMC".
 * @type:        3-character type string, e.g. "RMC".
 *
 * Returns true when sentence_id[3..5] == type[0..2].
 */
static bool nmea_type_is(const char *sentence_id, const char *type)
{
    /* Need at least '$' + 2 talker chars + 3 type chars = 6 */
    if (!sentence_id || strlen(sentence_id) < 6)
        return false;
    return sentence_id[3] == type[0] &&
           sentence_id[4] == type[1] &&
           sentence_id[5] == type[2];
}

   static void neo6m_parse_line(struct neo6m_priv *priv, const char *s, size_t len)
   {
    char *tmp = NULL;
    char *p;
    char *tok[32];
    int  ntok = 0;
    struct neo6m_gnss_fix_v2 newfix;
    u32  saved_utc_epoch;
    s32  old_lat_e7, old_lon_e7, old_speed, old_course;
    bool pos_updated   = false;
    bool vel_updated   = false;
    bool epoch_changed = false;
    bool write_fix     = false;
    u32  new_utc_epoch = 0;
    u32  now;

    /* Per-call stat deltas — applied under lock at done: */
    u32 d_badcs   = 0;
    u32 d_ok      = 0;
    u32 d_rmc     = 0, d_gga     = 0, d_vtg     = 0;
    u32 d_rmc_inv = 0, d_gga_nof = 0, d_vtg_ign = 0;
    u32 d_llfail  = 0, d_spdfail = 0;
    u32 d_pos     = 0, d_vel     = 0;

    if (!nmea_checksum_ok(s, len)) {
        dev_warn(priv->dev, "Bad checksum: %.*s\n", (int)len, s);
        d_badcs = 1;
        goto done;
    }

    tmp = kstrdup(s, GFP_KERNEL);
    if (!tmp)
        return;

    p = tmp;
    while ((ntok < ARRAY_SIZE(tok)) && (tok[ntok] = strsep(&p, ",")) != NULL)
        ntok++;

    if (ntok > 0) {
        char *star = strchr(tok[ntok - 1], '*');
        if (star)
            *star = '\0';
    }

    /* Snapshot latest fix and epoch marker under lock */
    spin_lock_bh(&priv->lock);
    newfix           = priv->fix_v2;
    saved_utc_epoch  = priv->last_utc_epoch;
    spin_unlock_bh(&priv->lock);

    /* Save pre-parse position/velocity for change detection */
    old_lat_e7 = newfix.lat_e7;
    old_lon_e7 = newfix.lon_e7;
    old_speed  = newfix.speed_mmps;
    old_course = newfix.course_deg_e5;

    if (ntok <= 0 || tok[0][0] != '$')
        goto done;

    if (nmea_type_is(tok[0], "RMC"))
    {
        /**
         * RMC: 1=time 2=status 3=lat 4=N/S 5=lon 6=E/W 7=sog 8=cog 9=date
         * Only status='A' provides a valid navigation solution.
         */
        const char *status = (ntok > 2) ? tok[2] : "";
        bool active = (status && *status == 'A');

        d_rmc = 1;

        if (active) {
            /* Detect new GNSS epoch via UTC second change */
            const char *t = (ntok > 1) ? tok[1] : NULL;
            if (t && *t) {
                long hh  = strtol_safe(t, 0) / 10000;
                long mmt = (strtol_safe(t, 0) / 100) % 100;
                long ss  = strtol_safe(t, 0) % 100;
                new_utc_epoch = (u32)(hh * 3600 + mmt * 60 + ss);
                if (new_utc_epoch != saved_utc_epoch)
                    epoch_changed = true;
            }

            newfix.have_fix = 1;
            neo6m_parse_utc(&newfix, tok, ntok);

            if (ntok > 5) {
                s32 lat_e7, lon_e7;
                bool lat_ok = nmea_latlon_to_e7(tok[3], tok[4], &lat_e7);
                bool lon_ok = nmea_latlon_to_e7(tok[5], tok[6], &lon_e7);
                if (lat_ok && lon_ok) {
                    newfix.lat_e7 = lat_e7;
                    newfix.lon_e7 = lon_e7;
                    pos_updated = true;
                } else {
                    d_llfail++;
                }
            }

            if (ntok > 7 && tok[7] && *tok[7]) {
                long mknots = strtod_milli(tok[7], -1);
                if (mknots < 0) {
                    d_spdfail++;
                } else {
                    newfix.speed_mmps = knots_to_mmps_maybe(tok[7]);
                    vel_updated = true;
                }
            }

            if (ntok > 8 && tok[8] && *tok[8]) {
                long hdg_milli = strtod_milli(tok[8], 0);
                newfix.course_deg_e5 = (s32)(hdg_milli * 100);
                newfix.heading_valid = true;
                vel_updated = true;
            } else {
                newfix.heading_valid = false;
            }
        } else {
            d_rmc_inv = 1;
            newfix.have_fix      = 0;
            newfix.heading_valid = false;
            newfix.speed_mmps    = 0;
            newfix.pos_valid     = 0;
            newfix.vel_valid     = 0;
            dev_dbg(priv->dev, "RMC status V — fix cleared\n");
        }
    }
    else if (nmea_type_is(tok[0], "GGA"))
    {
        /**
         * GGA: 1=time 2=lat 3=N/S 4=lon 5=E/W 6=fixq 7=nsats 8=hdop 9=alt
         * fixq==0 means no position fix.
         */
        long fixq = ((ntok > 6) && (*tok[6])) ? strtol_safe(tok[6], 0) : 0;

        d_gga = 1;

        if (fixq > 0) {
            newfix.have_fix = 1;

            if (ntok > 5) {
                s32 lat_e7, lon_e7;
                bool lat_ok = nmea_latlon_to_e7(tok[2], tok[3], &lat_e7);
                bool lon_ok = nmea_latlon_to_e7(tok[4], tok[5], &lon_e7);
                if (lat_ok && lon_ok) {
                    newfix.lat_e7 = lat_e7;
                    newfix.lon_e7 = lon_e7;
                    pos_updated = true;
                } else {
                    d_llfail++;
                }
            }

            if ((ntok > 9) && tok[9] && *tok[9]) {
                long alt_milli = strtod_milli(tok[9], 0);
                newfix.alt_mm = (s32)alt_milli;
            }

            if (ntok > 8 && tok[8] && *tok[8]) {
                long hdop_milli = strtod_milli(tok[8], 0);
                newfix.hdop_x100 = (u16)((hdop_milli + 5) / 10);
                newfix.hdop_valid = true;
            }

            if ((ntok > 1) && tok[1] && *tok[1]) {
                const char *t = tok[1];
                long hh  = strtol_safe(t, 0) / 10000;
                long mm  = (strtol_safe(t, 0) / 100) % 100;
                long ss  = strtol_safe(t, 0) % 100;
                const char *dot = strchr(t, '.');
                long ms = 0;
                if (dot) {
                    ms = strtol_safe(dot + 1, 0);
                    while (ms > 999) ms /= 10;
                }
                newfix.utc_hour   = clamp_t(int, hh, 0, 23);
                newfix.utc_min    = clamp_t(int, mm, 0, 59);
                newfix.utc_sec    = clamp_t(int, ss, 0, 60);
                newfix.utc_millis = clamp_t(int, ms, 0, 999);
            }
        } else {
            d_gga_nof = 1;
            newfix.have_fix      = 0;
            newfix.heading_valid = false;
            newfix.speed_mmps    = 0;
            newfix.pos_valid     = 0;
            newfix.vel_valid     = 0;
            /* HDOP is geometric — still informative without a position fix */
            if (ntok > 8 && tok[8] && *tok[8]) {
                long hdop_milli = strtod_milli(tok[8], 0);
                newfix.hdop_x100 = (u16)((hdop_milli + 5) / 10);
                newfix.hdop_valid = true;
            }
            dev_dbg(priv->dev, "GGA fixq=0 — position invalidated\n");
        }
    }
    else if (nmea_type_is(tok[0], "VTG"))
    {
        /**
         * VTG: 1=cog-true 5=speed-knots 9=mode(N=no-fix)
         * Relies on have_fix from the preceding RMC/GGA in the same epoch.
         */
        const char *vtg_mode  = (ntok > 9 && tok[9]) ? tok[9] : NULL;
        bool        vtg_no_fix = (vtg_mode && *vtg_mode == 'N');

        d_vtg = 1;

        if (!newfix.have_fix || vtg_no_fix) {
            d_vtg_ign = 1;
            newfix.heading_valid = false;
            newfix.speed_mmps    = 0;
            dev_dbg(priv->dev, "VTG ignored: %s\n",
                    vtg_no_fix ? "mode=N" : "no valid fix");
        } else {
            if (ntok > 1 && tok[1] && *tok[1]) {
                long hdg_milli = strtod_milli(tok[1], 0);
                newfix.course_deg_e5 = (s32)(hdg_milli * 100);
                newfix.heading_valid = true;
                vel_updated = true;
            } else {
                newfix.heading_valid = false;
            }

            if (ntok > 5 && tok[5] && *tok[5]) {
                long mknots = strtod_milli(tok[5], -1);
                if (mknots < 0) {
                    d_spdfail++;
                } else {
                    newfix.speed_mmps = knots_to_mmps_maybe(tok[5]);
                    vel_updated = true;
                }
            }
        }
    }
    else {
        /* Unrecognised sentence (GSV, GSA, …) — silently ignore */
        goto done;
    }

    /* ---- Sequence counter updates (only for recognised sentences) ---- */
    d_ok = 1;
    write_fix = true;

    now = jiffies_to_msecs(jiffies);
    newfix.monotonic_ms = now;
    newfix.nmea_seq++;

    /* pos_seq: fresh lat/lon when have_fix AND (values changed OR new epoch) */
    if (newfix.have_fix && pos_updated &&
        (newfix.lat_e7 != old_lat_e7 || newfix.lon_e7 != old_lon_e7 || epoch_changed)) {
        newfix.pos_seq++;
        newfix.pos_monotonic_ms = now;
        newfix.pos_valid = 1;
        d_pos = 1;
    }

    /* vel_seq: fresh speed/course when have_fix AND (values changed OR new epoch) */
    if (newfix.have_fix && vel_updated &&
        (newfix.speed_mmps != old_speed || newfix.course_deg_e5 != old_course || epoch_changed)) {
        newfix.vel_seq++;
        newfix.vel_monotonic_ms = now;
        newfix.vel_valid = 1;
        d_vel = 1;
    }

    /* Clear validity flags when fix is lost */
    if (!newfix.have_fix) {
        newfix.pos_valid = 0;
        newfix.vel_valid = 0;
    }

    /* fix_seq: new RMC-A epoch with valid fix */
    if (epoch_changed && newfix.have_fix)
        newfix.fix_seq++;

    newfix.stale = (u8)(!(pos_updated || vel_updated));

done:
    spin_lock_bh(&priv->lock);
    priv->nmea_rx_lines            += 1;
    priv->nmea_bad_checksum        += d_badcs;
    priv->nmea_parse_ok            += d_ok;
    priv->rmc_count                += d_rmc;
    priv->gga_count                += d_gga;
    priv->vtg_count                += d_vtg;
    priv->rmc_invalid_count        += d_rmc_inv;
    priv->gga_no_fix_count         += d_gga_nof;
    priv->vtg_ignored_no_fix_count += d_vtg_ign;
    priv->latlon_parse_fail_count  += d_llfail;
    priv->speed_parse_fail_count   += d_spdfail;
    priv->pos_update_count         += d_pos;
    priv->vel_update_count         += d_vel;
    if (write_fix) {
        priv->fix_v2 = newfix;
        if (epoch_changed)
            priv->last_utc_epoch = new_utc_epoch;
    }
    spin_unlock_bh(&priv->lock);

    kfree(tmp);
}

static void neo6m_parse_workfn(struct work_struct *w)
{
    struct neo6m_priv *priv = container_of(w, struct neo6m_priv, parse_work);

    /* Drain all queued lines in FIFO order.  Release qlock between each item
     * so the receive path can continue enqueuing without stalling. */
    for (;;) {
        struct nmea_line *nl;
        unsigned long flags;

        spin_lock_irqsave(&priv->qlock, flags);
        if (list_empty(&priv->parse_queue)) {
            spin_unlock_irqrestore(&priv->qlock, flags);
            break;
        }
        nl = list_first_entry(&priv->parse_queue, struct nmea_line, node);
        list_del(&nl->node);
        priv->parse_queue_depth--;
        spin_unlock_irqrestore(&priv->qlock, flags);

        neo6m_parse_line(priv, nl->buf, nl->len);
        kfree(nl);
    }
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
    v = p->fix_v2.lat_e7;
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
    v = p->fix_v2.lon_e7;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(lon, 0444, show_lon, NULL);

static ssize_t show_alt(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    s32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.alt_mm;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(alt_mm, 0444, show_alt, NULL);

static ssize_t show_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    s32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.speed_mmps;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(speed_mmps, 0444, show_speed, NULL);

static ssize_t show_have_fix(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    s8 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.have_fix;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(have_fix, 0444, show_have_fix, NULL);

static ssize_t show_utc(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    struct neo6m_gnss_fix_v2 f;
    spin_lock_irqsave(&p->lock, flags);
    f = p->fix_v2;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ\n",
        f.utc_year, f.utc_mon, f.utc_day,
        f.utc_hour, f.utc_min, f.utc_sec, f.utc_millis);
}
static DEVICE_ATTR(utc, 0444, show_utc, NULL);

static ssize_t show_hdop(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u16 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.hdop_x100;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(hdop_x100, 0444, show_hdop, NULL);

static ssize_t show_course(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    s32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.course_deg_e5;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}
static DEVICE_ATTR(course_deg_e5, 0444, show_course, NULL);

/* v2 sequence counter attributes */

static ssize_t show_fix_seq(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.fix_seq;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(fix_seq, 0444, show_fix_seq, NULL);

static ssize_t show_pos_seq(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.pos_seq;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(pos_seq, 0444, show_pos_seq, NULL);

static ssize_t show_vel_seq(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.vel_seq;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(vel_seq, 0444, show_vel_seq, NULL);

static ssize_t show_nmea_seq(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.nmea_seq;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(nmea_seq, 0444, show_nmea_seq, NULL);

static ssize_t show_pos_monotonic_ms(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.pos_monotonic_ms;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(pos_monotonic_ms, 0444, show_pos_monotonic_ms, NULL);

static ssize_t show_vel_monotonic_ms(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 v;
    spin_lock_irqsave(&p->lock, flags);
    v = p->fix_v2.vel_monotonic_ms;
    spin_unlock_irqrestore(&p->lock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(vel_monotonic_ms, 0444, show_vel_monotonic_ms, NULL);

/* Queue health attributes */

static ssize_t show_parse_queue_depth(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    unsigned int v;
    spin_lock_irqsave(&p->qlock, flags);
    v = p->parse_queue_depth;
    spin_unlock_irqrestore(&p->qlock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(parse_queue_depth, 0444, show_parse_queue_depth, NULL);

static ssize_t show_parse_queue_dropped(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    unsigned int v;
    spin_lock_irqsave(&p->qlock, flags);
    v = p->parse_queue_dropped;
    spin_unlock_irqrestore(&p->qlock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(parse_queue_dropped, 0444, show_parse_queue_dropped, NULL);

/* ---- Parser diagnostics attributes ---- */

#define DIAG_SHOW_LOCK(name, field)                                             \
static ssize_t show_##name(struct device *dev, struct device_attribute *attr,   \
                           char *buf)                                           \
{                                                                               \
    struct neo6m_priv *p = dev_get_drvdata(dev);                                \
    unsigned long flags;                                                        \
    u32 v;                                                                      \
    spin_lock_irqsave(&p->lock, flags);                                         \
    v = p->field;                                                               \
    spin_unlock_irqrestore(&p->lock, flags);                                    \
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);                                \
}                                                                               \
static DEVICE_ATTR(name, 0444, show_##name, NULL)

DIAG_SHOW_LOCK(nmea_rx_lines,             nmea_rx_lines);
DIAG_SHOW_LOCK(nmea_bad_checksum,         nmea_bad_checksum);
DIAG_SHOW_LOCK(nmea_parse_ok,             nmea_parse_ok);
DIAG_SHOW_LOCK(rmc_count,                 rmc_count);
DIAG_SHOW_LOCK(gga_count,                 gga_count);
DIAG_SHOW_LOCK(vtg_count,                 vtg_count);
DIAG_SHOW_LOCK(rmc_invalid_count,         rmc_invalid_count);
DIAG_SHOW_LOCK(gga_no_fix_count,          gga_no_fix_count);
DIAG_SHOW_LOCK(vtg_ignored_no_fix_count,  vtg_ignored_no_fix_count);
DIAG_SHOW_LOCK(latlon_parse_fail_count,   latlon_parse_fail_count);
DIAG_SHOW_LOCK(speed_parse_fail_count,    speed_parse_fail_count);
DIAG_SHOW_LOCK(pos_update_count,          pos_update_count);
DIAG_SHOW_LOCK(vel_update_count,          vel_update_count);

static ssize_t show_queue_drop_count(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 v;
    spin_lock_irqsave(&p->qlock, flags);
    v = p->queue_drop_count;
    spin_unlock_irqrestore(&p->qlock, flags);
    return scnprintf(buf, PAGE_SIZE, "%u\n", v);
}
static DEVICE_ATTR(queue_drop_count, 0444, show_queue_drop_count, NULL);

static ssize_t show_parser_stats(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct neo6m_priv *p = dev_get_drvdata(dev);
    unsigned long flags;
    u32 rx, ok, badcs, rmc, gga, vtg, pos, vel, drops;

    spin_lock_irqsave(&p->lock, flags);
    rx    = p->nmea_rx_lines;
    ok    = p->nmea_parse_ok;
    badcs = p->nmea_bad_checksum;
    rmc   = p->rmc_count;
    gga   = p->gga_count;
    vtg   = p->vtg_count;
    pos   = p->pos_update_count;
    vel   = p->vel_update_count;
    spin_unlock_irqrestore(&p->lock, flags);

    spin_lock_irqsave(&p->qlock, flags);
    drops = p->queue_drop_count;
    spin_unlock_irqrestore(&p->qlock, flags);

    return scnprintf(buf, PAGE_SIZE,
        "rx=%u ok=%u badcs=%u rmc=%u gga=%u vtg=%u pos=%u vel=%u drops=%u\n",
        rx, ok, badcs, rmc, gga, vtg, pos, vel, drops);
}
static DEVICE_ATTR(parser_stats, 0444, show_parser_stats, NULL);

static struct attribute *neo6m_attrs[] = {
    &dev_attr_lat.attr,
    &dev_attr_lon.attr,
    &dev_attr_alt_mm.attr,
    &dev_attr_speed_mmps.attr,
    &dev_attr_have_fix.attr,
    &dev_attr_utc.attr,
    &dev_attr_hdop_x100.attr,
    &dev_attr_course_deg_e5.attr,
    /* v2 sequence counters */
    &dev_attr_fix_seq.attr,
    &dev_attr_pos_seq.attr,
    &dev_attr_vel_seq.attr,
    &dev_attr_nmea_seq.attr,
    &dev_attr_pos_monotonic_ms.attr,
    &dev_attr_vel_monotonic_ms.attr,
    /* queue health */
    &dev_attr_parse_queue_depth.attr,
    &dev_attr_parse_queue_dropped.attr,
    /* parser diagnostics */
    &dev_attr_nmea_rx_lines.attr,
    &dev_attr_nmea_bad_checksum.attr,
    &dev_attr_nmea_parse_ok.attr,
    &dev_attr_rmc_count.attr,
    &dev_attr_gga_count.attr,
    &dev_attr_vtg_count.attr,
    &dev_attr_rmc_invalid_count.attr,
    &dev_attr_gga_no_fix_count.attr,
    &dev_attr_vtg_ignored_no_fix_count.attr,
    &dev_attr_latlon_parse_fail_count.attr,
    &dev_attr_speed_parse_fail_count.attr,
    &dev_attr_queue_drop_count.attr,
    &dev_attr_pos_update_count.attr,
    &dev_attr_vel_update_count.attr,
    &dev_attr_parser_stats.attr,
    NULL,
};

static const struct attribute_group neo6m_attr_group = {
    .attrs = neo6m_attrs,
};

/******************** Char device ********************/
static long neo6m_chr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct neo6m_priv *p = file->private_data;
    struct neo6m_gnss_fix_v2 fv2;
    unsigned long flags;

    spin_lock_irqsave(&p->lock, flags);
    fv2 = p->fix_v2;
    spin_unlock_irqrestore(&p->lock, flags);

    if (cmd == NEO6M_GNSS_IOC_GET_FIX) {
        /* v1 ABI: synthesise the old struct from the v2 snapshot */
        struct neo6m_gnss_fix f;
        memset(&f, 0, sizeof(f));
        f.monotonic_ms  = fv2.monotonic_ms;
        f.have_fix      = fv2.have_fix;
        f.lat_e7        = fv2.lat_e7;
        f.lon_e7        = fv2.lon_e7;
        f.alt_mm        = fv2.alt_mm;
        f.speed_mmps    = fv2.speed_mmps;
        f.course_deg_e5 = fv2.course_deg_e5;
        f.hdop_x100     = fv2.hdop_x100;
        f.utc_year      = fv2.utc_year;
        f.utc_mon       = fv2.utc_mon;
        f.utc_day       = fv2.utc_day;
        f.utc_hour      = fv2.utc_hour;
        f.utc_min       = fv2.utc_min;
        f.utc_sec       = fv2.utc_sec;
        f.utc_millis    = fv2.utc_millis;
        f.heading_valid = fv2.heading_valid;
        f.hdop_valid    = fv2.hdop_valid;
        if (copy_to_user((void __user *)arg, &f, sizeof(f)))
            return -EFAULT;
        return 0;
    }

    if (cmd == NEO6M_GNSS_IOC_GET_FIX_V2) {
        if (copy_to_user((void __user *)arg, &fv2, sizeof(fv2)))
            return -EFAULT;
        return 0;
    }

    return -ENOTTY;
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
    memset(&p->fix_v2, 0, sizeof(p->fix_v2));
    p->fix_v2.heading_valid = false;
    p->fix_v2.hdop_valid    = false;
    p->last_utc_epoch = U32_MAX;   /* sentinel: first valid RMC-A triggers epoch_changed */
    spin_lock_init(&p->lock);
    INIT_LIST_HEAD(&p->parse_queue);
    spin_lock_init(&p->qlock);
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
    struct nmea_line *nl, *tmp;
    unsigned long flags;

    sysfs_remove_group(&p->chardev->kobj, &neo6m_attr_group);
    device_destroy(p->cls, p->devt);
    cdev_del(&p->cdev);
    unregister_chrdev_region(p->devt, 1);
    class_destroy(p->cls);

    /* Stop the UART feed first, then wait for the worker to drain what it
     * picked up before close, then free anything left in the queue. */
    serdev_device_close(serdev);
    cancel_work_sync(&p->parse_work);

    spin_lock_irqsave(&p->qlock, flags);
    list_for_each_entry_safe(nl, tmp, &p->parse_queue, node) {
        list_del(&nl->node);
        kfree(nl);
    }
    p->parse_queue_depth = 0;
    spin_unlock_irqrestore(&p->qlock, flags);

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