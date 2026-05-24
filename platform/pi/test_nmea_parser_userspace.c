#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>

/*
 * Lightweight userspace test harness for the NMEA parsing logic used by
 * neo6m_gnss_serdev.c.
 *
 * Compile:
 *   gcc -O2 -Wall -Wextra -o test_nmea_parser_userspace test_nmea_parser_userspace.c
 */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define DEG_E7 10000000L

#define PASS() do { printf("PASS: %s\n", __func__); return 0; } while (0)
#define FAIL(fmt, ...) do { \
    fprintf(stderr, "FAIL: %s: " fmt "\n", __func__, ##__VA_ARGS__); \
    return 1; \
} while (0)

#define EXPECT_TRUE(expr) do { \
    if (!(expr)) FAIL("expected true: %s", #expr); \
} while (0)

#define EXPECT_FALSE(expr) do { \
    if ((expr)) FAIL("expected false: %s", #expr); \
} while (0)

#define EXPECT_EQ_INT(actual, expected) do { \
    long long _a = (long long)(actual); \
    long long _e = (long long)(expected); \
    if (_a != _e) FAIL("%s=%lld, expected %lld", #actual, _a, _e); \
} while (0)

#define EXPECT_NEAR_INT(actual, expected, tol) do { \
    long long _a = (long long)(actual); \
    long long _e = (long long)(expected); \
    long long _t = (long long)(tol); \
    long long _d = (_a > _e) ? (_a - _e) : (_e - _a); \
    if (_d > _t) FAIL("%s=%lld, expected %lld +/- %lld", #actual, _a, _e, _t); \
} while (0)

typedef struct {
    int64_t monotonic_ns;
    int8_t have_fix;

    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_mm;

    int32_t speed_mmps;
    int32_t course_deg_e5;

    uint16_t hdop_x100;

    uint16_t utc_year;
    uint8_t utc_mon;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_min;
    uint8_t utc_sec;
    uint16_t utc_millis;

    bool heading_valid;
    bool hdop_valid;

    uint32_t fix_seq;
    uint32_t pos_seq;
    uint32_t vel_seq;
    uint32_t nmea_seq;
    uint8_t pos_valid;
    uint8_t vel_valid;
    uint8_t stale;
    uint8_t reserved0;
    int64_t pos_monotonic_ns;
    int64_t vel_monotonic_ns;
} gnss_fix_t;

typedef struct {
    gnss_fix_t fix;
    uint32_t last_utc_epoch;
    int64_t fake_now_ns;
} parser_ctx_t;

static long clamp_long(long v, long lo, long hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static long strtol_safe(const char *p, long def)
{
    char *end = NULL;
    long v;

    if (!p || !*p)
        return def;

    v = strtol(p, &end, 10);
    if (end == p)
        return def;

    return v;
}

/* Convert decimal ASCII to milli-units, with simple rounding/truncation style
 * compatible with the original driver helper. Returns def if no digits exist. */
static long strtod_milli(const char *p, long def)
{
    long sign = 1;
    long whole = 0;
    long frac = 0;
    int frac_digits = 0;
    bool any_digit = false;
    const char *c = p;

    if (!p || !*p)
        return def;

    if (*c == '-') {
        sign = -1;
        c++;
    } else if (*c == '+') {
        c++;
    }

    while (isdigit((unsigned char)*c)) {
        any_digit = true;
        whole = whole * 10 + (*c - '0');
        c++;
    }

    if (*c == '.') {
        c++;
        while (isdigit((unsigned char)*c) && frac_digits < 3) {
            any_digit = true;
            frac = frac * 10 + (*c - '0');
            frac_digits++;
            c++;
        }
        while (isdigit((unsigned char)*c)) {
            any_digit = true;
            c++;
        }
    }

    if (!any_digit)
        return def;

    while (frac_digits < 3) {
        frac *= 10;
        frac_digits++;
    }

    return sign * (whole * 1000 + frac);
}

static bool nmea_checksum_ok(const char *s, size_t len)
{
    const char *star;
    unsigned int sum = 0;
    unsigned int got = 0;
    size_t i;

    if (!s || len < 4 || s[0] != '$')
        return false;

    star = memchr(s, '*', len);
    if (!star)
        return false;

    if (star + 3 > s + len)
        return false;

    for (i = 1; &s[i] < star; i++)
        sum ^= (unsigned char)s[i];

    if (!isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2]))
        return false;

    if (sscanf(star + 1, "%2x", &got) != 1)
        return false;

    return (sum & 0xFFU) == (got & 0xFFU);
}

static bool nmea_type_is(const char *sentence_id, const char *type)
{
    if (!sentence_id || !type || sentence_id[0] != '$' || strlen(sentence_id) < 6)
        return false;

    return sentence_id[3] == type[0] &&
           sentence_id[4] == type[1] &&
           sentence_id[5] == type[2];
}

static bool nmea_latlon_to_e7(const char *val, const char *hem, int32_t *out_e7)
{
    long sign;
    int exp_deg_digits;
    int dot_pos;
    const char *p;
    const char *dot;
    char deg_str[4] = {0};
    long deg;
    long mm_whole = 0;
    long mm_frac = 0;
    int frac_dig = 0;
    long minutes_e7_raw;
    long degrees_e7_from_min;
    long total;

    if (!val || !*val || !hem || !*hem || !out_e7)
        return false;

    switch (hem[0]) {
    case 'N': sign =  1; exp_deg_digits = 2; break;
    case 'S': sign = -1; exp_deg_digits = 2; break;
    case 'E': sign =  1; exp_deg_digits = 3; break;
    case 'W': sign = -1; exp_deg_digits = 3; break;
    default:
        return false;
    }

    dot = NULL;
    for (p = val; *p; p++) {
        if (*p == '.') {
            if (dot)
                return false;
            dot = p;
        } else if (!isdigit((unsigned char)*p)) {
            return false;
        }
    }

    dot_pos = (int)(dot ? (dot - val) : (p - val));
    if (dot_pos != exp_deg_digits + 2)
        return false;

    memcpy(deg_str, val, (size_t)exp_deg_digits);
    deg_str[exp_deg_digits] = '\0';
    deg = strtol_safe(deg_str, -1);

    if (exp_deg_digits == 2) {
        if (deg < 0 || deg > 90)
            return false;
    } else {
        if (deg < 0 || deg > 180)
            return false;
    }

    for (p = val + exp_deg_digits; p < val + dot_pos; p++)
        mm_whole = mm_whole * 10 + (*p - '0');

    if (mm_whole < 0 || mm_whole >= 60)
        return false;

    if (dot) {
        for (p = dot + 1; isdigit((unsigned char)*p) && frac_dig < 7; p++, frac_dig++)
            mm_frac = mm_frac * 10 + (*p - '0');
    }

    while (frac_dig < 7) {
        mm_frac *= 10;
        frac_dig++;
    }

    minutes_e7_raw = mm_whole * 10000000L + mm_frac;
    degrees_e7_from_min = (minutes_e7_raw + 30L) / 60L;
    total = deg * 10000000L + degrees_e7_from_min;

    *out_e7 = (int32_t)(sign * total);
    return true;
}

static uint32_t knots_to_mmps_maybe(const char *knots)
{
    long parsed;
    uint32_t mk, q, r, a, b;

    parsed = strtod_milli(knots, 0);
    if (parsed <= 0)
        return 0;

    if (parsed > 1000000L)
        return UINT_MAX;

    mk = (uint32_t)parsed;
    q = mk / 1000U;
    r = mk % 1000U;

    a = (q * 514444U + 500U) / 1000U;
    b = (r * 514444U + 500000U) / 1000000U;

    return a + b;
}

static uint8_t nmea_checksum_payload(const char *payload)
{
    uint8_t sum = 0;
    while (*payload)
        sum ^= (uint8_t)*payload++;
    return sum;
}

static void make_nmea(char *out, size_t out_len, const char *payload)
{
    uint8_t cs = nmea_checksum_payload(payload);
    snprintf(out, out_len, "$%s*%02X", payload, cs);
}

static int split_csv_preserve_empty(char *s, char **tok, int max_tok)
{
    int n = 0;
    char *start = s;
    char *c = s;

    while (n < max_tok) {
        if (*c == ',' || *c == '\0') {
            char saved = *c;
            *c = '\0';
            tok[n++] = start;
            if (saved == '\0')
                break;
            start = c + 1;
        }
        c++;
    }

    return n;
}

static void parse_utc(gnss_fix_t *f, char *const *fields, int n)
{
    const char *t = (n > 1) ? fields[1] : NULL;
    const char *d = (n > 9) ? fields[9] : NULL;

    if (t && *t) {
        long hh = strtol_safe(t, 0) / 10000;
        long mm = (strtol_safe(t, 0) / 100) % 100;
        long ss = strtol_safe(t, 0) % 100;
        const char *dot = strchr(t, '.');
        long ms = 0;

        if (dot) {
            ms = strtol_safe(dot + 1, 0);
            while (ms > 999)
                ms /= 10;
        }

        f->utc_hour = (uint8_t)clamp_long(hh, 0, 23);
        f->utc_min = (uint8_t)clamp_long(mm, 0, 59);
        f->utc_sec = (uint8_t)clamp_long(ss, 0, 60);
        f->utc_millis = (uint16_t)clamp_long(ms, 0, 999);
    }

    if (d && *d) {
        long dd = strtol_safe(d, 0) / 10000;
        long mo = (strtol_safe(d, 0) / 100) % 100;
        long yy = strtol_safe(d, 0) % 100;

        f->utc_day = (uint8_t)clamp_long(dd, 1, 31);
        f->utc_mon = (uint8_t)clamp_long(mo, 1, 12);
        f->utc_year = (uint16_t)((yy >= 80) ? (1900 + yy) : (2000 + yy));
    }
}

static int parse_line(parser_ctx_t *ctx, const char *s)
{
    char tmp[256];
    char *tok[32];
    int ntok;
    char *star;
    gnss_fix_t newfix;
    uint32_t saved_utc_epoch;
    int32_t old_lat_e7, old_lon_e7, old_speed, old_course;
    bool pos_updated = false;
    bool vel_updated = false;
    bool epoch_changed = false;
    uint32_t new_utc_epoch = 0;
    size_t len = strlen(s);

    if (!nmea_checksum_ok(s, len))
        return -1;

    if (len >= sizeof(tmp))
        return -2;

    memcpy(tmp, s, len + 1);
    ntok = split_csv_preserve_empty(tmp, tok, (int)ARRAY_SIZE(tok));

    if (ntok > 0) {
        star = strchr(tok[ntok - 1], '*');
        if (star)
            *star = '\0';
    }

    newfix = ctx->fix;
    saved_utc_epoch = ctx->last_utc_epoch;

    old_lat_e7 = newfix.lat_e7;
    old_lon_e7 = newfix.lon_e7;
    old_speed = newfix.speed_mmps;
    old_course = newfix.course_deg_e5;

    if (ntok <= 0 || tok[0][0] != '$')
        return -3;

    if (nmea_type_is(tok[0], "RMC")) {
        const char *status = (ntok > 2) ? tok[2] : "";
        bool active = (status && *status == 'A');

        if (active) {
            const char *t = (ntok > 1) ? tok[1] : NULL;

            if (t && *t) {
                long hh = strtol_safe(t, 0) / 10000;
                long mm = (strtol_safe(t, 0) / 100) % 100;
                long ss = strtol_safe(t, 0) % 100;
                new_utc_epoch = (uint32_t)(hh * 3600 + mm * 60 + ss);
                if (new_utc_epoch != saved_utc_epoch)
                    epoch_changed = true;
            }

            newfix.have_fix = 1;
            parse_utc(&newfix, tok, ntok);

            if (ntok > 6) {
                int32_t lat_e7, lon_e7;
                bool lat_ok = nmea_latlon_to_e7(tok[3], tok[4], &lat_e7);
                bool lon_ok = nmea_latlon_to_e7(tok[5], tok[6], &lon_e7);
                if (lat_ok && lon_ok) {
                    newfix.lat_e7 = lat_e7;
                    newfix.lon_e7 = lon_e7;
                    pos_updated = true;
                }
            }

            if (ntok > 7 && tok[7] && *tok[7]) {
                newfix.speed_mmps = (int32_t)knots_to_mmps_maybe(tok[7]);
                vel_updated = true;
            }

            if (ntok > 8 && tok[8] && *tok[8]) {
                long hdg_milli = strtod_milli(tok[8], 0);
                newfix.course_deg_e5 = (int32_t)(hdg_milli * 100);
                newfix.heading_valid = true;
                vel_updated = true;
            } else {
                newfix.heading_valid = false;
            }
        } else {
            newfix.have_fix = 0;
            newfix.heading_valid = false;
            newfix.speed_mmps = 0;
            newfix.pos_valid = 0;
            newfix.vel_valid = 0;
        }
    } else if (nmea_type_is(tok[0], "GGA")) {
        long fixq = ((ntok > 6) && tok[6] && *tok[6]) ? strtol_safe(tok[6], 0) : 0;

        if (fixq > 0) {
            newfix.have_fix = 1;

            if (ntok > 5) {
                int32_t lat_e7, lon_e7;
                bool lat_ok = nmea_latlon_to_e7(tok[2], tok[3], &lat_e7);
                bool lon_ok = nmea_latlon_to_e7(tok[4], tok[5], &lon_e7);
                if (lat_ok && lon_ok) {
                    newfix.lat_e7 = lat_e7;
                    newfix.lon_e7 = lon_e7;
                    pos_updated = true;
                }
            }

            if (ntok > 9 && tok[9] && *tok[9])
                newfix.alt_mm = (int32_t)strtod_milli(tok[9], 0);

            if (ntok > 8 && tok[8] && *tok[8]) {
                long hdop_milli = strtod_milli(tok[8], 0);
                newfix.hdop_x100 = (uint16_t)((hdop_milli + 5) / 10);
                newfix.hdop_valid = true;
            }

            if (ntok > 1 && tok[1] && *tok[1]) {
                long hh = strtol_safe(tok[1], 0) / 10000;
                long mm = (strtol_safe(tok[1], 0) / 100) % 100;
                long ss = strtol_safe(tok[1], 0) % 100;
                const char *dot = strchr(tok[1], '.');
                long ms = 0;
                if (dot) {
                    ms = strtol_safe(dot + 1, 0);
                    while (ms > 999)
                        ms /= 10;
                }
                newfix.utc_hour = (uint8_t)clamp_long(hh, 0, 23);
                newfix.utc_min = (uint8_t)clamp_long(mm, 0, 59);
                newfix.utc_sec = (uint8_t)clamp_long(ss, 0, 60);
                newfix.utc_millis = (uint16_t)clamp_long(ms, 0, 999);
            }
        } else {
            newfix.have_fix = 0;
            newfix.heading_valid = false;
            newfix.speed_mmps = 0;
            newfix.pos_valid = 0;
            newfix.vel_valid = 0;

            if (ntok > 8 && tok[8] && *tok[8]) {
                long hdop_milli = strtod_milli(tok[8], 0);
                newfix.hdop_x100 = (uint16_t)((hdop_milli + 5) / 10);
                newfix.hdop_valid = true;
            }
        }
    } else if (nmea_type_is(tok[0], "VTG")) {
        const char *vtg_mode = (ntok > 9 && tok[9]) ? tok[9] : NULL;
        bool vtg_no_fix = (vtg_mode && *vtg_mode == 'N');

        if (!newfix.have_fix || vtg_no_fix) {
            newfix.heading_valid = false;
            newfix.speed_mmps = 0;
        } else {
            if (ntok > 1 && tok[1] && *tok[1]) {
                long hdg_milli = strtod_milli(tok[1], 0);
                newfix.course_deg_e5 = (int32_t)(hdg_milli * 100);
                newfix.heading_valid = true;
                vel_updated = true;
            } else {
                newfix.heading_valid = false;
            }

            if (ntok > 5 && tok[5] && *tok[5]) {
                newfix.speed_mmps = (int32_t)knots_to_mmps_maybe(tok[5]);
                vel_updated = true;
            }
        }
    } else {
        return 1; /* ignored unknown sentence */
    }

    ctx->fake_now_ns += 100000000LL;
    newfix.monotonic_ns = ctx->fake_now_ns;
    newfix.nmea_seq++;

    if (newfix.have_fix && pos_updated &&
        (newfix.lat_e7 != old_lat_e7 || newfix.lon_e7 != old_lon_e7 || epoch_changed)) {
        newfix.pos_seq++;
        newfix.pos_monotonic_ns = newfix.monotonic_ns;
        newfix.pos_valid = 1;
    }

    if (newfix.have_fix && vel_updated &&
        (newfix.speed_mmps != old_speed || newfix.course_deg_e5 != old_course || epoch_changed)) {
        newfix.vel_seq++;
        newfix.vel_monotonic_ns = newfix.monotonic_ns;
        newfix.vel_valid = 1;
    }

    if (!newfix.have_fix) {
        newfix.pos_valid = 0;
        newfix.vel_valid = 0;
    }

    if (epoch_changed && newfix.have_fix)
        newfix.fix_seq++;

    newfix.stale = (uint8_t)(!(pos_updated || vel_updated));

    ctx->fix = newfix;
    if (epoch_changed)
        ctx->last_utc_epoch = new_utc_epoch;

    return 0;
}

static void parser_init(parser_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->last_utc_epoch = UINT_MAX;
}

static int expected_e7(int deg, double minutes, int sign)
{
    double v = ((double)deg + minutes / 60.0) * 10000000.0;
    long rounded = (long)(v + 0.5);
    return (int)(sign * rounded);
}

static int test_speed_conversion(void)
{
    EXPECT_NEAR_INT(knots_to_mmps_maybe("0.50"), 257, 1);
    EXPECT_NEAR_INT(knots_to_mmps_maybe("2.744"), 1412, 1);
    EXPECT_NEAR_INT(knots_to_mmps_maybe("10.0"), 5144, 1);
    EXPECT_NEAR_INT(knots_to_mmps_maybe("29.05"), 14945, 1);

    /* This specifically catches the old 1000x bug. Old code returned ~257222. */
    if (knots_to_mmps_maybe("0.50") > 1000)
        FAIL("old 1000x speed bug appears present");

    PASS();
}

static int test_latlon_parser(void)
{
    int32_t out;
    int32_t exp_lat = expected_e7(12, 59.681388, 1);
    int32_t exp_lon = expected_e7(77, 28.764630, 1);

    EXPECT_TRUE(nmea_latlon_to_e7("1259.681388", "N", &out));
    EXPECT_NEAR_INT(out, exp_lat, 1);

    EXPECT_TRUE(nmea_latlon_to_e7("07728.764630", "E", &out));
    EXPECT_NEAR_INT(out, exp_lon, 1);

    EXPECT_FALSE(nmea_latlon_to_e7("1259.681388", "X", &out));
    EXPECT_FALSE(nmea_latlon_to_e7("9100.0000", "N", &out));
    EXPECT_FALSE(nmea_latlon_to_e7("18100.0000", "E", &out));
    EXPECT_FALSE(nmea_latlon_to_e7("1260.0000", "N", &out));

    PASS();
}

static int test_rmc_valid_stationary(void)
{
    parser_ctx_t ctx;
    char line[256];
    int32_t exp_lat = expected_e7(12, 59.681388, 1);
    int32_t exp_lon = expected_e7(77, 28.764630, 1);

    parser_init(&ctx);
    make_nmea(line, sizeof(line),
              "GNRMC,004526.000,A,1259.681388,N,07728.764630,E,0.50,123.45,070526,,,A");

    EXPECT_EQ_INT(parse_line(&ctx, line), 0);
    EXPECT_EQ_INT(ctx.fix.have_fix, 1);
    EXPECT_NEAR_INT(ctx.fix.speed_mmps, 257, 1);
    EXPECT_EQ_INT(ctx.fix.heading_valid, true);
    EXPECT_NEAR_INT(ctx.fix.course_deg_e5, 12345000, 1);
    EXPECT_NEAR_INT(ctx.fix.lat_e7, exp_lat, 1);
    EXPECT_NEAR_INT(ctx.fix.lon_e7, exp_lon, 1);
    EXPECT_EQ_INT(ctx.fix.pos_seq, 1);
    EXPECT_EQ_INT(ctx.fix.vel_seq, 1);
    EXPECT_EQ_INT(ctx.fix.fix_seq, 1);

    PASS();
}

static int test_rmc_invalid_does_not_update_stale_fix(void)
{
    parser_ctx_t ctx;
    char line[256];

    parser_init(&ctx);
    ctx.fix.have_fix = 1;
    ctx.fix.lat_e7 = 111111111;
    ctx.fix.lon_e7 = 222222222;
    ctx.fix.speed_mmps = 12345;
    ctx.fix.course_deg_e5 = 33333000;
    ctx.fix.heading_valid = true;
    ctx.fix.pos_valid = 1;
    ctx.fix.vel_valid = 1;

    make_nmea(line, sizeof(line),
              "GNRMC,004527.000,V,1259.681388,N,07728.764630,E,29.05,123.45,070526,,,N");

    EXPECT_EQ_INT(parse_line(&ctx, line), 0);
    EXPECT_EQ_INT(ctx.fix.have_fix, 0);
    EXPECT_EQ_INT(ctx.fix.speed_mmps, 0);
    EXPECT_EQ_INT(ctx.fix.heading_valid, false);
    EXPECT_EQ_INT(ctx.fix.pos_valid, 0);
    EXPECT_EQ_INT(ctx.fix.vel_valid, 0);
    EXPECT_EQ_INT(ctx.fix.lat_e7, 111111111);
    EXPECT_EQ_INT(ctx.fix.lon_e7, 222222222);

    PASS();
}

static int test_gga_valid(void)
{
    parser_ctx_t ctx;
    char line[256];
    int32_t exp_lat = expected_e7(12, 59.681388, 1);
    int32_t exp_lon = expected_e7(77, 28.764630, 1);

    parser_init(&ctx);
    make_nmea(line, sizeof(line),
              "GNGGA,004526.000,1259.681388,N,07728.764630,E,1,12,1.23,494.2,M,0.0,M,,");

    EXPECT_EQ_INT(parse_line(&ctx, line), 0);
    EXPECT_EQ_INT(ctx.fix.have_fix, 1);
    EXPECT_NEAR_INT(ctx.fix.lat_e7, exp_lat, 1);
    EXPECT_NEAR_INT(ctx.fix.lon_e7, exp_lon, 1);
    EXPECT_EQ_INT(ctx.fix.hdop_valid, true);
    EXPECT_EQ_INT(ctx.fix.hdop_x100, 123);
    EXPECT_EQ_INT(ctx.fix.alt_mm, 494200);
    EXPECT_EQ_INT(ctx.fix.pos_seq, 1);

    PASS();
}

static int test_gga_invalid_does_not_update_stale_position(void)
{
    parser_ctx_t ctx;
    char line[256];

    parser_init(&ctx);
    ctx.fix.have_fix = 1;
    ctx.fix.lat_e7 = 111111111;
    ctx.fix.lon_e7 = 222222222;
    ctx.fix.alt_mm = 333333;
    ctx.fix.speed_mmps = 9999;
    ctx.fix.heading_valid = true;
    ctx.fix.pos_valid = 1;
    ctx.fix.vel_valid = 1;

    make_nmea(line, sizeof(line),
              "GNGGA,004526.000,1259.681388,N,07728.764630,E,0,00,9.99,999.9,M,0.0,M,,");

    EXPECT_EQ_INT(parse_line(&ctx, line), 0);
    EXPECT_EQ_INT(ctx.fix.have_fix, 0);
    EXPECT_EQ_INT(ctx.fix.speed_mmps, 0);
    EXPECT_EQ_INT(ctx.fix.heading_valid, false);
    EXPECT_EQ_INT(ctx.fix.pos_valid, 0);
    EXPECT_EQ_INT(ctx.fix.vel_valid, 0);
    EXPECT_EQ_INT(ctx.fix.lat_e7, 111111111);
    EXPECT_EQ_INT(ctx.fix.lon_e7, 222222222);
    EXPECT_EQ_INT(ctx.fix.alt_mm, 333333);
    EXPECT_EQ_INT(ctx.fix.hdop_valid, true);
    EXPECT_EQ_INT(ctx.fix.hdop_x100, 999);

    PASS();
}

static int test_vtg_valid_with_existing_fix(void)
{
    parser_ctx_t ctx;
    char line[256];

    parser_init(&ctx);
    ctx.fix.have_fix = 1;
    ctx.fix.pos_valid = 1;

    make_nmea(line, sizeof(line),
              "GNVTG,222.22,T,,M,2.744,N,5.081,K,A");

    EXPECT_EQ_INT(parse_line(&ctx, line), 0);
    EXPECT_EQ_INT(ctx.fix.have_fix, 1);
    EXPECT_NEAR_INT(ctx.fix.speed_mmps, 1412, 1);
    EXPECT_EQ_INT(ctx.fix.heading_valid, true);
    EXPECT_NEAR_INT(ctx.fix.course_deg_e5, 22222000, 1);
    EXPECT_EQ_INT(ctx.fix.vel_seq, 1);
    EXPECT_EQ_INT(ctx.fix.vel_valid, 1);

    PASS();
}

static int test_vtg_with_no_fix_is_ignored(void)
{
    parser_ctx_t ctx;
    char line[256];

    parser_init(&ctx);
    ctx.fix.have_fix = 0;
    ctx.fix.speed_mmps = 7777;
    ctx.fix.course_deg_e5 = 88888000;
    ctx.fix.heading_valid = true;

    make_nmea(line, sizeof(line),
              "GNVTG,222.22,T,,M,2.744,N,5.081,K,A");

    EXPECT_EQ_INT(parse_line(&ctx, line), 0);
    EXPECT_EQ_INT(ctx.fix.have_fix, 0);
    EXPECT_EQ_INT(ctx.fix.speed_mmps, 0);
    EXPECT_EQ_INT(ctx.fix.heading_valid, false);
    EXPECT_EQ_INT(ctx.fix.vel_seq, 0);

    PASS();
}

static int test_talker_id_generalization(void)
{
    parser_ctx_t ctx;
    char line[256];

    parser_init(&ctx);
    make_nmea(line, sizeof(line),
              "BDRMC,004526.000,A,1259.681388,N,07728.764630,E,0.50,123.45,070526,,,A");

    EXPECT_EQ_INT(parse_line(&ctx, line), 0);
    EXPECT_EQ_INT(ctx.fix.have_fix, 1);
    EXPECT_NEAR_INT(ctx.fix.speed_mmps, 257, 1);
    EXPECT_EQ_INT(ctx.fix.pos_seq, 1);

    PASS();
}

static int test_bad_checksum_rejected(void)
{
    parser_ctx_t ctx;
    const char *bad = "$GNRMC,004526.000,A,1259.681388,N,07728.764630,E,0.50,123.45,070526,,,A*00";

    parser_init(&ctx);
    EXPECT_EQ_INT(parse_line(&ctx, bad), -1);
    EXPECT_EQ_INT(ctx.fix.nmea_seq, 0);

    PASS();
}

int main(void)
{
    int failures = 0;

    failures += test_speed_conversion();
    failures += test_latlon_parser();
    failures += test_rmc_valid_stationary();
    failures += test_rmc_invalid_does_not_update_stale_fix();
    failures += test_gga_valid();
    failures += test_gga_invalid_does_not_update_stale_position();
    failures += test_vtg_valid_with_existing_fix();
    failures += test_vtg_with_no_fix_is_ignored();
    failures += test_talker_id_generalization();
    failures += test_bad_checksum_rejected();

    if (failures) {
        fprintf(stderr, "\nRESULT: FAIL (%d failing test group%s)\n",
                failures, failures == 1 ? "" : "s");
        return 1;
    }

    printf("\nRESULT: PASS (all parser tests passed)\n");
    return 0;
}
