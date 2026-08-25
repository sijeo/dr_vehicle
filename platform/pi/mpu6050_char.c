// SPDX-License-Identifier: GPL-2.0

/**
 * @file mpu6050_char.c
 * @brief Invensense MPU6050 character device interface.(I2C) - IRQ streaming or on-demand snapshot
 * 
 * This non-IIO driver exposes a compact ABI via /dev/mpu6050-X and supports **two** modes:
 * 
 * 
 * 1) ** IRQ Mode (preferred)**: If an INT pin is wired and enabled, the driver
 *    captures samples on each DATA_RDY interrupt  and pushes them into an in-kernel FIFO.
 *   read()/poll() operates on that FIFO (low-jitter streaming).
 * 
 * 2) **On-demand snapshot mode**: If IRQ is disabled (or no IRQ is present), the driver 
 *    read() performs a **synchronous I2C burst** of the 14 bytes raw data window and returns
 *   exactly one sample. No background timers are used. 
 * 
 * There is **no hrtimer fallback** in this driver. Sampling occurs only via IRQ or when 
 * userspace explicitly calls read()
 * 
 * Kernel module floating point is not used anywhere; all data are integers.
 */

#include <linux/bitfield.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sysfs.h>
#include <linux/ktime.h>

#ifndef sysfs_emit
#define sysfs_emit(buf, fmt, ...) scnprintf((buf), PAGE_SIZE, (fmt), ##__VA_ARGS__)
#endif

#include "mpu6050_ioctl.h" // User-space API definitions

#define DRIVER_NAME "mpu6050-char"
#define DEVICE_NAME "mpu6050"
#define MPU6050_MAX_DEVICES 8
#define MPU6050_FIFO_SAMPLES   2048 /* in-kernel sample FIFO depth */

/* ---------------------------------------------------------------------
 * ISM330DLC register map.
 *
 * The physical chip on the board is an ST ISM330DLC (I²C address 0x6A),
 * NOT an Invensense MPU6050. File names, /dev node, struct mpu6050_sample,
 * and the ioctl ABI are kept unchanged so that userspace (the DR app, the
 * test_app, and any legacy tooling) does not have to change. Only the
 * driver internals below actually talk to the chip. Register names are
 * ISM330-native to avoid confusion — the MPU6050_REG_* prefix used
 * previously mapped onto reserved / repurposed ISM330 addresses and was
 * silently misconfiguring the sensor.
 *
 * Datasheet: STMicroelectronics DocID028901 (ISM330DLC).
 * ------------------------------------------------------------------- */
#define ISM330_REG_WHO_AM_I         0x0F   /* r/o, returns 0x71 */
#define ISM330_REG_INT1_CTRL        0x0D   /* which sources drive INT1 pin */
#define ISM330_REG_INT2_CTRL        0x0E
#define ISM330_REG_CTRL1_XL         0x10   /* accel: ODR_XL[7:4], FS_XL[3:2], LPF1_BW_SEL[1], BW0_XL[0] */
#define ISM330_REG_CTRL2_G          0x11   /* gyro:  ODR_G[7:4],  FS_G[3:2],  FS_125[1] */
#define ISM330_REG_CTRL3_C          0x12   /* BOOT[7] BDU[6] H_LACTIVE[5] PP_OD[4] SIM[3] IF_INC[2] BLE[1] SW_RESET[0] */
#define ISM330_REG_CTRL4_C          0x13
#define ISM330_REG_CTRL5_C          0x14
#define ISM330_REG_CTRL6_C          0x15
#define ISM330_REG_CTRL7_G          0x16
#define ISM330_REG_CTRL8_XL         0x17
#define ISM330_REG_CTRL9_XL         0x18
#define ISM330_REG_CTRL10_C         0x19
#define ISM330_REG_STATUS_REG       0x1E   /* XLDA[0], GDA[1], TDA[2] */
#define ISM330_REG_OUT_TEMP_L       0x20   /* temp L/H (little-endian pair) */
#define ISM330_REG_OUTX_L_G         0x22   /* gyro  X L, X H, Y L, Y H, Z L, Z H (LE) */
#define ISM330_REG_OUTX_L_XL        0x28   /* accel X L, X H, Y L, Y H, Z L, Z H (LE) */

/* ============================================================
 * Chip-family support.
 *
 * The driver targets two distinct ST 6-axis IMU register-layout
 * families that share the classic 0x0F WHO_AM_I / 0x20 temp / 0x22
 * gyro / 0x28 accel data addresses, but differ in HOW you configure
 * full-scale and output data rate:
 *
 *   CLASSIC (ISM330DLC, DHCX, LSM6DSL/DSM/DSR/DSO/DSOX):
 *     - CTRL1_XL (0x10)  ODR_XL[7:4] + FS_XL[3:2]  (non-monotonic 2/16/4/8)
 *     - CTRL2_G  (0x11)  ODR_G [7:4] + FS_G [3:2]  (monotonic 250/500/1000/2000)
 *     - Discrete ODR: 12.5/26/52/104/208/416/833/1666 Hz
 *
 *   NEW-GEN (LSM6DSV, LSM6DSV16X, LSM6DSV32X, ISM330BX):
 *     - CTRL1    (0x10)  OP_MODE_XL[6:4] + ODR_XL[3:0]  (ODR only)
 *     - CTRL2    (0x11)  OP_MODE_G [6:4] + ODR_G [3:0]  (ODR only)
 *     - CTRL6    (0x15)  LPF1_G_BW[5:4] + FS_G[3:0]     (monotonic 125/250/…)
 *     - CTRL8    (0x17)  HP_LPF2_XL_BW[7:5] + FS_XL[1:0] (monotonic 2/4/8/16)
 *     - Discrete ODR: 1.875/7.5/15/30/60/120/240/480/960/1920/3840/7680 Hz
 * ============================================================ */

/* WHO_AM_I IDs the driver understands */
#define ISM330_WHO_AM_I_ID          0x6A  /* ISM330DLC (classic) */
#define ISM330DHCX_WHO_AM_I_ID      0x6B  /* ISM330DHCX / LSM6DSR (classic) */
#define LSM6DSOX_WHO_AM_I_ID        0x6C  /* LSM6DSO / LSM6DSOX  (classic) */
#define LSM6DSV_WHO_AM_I_ID         0x70  /* LSM6DSV / LSM6DSV16X (new-gen) */
#define ISM330BX_WHO_AM_I_ID        0x71  /* ISM330BX (new-gen) — SAME ID as MPU-9250 but different vendor */
#define LSM6DSV32X_WHO_AM_I_ID      0x73  /* LSM6DSV32X (new-gen) */

enum ism330_family {
    ISM330_FAMILY_UNKNOWN = 0,
    ISM330_FAMILY_CLASSIC,   /* DLC/DHCX/DSL/DSM/DSR/DSO/DSOX */
    ISM330_FAMILY_NEWGEN,    /* LSM6DSV / ISM330BX / LSM6DSV32X */
};

static enum ism330_family ism330_family_from_who(u8 v) {
    switch (v) {
    case ISM330_WHO_AM_I_ID:
    case ISM330DHCX_WHO_AM_I_ID:
    case LSM6DSOX_WHO_AM_I_ID:
        return ISM330_FAMILY_CLASSIC;
    case LSM6DSV_WHO_AM_I_ID:
    case ISM330BX_WHO_AM_I_ID:
    case LSM6DSV32X_WHO_AM_I_ID:
        return ISM330_FAMILY_NEWGEN;
    default:
        return ISM330_FAMILY_UNKNOWN;
    }
}
static const char *ism330_family_name(enum ism330_family f) {
    switch (f) {
    case ISM330_FAMILY_CLASSIC: return "classic (CTRL1_XL/CTRL2_G FS layout)";
    case ISM330_FAMILY_NEWGEN:  return "new-gen (CTRL8/CTRL6 FS layout)";
    default:                    return "UNKNOWN";
    }
}
static const char *ism330_who_am_i_name(u8 v) {
    switch (v) {
    case ISM330_WHO_AM_I_ID:     return "ISM330DLC (or LSM6DSL/DSM)";
    case ISM330DHCX_WHO_AM_I_ID: return "ISM330DHCX (or LSM6DSR)";
    case LSM6DSOX_WHO_AM_I_ID:   return "LSM6DSO/LSM6DSOX";
    case LSM6DSV_WHO_AM_I_ID:    return "LSM6DSV/LSM6DSV16X";
    case ISM330BX_WHO_AM_I_ID:   return "ISM330BX (LSM6DSV-family)";
    case LSM6DSV32X_WHO_AM_I_ID: return "LSM6DSV32X";
    default:                     return "UNSUPPORTED";
    }
}

/* --- New-gen (LSM6DSV / ISM330BX) additional register addresses --- */
/* CTRL1 / CTRL2 are the same address as the classic CTRL1_XL / CTRL2_G
 * but hold different fields (ODR + OP_MODE only, no FS). FS lives at: */
#define ISM330BX_REG_CTRL6          0x15  /* bits [3:0] = FS_G[3:0] */
#define ISM330BX_REG_CTRL8          0x17  /* bits [1:0] = FS_XL[1:0] */

/* New-gen ODR encoding (CTRL1[3:0] for accel, CTRL2[3:0] for gyro) */
#define ISM330BX_ODR_POWER_DOWN     0x0
#define ISM330BX_ODR_1_875_HZ       0x1
#define ISM330BX_ODR_7_5_HZ         0x2
#define ISM330BX_ODR_15_HZ          0x3
#define ISM330BX_ODR_30_HZ          0x4
#define ISM330BX_ODR_60_HZ          0x5
#define ISM330BX_ODR_120_HZ         0x6
#define ISM330BX_ODR_240_HZ         0x7   /* nearest to 200 Hz */
#define ISM330BX_ODR_480_HZ         0x8
#define ISM330BX_ODR_960_HZ         0x9
#define ISM330BX_ODR_1920_HZ        0xA
#define ISM330BX_ODR_3840_HZ        0xB
#define ISM330BX_ODR_7680_HZ        0xC

/* New-gen FS_XL encoding (CTRL8[1:0]) — MONOTONIC (unlike classic FS_XL) */
#define ISM330BX_FS_XL_2G           0x0
#define ISM330BX_FS_XL_4G           0x1
#define ISM330BX_FS_XL_8G           0x2
#define ISM330BX_FS_XL_16G          0x3

/* New-gen FS_G encoding (CTRL6[3:0]) — starts at 125 dps */
#define ISM330BX_FS_G_125DPS        0x0
#define ISM330BX_FS_G_250DPS        0x1
#define ISM330BX_FS_G_500DPS        0x2
#define ISM330BX_FS_G_1000DPS       0x3
#define ISM330BX_FS_G_2000DPS       0x4
#define ISM330BX_FS_G_4000DPS       0x5   /* not present on all variants */

static bool ism330_who_am_i_is_supported(u8 v) {
    return ism330_family_from_who(v) != ISM330_FAMILY_UNKNOWN;
}

/* CTRL3_C bit fields */
#define ISM330_CTRL3C_SW_RESET      BIT(0)
#define ISM330_CTRL3C_IF_INC        BIT(2)  /* auto-increment register address on burst read */
#define ISM330_CTRL3C_BDU           BIT(6)  /* block-data-update: prevents low/high byte mismatch */

/* INT1_CTRL bit fields — route data-ready to the INT1 pin */
#define ISM330_INT1_DRDY_XL         BIT(0)  /* accel data-ready */
#define ISM330_INT1_DRDY_G          BIT(1)  /* gyro  data-ready */

/* STATUS_REG bit fields */
#define ISM330_STATUS_XLDA          BIT(0)  /* new accel sample available */
#define ISM330_STATUS_GDA           BIT(1)  /* new gyro  sample available */
#define ISM330_STATUS_TDA           BIT(2)  /* new temp  sample available */

/* ODR encoding (bits [7:4] of CTRL1_XL / CTRL2_G) — discrete values only */
#define ISM330_ODR_POWER_DOWN       0x0
#define ISM330_ODR_12_5_HZ          0x1
#define ISM330_ODR_26_HZ            0x2
#define ISM330_ODR_52_HZ            0x3
#define ISM330_ODR_104_HZ           0x4
#define ISM330_ODR_208_HZ           0x5    /* nearest supported to 200 Hz */
#define ISM330_ODR_416_HZ           0x6
#define ISM330_ODR_833_HZ           0x7
#define ISM330_ODR_1666_HZ          0x8

/* FS_XL encoding (bits [3:2] of CTRL1_XL) — NOTE non-monotonic ordering
 * (2/16/4/8, not 2/4/8/16 like MPU6050) — a lookup table is used below. */
#define ISM330_FS_XL_2G             0x0
#define ISM330_FS_XL_16G            0x1
#define ISM330_FS_XL_4G             0x2
#define ISM330_FS_XL_8G             0x3

/* FS_G encoding (bits [3:2] of CTRL2_G) — monotonic; coincidentally lines
 * up with the mpu6050_gyro_fs enum so a direct shift works. */
#define ISM330_FS_G_250DPS          0x0
#define ISM330_FS_G_500DPS          0x1
#define ISM330_FS_G_1000DPS         0x2
#define ISM330_FS_G_2000DPS         0x3

/* Module parameter: prefer FIFO path */
#if 0
static bool use_fifo = true;
module_param(use_fifo, bool, 0644);
MODULE_PARM_DESC(use_fifo, "Use FIFO for data capture (default: true)");
#endif
/*--------------regmap---------------*/
static const struct regmap_config mpu6050_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0x7F,
    .cache_type = REGCACHE_NONE,
};

/*--------------- Driver private state ----------------*/
struct mpu6050_priv {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;

    /* configuration cache*/
    u32 odr_hz; /* Output Data Rate in Hz */
    mpu6050_accel_fs accel_fs; /* accel FSR enum */
    mpu6050_gyro_fs gyro_fs; /* gyro FSR enum */
    bool running;                 /* Streaming enabled */

    /* Chip register-layout family — detected from WHO_AM_I at probe.
     * All FS/ODR access goes through mpu6050_set_ranges / mpu6050_set_odr
     * which dispatch to the correct implementation based on this. */
    enum ism330_family family;

    /* Calibration bias/scale */
    struct mpu6050_cal cal_accel;
    struct mpu6050_cal cal_gyro;

    /* char device */
    dev_t devt;
    struct cdev cdev;
    struct class *cls;
    struct device *chardev;

    /* sample FIFO and synchronization */
    DECLARE_KFIFO(sample_fifo, struct mpu6050_sample, MPU6050_FIFO_SAMPLES);
    wait_queue_head_t wq;
    spinlock_t fifo_lock; /* protects sample_fifo */

/* IRQ + Timer */
    int irq;
    bool irq_mode; /* true if using IRQ path */
    ktime_t hrt_period;
    struct mutex io_lock; /* serialize read/write/ioctl */
};

/* ------------Helpers---------------*/
static int mpu6050_write( struct mpu6050_priv *p, u8 reg, u8 val)
{
    return regmap_write(p->regmap, reg, val);
}

static int mpu6050_read( struct mpu6050_priv *p, u8 reg, unsigned int *val)
{
    return regmap_read(p->regmap, reg, val);
}

static inline int mpu6050_read_burst(struct mpu6050_priv *p, struct mpu6050_sample *s)
{
    u8 buf[14];
    int ret;

    /* ISM330 layout with IF_INC=1 (auto-increment) starting at OUT_TEMP_L:
     *   buf[0..1]   OUT_TEMP  (temp L, temp H)             — little-endian
     *   buf[2..7]   gyro  X, Y, Z                          — LE pairs each
     *   buf[8..13]  accel X, Y, Z                          — LE pairs each
     *
     * NOTE the two differences vs. the MPU6050 layout the driver used to
     * assume: (a) LSB-first, and (b) temp precedes gyro precedes accel. */
    ret = regmap_bulk_read(p->regmap, ISM330_REG_OUT_TEMP_L, buf, sizeof(buf));
    if (ret) return ret;

    s->temp = (s16)((buf[1]  << 8) | buf[0]);
    s->gx   = (s16)((buf[3]  << 8) | buf[2]);
    s->gy   = (s16)((buf[5]  << 8) | buf[4]);
    s->gz   = (s16)((buf[7]  << 8) | buf[6]);
    s->ax   = (s16)((buf[9]  << 8) | buf[8]);
    s->ay   = (s16)((buf[11] << 8) | buf[10]);
    s->az   = (s16)((buf[13] << 8) | buf[12]);
    s->t_ns = ktime_get_boottime_ns();
    return 0;
}

/* ---- Classic-family set_ranges (ISM330DLC/DHCX/DSO/DSOX/DSL/DSM/DSR) ----
 * FS_XL at CTRL1_XL[3:2] (non-monotonic 2/16/4/8), FS_G at CTRL2_G[3:2]. */
static int ism330_classic_set_ranges(struct mpu6050_priv *p)
{
    static const u8 fs_xl_map[4] = {
        [ACCEL_2G]  = ISM330_FS_XL_2G,   /* 0 */
        [ACCEL_4G]  = ISM330_FS_XL_4G,   /* 2 */
        [ACCEL_8G]  = ISM330_FS_XL_8G,   /* 3 */
        [ACCEL_16G] = ISM330_FS_XL_16G,  /* 1 */
    };
    static const u8 fs_g_map[4] = {
        [GYRO_250DPS]  = ISM330_FS_G_250DPS,
        [GYRO_500DPS]  = ISM330_FS_G_500DPS,
        [GYRO_1000DPS] = ISM330_FS_G_1000DPS,
        [GYRO_2000DPS] = ISM330_FS_G_2000DPS,
    };
    int ret;
    u8 fs_xl = fs_xl_map[p->accel_fs];
    u8 fs_g  = fs_g_map[p->gyro_fs];

    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL1_XL,
                             GENMASK(3, 2), (u8)(fs_xl << 2));
    if (ret) return ret;
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL2_G,
                             GENMASK(3, 2), (u8)(fs_g << 2));
    return ret;
}

/* ---- New-gen set_ranges (LSM6DSV / ISM330BX / LSM6DSV32X) ----
 * FS_XL at CTRL8[1:0] (monotonic 2/4/8/16), FS_G at CTRL6[3:0]. Note that
 * CTRL1/CTRL2 hold ONLY ODR + OP_MODE here — no FS bits at all. */
static int ism330_newgen_set_ranges(struct mpu6050_priv *p)
{
    /* Monotonic mapping — mpu6050_accel_fs enum (0/1/2/3 = 2/4/8/16 g)
     * maps 1:1 to ISM330BX FS_XL (0/1/2/3 = 2/4/8/16 g). Table kept for
     * clarity + future-proofing against enum reordering. */
    static const u8 fs_xl_map[4] = {
        [ACCEL_2G]  = ISM330BX_FS_XL_2G,
        [ACCEL_4G]  = ISM330BX_FS_XL_4G,
        [ACCEL_8G]  = ISM330BX_FS_XL_8G,
        [ACCEL_16G] = ISM330BX_FS_XL_16G,
    };
    /* mpu6050_gyro_fs enum starts at 250 dps, but ISM330BX FS_G starts at
     * 125 dps — so ADD 1 to shift the mapping (250→code 1, 2000→code 4). */
    static const u8 fs_g_map[4] = {
        [GYRO_250DPS]  = ISM330BX_FS_G_250DPS,   /* 1 */
        [GYRO_500DPS]  = ISM330BX_FS_G_500DPS,   /* 2 */
        [GYRO_1000DPS] = ISM330BX_FS_G_1000DPS,  /* 3 */
        [GYRO_2000DPS] = ISM330BX_FS_G_2000DPS,  /* 4 */
    };
    int ret;
    u8 fs_xl = fs_xl_map[p->accel_fs];
    u8 fs_g  = fs_g_map[p->gyro_fs];

    /* Accel FS at CTRL8[1:0] — preserve HP_LPF2_XL_BW bits [7:5]. */
    ret = regmap_update_bits(p->regmap, ISM330BX_REG_CTRL8,
                             GENMASK(1, 0), fs_xl);
    if (ret) return ret;
    /* Gyro FS at CTRL6[3:0] — preserve LPF1_G_BW bits [5:4]. */
    ret = regmap_update_bits(p->regmap, ISM330BX_REG_CTRL6,
                             GENMASK(3, 0), fs_g);
    return ret;
}

/* Dispatch to the correct set_ranges based on detected chip family. */
static int mpu6050_set_ranges(struct mpu6050_priv *p)
{
    if ((unsigned)p->accel_fs > ACCEL_16G || (unsigned)p->gyro_fs > GYRO_2000DPS)
        return -EINVAL;
    switch (p->family) {
    case ISM330_FAMILY_CLASSIC: return ism330_classic_set_ranges(p);
    case ISM330_FAMILY_NEWGEN:  return ism330_newgen_set_ranges(p);
    default:
        dev_err(p->dev, "set_ranges: chip family not detected\n");
        return -ENODEV;
    }
}

/* Common: snap a requested Hz to the nearest supported ODR from a lookup
 * table. Returns the table index of the best match. best_diff is set to 0
 * when the requested rate is exactly supported. */
struct odr_entry { u32 hz; u8 code; };
static size_t odr_snap_nearest(const struct odr_entry *tbl, size_t n,
                               u32 hz, u32 *best_diff)
{
    size_t i, best = 0;
    u32 bd = U32_MAX;
    for (i = 0; i < n; i++) {
        u32 d = (tbl[i].hz > hz) ? (tbl[i].hz - hz) : (hz - tbl[i].hz);
        if (d < bd) { bd = d; best = i; }
    }
    if (best_diff) *best_diff = bd;
    return best;
}

/* ---- Classic-family set_odr (ODR at CTRL1_XL[7:4] / CTRL2_G[7:4]) ---- */
static int ism330_classic_set_odr(struct mpu6050_priv *p, u32 hz)
{
    static const struct odr_entry tbl[] = {
        {   13, ISM330_ODR_12_5_HZ },
        {   26, ISM330_ODR_26_HZ   },
        {   52, ISM330_ODR_52_HZ   },
        {  104, ISM330_ODR_104_HZ  },
        {  208, ISM330_ODR_208_HZ  },
        {  416, ISM330_ODR_416_HZ  },
        {  833, ISM330_ODR_833_HZ  },
        { 1666, ISM330_ODR_1666_HZ },
    };
    u8 code;
    int ret;

    if (hz == 0) {
        code = ISM330_ODR_POWER_DOWN;
        p->odr_hz = 0;
    } else {
        u32 diff;
        size_t i = odr_snap_nearest(tbl, ARRAY_SIZE(tbl), hz, &diff);
        code = tbl[i].code;
        p->odr_hz = tbl[i].hz;
        if (diff != 0)
            dev_info(p->dev, "classic: ODR request %u Hz snapped to %u Hz\n",
                     hz, p->odr_hz);
    }
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL1_XL,
                             GENMASK(7, 4), (u8)(code << 4));
    if (ret) return ret;
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL2_G,
                             GENMASK(7, 4), (u8)(code << 4));
    return ret;
}

/* ---- New-gen set_odr (ODR at CTRL1[3:0] / CTRL2[3:0], DIFFERENT rates) ---- */
static int ism330_newgen_set_odr(struct mpu6050_priv *p, u32 hz)
{
    /* Fractional rates rounded to nearest whole Hz for the integer table
     * (1.875→2, 7.5→8). The actual physical rate is what the chip emits. */
    static const struct odr_entry tbl[] = {
        {    2, ISM330BX_ODR_1_875_HZ  },
        {    8, ISM330BX_ODR_7_5_HZ    },
        {   15, ISM330BX_ODR_15_HZ     },
        {   30, ISM330BX_ODR_30_HZ     },
        {   60, ISM330BX_ODR_60_HZ     },
        {  120, ISM330BX_ODR_120_HZ    },
        {  240, ISM330BX_ODR_240_HZ    },   /* nearest to 200 */
        {  480, ISM330BX_ODR_480_HZ    },
        {  960, ISM330BX_ODR_960_HZ    },
        { 1920, ISM330BX_ODR_1920_HZ   },
        { 3840, ISM330BX_ODR_3840_HZ   },
        { 7680, ISM330BX_ODR_7680_HZ   },
    };
    u8 code;
    int ret;

    if (hz == 0) {
        code = ISM330BX_ODR_POWER_DOWN;
        p->odr_hz = 0;
    } else {
        u32 diff;
        size_t i = odr_snap_nearest(tbl, ARRAY_SIZE(tbl), hz, &diff);
        code = tbl[i].code;
        p->odr_hz = tbl[i].hz;
        if (diff != 0)
            dev_info(p->dev, "new-gen: ODR request %u Hz snapped to %u Hz\n",
                     hz, p->odr_hz);
    }
    /* ODR field is bits [3:0] on new-gen — preserve OP_MODE bits [6:4]. */
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL1_XL,
                             GENMASK(3, 0), code);
    if (ret) return ret;
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL2_G,
                             GENMASK(3, 0), code);
    return ret;
}

/* Dispatch to the correct set_odr based on chip family. */
static int mpu6050_set_odr(struct mpu6050_priv *p, u32 hz)
{
    int ret;
    switch (p->family) {
    case ISM330_FAMILY_CLASSIC: ret = ism330_classic_set_odr(p, hz); break;
    case ISM330_FAMILY_NEWGEN:  ret = ism330_newgen_set_odr(p, hz);  break;
    default:
        dev_err(p->dev, "set_odr: chip family not detected\n");
        return -ENODEV;
    }
    /* hrt_period is kept in step for any downstream code that reads it;
     * this driver itself does not run an hrtimer. */
    p->hrt_period = ktime_set(0, NSEC_PER_SEC / (p->odr_hz ? p->odr_hz : 1));
    return ret;
}

/*-------------IRQ and Timer -----------------*/
static irqreturn_t mpu6050_irq_thread(int irq, void *data)
{
    struct mpu6050_priv *p = data;
    unsigned int st;
    struct mpu6050_sample s;
    unsigned long flags;

    if (!p->irq_mode) return IRQ_NONE;

    /* Consult STATUS_REG for XLDA (accel data-ready). Reading the data
     * register (below) is what actually deasserts INT1 on ISM330. */
    if (regmap_read(p->regmap, ISM330_REG_STATUS_REG, &st)) return IRQ_NONE;
    if (!(st & ISM330_STATUS_XLDA))                       return IRQ_NONE;

    if (mpu6050_read_burst(p, &s) == 0) {
        spin_lock_irqsave(&p->fifo_lock, flags);
        if (!kfifo_is_full(&p->sample_fifo))
            kfifo_in(&p->sample_fifo, &s, 1);
        spin_unlock_irqrestore(&p->fifo_lock, flags);
        wake_up_interruptible(&p->wq);
    }
    return IRQ_HANDLED;
}

static int mpu6050_set_irq_mode(struct mpu6050_priv *p, bool enable)
{
    int ret;
    /* On ISM330, INT1 is routed via INT1_CTRL: bit 0 = accel DRDY,
     * bit 1 = gyro DRDY. We route accel DRDY only — accel and gyro
     * data-ready are synchronised at the same ODR, so routing both
     * would just double-fire the interrupt. */
    if (enable && p->irq) {
        ret = regmap_write(p->regmap, ISM330_REG_INT1_CTRL, ISM330_INT1_DRDY_XL);
        if (!ret) p->irq_mode = true;
    } else {
        ret = regmap_write(p->regmap, ISM330_REG_INT1_CTRL, 0);
        if (!ret) p->irq_mode = false;
    }
    return ret;
}


/* ---------------------Character Device --------------------- */
static long mpu6050_unlocked_ioctl( struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct mpu6050_priv *p = filp->private_data;
    int ret = 0;
    unsigned int v;
    struct mpu6050_fs fs ;
    struct mpu6050_cal_pair cp;
    
    switch(cmd) {
        case MPU6050_IOC_GET_WHOAMI: {
            v = 0;
            mpu6050_read(p, ISM330_REG_WHO_AM_I, &v);
            if (copy_to_user((void __user*)arg, &v, sizeof(v)))
                return -EFAULT;
            break;
        }
        case MPU6050_IOC_GET_ODR: {
            v = p->odr_hz;
            if (copy_to_user((void __user *)arg, &v, sizeof(v)))
                return -EFAULT;
            break;
        }
        case MPU6050_IOC_SET_ODR: {
            if (copy_from_user(&v, (void __user *)arg, sizeof(v)))
                return -EFAULT;
            mutex_lock(&p->io_lock);
            ret = mpu6050_set_odr(p, v);
            mutex_unlock(&p->io_lock);
            if (ret)
                return ret;
            break;
        }
        case MPU6050_IOC_SET_FS: {
            memset(&fs, 0, sizeof(fs));
            if( copy_from_user(&fs, (void __user *)arg, sizeof(fs)))
                return -EFAULT;
            mutex_lock(&p->io_lock);
            p->accel_fs = fs.accel;
            p->gyro_fs = fs.gyro;
            ret = mpu6050_set_ranges(p);
            mutex_unlock(&p->io_lock);
            if (ret)
                return ret;
            break;
        }
        case MPU6050_IOC_GET_FS: {
            memset(&fs, 0, sizeof(fs));
            fs.accel = p->accel_fs;
            fs.gyro  = p->gyro_fs;
            if (copy_to_user((void __user *)arg, &fs, sizeof(fs)))
                return -EFAULT;
            break;
        }
       
        default:
            return -ENOTTY;
    }
    return 0;
}

static ssize_t mpu6050_read_file(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    struct mpu6050_priv *p = f->private_data;
    size_t count = len/sizeof(struct mpu6050_sample);
    size_t done = 0;

    if( count == 0 ) return -EINVAL;
    dev_info(p->dev, "read request for %zu samples\n", count);

    while( done < count ){
        if (p->irq_mode) {
            struct mpu6050_sample s;
            if (kfifo_is_empty(&p->sample_fifo)) {
                if (f->f_flags & O_NONBLOCK) {
                    return done ? (ssize_t)(done * sizeof(s)) : -EAGAIN;
                }
                if (wait_event_interruptible(p->wq, !kfifo_is_empty(&p->sample_fifo))) {
                return done ? (ssize_t)(done*sizeof(s)) : -ERESTARTSYS;
            }
        }
        spin_lock(&p->fifo_lock);
        if(!kfifo_out(&p->sample_fifo, &s, 1))
        {
            spin_unlock(&p->fifo_lock);
            continue;
        }
        spin_unlock(&p->fifo_lock);
        if( copy_to_user(buf+done*sizeof(s), &s, sizeof(s)))
        return -EFAULT;
        done++;

        } else {
            /* On demand snapshot mode */
            struct mpu6050_sample s; int ret;
            size_t off;
            mutex_lock(&p->io_lock);
            ret = mpu6050_read_burst(p, &s);
            
            mutex_unlock(&p->io_lock);
            if (ret)
                return done ? (ssize_t)(done * sizeof(s)) : ret;
            off = done * sizeof(struct mpu6050_sample);
            if (copy_to_user((void __user *)(buf + off), &s, sizeof(s)))
                return -EFAULT;
            dev_info(p->dev, "snapshot read: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d temp=%d\n",
                s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp);
            done++;
    
        }
        
    }
    return (ssize_t)(done * sizeof(struct mpu6050_sample));
}

static __poll_t mpu6050_poll(struct file *f, struct poll_table_struct *pt)
{
    struct mpu6050_priv *p = f->private_data;
    __poll_t mask = 0;
    dev_info(p->dev, "poll\n");
    if( !p->irq_mode ){
        poll_wait(f, &p->wq, pt);
        if( !kfifo_is_empty(&p->sample_fifo)) mask |= POLLIN | POLLRDNORM;
    } else {
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}

static int mpu6050_open(struct inode *ino, struct file *f)
{
    struct mpu6050_priv *p = container_of(ino->i_cdev, struct mpu6050_priv, cdev);
    f->private_data = p;
    return 0;
}

static int mpu6050_release( struct inode *ino, struct file *f)
{
    return 0;
}

static const struct file_operations mpu6050_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = mpu6050_unlocked_ioctl,
    .open = mpu6050_open,
    .release = mpu6050_release,
    .read = mpu6050_read_file,
    .poll = mpu6050_poll,
    #ifdef CONFIG_COMPAT
    .compat_ioctl = mpu6050_unlocked_ioctl,
    #endif
};

/*----------SYSFS----------*/
static ssize_t show_odr(struct device *dev, struct device_attribute *a, char *buf)
{
    struct mpu6050_priv *p = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%u\n", p->odr_hz);
}

static ssize_t store_odr(struct device *dev, struct device_attribute *a, const char *b, size_t c)
{
    struct mpu6050_priv *p = dev_get_drvdata(dev);
    unsigned int v;
    if( kstrtouint(b, 0, &v))
        return -EINVAL;
    mutex_lock(&p->io_lock);
    mpu6050_set_odr(p, v);
    mutex_unlock(&p->io_lock);
    return c;
}

static DEVICE_ATTR(odr_hz, 0644, show_odr, store_odr);

static ssize_t show_fs(struct device *dev, struct device_attribute *a, char *buf)
{
    struct mpu6050_priv *p = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%u %u\n", p->accel_fs, p->gyro_fs);
}

static ssize_t store_fs( struct device *dev, struct device_attribute *a, const char *b, size_t c)
{
    struct mpu6050_priv *p = dev_get_drvdata(dev);
    unsigned int af, gf;
    if( sscanf(b, "%u %u", &af, &gf ) != 2)
        return -EINVAL;
    mutex_lock(&p->io_lock);
    p->accel_fs = af;
    p->gyro_fs = gf;
    mpu6050_set_ranges(p);
    mutex_unlock(&p->io_lock);
    return c;
}

static DEVICE_ATTR(fullscale, 0644, show_fs, store_fs);

static ssize_t show_irq_mode(struct device *dev, struct device_attribute *a, char *buf)
{
    struct mpu6050_priv *p = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%u", p->irq_mode ? 1 : 0);
}

static ssize_t store_irq_mode(struct device *dev, struct device_attribute *a, const char *buf, size_t count)
{
    struct mpu6050_priv *p = dev_get_drvdata(dev);
    unsigned int v;
    if( kstrtouint(buf, 0, &v))
        return -EINVAL;
    mutex_lock(&p->io_lock);
    mpu6050_set_irq_mode(p, v != 0);
    mutex_unlock(&p->io_lock);
    return count;
}
static DEVICE_ATTR(irq_mode, 0644, show_irq_mode, store_irq_mode);

static struct attribute *mpu6050_attrs[] = {
    &dev_attr_odr_hz.attr,
    &dev_attr_fullscale.attr,
    &dev_attr_irq_mode.attr,
    NULL,
};

static const struct attribute_group mpu6050_attr_group = {
    .attrs = mpu6050_attrs,
};

/*--------------probe/remove/pm-------------------*/

/* Decode-and-print every register the driver touches. Useful for confirming
 * that writes actually landed on the chip and that the FS/ODR encoding is
 * what the datasheet says. All fields are labelled and printed in both raw
 * hex and decoded engineering units. Safe to call multiple times (e.g.
 * BEFORE and AFTER a write) — it only issues reads. */
static void mpu6050_dump_regs(struct mpu6050_priv *p, const char *tag)
{
    /* Classic FS_XL (non-monotonic 2/16/4/8) and monotonic FS_G */
    static const u16 classic_fs_xl_g[4]   = { 2, 16, 4, 8 };
    static const u16 classic_fs_g_dps[4]  = { 250, 500, 1000, 2000 };
    static const u16 classic_odr_hz[16]   = {
        [ISM330_ODR_POWER_DOWN] = 0,
        [ISM330_ODR_12_5_HZ]    = 13,
        [ISM330_ODR_26_HZ]      = 26,
        [ISM330_ODR_52_HZ]      = 52,
        [ISM330_ODR_104_HZ]     = 104,
        [ISM330_ODR_208_HZ]     = 208,
        [ISM330_ODR_416_HZ]     = 416,
        [ISM330_ODR_833_HZ]     = 833,
        [ISM330_ODR_1666_HZ]    = 1666,
    };
    /* New-gen FS_XL (monotonic 2/4/8/16) and FS_G starting at 125 */
    static const u16 newgen_fs_xl_g[4]    = { 2, 4, 8, 16 };
    static const u16 newgen_fs_g_dps[16]  = {
        [ISM330BX_FS_G_125DPS]  = 125,
        [ISM330BX_FS_G_250DPS]  = 250,
        [ISM330BX_FS_G_500DPS]  = 500,
        [ISM330BX_FS_G_1000DPS] = 1000,
        [ISM330BX_FS_G_2000DPS] = 2000,
        [ISM330BX_FS_G_4000DPS] = 4000,
    };
    static const u16 newgen_odr_hz[16] = {
        [ISM330BX_ODR_POWER_DOWN] = 0,
        [ISM330BX_ODR_1_875_HZ]   = 2,
        [ISM330BX_ODR_7_5_HZ]     = 8,
        [ISM330BX_ODR_15_HZ]      = 15,
        [ISM330BX_ODR_30_HZ]      = 30,
        [ISM330BX_ODR_60_HZ]      = 60,
        [ISM330BX_ODR_120_HZ]     = 120,
        [ISM330BX_ODR_240_HZ]     = 240,
        [ISM330BX_ODR_480_HZ]     = 480,
        [ISM330BX_ODR_960_HZ]     = 960,
        [ISM330BX_ODR_1920_HZ]    = 1920,
        [ISM330BX_ODR_3840_HZ]    = 3840,
        [ISM330BX_ODR_7680_HZ]    = 7680,
    };
    unsigned int who = 0, c1 = 0, c2 = 0, c3 = 0, i1 = 0, st = 0;
    unsigned int c6 = 0, c8 = 0;   /* only meaningful on new-gen */
    int r = 0;

    r |= mpu6050_read(p, ISM330_REG_WHO_AM_I,   &who);
    r |= mpu6050_read(p, ISM330_REG_CTRL1_XL,   &c1);
    r |= mpu6050_read(p, ISM330_REG_CTRL2_G,    &c2);
    r |= mpu6050_read(p, ISM330_REG_CTRL3_C,    &c3);
    r |= mpu6050_read(p, ISM330_REG_INT1_CTRL,  &i1);
    r |= mpu6050_read(p, ISM330_REG_STATUS_REG, &st);
    if (p->family == ISM330_FAMILY_NEWGEN) {
        r |= mpu6050_read(p, ISM330BX_REG_CTRL6, &c6);
        r |= mpu6050_read(p, ISM330BX_REG_CTRL8, &c8);
    }
    if (r) {
        dev_warn(p->dev, "[%s] register dump: read failed (rc=%d)\n", tag, r);
        return;
    }

    dev_info(p->dev,
        "[%s] family=%s  WHO_AM_I=0x%02X  CTRL1=0x%02X CTRL2=0x%02X CTRL3_C=0x%02X "
        "INT1_CTRL=0x%02X STATUS=0x%02X\n",
        tag, ism330_family_name(p->family), who, c1, c2, c3, i1, st);

    if (p->family == ISM330_FAMILY_CLASSIC) {
        u8 odr_xl = (c1 >> 4) & 0x0F;
        u8 fs_xl  = (c1 >> 2) & 0x03;
        u8 odr_g  = (c2 >> 4) & 0x0F;
        u8 fs_g   = (c2 >> 2) & 0x03;
        u16 acc_g   = classic_fs_xl_g[fs_xl];
        u16 gyr_dps = classic_fs_g_dps[fs_g];
        u16 acc_hz  = classic_odr_hz[odr_xl];
        u16 gyr_hz  = classic_odr_hz[odr_g];
        dev_info(p->dev,
            "[%s]   accel: ODR=%u Hz FS=±%u g (bits: ODR_XL=%u FS_XL=%u)  --  "
            "gyro: ODR=%u Hz FS=±%u dps (bits: ODR_G=%u FS_G=%u)\n",
            tag, acc_hz, acc_g, odr_xl, fs_xl, gyr_hz, gyr_dps, odr_g, fs_g);
    } else if (p->family == ISM330_FAMILY_NEWGEN) {
        u8 op_xl  = (c1 >> 4) & 0x07;
        u8 odr_xl = c1 & 0x0F;
        u8 op_g   = (c2 >> 4) & 0x07;
        u8 odr_g  = c2 & 0x0F;
        u8 fs_xl  = c8 & 0x03;
        u8 fs_g   = c6 & 0x0F;
        u16 acc_g   = newgen_fs_xl_g[fs_xl];
        u16 gyr_dps = newgen_fs_g_dps[fs_g];
        u16 acc_hz  = newgen_odr_hz[odr_xl];
        u16 gyr_hz  = newgen_odr_hz[odr_g];
        dev_info(p->dev,
            "[%s]   CTRL6=0x%02X (FS_G bits=%u)  CTRL8=0x%02X (FS_XL bits=%u)\n",
            tag, c6, fs_g, c8, fs_xl);
        dev_info(p->dev,
            "[%s]   accel: ODR=%u Hz FS=±%u g (bits: OP_MODE=%u ODR=%u FS=%u)  --  "
            "gyro: ODR=%u Hz FS=±%u dps (bits: OP_MODE=%u ODR=%u FS=%u)\n",
            tag, acc_hz, acc_g, op_xl, odr_xl, fs_xl,
                 gyr_hz, gyr_dps, op_g,  odr_g,  fs_g);
    }

    dev_info(p->dev,
        "[%s]   CTRL3_C: BDU=%d IF_INC=%d SW_RESET=%d   INT1_CTRL: DRDY_XL=%d DRDY_G=%d   "
        "STATUS: XLDA=%d GDA=%d TDA=%d\n",
        tag,
        !!(c3 & ISM330_CTRL3C_BDU),
        !!(c3 & ISM330_CTRL3C_IF_INC),
        !!(c3 & ISM330_CTRL3C_SW_RESET),
        !!(i1 & ISM330_INT1_DRDY_XL),
        !!(i1 & ISM330_INT1_DRDY_G),
        !!(st & ISM330_STATUS_XLDA),
        !!(st & ISM330_STATUS_GDA),
        !!(st & ISM330_STATUS_TDA));
}

static int mpu6050_hw_init(struct mpu6050_priv *p)
{
    int ret, tries;
    unsigned int v = 0;

    /* 1. Software reset via CTRL3_C SW_RESET. Datasheet says the bit
     *    self-clears within ~50 µs — we poll with a generous timeout. */
    ret = mpu6050_write(p, ISM330_REG_CTRL3_C, ISM330_CTRL3C_SW_RESET);
    if (ret) {
        dev_err(p->dev, "CTRL3_C SW_RESET write failed: %d\n", ret);
        return ret;
    }
    for (tries = 0; tries < 20; tries++) {
        usleep_range(1000, 2000);
        ret = mpu6050_read(p, ISM330_REG_CTRL3_C, &v);
        if (ret) return ret;
        if (!(v & ISM330_CTRL3C_SW_RESET)) break;
    }
    if (v & ISM330_CTRL3C_SW_RESET) {
        dev_err(p->dev, "SW reset did not clear within timeout\n");
        return -EIO;
    }

    /* 2. WHO_AM_I probe.
     *    Read BOTH candidate WHO_AM_I addresses so we can positively
     *    identify the chip family without guessing:
     *      0x0F  — ST convention (ISM330DLC/DHCX, LSM6DS*, LSM6DSV, ISM330BX)
     *      0x75  — Invensense convention (MPU-6050/6500/9250)
     *
     *    A chip on the I²C bus responds to ANY address you read, whether
     *    that register is defined or not — so both reads will return SOME
     *    value. What matters is which one matches a KNOWN chip ID.
     *
     *    Known WHO_AM_I values that reach this driver:
     *      0x68 → MPU-6050          (Invensense, MPU6050 register map)
     *      0x6A → ISM330DLC         (ST, ISM330DLC layout)
     *      0x6B → ISM330DHCX        (ST, ISM330DHCX layout)
     *      0x6C → LSM6DSO/LSM6DSOX  (ST, LSM6DSO layout)
     *      0x70 → LSM6DSV / MPU-6500 depending on which register hits
     *      0x71 → MPU-9250 (@0x75) OR ISM330BX (@0x0F)  ← ambiguous
     *      0x73 → LSM6DSV32X / MPU-9255
     */
    {
        unsigned int who_st = 0, who_inv = 0;
        int r_st, r_inv;

        r_st  = mpu6050_read(p, 0x0F, &who_st);
        r_inv = mpu6050_read(p, 0x75, &who_inv);
        dev_info(p->dev,
            "WHO_AM_I probe: ST@0x0F=0x%02X (rc=%d)  Invensense@0x75=0x%02X (rc=%d)\n",
            who_st, r_st, who_inv, r_inv);

        /* Best-effort identity — the FIRST address that yields a plausibly-
         * valid non-zero, non-0xFF WHO_AM_I wins. The full driver still
         * needs to target the corresponding register map though; if this
         * comes back showing e.g. 0x71 at BOTH addresses, that strongly
         * suggests MPU-9250 (0x75 is the real WHO_AM_I and 0x0F is a
         * factory self-test byte that coincidentally holds 0x71). */
        if (r_st == 0 && who_st != 0x00 && who_st != 0xFF)
            v = who_st;
        else if (r_inv == 0 && who_inv != 0x00 && who_inv != 0xFF)
            v = who_inv;
        else {
            dev_err(p->dev, "No usable WHO_AM_I from either 0x0F or 0x75\n");
            return -ENODEV;
        }
    }

    if (!ism330_who_am_i_is_supported((u8)v)) {
        dev_err(p->dev,
            "WHO_AM_I=0x%02X is NOT a supported ST 6-axis IMU ID. "
            "Classic family: 0x6A=ISM330DLC, 0x6B=ISM330DHCX/LSM6DSR, 0x6C=LSM6DSO/LSM6DSOX. "
            "New-gen family: 0x70=LSM6DSV, 0x71=ISM330BX, 0x73=LSM6DSV32X. "
            "0x71 is AMBIGUOUS — it also matches MPU-9250 (Invensense); compare "
            "the 'ST@0x0F' vs 'Invensense@0x75' probe above to disambiguate.\n",
            v);
        return -ENODEV;
    }
    p->family = ism330_family_from_who((u8)v);
    dev_info(p->dev, "Detected %s (WHO_AM_I=0x%02X, family=%s)\n",
             ism330_who_am_i_name((u8)v), v,
             ism330_family_name(p->family));

    /* 3. CTRL3_C: enable BDU (block-data-update — freezes low/high byte
     *    pair until both are read, preventing torn reads) and IF_INC
     *    (auto-increment register address for burst read). */
    ret = mpu6050_write(p, ISM330_REG_CTRL3_C,
                        ISM330_CTRL3C_BDU | ISM330_CTRL3C_IF_INC);
    if (ret) {
        dev_err(p->dev, "CTRL3_C config failed: %d\n", ret);
        return ret;
    }

    /* 4. Defaults — accel ±8g / gyro ±2000 dps @ 208 Hz.
     *    Sensitivities: 4096 LSB/g and 16.384 LSB/dps (= 32768/2000).
     *    Userspace calibration constants MUST match (see
     *    dr_dead_reckoning_app_ins15_v*.c: cal_set_defaults_from_lsq()
     *    and GYRO_LSB_PER_DPS).
     *    NOTE ISM330 has no exact 200 Hz — the nearest supported ODR is
     *    208 Hz. If a legacy config passes 200, set_odr() snaps it up. */
    if (!p->odr_hz)   p->odr_hz   = 208;
    if (!p->accel_fs) p->accel_fs = ACCEL_8G;
    if (!p->gyro_fs)  p->gyro_fs  = GYRO_2000DPS;

    /* Snapshot BEFORE any FS/ODR write so we can attribute any mismatch
     * later to a specific write step. Registers should read all zeros
     * here immediately after CTRL3_C reconfiguration (post-reset). */
    mpu6050_dump_regs(p, "pre-configure");

    ret = mpu6050_set_ranges(p);
    if (ret) {
        dev_err(p->dev, "FS range config failed: %d\n", ret);
        return ret;
    }
    mpu6050_dump_regs(p, "after set_ranges");

    ret = mpu6050_set_odr(p, p->odr_hz);
    if (ret) {
        dev_err(p->dev, "ODR config failed: %d\n", ret);
        return ret;
    }
    mpu6050_dump_regs(p, "after set_odr");

    /* Final verify-and-warn against driver intent. Extracts family-specific
     * register bits and cross-checks against what the driver asked for.
     * Probe stays successful either way — userspace can still inspect via
     * /sys/…/fullscale and /sys/…/odr_hz. */
    {
        static const u16 classic_fs_xl_g[4]   = { 2, 16, 4, 8 };
        static const u16 classic_fs_g_dps[4]  = { 250, 500, 1000, 2000 };
        static const u16 classic_odr_hz[16]   = {
            [ISM330_ODR_POWER_DOWN] = 0,
            [ISM330_ODR_12_5_HZ]    = 13,
            [ISM330_ODR_26_HZ]      = 26,
            [ISM330_ODR_52_HZ]      = 52,
            [ISM330_ODR_104_HZ]     = 104,
            [ISM330_ODR_208_HZ]     = 208,
            [ISM330_ODR_416_HZ]     = 416,
            [ISM330_ODR_833_HZ]     = 833,
            [ISM330_ODR_1666_HZ]    = 1666,
        };
        static const u16 newgen_fs_xl_g[4]    = { 2, 4, 8, 16 };
        static const u16 newgen_fs_g_dps[16]  = {
            [ISM330BX_FS_G_125DPS]  = 125,
            [ISM330BX_FS_G_250DPS]  = 250,
            [ISM330BX_FS_G_500DPS]  = 500,
            [ISM330BX_FS_G_1000DPS] = 1000,
            [ISM330BX_FS_G_2000DPS] = 2000,
            [ISM330BX_FS_G_4000DPS] = 4000,
        };
        static const u16 newgen_odr_hz[16] = {
            [ISM330BX_ODR_POWER_DOWN] = 0,
            [ISM330BX_ODR_1_875_HZ]   = 2,
            [ISM330BX_ODR_7_5_HZ]     = 8,
            [ISM330BX_ODR_15_HZ]      = 15,
            [ISM330BX_ODR_30_HZ]      = 30,
            [ISM330BX_ODR_60_HZ]      = 60,
            [ISM330BX_ODR_120_HZ]     = 120,
            [ISM330BX_ODR_240_HZ]     = 240,
            [ISM330BX_ODR_480_HZ]     = 480,
            [ISM330BX_ODR_960_HZ]     = 960,
            [ISM330BX_ODR_1920_HZ]    = 1920,
            [ISM330BX_ODR_3840_HZ]    = 3840,
            [ISM330BX_ODR_7680_HZ]    = 7680,
        };
        static const u16 fs_xl_expect_g[4]   = {
            [ACCEL_2G] = 2, [ACCEL_4G] = 4, [ACCEL_8G] = 8, [ACCEL_16G] = 16,
        };
        static const u16 fs_g_expect_dps[4]  = {
            [GYRO_250DPS] = 250, [GYRO_500DPS] = 500,
            [GYRO_1000DPS] = 1000, [GYRO_2000DPS] = 2000,
        };
        unsigned int c1 = 0, c2 = 0, c6 = 0, c8 = 0;
        u16 read_accel_g = 0, read_gyro_dps = 0;
        u16 read_odr_xl_hz = 0, read_odr_g_hz = 0;
        u16 want_accel_g   = fs_xl_expect_g[p->accel_fs];
        u16 want_gyro_dps  = fs_g_expect_dps[p->gyro_fs];
        int rd;

        rd  = mpu6050_read(p, ISM330_REG_CTRL1_XL, &c1);
        rd |= mpu6050_read(p, ISM330_REG_CTRL2_G,  &c2);
        if (p->family == ISM330_FAMILY_NEWGEN) {
            rd |= mpu6050_read(p, ISM330BX_REG_CTRL6, &c6);
            rd |= mpu6050_read(p, ISM330BX_REG_CTRL8, &c8);
        }
        if (rd) {
            dev_warn(p->dev, "final read-back failed (ret=%d)\n", rd);
            return 0;
        }

        if (p->family == ISM330_FAMILY_CLASSIC) {
            u8 odr_xl = (c1 >> 4) & 0x0F;
            u8 fs_xl  = (c1 >> 2) & 0x03;
            u8 odr_g  = (c2 >> 4) & 0x0F;
            u8 fs_g   = (c2 >> 2) & 0x03;
            read_accel_g   = classic_fs_xl_g[fs_xl];
            read_gyro_dps  = classic_fs_g_dps[fs_g];
            read_odr_xl_hz = classic_odr_hz[odr_xl];
            read_odr_g_hz  = classic_odr_hz[odr_g];
        } else if (p->family == ISM330_FAMILY_NEWGEN) {
            u8 odr_xl = c1 & 0x0F;
            u8 odr_g  = c2 & 0x0F;
            u8 fs_xl  = c8 & 0x03;
            u8 fs_g   = c6 & 0x0F;
            read_accel_g   = newgen_fs_xl_g[fs_xl];
            read_gyro_dps  = newgen_fs_g_dps[fs_g];
            read_odr_xl_hz = newgen_odr_hz[odr_xl];
            read_odr_g_hz  = newgen_odr_hz[odr_g];
        }

        if (read_accel_g   != want_accel_g  ||
            read_gyro_dps  != want_gyro_dps ||
            read_odr_xl_hz != p->odr_hz     ||
            read_odr_g_hz  != p->odr_hz) {
            dev_warn(p->dev,
                "READBACK MISMATCH (%s): wanted accel=±%ug gyro=±%u dps ODR=%u Hz "
                "-- got accel=±%ug gyro=±%u dps ODR_XL=%u ODR_G=%u\n",
                ism330_family_name(p->family),
                want_accel_g, want_gyro_dps, p->odr_hz,
                read_accel_g, read_gyro_dps, read_odr_xl_hz, read_odr_g_hz);
            dev_warn(p->dev,
                "  → compare the checkpoint dumps above ('pre-configure' vs 'after set_ranges' "
                "vs 'after set_odr') to see which register/bit didn't take our write.\n");
        } else {
            dev_info(p->dev,
                "READBACK OK (%s): accel=±%ug gyro=±%u dps ODR=%u Hz — all match driver intent\n",
                ism330_family_name(p->family),
                read_accel_g, read_gyro_dps, read_odr_xl_hz);
        }
    }

    return 0;
}

static int mpu6050_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct mpu6050_priv *p;
    int ret;
    u32 ag = 8, gd = 2000; /* defaults — ±8g / ±2000 dps (see mpu6050_hw_init) */

    p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
    if (!p) {
        dev_err(dev, "Failed to allocate memory\n");
        return -ENOMEM;
    }
    p->dev = dev;
    p->client = client;
    p->regmap = devm_regmap_init_i2c(client, &mpu6050_regmap_config);
    if (IS_ERR(p->regmap)) {
        ret = PTR_ERR(p->regmap);
        dev_err(dev, "Failed to initialize regmap: %d\n", ret);
        return ret;
    }
    mutex_init(&p->io_lock);
    init_waitqueue_head(&p->wq);
    spin_lock_init(&p->fifo_lock);
    INIT_KFIFO(p->sample_fifo);

    // Keep device always active (no pm_runtime)

    /* Parse DT (optional) */
    device_property_read_u32(dev, "st,odr-hz", &p->odr_hz);
    device_property_read_u32(dev, "st,accel-fsr-g", &ag);
    device_property_read_u32(dev, "st,gyro-fsr-dps", &gd);
    p->accel_fs = (ag <= 2) ? ACCEL_2G :
                   (ag <= 4) ? ACCEL_4G :
                   (ag <= 8) ? ACCEL_8G : ACCEL_16G;
    p->gyro_fs = (gd <= 250) ? GYRO_250DPS :
                  (gd <= 500) ? GYRO_500DPS :
                  (gd <= 1000) ? GYRO_1000DPS : GYRO_2000DPS;

    ret = mpu6050_hw_init(p);            // wakes device and sets ranges/ODR
    if (ret) {
        dev_err(dev, "Failed to initialize device: %d\n", ret);
        return ret;
    }

    /* Character device */
    ret = alloc_chrdev_region(&p->devt, 0, 1, DEVICE_NAME);
    if (ret) {
        dev_err(dev, "Failed to allocate char device region: %d\n", ret);
        return ret;                      // was: goto err_pm;
    }
    cdev_init(&p->cdev, &mpu6050_fops);
    p->cdev.owner = THIS_MODULE;
    ret = cdev_add(&p->cdev, p->devt, 1);
    if (ret) {
        dev_err(dev, "Failed to add char device: %d\n", ret);
        goto err_unreg;
    }
    p->cls = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(p->cls)) {
        ret = PTR_ERR(p->cls);
        dev_err(dev, "Failed to create class: %d\n", ret);
        goto err_cdev;
    }
    p->chardev = device_create(p->cls, dev, p->devt, p, "mpu6050-%d", MINOR(p->devt));
    if (IS_ERR(p->chardev)) {
        ret = PTR_ERR(p->chardev);
        dev_err(dev, "Failed to create device: %d\n", ret);
        goto err_class;
    }
    dev_set_drvdata(p->chardev, p);
    ret = sysfs_create_group(&p->chardev->kobj, &mpu6050_attr_group);
    if (ret) {
        dev_err(dev, "Failed to create sysfs group: %d\n", ret);
        goto err_dev;
    }

    /* IRQ path (optional) */
    p->irq = client->irq;
    if (p->irq) {
        ret = devm_request_threaded_irq(dev, p->irq, NULL, mpu6050_irq_thread,
                                        IRQF_ONESHOT | IRQF_TRIGGER_RISING,
                                        DRIVER_NAME, p);
        if (ret) {
            dev_warn(dev, "Failed to request IRQ %d: %d\n", p->irq, ret);
            p->irq = 0;
        }
    }
    /* Default mode: enable IRQ mode only if an IRQ exists */
    p->irq_mode = false;
    if( p->irq ){
        ret = mpu6050_set_irq_mode(p, true);
        if (ret) {
            dev_warn(dev, "Failed to enable IRQ mode: %d\n", ret);
        }
    }
    
    i2c_set_clientdata(client, p);

    dev_info(dev, "MPU6050 char driver ready (odr=%uHz, af=%d, gf=%d, irq=%d)\n",
             p->odr_hz, p->accel_fs, p->gyro_fs, p->irq);
    return 0;

err_dev:
    device_destroy(p->cls, p->devt);
err_class:
    class_destroy(p->cls);
err_cdev:
    cdev_del(&p->cdev);
err_unreg:
    unregister_chrdev_region(p->devt, 1);
    return ret;
}

// Remove: no PM calls; keep device nodes cleanup only
static int mpu6050_remove(struct i2c_client *client)
{
    struct mpu6050_priv *p = i2c_get_clientdata(client);
    sysfs_remove_group(&p->chardev->kobj, &mpu6050_attr_group);
    device_destroy(p->cls, p->devt);
    class_destroy(p->cls);
    cdev_del(&p->cdev);
    unregister_chrdev_region(p->devt, 1);
    return 0;
}

// Remove all PM ops (system sleep/runtime)
// #ifdef CONFIG_PM_SLEEP
// static int mpu6050_suspend(struct device *dev) { return 0; }
// static int mpu6050_resume(struct device *dev) { return 0; }
// #endif
// static int mpu6050_runtime_suspend(struct device *dev) { return 0; }
// static int mpu6050_runtime_resume(struct device *dev) { return 0; }
// static const struct dev_pm_ops mpu6050_pm_ops = { ... };

static const struct of_device_id mpu6050_of_match[] = {
    /* Current overlay uses this string — ST ISM330DLC is the actual chip. */
    { .compatible = "stm,ism330dlctr" },
    { .compatible = "st,ism330dlc"    },
    /* Legacy alias kept for older overlays / dtbo files that still identify
     * the device as an Invensense MPU6050-compatible node. */
    { .compatible = "invensense,mpu6050-custom" },
    { }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static const struct i2c_device_id mpu6050_id[] = {
    { "mpu6050", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

static struct i2c_driver mpu6050_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = mpu6050_of_match,
        // .pm = &mpu6050_pm_ops,   // removed: no power management
    },
    .probe_new = mpu6050_probe,
    .remove = mpu6050_remove,
    .id_table = mpu6050_id,
};
module_i2c_driver(mpu6050_driver);

MODULE_AUTHOR("Sijeo Philip <sijeo80@gmail.com>");
MODULE_DESCRIPTION("Invensense MPU6050 Character Device Driver");
MODULE_LICENSE("GPL v2");

