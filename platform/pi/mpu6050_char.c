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
#define ISM330_REG_WHO_AM_I         0x0F   /* r/o, returns 0x6A */
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

#define ISM330_WHO_AM_I_ID          0x71

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

static int mpu6050_set_ranges(struct mpu6050_priv *p)
{
    /* mpu6050_accel_fs enum values are (0,1,2,3) = (2g, 4g, 8g, 16g).
     * ISM330 FS_XL bits are (0,1,2,3) = (2g, 16g, 4g, 8g). Lookup table
     * translates the enum to the register value. */
    static const u8 fs_xl_map[4] = {
        [ACCEL_2G]  = ISM330_FS_XL_2G,
        [ACCEL_4G]  = ISM330_FS_XL_4G,
        [ACCEL_8G]  = ISM330_FS_XL_8G,
        [ACCEL_16G] = ISM330_FS_XL_16G,
    };
    /* mpu6050_gyro_fs enum values coincidentally match ISM330 FS_G bits
     * 1:1 (0,1,2,3 = 250,500,1000,2000 dps). Kept as a table for symmetry
     * and to future-proof against enum reordering. */
    static const u8 fs_g_map[4] = {
        [GYRO_250DPS]  = ISM330_FS_G_250DPS,
        [GYRO_500DPS]  = ISM330_FS_G_500DPS,
        [GYRO_1000DPS] = ISM330_FS_G_1000DPS,
        [GYRO_2000DPS] = ISM330_FS_G_2000DPS,
    };
    int ret;
    u8 fs_xl, fs_g;

    if ((unsigned)p->accel_fs > ACCEL_16G || (unsigned)p->gyro_fs > GYRO_2000DPS)
        return -EINVAL;

    fs_xl = fs_xl_map[p->accel_fs];
    fs_g  = fs_g_map[p->gyro_fs];

    /* FS occupies bits [3:2] of each control register — preserve ODR bits. */
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL1_XL,
                             GENMASK(3, 2), (u8)(fs_xl << 2));
    if (ret) return ret;
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL2_G,
                             GENMASK(3, 2), (u8)(fs_g << 2));
    return ret;
}

static int mpu6050_set_odr(struct mpu6050_priv *p, u32 hz)
{
    /* ISM330 has DISCRETE ODRs (no divisor). We snap the requested rate to
     * the nearest supported value, store the achieved rate for reporting,
     * and write the code to bits [7:4] of BOTH CTRL1_XL and CTRL2_G so
     * accel and gyro tick in lockstep. hz == 0 powers both blocks down. */
    static const struct { u32 hz; u8 code; } odr_table[] = {
        {   13, ISM330_ODR_12_5_HZ }, /* nominal 12.5 Hz — stored as 13 for integer table */
        {   26, ISM330_ODR_26_HZ   },
        {   52, ISM330_ODR_52_HZ   },
        {  104, ISM330_ODR_104_HZ  },
        {  208, ISM330_ODR_208_HZ  },
        {  416, ISM330_ODR_416_HZ  },
        {  833, ISM330_ODR_833_HZ  },
        { 1666, ISM330_ODR_1666_HZ },
    };
    size_t i, best = 0;
    u32 best_diff = U32_MAX;
    u8 code;
    int ret;

    if (hz == 0) {
        code = ISM330_ODR_POWER_DOWN;
        p->odr_hz = 0;
    } else {
        for (i = 0; i < ARRAY_SIZE(odr_table); i++) {
            u32 d = (odr_table[i].hz > hz)
                    ? (odr_table[i].hz - hz)
                    : (hz - odr_table[i].hz);
            if (d < best_diff) { best_diff = d; best = i; }
        }
        code       = odr_table[best].code;
        p->odr_hz  = odr_table[best].hz;
        if (best_diff != 0)
            dev_info(p->dev, "ODR request %u Hz snapped to %u Hz (ISM330 has discrete ODRs)\n",
                     hz, p->odr_hz);
    }

    /* ODR field occupies bits [7:4] of each control register — preserve FS bits. */
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL1_XL,
                             GENMASK(7, 4), (u8)(code << 4));
    if (ret) return ret;
    ret = regmap_update_bits(p->regmap, ISM330_REG_CTRL2_G,
                             GENMASK(7, 4), (u8)(code << 4));
    if (ret) return ret;

    /* hrt_period stays for downstream code that may consult it, but this
     * driver does not use an hrtimer (see file header). */
    p->hrt_period = ktime_set(0, NSEC_PER_SEC / (p->odr_hz ? p->odr_hz : 1));
    return 0;
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

    /* 2. WHO_AM_I check — now that the chip has settled, verify identity.
     *    This is a HARD fail: previously this was silently commented out
     *    while the driver was writing MPU6050 register addresses to an
     *    ISM330 chip, misconfiguring it. Never disable this check again. */
    ret = mpu6050_read(p, ISM330_REG_WHO_AM_I, &v);
    if (ret) {
        dev_err(p->dev, "WHO_AM_I read failed: %d\n", ret);
        return ret;
    }
    if (v != ISM330_WHO_AM_I_ID) {
        dev_err(p->dev, "WHO_AM_I mismatch: 0x%02X (expected 0x%02X for ISM330DLC)\n",
                v, ISM330_WHO_AM_I_ID);
        return -ENODEV;
    }
    dev_info(p->dev, "ISM330DLC detected (WHO_AM_I=0x%02X)\n", v);

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

    ret = mpu6050_set_ranges(p);
    if (ret) {
        dev_err(p->dev, "FS range config failed: %d\n", ret);
        return ret;
    }
    ret = mpu6050_set_odr(p, p->odr_hz);
    if (ret) {
        dev_err(p->dev, "ODR config failed: %d\n", ret);
        return ret;
    }

    dev_info(p->dev, "ISM330DLC configured: ODR=%u Hz  accel_fs_enum=%u  gyro_fs_enum=%u\n",
             p->odr_hz, p->accel_fs, p->gyro_fs);
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

