// SPDX-License-Identifier: GPL-2.0
/**
 * @file ism330dlctr_char.c
 * @brief ST ISM330DLC-TR IMU character device interface (I2C) - IRQ streaming or on-demand snapshot
 * 
 * DROP-IN REPLACEMENT FOR MPU6050_CHAR.C (same API, different sensor)
 * - Keep the SAME userspace ABI as mpu6050_char.c via mpu6050_ioctl.h
 * - Keep the SAME device node naming: /dev/mpu6050-0
 * - Keep the SAME sysfs attributes: odr_hz, fullscale, irq_mode
 * - Calibration remains in userspace; kernel never does the float math
 * 
 * Modes:
 * 1) IRQ mode (optional): INT1 DATA_READY pushes samples into an in-kernel FIFO.
 * 2) Snapshot mode (default): each read() performs one synchronous I2C burst and returns one sample 
 * 
 * Notes:
 * - IRQ is disables by default, even if an IRQ is present
 *  You can enable via: echo 1 > /sys/class/mpu6050/mpu6050-0/irq_mode
 * - IMPORTANT: ABI leak fix: struct mpu6050_sample contains floats; we always memset() sample to zero
 * 
 * Build:
 * - Keep your existing application unchanged (same header)
 * 
 * Optional DT properties (we accept both old, "invensense, *" and new "st, *" compatible properties for flexibility):
 * - "invensense,odr-hz" or "st,odr-hz" (default 104Hz, u32)
 * - "invensense,accel-fsr-g " or "st,accel-fsr-g" (default 2/4/8/16g, u32)
 * - "invensense,gyro-fsr-dps" or "st,gyro-fsr-dps" (default 250/500/1000/2000 dps, u32)
 * 
 * 
 */

 #include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
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
#define DRIVER_NAME "ism330dlctr-char"
#define DEVICE_NAME "mpu6050"  // Keep same device name for drop-in replacement
#define ISM330DLCTR_MAX_DEVICES 8
#define ISM330DLCTR_FIFO_SAMPLES   2048 /* in-kernel sample FIFO depth */

/* -------------------------------- ISM330DLCTR register map (Common ST Layout)--------------------*/
#define ISM_REG_FUNC_CFG_ACCESS    0x01

#define ISM_REG_WHO_AM_I           0x0F
#define ISM_REG_INT1_CTRL          0x0D
#define ISM_REG_INT2_CTRL          0x0E
#define ISM_REG_CTRL1_XL           0x10
#define ISM_REG_CTRL2_G            0x11
#define ISM_REG_CTRL3_C            0x12

#define ISM_REG_STATUS_REG         0x1E
#define ISM_REG_OUT_TEMP_L        0x20 /* 14 bytes: TEMP(2), G(6), XL(6)*/

#define ISM_WHO_AM_I_ID            0x6A

#define ISM_CTRL3_SW_RESET        BIT(0)
#define ISM_CTRL3_IF_INC          BIT(2)
#define ISM_CTRL3_BDU            BIT(6) /* block data update */

/* INT1_CTRL bits (common): DRDY routing */
#define ISM_INT1_DRDY_XL         BIT(0)
#define ISM_INT1_DRDY_G          BIT(1)

/* Masks */
#define ISM_ODR_MASK             0xF0 /* ODR bits are [7:4] */
#define ISM_FS_MASK             0x0C /* FS bits are [3:2] */

/*---------------------------regmap-----------------------------*/
static const struct regmap_config ism330_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0x7F,
    .cache_type = REGCACHE_NONE,
};

/*--------------------Driver Private Data ----------------------*/
struct ism330_priv {
    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;

    /* configuration cache  */
    u32 odr_hz; /* Output Data Rate in Hz */
    mpu6050_accel_fs accel_fs; /* accel FSR enum */
    mpu6050_gyro_fs gyro_fs;   /* gyro FSR enum */

    /* (kept for ABI parity; not used in kernel )*/
    struct mpu6050_cal cal_accel;
    struct mpu6050_cal cal_gyro;

    /* char device */
    dev_t devt;
    struct cdev cdev;
    struct class *cls;
    struct device *chardev;

    /* sample FIFO and synchronization */
    DECLARE_KFIFO(sample_fifo, struct mpu6050_sample, ISM330DLCTR_FIFO_SAMPLES);
    wait_queue_head_t wq;
    spinlock_t fifo_lock;

    /*IRQ*/
    int irq;
    bool irq_mode; /* true if using IRQ path */
    struct mutex io_lock; /* protects I2C operations */
};

/*------------------ Helpers ----------------------------*/
static int ism_write( struct ism330_priv *p, u8 reg, u8 val)
{
    return regmap_write(p->regmap, reg, val);
}

static int ism_read( struct ism330_priv *p, u8 reg, unsigned int *val)
{
    return regmap_read(p->regmap, reg, val);
}

/* Convert requested Hz to ISM330 ODR code in [7:4] */
static u8 ism_odr_code_from_hz(u32 hz)
{
    /**
     * Common ST ODR encoding (see datasheet):
     * 0 : Power-down
     * 1 : 12.5 Hz
     * 2 : 26 Hz
     * 3 : 52 Hz
     * 4 : 104 Hz
     * 5 : 208 Hz
     * 6 : 416 Hz
     * 7 : 833 Hz
     * 8 : 1.66 kHz
     */
    if (hz == 0) return 0x00;
    if (hz <= 13) return 0x10;
    if (hz <= 26) return 0x20;
    if (hz <= 52) return 0x30;
    if (hz <= 104) return 0x40;
    if (hz <= 208) return 0x50;
    if (hz <= 416) return 0x60;
    if (hz <= 833) return 0x70;
    return 0x80; /* max */
}

static u32 ism_hz_from_odr_code(u8 code)
{
    switch (code & 0xF0) {
        case 0x00: return 0;
        case 0x10: return 12;
        case 0x20: return 26;
        case 0x30: return 52;
        case 0x40: return 104;
        case 0x50: return 208;
        case 0x60: return 416;
        case 0x70: return 833;
        case 0x80: return 1660;
        default: return 0;
    }
}

static int ism_set_odr(struct ism330_priv *p, u32 hz)
{
    int ret;
    u8 odr = ism_odr_code_from_hz(hz);

    /* CTRL1_XL ODR_XL[7:4], CTRL2_G ODR_G[7:4] */
    ret = regmap_update_bits(p->regmap, ISM_REG_CTRL1_XL, ISM_ODR_MASK, odr);
    if (ret) return ret;

    ret = regmap_update_bits(p->regmap, ISM_REG_CTRL2_G, ISM_ODR_MASK, odr);
    if (ret) return ret;

    p->odr_hz = ism_hz_from_odr_code(odr);
    return 0;
}

/**
 * Full-scale mapping 
 * We reuse your existing enums from mpu6050_ioctl.h for compatibility
 * Accel: 2/4/8/16g
 * Gyro: 250/500/1000/2000 dps
 * 
 * Many ST IMUs use:
 * XL FS_XL[3:2] = 00:2g, 10:4g, 11:8g, 01:16g
 * G FS_G[3:2]  = 00:250dps, 01:500dps, 10:1000dps, 11:2000dps
 */

 static int ism_set_ranges(struct ism330_priv *p)
 {
    int ret;
    u8 xl_fs_bits, g_fs_bits;

    switch (p->accel_fs) {
        case ACCEL_2G: xl_fs_bits = 0x00; break;
        case ACCEL_4G: xl_fs_bits = 0x08; break;
        case ACCEL_8G: xl_fs_bits = 0x0C; break;
        case ACCEL_16G: xl_fs_bits = 0x04; break;
        default:  xl_fs_bits = 0x00; break; /* default to 2g */
 }

    switch (p->gyro_fs) {
        case GYRO_250DPS: g_fs_bits = 0x00; break;
        case GYRO_500DPS: g_fs_bits = 0x04; break;
        case GYRO_1000DPS: g_fs_bits = 0x08; break;
        case GYRO_2000DPS: g_fs_bits = 0x0C; break;
        default: g_fs_bits = 0x00; break; /* default to 250dps */
    }

    ret = regmap_update_bits(p->regmap, ISM_REG_CTRL1_XL, ISM_FS_MASK, xl_fs_bits);
    if (ret) return ret;

    ret = regmap_update_bits(p->regmap, ISM_REG_CTRL2_G, ISM_FS_MASK, g_fs_bits);
    return ret;
}

/* Read 14 bytes: TEMP, GXYZ, AXYZ, Fill ABI Sample (raw int 16 + timestamp )*/
static inline int ism_read_sample(struct ism330_priv *p, struct mpu6050_sample *s)
{
    u8 buf[14];
    int ret;

    ret = regmap_bulk_read(p->regmap, ISM_REG_OUT_TEMP_L, buf, sizeof(buf));
    if (ret) return ret;

    s->temp  = (s16)((buf[1] << 8) | buf[0]);

    s->gx = (s16)((buf[3] << 8) | buf[2]);
    s->gy = (s16)((buf[5] << 8) | buf[4]);
    s->gz = (s16)((buf[7] << 8) | buf[6]);
    s->ax = (s16)((buf[9] << 8) | buf[8]);
    s->ay = (s16)((buf[11] << 8) | buf[10]);
    s->az = (s16)((buf[13] << 8) | buf[12]);

    s->t_ns = ktime_get_boottime_ns();
    return 0;
}

static void ism_fifo_reset(struct ism330_priv *p)
{
    unsigned long flags;
    spin_lock_irqsave(&p->fifo_lock, flags);
    kfifo_reset(&p->sample_fifo);
    spin_unlock_irqrestore(&p->fifo_lock, flags);
}

/* IRQ enable/disable: INT1 data-ready */
static int ism_set_irq_mode(struct ism330_priv *p, bool enable)
{
    int ret;

    if( enable && p->irq ) {
        /* Route DRDY (XL+G) to INT1 */
        ret = regmap_write(p->regmap, ISM_REG_INT1_CTRL, ISM_INT1_DRDY_XL | ISM_INT1_DRDY_G);
        if ( !ret ){
            p->irq_mode = true;
            ism_fifo_reset(p); /* clear FIFO when enabling IRQ mode */
        } 
    }else {
        ret = regmap_write(p->regmap, ISM_REG_INT1_CTRL, 0x00);
        if( !ret ){
            p->irq_mode = false;
            ism_fifo_reset(p); /* clear FIFO when disabling IRQ mode */
        }
    }
    return ret;
}

/*----------------------IRQ Thread ----------------------------*/
static irqreturn_t ism_irq_thread(int irq, void *data)
{
    struct ism330_priv *p = data;
    struct mpu6050_sample s;
    unsigned long flags;
    if (!p->irq_mode)
        return IRQ_NONE;

    /* ABI leak fix: always clear full struct (includes float bytes )*/
    memset(&s, 0, sizeof(s));

    if( ism_read_sample(p, &s) == 0){
        spin_lock_irqsave(&p->fifo_lock, flags);
        if (!kfifo_is_full(&p->sample_fifo)) {
            kfifo_in(&p->sample_fifo, &s, 1);
        }
        spin_unlock_irqrestore(&p->fifo_lock, flags);
        wake_up_interruptible(&p->wq);
    }

    return IRQ_HANDLED;

}

/* ----------------------Character Device -----------------------------*/
static long ism330_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct ism330_priv *p = filp->private_data;
    int ret=0;
    unsigned int v = 0;
    struct mpu6050_fs fs ;

    switch(cmd) {
        case MPU6050_IOC_GET_WHOAMI: 
            ret = ism_read(p, ISM_REG_WHO_AM_I, &v);
            if (ret) return ret;
            if (copy_to_user((void __user*)arg, &v, sizeof(v)))
                return -EFAULT;
            return 0;

        case MPU6050_IOC_GET_ODR:
            v = p->odr_hz;
            if (copy_to_user((void __user*)arg, &v, sizeof(v)))
                return -EFAULT;
            return 0;

        case MPU6050_IOC_SET_ODR:
            if (copy_from_user(&v, (void __user *)arg, sizeof(v)))
                return -EFAULT;
            mutex_lock(&p->io_lock);
            ret = ism_set_odr(p, v);
            mutex_unlock(&p->io_lock);
            return ret;
        
        case MPU6050_IOC_SET_FS:
            memset(&fs, 0, sizeof(fs));
            if( copy_from_user(&fs, (void __user *)arg, sizeof(fs)))
                return -EFAULT;
            mutex_lock(&p->io_lock);
            p->accel_fs = fs.accel;
            p->gyro_fs = fs.gyro;
            ret = ism_set_ranges(p);
            mutex_unlock(&p->io_lock);
            return ret;

        case MPU6050_IOC_GET_FS:
            memset(&fs, 0, sizeof(fs));
            fs.accel = p->accel_fs;
            fs.gyro  = p->gyro_fs;
            if (copy_to_user((void __user *)arg, &fs, sizeof(fs)))
                return -EFAULT;
            return 0;

        default:
            return -ENOTTY;
        
    }
    
}


static ssize_t ism330_read_file(struct file *f, char __user *buf, size_t len, loff_t *ppos)
{
    struct ism330_priv *p = f->private_data;
    size_t count = len/sizeof(struct mpu6050_sample);
    size_t done = 0;

    if (count == 0)
        return -EINVAL;

    while( done < count ){
        if (p->irq_mode) {
            struct mpu6050_sample s;
            if (kfifo_is_empty(&p->sample_fifo)) {
                if (f->f_flags & O_NONBLOCK) {
                    return done ? (ssize_t)(done * sizeof(s)) : -EAGAIN;
                }
                if (wait_event_interruptible(p->wq, !kfifo_is_empty(&p->sample_fifo))) {
                    return done ? (ssize_t)(done * sizeof(s)) : -ERESTARTSYS;
                }
            }
            spin_lock(&p->fifo_lock);
            if(!kfifo_out(&p->sample_fifo, &s, 1))
            {
                spin_unlock(&p->fifo_lock);
                continue;
            }
            spin_unlock(&p->fifo_lock);
            if( copy_to_user(buf + done * sizeof(s), &s, sizeof(s)))
                return -EFAULT;
            done++;

        } else {
            /* Snapshot mode */
            struct mpu6050_sample s; int ret;
            size_t off = done * sizeof(struct mpu6050_sample);

            memset(&s, 0, sizeof(s)); /* ABI leak fix: clear full struct including float bytes */
            mutex_lock(&p->io_lock);
            ret = ism_read_sample(p, &s);
            mutex_unlock(&p->io_lock);
            if (ret)
                return done ? (ssize_t)(done * sizeof(s)) : ret;
            if (copy_to_user((void __user *)(buf + off), &s, sizeof(s)))
                return -EFAULT;
            done++;
        }
    }
    return (ssize_t)(done * sizeof(struct mpu6050_sample));
}

static __poll_t ism330_poll(struct file *f, struct poll_table_struct *pt)
{
    struct ism330_priv *p = f->private_data;
    __poll_t mask = 0;

    if (!p->irq_mode) {
        poll_wait(f, &p->wq, pt);
        if (!kfifo_is_empty(&p->sample_fifo))
            mask |= POLLIN | POLLRDNORM;
    } else {
        /* In IRQ mode, we can always read (either data or latest sample) */
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}

static int ism330_open(struct inode *ino, struct file *f)
{
    struct ism330_priv *p = container_of(ino->i_cdev, struct ism330_priv, cdev);
    f->private_data = p;
    return 0;   
}

static int ism330_release(struct inode *ino, struct file *f)
{
    return 0;
}

static const struct file_operations ism330_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = ism330_unlocked_ioctl,
    .open = ism330_open,
    .release = ism330_release,
    .read = ism330_read_file,
    .poll = ism330_poll,
    #ifdef CONFIG_COMPAT
    .compat_ioctl = ism330_unlocked_ioctl,
    #endif
};

/*----------------------------------SYSFS---------------------------*/
static ssize_t show_odr(struct device *dev, struct device_attribute *a, char *buf)
{
    struct ism330_priv *p = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%u\n", p->odr_hz);
}

static ssize_t store_odr(struct device *dev, struct device_attribute *a, const char *b, size_t c)
{
    struct ism330_priv *p = dev_get_drvdata(dev);
    unsigned int v;
    if( kstrtouint(b, 0, &v))
        return -EINVAL;
    mutex_lock(&p->io_lock);
    ism_set_odr(p, v);
    mutex_unlock(&p->io_lock);
    return c;
}
static DEVICE_ATTR(odr_hz, 0644, show_odr, store_odr);


static ssize_t show_fs(struct device *dev, struct device_attribute *a, char *buf)
{
    struct ism330_priv *p = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%u %u\n", p->accel_fs, p->gyro_fs);
}

static ssize_t store_fs( struct device *dev, struct device_attribute *a, const char *b, size_t c)
{
    struct ism330_priv *p = dev_get_drvdata(dev);
    unsigned int af, gf;

    if (sscanf(b, "%u %u", &af, &gf) != 2)
        return -EINVAL;

    mutex_lock(&p->io_lock);
    p->accel_fs = af;
    p->gyro_fs = gf;
    ism_set_ranges(p);
    mutex_unlock(&p->io_lock);
    return c;
}
static DEVICE_ATTR(fullscale, 0644, show_fs, store_fs);

static ssize_t show_irq_mode(struct device *dev, struct device_attribute *a, char *buf)
{
    struct ism330_priv *p = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%u", p->irq_mode ? 1 : 0);
}

static ssize_t store_irq_mode(struct device *dev, struct device_attribute *a, const char *buf, size_t count)
{
    struct ism330_priv *p = dev_get_drvdata(dev);
    unsigned int v;
    if( kstrtouint(buf, 0, &v))
        return -EINVAL;
    mutex_lock(&p->io_lock);
    ism_set_irq_mode(p, v != 0);
    mutex_unlock(&p->io_lock);
    return count;
}
static DEVICE_ATTR(irq_mode, 0644, show_irq_mode, store_irq_mode);


static struct attribute *ism330_attrs[] = {
    &dev_attr_odr_hz.attr,
    &dev_attr_fullscale.attr,
    &dev_attr_irq_mode.attr,
    NULL,
};

static const struct attribute_group ism330_attr_group = {
    .attrs = ism330_attrs,
};

/*--------------------- HW Init -------------------------- */
static int ism330_hw_init(struct ism330_priv *p)
{
    int ret;
    unsigned int v;

    /* SW reset */
    ret = ism_write(p, ISM_REG_CTRL3_C, ISM_CTRL3_SW_RESET);
    if (ret) {
        dev_err(p->dev, "Failed to reset device: %d\n", ret);
        return ret;
    }
    usleep_range(10000, 15000); /* wait 10-15ms */

    /* Enable IF_INC + BDU for clean multibyte reading */
    ret = ism_write(p, ISM_REG_CTRL3_C, ISM_CTRL3_IF_INC | ISM_CTRL3_BDU);
    if (ret) {
        dev_err(p->dev, "Failed to configure CTRL3_C: %d\n", ret);
        return ret;
    }

    /* WHO AM I Check */
    ret = ism_read(p, ISM_REG_WHO_AM_I, &v);
    if (ret) {
        dev_err(p->dev, "Failed to read WHO_AM_I: %d\n", ret);
        return ret;
    }
    if ((v & 0xFF) != ISM_WHO_AM_I_ID) {
        dev_err(p->dev, "WHO_AM_I mismatch: expected 0x%02X, got 0x%02X\n",
                ISM_WHO_AM_I_ID, v & 0xFF);
        return -ENODEV;
    }

    /* Defaults (drop-in style like the MPU driver )*/
    if(!p->odr_hz) p->odr_hz = 200; /* will quantize to 208 */
    if(!p->accel_fs) p->accel_fs = ACCEL_4G;
    if(!p->gyro_fs) p->gyro_fs = GYRO_500DPS;

    ret = ism_set_odr(p, p->odr_hz);
    if (ret) {
        dev_err(p->dev, "Failed to set ODR: %d\n", ret);
        return ret;
    }
    ret = ism_set_ranges(p);
    if (ret) {  
        dev_err(p->dev, "Failed to set ranges: %d\n", ret);
        return ret;
    }

    /* Ensure IRQ routing off by default */
    ret = ism_write(p, ISM_REG_INT1_CTRL, 0x00);
    if (ret) {
        dev_err(p->dev, "Failed to disable IRQs: %d\n", ret);
        return ret;
    }
    p->irq_mode = false;
    return 0;
}

/*--------------------- Probe/Remove -------------------------- */
static void read_dt_u32_compat(struct device *dev, const char *k1, const char *k2, u32 *out)
{
    u32 v;
    if (device_property_read_u32(dev, k1, &v) == 0) {
        *out = v;
        return;
    } 
    if (device_property_read_u32(dev, k2, &v) == 0) {
        *out = v;
        return;
    }
}

static int ism330_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct ism330_priv *p;
    int ret;
    u32 ag = 4, gd = 500; /* defaults */

    p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
    if (!p) {
        dev_err(dev, "Failed to allocate memory\n");
        return -ENOMEM;
    }
    p->dev = dev;
    p->client = client;

    p->regmap = devm_regmap_init_i2c(client, &ism330_regmap_config);
    if (IS_ERR(p->regmap)) {
        ret = PTR_ERR(p->regmap);
        dev_err(dev, "Failed to initialize regmap: %d\n", ret);
        return ret;
    }

    mutex_init(&p->io_lock);
    init_waitqueue_head(&p->wq);
    spin_lock_init(&p->fifo_lock);
    INIT_KFIFO(p->sample_fifo);

    /* Parse DT (optional) */
    read_dt_u32_compat(dev, "invensense,odr-hz", "st,odr-hz", &p->odr_hz);
    read_dt_u32_compat(dev, "invensense,accel-fsr-g", "st,accel-fsr-g", &ag);
    read_dt_u32_compat(dev, "invensense,gyro-fsr-dps", "st,gyro-fsr-dps", &gd);

    p->accel_fs = (ag <= 2) ? ACCEL_2G :
                  (ag <= 4) ? ACCEL_4G :
                  (ag <= 8) ? ACCEL_8G : ACCEL_16G;

    p->gyro_fs = (gd <= 250) ? GYRO_250DPS :
                 (gd <= 500) ? GYRO_500DPS :
                    (gd <= 1000) ? GYRO_1000DPS : GYRO_2000DPS;

    ret = ism330_hw_init(p);
    if (ret) {
        dev_err(dev, "Failed to initialize device: %d\n", ret);
        return ret;
    }

    /* Character device */
    ret = alloc_chrdev_region(&p->devt, 0, 1, DEVICE_NAME);
    if (ret) {
        dev_err(dev, "Failed to allocate char device region: %d\n", ret);
        return ret;
    }
    cdev_init(&p->cdev, &ism330_fops);
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
    ret = sysfs_create_group(&p->chardev->kobj, &ism330_attr_group);
    if (ret) {
        dev_err(dev, "Failed to create sysfs group: %d\n", ret);
        goto err_dev;
    }

    /* IRQ setup (optional) */
    p->irq = client->irq;
    p->irq_mode = false; /* disabled by default */
    if( p->irq ){
       ret = devm_request_threaded_irq(dev, p->irq, NULL, ism_irq_thread,
                                        IRQF_ONESHOT | IRQF_TRIGGER_RISING,
                                        DRIVER_NAME, p);
        if (ret) {
            dev_warn(dev, "Failed to request IRQ %d: %d\n", p->irq, ret);
            p->irq = 0;
        }
    }

    i2c_set_clientdata(client, p);
    dev_info(dev, "ISM330DLC-TR character device probed successfully\n");
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

static int ism330_remove(struct i2c_client *client)
{
    struct ism330_priv *p = i2c_get_clientdata(client);

    sysfs_remove_group(&p->chardev->kobj, &ism330_attr_group);
    device_destroy(p->cls, p->devt);
    class_destroy(p->cls);
    cdev_del(&p->cdev);
    unregister_chrdev_region(p->devt, 1);
    return 0;
}

/* --------------------------DT Match + i2c id --------------------------*/

static const struct of_device_id ism330_of_match[] = {
    { .compatible = "st,ism330dlctr", },
    { /* sentinel */ }
};

static const struct i2c_device_id ism330_i2c_id[] = {
    { "ism330dlctr", 0 },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(i2c, ism330_i2c_id);

static struct i2c_driver ism330_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = ism330_of_match,
    },
    .probe_new = ism330_probe,
    .remove = ism330_remove,
    .id_table = ism330_i2c_id,
};

module_i2c_driver(ism330_driver);

MODULE_AUTHOR("Sijeo Philip <sijeo80@gmail.com>");
MODULE_DESCRIPTION("ISM330DLC-TR IMU Character Device Driver (I2C)");
MODULE_LICENSE("GPL v2");

