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

/* MPU6050 register map */
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_FIFO_EN       0x23
#define MPU6050_REG_INT_ENABLE    0x38
#define MPU6050_REG_INT_STATUS    0x3A
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_USER_CTRL     0x6A
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_PWR_MGMT_2    0x6C
#define MPU6050_REG_FIFO_COUNT_H  0x72
#define MPU6050_REG_FIFO_R_W      0x74
#define MPU6050_REG_WHO_AM_I      0x75

#define MPU6050_WHO_AM_I_ID       0x68

/* Bit field */
#define MPU6050_PWR1_DEVICE_RESET   BIT(7)
#define MPU6050_PWR1_CLKSEL_PLL_X   0x01
#define MPU6050_USERCTRL_FIFO_EN    BIT(6)
#define MPU6050_USERCTRL_FIFO_RST   BIT(2)
#define MPU6050_INT_DATA_RDY_EN     BIT(0)
#define MPU6050_INT_DATA_RDY        BIT(0)
#define MPU6050_PWR1_SLEEP         BIT(6)

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

    ret = regmap_bulk_read(p->regmap, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (ret) return ret;
    s->ax = (s16)((buf[0] << 8) | buf[1]);
    s->ay = (s16)((buf[2] << 8) | buf[3]);
    s->az = (s16)((buf[4] << 8) | buf[5]);
    s->temp = (s16)((buf[6] << 8) | buf[7]);
    s->gx = (s16)((buf[8] << 8) | buf[9]);
    s->gy = (s16)((buf[10] << 8) | buf[11]);
    s->gz = (s16)((buf[12] << 8) | buf[13]);
    s->t_ns = ktime_get_boottime_ns(); 
    return 0;
}

static int mpu6050_set_ranges(struct mpu6050_priv *p )
{
    int ret;
    u8 a = (u8)(p->accel_fs << 3); /* AFS_SEL bits 4:3 */
    u8 g = (u8)(p->gyro_fs << 3);  /* FS_SEL bits 4:3 */
    ret = regmap_update_bits(p->regmap, MPU6050_REG_ACCEL_CONFIG, GENMASK(4, 3), a);
    if (ret) return ret;
    ret = regmap_update_bits(p->regmap, MPU6050_REG_GYRO_CONFIG, GENMASK(4, 3), g);
    return ret;
}

static int mpu6050_set_odr(struct mpu6050_priv *p, u32 hz)
{
    int ret;
    u32 base = 1000; /* with DLPF, gyro output rate is 1KHz */
    u32 div = (hz == 0) ? 0 : clamp_val(base / hz - 1, 0, 255);
    ret = regmap_write(p->regmap, MPU6050_REG_SMPLRT_DIV, div);
    if (ret) return ret;
    p->odr_hz = base / (1 + div);
    p->hrt_period = ktime_set(0, NSEC_PER_SEC / (p->odr_hz ? p->odr_hz : 1));
    return 0;
}

/**
 * @brief Read a single raw sensor burst and push a calibrated, timestamped sample into the FIFO
 */
#if 0
static int mpu6050_read_and_push(struct mpu6050_priv *p)
{
    u8 buf[14];
    int ret;
    struct mpu6050_sample s = {0};

    ret = regmap_bulk_read(p->regmap, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (ret) return ret;
    s.ax = (s16)((buf[0] << 8) | buf[1]);
    s.ay = (s16)((buf[2] << 8) | buf[3]);
    s.az = (s16)((buf[4] << 8) | buf[5]);
    s.temp = (s16)((buf[6] << 8) | buf[7]);
    s.gx = (s16)((buf[8] << 8) | buf[9]);
    s.gy = (s16)((buf[10] << 8) | buf[11]);
    s.gz = (s16)((buf[12] << 8) | buf[13]);
    s.t_ns = ktime_get_boottime_ns(); 
    
    /* Push to FIFO */
    spin_lock(&p->fifo_lock);
    if (!kfifo_is_full(&p->sample_fifo)) {
        kfifo_in(&p->sample_fifo, &s, 1);
        spin_unlock(&p->fifo_lock);
        wake_up_interruptible(&p->wq);
    } else {
        spin_unlock(&p->fifo_lock);
        dev_warn_ratelimited(p->dev, "Sample FIFO Overrun\n");
    }
    return 0;

}
#endif 
/*-------------IRQ and Timer -----------------*/
static irqreturn_t mpu6050_irq_thread(int irq, void *data)
{
    struct mpu6050_priv *p = data;
    unsigned int st;
    struct mpu6050_sample s;
    unsigned long flags;
    
    if(!p->irq_mode)
        return IRQ_NONE;
    
    if( regmap_read(p->regmap, MPU6050_REG_INT_STATUS, &st))
        return IRQ_NONE;
    if( !(st & MPU6050_INT_DATA_RDY))
        return IRQ_NONE;
    
        if( mpu6050_read_burst(p, &s) == 0){
            spin_lock_irqsave(&p->fifo_lock, flags);
            if (!kfifo_is_full(&p->sample_fifo)) {
                kfifo_in(&p->sample_fifo, &s, 1);
            }
            spin_unlock_irqrestore(&p->fifo_lock, flags);
            wake_up_interruptible(&p->wq);
        }
    return IRQ_HANDLED;
}

static int mpu6050_set_irq_mode(struct mpu6050_priv *p, bool enable)
{
    int ret = 0;
    if( enable && p->irq ){
        /* Enable DATA_RDY interrupt on the sensor */
        ret = regmap_write(p->regmap, MPU6050_REG_INT_ENABLE, MPU6050_INT_DATA_RDY_EN);
        if (!ret)
            p->irq_mode = true;
    } else {
        /* Disable interrupts */
        ret = regmap_write(p->regmap, MPU6050_REG_INT_ENABLE, 0);
        if (!ret)
            p->irq_mode = false;
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
            mpu6050_read(p, MPU6050_REG_WHO_AM_I, &v);
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
            mutex_lock(&p->io_lock);
            ret = mpu6050_read_burst(p, &s);
            
            mutex_unlock(&p->io_lock);
            if (ret)
                return done ? (ssize_t)(done * sizeof(s)) : ret;
            size_t off = done * sizeof(struct mpu6050_sample);
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
    int ret;
    unsigned int v;
    /* Reset Device */
    ret = mpu6050_write(p, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (ret) {
        dev_err(p->dev, "Device reset failed: %d\n", ret);
        return ret;
    }

    usleep_range(10000, 15000); /* 10-15ms reset time */
    /* DLPF ~42/44Hz, see datasheet */
    ret = mpu6050_write(p, MPU6050_REG_CONFIG, 0x03);
    if (ret) {
        dev_err(p->dev, "LPF config failed: %d\n", ret);
        return ret;
    
    }
    /* Defaults */
    if(!p->odr_hz) p->odr_hz = 200;
    if(!p->accel_fs) p->accel_fs = ACCEL_4G;
    if(!p->gyro_fs) p->gyro_fs = GYRO_500DPS;
    ret = mpu6050_set_odr(p, p->odr_hz);
    if (ret) {
        dev_err(p->dev, "ODR config failed: %d\n", ret);
        return ret;
    }
    ret = mpu6050_set_ranges(p);
    if (ret) {
        dev_err(p->dev, "Range config failed: %d\n", ret);
        return ret;
    }
    /* Verify WHO_AM_I */
    ret = mpu6050_read(p, MPU6050_REG_WHO_AM_I, &v);
    if (ret) {
        dev_err(p->dev, "Failed to read WHO_AM_I: %d\n", ret);
        return ret;
    }
    if ((v & 0x7E) != MPU6050_WHO_AM_I_ID) {
        dev_err(p->dev, "WHO_AM_I mismatch: 0x%02X\n", v);
        //return -ENODEV;
    }
    dev_info(p->dev, "MPU6050 WHO_AM_I=0x%02X\n", v);
    ret = mpu6050_write(p, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_X);
    if (ret) {
        dev_err(p->dev, "Failed to wake device: %d\n", ret);
        return ret;
    }

    /* Ensure accel axes enabled */
    regmap_write(p->regmap, MPU6050_REG_PWR_MGMT_2, 0x00);
    return 0;
}

static int mpu6050_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct mpu6050_priv *p;
    int ret;
    u32 ag = 4, gd = 500; /* defaults */

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
    device_property_read_u32(dev, "invensense,odr-hz", &p->odr_hz);
    device_property_read_u32(dev, "invensense,accel-fsr-g", &ag);
    device_property_read_u32(dev, "invensense,gyro-fsr-dps", &gd);
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

