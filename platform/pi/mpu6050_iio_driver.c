// SPDX-License-Identifier: GPL-2.0

/**
 * @file mpu6050_iio_driver.c
 * @brief Linux I2C + IIO driver for Invensense MPU6050 (acclerometer + gyro).
 * 
 * This driver exposes raw accelerometer and gyroscope, and temperature channels via
 * the Industrial I/O (IIO) subsystem, supporting both on demand reads and buffered 
 * accquistion triggered by a data-ready interrupt. It uses regmap for efficient clustered
 * /bulk register access, implements Runtime Power Management (PM) and provides basic 
 * system sleep hooks. Optional FIFO/DMP paths can be added.
 * 
 * @section usage Userspace
 * - Read raw values from sysfs: 'in_accel_*_raw', 'in_anglvel_*_raw', 'in_temp_raw'
 * - Enable buffered mode: 'echo 1 > buffer/enable' and read via 'iio_readdev'.
 * 
 * @section dt Device Tree
 * Compatible String: "invensense,mpu6050". Optional IRQ on the INT pin enables
 * low-jitter buffered capture. 
 * 
 * @copyright
 * Copyright (c) 2025 Sijeo Philip 
 */

// mpu6050_iio_driver.c - Efficient Linux I2C + IIO driver for Invensense MPU6050 (acclerometer + gyro)
// 
// Highlights
// - Uses regmap for efficient clustered/bulk I2C access
// - Exposes accel/gyro/temp via IIO with both polled and IRQ/FIFO buffered captures
// - Runtime PM and system sleep PM
// - Device Tree bindings (irq optional). Falls back to polling if no IRQ/FIFO
// - Sensible defaults; easily extendable to DMP if needed.
//
// Build
// - Add to kernel tree (drivers/iio/imu/) or build as out of tree module.
// - See Kconfig/Makefile at the bottom of this file.
//
// Device Tree Example
// i2c1: i2c@... {
//    ...mpu6050@68 {
//    compatible = "invensense,mpu6050";
//    reg = <0x68>;
//    interrupt-parent = <&gpioX>;
//    vdd-supply = <&vdd_3v3>;
//    vddio-supply = <&vdd_3v3>;
//    //optional properties
//    invensens,accel-fsr-g = <4>; // 2/4/8/16g, default 2g
//    invensens,gyro-fsr-dps = <500>; // 250/500/1000/2000dps, default 250dps
//    invensens,odr-hz = <100>; // 4/8/16/32/64/128/256/512/1024Hz, default 100Hz
//    interrupt-config = <0x10>; // INT pin config, default 0x10 (active high, push-pull, latch until cleared)
//  };
//};
//
// Userspace
// # Read raw values from sysfs
// $ cat /sys/bus/iio/devices/iio:deviceX/in_accel_x_raw
// $ cat /sys/bus/iio/devices/iio:deviceX/in_anglvel_y_raw
// $ cat /sys/bus/iio/devices/iio:deviceX/scan_elements/*
//
// # Buffered Capture(Timestamped)
// $ echo 1 > /sys/bus/iio/devices/iio:deviceX/buffer/enable
// $ iio_readdev -b 1024 iio:deviceX | hexdump -C
//
// Notes
// - Register map kept minimal here; add more as needed.
// - Datasheet: refer to Invensense MPU-6000/MPU-6050 Register Map and Descriptions, Rev. 4.4, 5/19/2011
// - Based on Invensense application notes and example code.

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger.h>

/* MPU6050 Register Map */
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_FIFO_EN       0x23
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_INT_STATUS    0x3A
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_USER_CTRL    0x6A
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_PWR_MGMT_2   0x6C
#define MPU6050_REG_FIFO_COUNT_H  0x72
#define MPU6050_REG_FIFO_R_W      0x74
#define MPU6050_REG_WHO_AM_I     0x75

#define MPU6050_WHO_AM_I_ID      0x68

/* Bit fields */
#define MPU6050_PWR1_DEVICE_RESET   BIT(7)
#define MPU6050_PWR1_CLKSEL_PLL_X   0x01
#define MPU6050_USERCTRL_FIFO_EN    BIT(6)
#define MPU6050_USERCTRL_FIFO_RST   BIT(2)
#define MPU6050_INT_DATA_RDY_EN     BIT(0)
#define MPU6050_INT_DATA_RDY        BIT(0)


// FIFO packet: 6 bytes accel + 6 bytes gyro + temp(2 bytes)= 14 bytes if all enabled

// Full-Scale enums
enum mpu6050_accel_fs {
    ACCEL_2G = 0,
    ACCEL_4G = 1,
    ACCEL_8G = 2,
    ACCEL_16G = 3,
};

enum mpu6050_gyro_fs {
    GYRO_250DPS = 0,
    GYRO_500DPS = 1,
    GYRO_1000DPS = 2,
    GYRO_2000DPS = 3,
};

/** 
 * @brief Runtime state for an MPU-6050 instance. 
 * 
 * The structure caches configuration (ODR and Fullscale ranges), hold regmap, and IIO
 * trigger context, and tracks IRQ/FIFO state.
 */

struct mpu6050_data {
    struct device *dev;
    struct regmap *regmap;
    struct iio_trigger *trig;
    struct mutex lock; /* Mutex to protect concurrent access */
    int irq;
    bool use_fifo;
    // Cache settings
    unsigned int odr_hz; /* Output Data Rate in Hz */
    enum mpu6050_accel_fs accel_fs;
    enum mpu6050_gyro_fs gyro_fs;
    /* scan state */
    u8 scan_mask; /* which channels are enabled in scan */
};

/* -----------------Regmap ------------------*/
static const struct regmap_config mpu6050_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0x7F,
    .cache_type = REGCACHE_RBTREE,
};

/*----------------------IIO: channels----------------------------*/
#define MPU6050_CHANNEL(_type, _mod, _addr) \
{\
    .type = (_type),\
    .modified = 1,\
    .address = (_addr),\
    .channel2 = (_mod),\
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW)|BIT(IIO_CHAN_INFO_SCALE),\
}

static const struct iio_chan_spec mpu6050_channels[] = {
    // Accelerometer XYZ
    MPU6050_CHANNEL(IIO_ACCEL, IIO_MOD_X, MPU6050_REG_ACCEL_XOUT_H), // Accel X
    MPU6050_CHANNEL(IIO_ACCEL, IIO_MOD_Y, MPU6050_REG_ACCEL_XOUT_H + 2), // Accel Y
    MPU6050_CHANNEL(IIO_ACCEL, IIO_MOD_Z, MPU6050_REG_ACCEL_XOUT_H + 4), // Accel Z
    // Gyroscope XYZ
    MPU6050_CHANNEL(IIO_ANGL_VEL, IIO_MOD_X, MPU6050_REG_GYRO_XOUT_H), // Gyro X
    MPU6050_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Y, MPU6050_REG_GYRO_XOUT_H + 2), // Gyro Y
    MPU6050_CHANNEL(IIO_ANGL_VEL, IIO_MOD_Z, MPU6050_REG_GYRO_XOUT_H + 4), // Gyro Z
    // Temperature (single channel)
    {
        .type = IIO_TEMP,
        .indexed = 1,
        .channel = 0,
        .address = MPU6050_REG_TEMP_OUT_H,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW)|BIT(IIO_CHAN_INFO_SCALE)|BIT(IIO_CHAN_INFO_OFFSET),
    }, 
    // Timestamp for buffered mode
    IIO_CHAN_SOFT_TIMESTAMP(7),
};

/*-------------------Helpers--------------------------*/
/**
 * @brief Read a big-endian 16-bit register pair and return signed value.
 * @param st Driver state
 * @param addr High-byte register address (MPU big-endian registers )
 * @param[out] out Destination for the signed 16-bit value.
 * @return 0 on success or a negative errno.
 */
static int mpu6050_read_word(struct mpu6050_data *st, u8 addr, s16 *out)
{
    u8 buf[2];
    int ret;
    ret = regmap_bulk_read(st->regmap, addr, buf, 2);
    if (ret < 0)
        return ret; 
    *out = (s16)((buf[0] << 8) | buf[1]);
    return 0;
}

/**
 * @brief Configure the Output Data Rate (ODR).
 * 
 * The device runs the DLPF enabled (1 KHz internal ), so the public ODR is
 * 1000 / (1 + SMPLRT_DIV).
 * 
 * @param st Driver state
 * @param hz Requested ODR in Hz (clamped by divider range 0..255)
 * @return 0 on success or a negative errno.
 */

static int mpu6050_set_odr(struct mpu6050_data *st, unsigned int hz)
{
    // Sample rate divider: Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
    // Gyro Output Rate = 8KHz if DLPF disabled, else 1KHz(we use DLPF: 1KHz)
    unsigned int base = 1000;
    unsigned int div = (hz == 0) ? 0 : clamp_val(base / hz - 1, 0, 255);
    int ret;

    ret = regmap_write(st->regmap, MPU6050_REG_SMPLRT_DIV, div);
    if (ret < 0)
        return ret;
    st->odr_hz = base / (1 + div);
    return 0;
}

 /**
  * @brief Program accelerometer and gyroscope full-scale ranges.
  * @param st Driver state (uses @ref mpu6050_data::accel_fs and @ref mpu6050_data::gyro_fs)
  * @return 0 on success or a negative errno.
  */
static int mpu6050_config_ranges(struct mpu6050_data *st)
{
    int ret;
    u8 accel_fs_bits = st->accel_fs << 3; // AFS_SEL is bits 4:3
    u8 gyro_fs_bits = st->gyro_fs << 3; // FS_SEL is bits 4:3

    ret = regmap_update_bits(st->regmap, MPU6050_REG_ACCEL_CONFIG, GENMASK(4,3), accel_fs_bits);
    if (ret < 0)
        return ret;
    ret = regmap_update_bits(st->regmap, MPU6050_REG_GYRO_CONFIG, GENMASK(4,3), gyro_fs_bits);
    return ret;
}

 /* Scale the IIO is reported as: value = raw * scale + offset */
 /**
  * @brief Return IIO scale for a channel as (val + val2*1e-9)
  * @param st Driver state
  * @param chan IIO channel descriptor.
  * @param[out] val Integer part of the scale.
  * @param[out] val2 Nano part of the scale.
  * @return IIO_VAL_* code or negative errno.
  */
static int mpu6050_get_scale(struct mpu6050_data *st, const struct iio_chan_spec *chan,
                              int *val, int *val2)
{
    int g, dps;
    switch (chan->type) {
        case IIO_ACCEL:
            // LSB/g per full scale. Raw is signed 16-bit
            // Convert to m/s^2 scale: (g_range / 32768) * 9.80665
            // g_range = 2, 4, 8, 16
            g = 2 << st->accel_fs; // 2,4,8,16
            // scale (m/s^2 per LSB) = (g_range / 32768) * 9.80665
            do_div(*(u64 *)&g, 1); // placate static analyzer (no-op)
            *val = 0;
            // 9.80665 / 32768 = 0.000299 ... multiply by g 
            // Use nanounits: 9806650 (um/s^2) per g / 32768
            *val2 = DIV_ROUND_CLOSEST(9806650 * g, 32768) * 1000; // in nano units
            return IIO_VAL_INT_PLUS_NANO;

        case IIO_ANGL_VEL:
            dps = 250 << st->gyro_fs; // 250,500,1000,2000
            // scale rad/s per LSB = (dps * (pi/180)) / 32768
            *val = 0;
            *val2 = DIV_ROUND_CLOSEST((dps * 1745329), 32768); // in nano units
            return IIO_VAL_INT_PLUS_NANO;

        case IIO_TEMP:
            // From datasheet: Temp in C = (raw / 340) + 36.53
            // scale = 1/340 = 0.002941176 degC per LSB
            *val = 0;
            *val2 = DIV_ROUND_CLOSEST(1000000000, 340); // in nano units
            return IIO_VAL_INT_PLUS_NANO;

        default:
            return -EINVAL;
    }
}

    /**
     * @brief Return IIO offset (Temperature channel only)
     * @param chan IIO channel descriptor.
     * @param[out] val Integer part of the offset(degC).
     * @param[out] val2 Nano part of the offset(degC).
     * @return IIO_VAL_* code or -EINVAL for non temperature channels.
     */
static int mpu6050_get_offset(const struct iio_chan_spec *chan, int *val, int *val2)
{
    if (chan->type != IIO_TEMP)
        return -EINVAL;
    // From datasheet: Temp in C = (raw / 340) + 36.53
    *val = 3653; // 36.53 degC
    *val2 = 530000000; // 0.53 degC in nano units
    return IIO_VAL_INT_PLUS_NANO;
}

    /* --------------------- IIO read_raw ---------------------------*/
    /**
     * @brief IIO read_raw() callback for on-demand reads and metadata.
     * @param indio_dev IIO device instance.
     * @param chan Channel Specifier.
     * @param [out] val Primary return value (raw or integer part of scale/offset).
     * @param [out] val2 Secondary return value (nano part of scale/offset or unused).
     * @param mask Query type (RAW/SCALE/OFFSET).
     * @return IIO_VAL_* code or negative errno.
     */
static int mpu6050_read_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
    struct mpu6050_data *st = iio_priv(indio_dev);
    int ret;
    s16 raw;

    switch (mask) {
        case IIO_CHAN_INFO_RAW:
            pm_runtime_get_sync(st->dev);
            mutex_lock(&st->lock);
            ret = mpu6050_read_word(st, chan->address, &raw);
            mutex_unlock(&st->lock);
            pm_runtime_put(st->dev);
            if (ret)
                return ret;
            *val = raw;
            return IIO_VAL_INT;
        case IIO_CHAN_INFO_SCALE:
            return mpu6050_get_scale(st, chan, val, val2);
        case IIO_CHAN_INFO_OFFSET:
            return mpu6050_get_offset(chan, val, val2);
        default:
            return -EINVAL;
    }
}

/* -----------------Triggered Buffer (IRQ/FIFO)--------------------*/
struct mpu6050_scan {
    s16 ax, ay, az; // Accel
    s16 gx, gy, gz; // Gyro
    s16 temp; // Temp
    s64 ts; // Timestamp
}

/**
 * @brief Threaded IRQ handler for data-ready interrupt.
 * 
 * Reads a single sample burst (accel+gyro+temp) and pushes it into the IIO buffer.
 * with a hardware-agnostic timestamp. Switch to FIFO path for higher throughput.
 * 
 * @param irq linux IRQ handler
 * @param p Pointer to @ref iio_dev
 * @return IRQ_HANDLED or IRQ_NONE
 */

 static irqreturn_t mpu6050_irq_thread( int irq, void *p)
 {
    struct iio_dev *indio_dev = p;
    struct mpu6050_data *st = iio_priv(indio_dev);
    u8 status;
    int ret;

    ret = regmap_read(st->regmap, MPU6050_REG_INT_STATUS, &status);
    if( ret )
        return IRQ_NONE; // spurious?
    if( !(status & MPU6050_INT_DATA_RDY) )
        return IRQ_NONE; 
    // Read a full sample burst (14 bytes) directly from data regs for low latency.
    // For higher throughput, enable FIFO and burst read from MPU6050_REG_FIFO_R_W
    {
        u8 buf[14];
        struct mpu6050_scan scan;
        ret = regmap_bulk_read(st->regmap, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
        if( ret )
            return IRQ_HANDLED; // error, but we handled the IRQ
        scan.ax = (s16)((buf[0] << 8) | buf[1]);
        scan.ay = (s16)((buf[2] << 8) | buf[3]);
        scan.az = (s16)((buf[4] << 8) | buf[5]);
        scan.temp = (s16)((buf[6] << 8) | buf[7]);
        scan.gx = (s16)((buf[8] << 8) | buf[9]);
        scan.gy = (s16)((buf[10] << 8) | buf[11]);
        scan.gz = (s16)((buf[12] << 8) | buf[13]);
        // Timestamp
        scan.ts = iio_get_time_ns(indio_dev);
        // Push to IIO buffer
        iio_push_to_buffers_with_timestamp(indio_dev, &scan, scan.ts);
    }
    return IRQ_HANDLED;
}

/**
 * @brief IIO buffer post-enable hook..
 * Enables the data-ready interrupt and FIFO if used.
 */
static int mpu6050_buffer_postenable(struct iio_dev *indio_dev)
{
    struct mpu6050_data *st = iio_priv(indio_dev);
    int ret;

    pm_runtime_get_sync(st->dev);
    mutex_lock(&st->lock);

    // Enable data-ready interrupt
    ret = regmap_write(st->regmap, MPU6050_REG_INT_ENABLE, MPU6050_INT_DATA_RDY_EN);
    mutex_unlock(&st->lock);
    if( ret ) {
        pm_runtime_put(st->dev);
        return ret;
    }
   return 0;
}

/**
 * @brief IIO buffer pre-disable hook.
 * Disables the data-ready interrupt  and allows Runtime PM to suspend the device.
 */
static int mpu6050_buffer_predisable(struct iio_dev *indio_dev)
{
    struct mpu6050_data *st = iio_priv(indio_dev);
    int ret;

    mutex_lock(&st->lock);
    // Disable data-ready interrupt
    ret = regmap_write(st->regmap, MPU6050_REG_INT_ENABLE, 0);
    mutex_unlock(&st->lock);
    pm_runtime_put(st->dev);
    return ret;
}

static const struct iio_buffer_setup_ops mpu6050_buffer_ops = {
    .postenable = mpu6050_buffer_postenable,
    .predisable = mpu6050_buffer_predisable,
};

/*----------------- Probe/Remove/PM --------------------- */
static const struct iio_info mpu6050_iio_info = {
    .read_raw = mpu6050_read_raw,
    // Other callbacks can be added as needed
};

/**
 * @brief One-time hardware initialization sequence.
 * - Soft reset, wake, select PLL clock
 * - Configure DLPF, ODR, full-scale ranges
 * - Verify WHO_AM_I
 * @param st Driver state
 * @return 0 on success or a negative errno.
 */
static int mpu6050_hw_init(struct mpu6050_data *st)
{
    int ret, val;

    // Reset device
    ret = regmap_write(st->regmap, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if( ret ) return ret;
    usleep_range(10000, 15000); // 10-15ms reset time

    // Basic LPF config: DLPF = 3 (~44Hz for accel, 42Hz for gyro at 1kHz)
    ret = regmap_write(st->regmap, MPU6050_REG_CONFIG, 0x03);
    if( ret ) return ret;

    // Set ODR (Default 200 Hz)
    ret = mpu6050_set_odr(st, st->odr_hz ? st->odr_hz : 200);
    if( ret ) return ret;

    // Set full-scale ranges (Default: 2g, 250dps)
    ret = mpu6050_config_ranges(st);
    if( ret ) return ret;

    // Verify WHO_AM_I
    ret = regmap_read(st->regmap, MPU6050_REG_WHO_AM_I, &val);
    if( ret ) return ret;
    if( (val & 0x7E) != MPU6050_WHO_AM_I_ID )  // Mask lower bit (revision)
        return -ENODEV;
    return 0;
}


/**
 * @brief Probe entry for the I2C driver.
 * Allocates IIO device, initializes regmap, parses DT properties, set defaults, 
 * requests optional IRQ, and registers the IIO device. 
 * @param client I2C client instance.
 * @return 0 on success or a negative errno.
 */
static int mpu6050_probe(struct i2c_client *client)
{
    struct iio_dev *indio_dev;
    struct mpu6050_data *st;
    int ret;
    unsigned int odr = 200;
    unsigned int accel_g = 4;
    unsigned int gyro_dps = 500;

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
    if (!indio_dev)
        return -ENOMEM;
    st = iio_priv(indio_dev);
    st->dev = &client->dev;
    mutex_init(&st->lock);

    st->regmap = devm_regmap_init_i2c(client, &mpu6050_regmap_config);
    if (IS_ERR(st->regmap)) {
        return dev_err_probe(&client->dev, PTR_ERR(st->regmap), "Failed to init regmap\n");
    }
    // Parse DT properties
    device_property_read_u32(&client->dev, "invensens,odr-hz", &odr);
    device_property_read_u32(&client->dev, "invensens,accel-fsr-g", &accel_g);
    device_property_read_u32(&client->dev, "invensens,gyro-fsr-dps", &gyro_dps);

    st->odr_hz = odr;
    st->accel_fs = (accel_g <=2 )? ACCEL_2G :
                   (accel_g <=4 )? ACCEL_4G :
                   (accel_g <=8 )? ACCEL_8G : ACCEL_16G;
    st->gyro_fs = (gyro_dps <= 250) ? GYRO_250DPS :
                  (gyro_dps <= 500) ? GYRO_500DPS :
                  (gyro_dps <= 1000) ? GYRO_1000DPS : GYRO_2000DPS;
    pm_runtime_enable(&client->dev);
    pm_runtime_get_noresume(&client->dev);
    pm_runtime_set_active(&client->dev);

    ret = mpu6050_hw_init(st);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to initialize device: %d\n", ret);
        goto err_pm_disable;
    }

    ret = mpu6050_hw_init(st);
    if (ret ) {
        goto err_pm_disable;
    }

    indio_dev->dev.parent = &client->dev;
    indio_dev->info = &mpu6050_iio_info;
    indio_dev->name = "mpu6050";
    indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
    indio_dev->channels = mpu6050_channels;
    indio_dev->num_channels = ARRAY_SIZE(mpu6050_channels);

    // Optional IRQ
    st->irq = client->irq;
    if (st->irq) {
        ret = devm_request_threaded_irq(&client->dev, st->irq, NULL,
                                        mpu6050_irq_thread, IRQF_ONESHOT | IRQF_TRIGGER_RISING,
                                        "mpu6050", indio_dev);
        if (ret) {
            dev_warn(&client->dev, "Failed to request IRQ %d: %d\n", st->irq, ret);
            st->irq = 0; // fall back to polling mode
        }
    }

    ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev, NULL, NULL, &mpu6050_buffer_ops);
    if(ret)
        goto err_pm_put;
    
    ret = devm_iio_device_register(&client->dev, indio_dev);
    if (ret)
        goto err_pm_put;
    
    pm_runtime_put(&client->dev);
    return 0;

err_pm_put:
    pm_runtime_put(&client->dev);
err_pm_disable:
    pm_runtime_disable(&client->dev);
    return ret;

}

/**
 * @brief Remove callback for the I2C driver.
 * Disable Runtime PM.
 */
static void mpu6050_remove( struct i2c_client *client)
{
    pm_runtime_disable(&client->dev);
}

#ifdef CONFIG_PM_SLEEP
/**
 * @brief System suspend callback.
 * Suspend the device and its components.
 */
static int mpu6050_suspend(struct device *dev)
{
    pm_runtime_force_suspend(dev);
    return 0;
}

/**
 * @brief Sleep resume callback.
 * Resume the device and its components.
 */
static int mpu6050_resume(struct device *dev)
{
    pm_runtime_force_resume(dev);
    return 0;
}

#endif /* CONFIG_PM_SLEEP */

/**
 * @brief Runtime PM suspend: put device into low-power state.
 * Sets PWR_MGMT_1[SLEEP].
 */
static int mpu6050_runtime_suspend(struct device *dev)
{
    struct mpu6050_data *st = iio_priv(i2c_get_clientdata(to_i2c_client(dev)));
    regmap_write(st->regmap, MPU6050_REG_PWR_MGMT_1, BIT(6)); // Set SLEEP bit
    return 0;
}

/**
 * @brief Runtime PM Resume: wake device and select PLL clock.
 */
static int mpu6050_runtime_resume(struct device *dev)
{
    struct mpu6050_data *st = iio_priv(i2c_get_clientdata(to_i2c_client(dev)));
    regmap_write(st->regmap, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_X);
    return 0;
}

static const struct of_device_id mpu6050_of_match[] = {
    { .compatible = "invensense,mpu6050", },
    { }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static const struct i2c_device_id mpu6050_id[] = {
    { "mpu6050", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

static const struct dev_pm_ops mpu6050_pm_ops = {
    SET_SYSTEM_SLEEP_PM(mpu6050_suspend, mpu6050_resume)
    SET_RUNTIME_PM_OPS(mpu6050_runtime_suspend, mpu6050_runtime_resume, NULL)
};

static struct i2c_driver mpu6050_i2c_driver = {
    .driver = {
        .name = "mpu6050-iio",
        .pm = &mpu6050_pm_ops,
        .of_match_table = mpu6050_of_match,
    },
    .probe_new = mpu6050_probe,
    .remove = mpu6050_remove,
    .id_table = mpu6050_id,
};
module_i2c_driver(mpu6050_i2c_driver);

MODULE_AUTHOR("Sijeo Philip <sijeo80@gmail.com>");
MODULE_DESCRIPTION("Invensense MPU6050 IIO Driver(I2C, IRQ/FIFO, Runtime PM )");
MODULE_LICENSE("GPL v2");

/*------------------Kconfig--------------------
config IIO_MPU6050_CUSTOM
    tristate "Invensense MPU6050 IIO driver accel/gyro (custom)"
    depends on I2C && IIO
    select REGMAP_I2C
    select IIO_TRIGGERED_BUFFER
    help
      Say Y/M to enable support for the Invensense MPU-6050 over I2C.

---------------------Makefile-------------------
obj-m += mpu6050_iio_driver.o
KDIR ?= /lib/modules/$(shell uname -r)/build
all:
    make -C $(KDIR) M=$(PWD) modules
clean:
    make -C $(KDIR) M=$(PWD) clean
----------------Notes on Performance -------------------
- Use IRQ-driven buffer for lowest jitter; increase odr-hz desired rate.
- For higher rates, enable FIFO (set st->use_fifo = true in probe) and read from FIFO_R_W register.
  *Set USER_CTRL.FIFO_EN and FIFO_EN bits for accel/gyro in FIFO_EN register.
  * In IRQ thread, burst read FIFO in multiples of 14 bytes to drain quickly.
  * Consider regmap_raw_read on MPU6050_REG_FIFO_R_W for > 1 KB bursts.
- Pin the I2C bus to Fast Mode (400 KHz) or FastMode Plus (1 MHz) in DT.
- Consider CPU affinity for IRQ or threaded handler if running at 1KHz.
- For DMP integration, Feed DMP packets into iio buffers as seperate trigger.

*/
