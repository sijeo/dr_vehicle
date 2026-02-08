# MPU-6050 IIO on Raspberry Pi 3 Model B

Guide to wire an MPU-6050 to a Raspberry Pi 3B, load the Industrial I/O (IIO) driver, verify data via sysfs and buffered reads, and run the included test apps.

## Build and load

```sh
make -j"$(nproc)"
sudo insmod mpu6050_iio_driver.ko    # or copy to /lib/modules/... and run depmod
# If loaded as a module and not built-in, you may need:
sudo modprobe industrialio
sudo modprobe industrialio-triggered-buffer
```

## Verify device

```sh
dmesg | tail -n 50
ls -l /sys/bus/iio/devices/
iio_info | sed -n '/Device.*mpu6050/,$p'
```

## Raw reads (sysfs)

```sh
for f in in_accel_x_raw in_accel_y_raw in_accel_z_raw in_anglvel_x_raw in_temp_raw; do
  echo "$f: $(cat /sys/bus/iio/devices/iio:device*/$f)"
done
```

## Buffered stream (timestamped)

```sh
cd /sys/bus/iio/devices/iio:device*
echo 1 | sudo tee buffer/enable
sudo iio_readdev -b 512 . | hexdump -C | head
echo 0 | sudo tee buffer/enable
```

## Raspberry Pi I2C performance notes

- Use a 400 kHz bus for lower latency (add to /boot/config.txt):
  ```ini
  dtparam=i2c_arm_baudrate=400000
  ```
- Keep INT line short; if noisy, try level-high IRQ (0x4) and add simple deglitch in software by discarding repeated IRQs without data-ready status.
- For sustained >400 Hz, enable the MPU FIFO and burst-read from FIFO_R_W in the IRQ thread.

## Troubleshooting

- i2cdetect shows nothing at 0x68/0x69: check 3.3 V power, pull-ups (Pi has 1.8 kΩ), wiring, and AD0 state.
- Driver fails WHO_AM_I: many clones still report 0x68; if not, scope SCL/SDA.
- IRQ never triggers: verify GPIO17 wiring; e.g.:
  ```sh
  grep . /proc/interrupts | grep -i mpu6050
  ```
- Scale off by ~2x/4x: confirm DT full-scale properties match your driver settings.

## Generate docs (optional)

```sh
sudo apt-get update
sudo apt-get install -y doxygen graphviz
cd /path/to/driver
cat > Doxyfile <<'EOF'
# (Add your Doxygen configuration here)
EOF
doxygen Doxyfile
xdg-open docs/html/index.html
```

---

# Test Applications

Two example apps expect an IIO device named "mpu6050" and (for buffered IO) a valid IRQ. Use the overlay below to instantiate the device on I2C1 and wire INT → GPIO17.

## Device Tree overlay (RPi3B)

Save as mpu6050-rpi3-overlay.dts:

```dts
/dts-v1/;
/plugin/;

/ {
    compatible = "brcm,bcm2835";

    fragment@0 {
        target = <&i2c1>;
        __overlay__ {
            #address-cells = <1>;
            #size-cells = <0>;

            mpu6050: mpu6050@68 {
                compatible = "invensense,mpu6050";
                reg = <0x68>;                 // use <0x69> if AD0 is tied high
                interrupt-parent = <&gpio>;
                interrupts = <17 0x8>;        // GPIO17, rising edge (IRQ_TYPE_EDGE_RISING=0x8)

                // Optional defaults (match your driver's DT parsing)
                invensense,accel-fsr-g = <4>;     // 2/4/8/16 g
                invensense,gyro-fsr-dps = <500>;  // 250/500/1000/2000 dps
                invensense,odr-hz = <200>;        // effective ODR with DLPF=on
            };
        };
    };
};
```

## Build & install the overlay

```sh
# 1) Enable I2C if not already
sudo raspi-config    # Interface Options -> I2C -> Enable
# or ensure /boot/config.txt contains:
# dtparam=i2c_arm=on

# (Optional) faster I2C:
# dtparam=i2c_arm_baudrate=400000

# 2) Compile overlay
dtc -@ -I dts -O dtb -o mpu6050-rpi3-overlay.dtbo mpu6050-rpi3-b-overlay.dts

# 3) Install overlay
sudo cp mpu6050-rpi3-overlay.dtbo /boot/firmware/overlays/

# 4) Load overlay via firmware
echo 'dtoverlay=mpu6050-rpi3-overlay' | sudo tee -a /boot/firmware/config.txt

# 5) Reboot
sudo reboot
```

## Post-reboot checks

```sh
sudo apt-get update && sudo apt-get install -y i2c-tools
sudo i2cdetect -y 1                  # Should show 0x68 (or 0x69)
sudo insmod ./mpu6050_iio_driver.ko || sudo modprobe mpu6050-iio
iio_info | sed -n '/Device.*mpu6050/,$p'
ls -l /sys/bus/iio/devices/
```

## Why the overlay is needed

- Instantiates the MPU-6050 on I2C1 at 0x68 (or 0x69 if AD0=1).
- Connects INT to GPIO17, enabling low-jitter buffered capture.
- Sets optional defaults (FSR/ODR) read by your driver from DT.
- Without the DTS/overlay, Linux won't create the IIO node and the test apps won't find iio:deviceN.

## Build and run the test apps

```sh
# Polled sysfs reader (100 ms)
gcc -O2 -Wall -Wextra -o test_mpu6050_iio test_mpu6050_iio.c
./test_mpu6050_iio

# Buffered IIO character-device reader
gcc -O2 -Wall -Wextra -o test_mpu6050_iio_buffered test_mpu6050_iio_buffered.c
sudo ./test_mpu6050_iio_buffered   # buffer/enable usually requires root
```

Tips:
- If buffered app reports "Missing scan enable" or "Permission denied," run with sudo or adjust udev rules for /sys/bus/iio/devices/** and /dev/iio:device*.
- If IRQ doesn't fire:
  ```sh
  grep -i mpu6050 /proc/interrupts
  dmesg | tail -n 100
  ```

## Common gotchas

- Wrong address: If i2cdetect shows 0x69, set AD0 → 3V3 and use reg = <0x69> in the overlay.
- No I²C devices: Ensure dtparam=i2c_arm=on and probe bus 1 (`i2cdetect -y 1`).
- INT noise: Keep wire short, use rising edge (0x8). If spurious interrupts occur, try level-high (0x4) and/or debounce in the driver.
- Power: Some breakout boards add regulators/level shifters—ensure they're 3.3 V-safe on SDA/SCL and INT.

---

# Physical/Electrical connections (MPU-6050 ↔ RPi 3B)

- Voltage: 3.3 V only (do not use 5 V).
- INT is push-pull, active-high by default—connect directly to GPIO17.

| MPU-6050 pin | Raspberry Pi 3B pin           | Signal               |
|--------------|-------------------------------|----------------------|
| VCC          | Pin 1 (3V3) or Pin 17 (3V3)   | 3.3 V power          |
| GND          | Any GND (e.g., Pin 6)         | Ground               |
| SDA          | Pin 3 (GPIO2 / SDA1)          | I2C data             |
| SCL          | Pin 5 (GPIO3 / SCL1)          | I2C clock            |
| INT          | Pin 11 (GPIO17)               | Data-ready interrupt |
| AD0          | GND = 0x68, 3V3 = 0x69        | I2C address select   |



# NEO-6M GNSS (NMEA over UART) — serdev kernel driver

- Exposes parsed fields:
  - `/sys/class/neo6m_gnss/neo6m0/lat` (degrees × 1e7, signed)
  - `/sys/class/neo6m_gnss/neo6m0/lon` (degrees × 1e7, signed)
  - `/sys/class/neo6m_gnss/neo6m0/alt_mm` (mm)
  - `/sys/class/neo6m_gnss/neo6m0/speed_mmps` (mm/s)
  - `/sys/class/neo6m_gnss/neo6m0/have_fix` (0/1)
  - `/sys/class/neo6m_gnss/neo6m0/utc` (ISO 8601)

- Character device: `/dev/neo6m_gnss` → `ioctl(NEO6M_GNSS_IOC_GET_FIX, struct neo6m_gnss_fix*)`

## Build (out of tree)

```bash
make
sudo insmod neo6m_gnss_serdev.ko
# or
sudo modprobe neo6m_gnss_serdev
```

## NEO-6M Wiring (Raspberry Pi 3 Model B)

### Physical Connections

| NEO-6M Pin | Raspberry Pi 3B Pin   | Signal           | Notes     |
|------------|-----------------------|------------------|-----------|
| **VCC**    | Pin 1 or 17 (3V3)     | 3.3V Power       | Required  |
| **GND**    | Pin 6 (GND) | Ground  | Required         |           |
| **TX**     | Pin 10 (RXD0/GPIO15)  | UART Data        | Required  |
| **RX**     | Pin 8 (TXD0/GPIO14)   | UART Data        | Optional* |
| **PPS**    | Pin 12 (GPIO18)       | Pulse Per Second | Optional  |

> **\*** RX connection is only needed if you plan to send UBX/NMEA configuration commands to the module.

### Wiring Guidelines

⚠️ **Important Safety Notes:**
- **Use 3.3V only** - Raspberry Pi UART pins are **NOT** 5V tolerant
- Most NEO-6M breakout boards accept 5V on VCC (internal regulator) but output 3.3V logic
- When in doubt, power from 3.3V to be safe

### Minimum Setup
For basic NMEA data reception, you only need:
1. **VCC** → 3.3V
2. **GND** → Ground  
3. **TX** (NEO-6M) → **RX** (Pi)

### UART Configuration
The NEO-6M uses **9600 baud, 8N1** by default, connected to Raspberry Pi's primary UART (UART0).
Remove serial console from /boot/cmdline.txt (no console=ttyAMA0,...).

Device Tree should bind to uart0:
&uart1 {
    status = "okay";
    neo6m: gnss@0 {
        compatible = "sijeo,neo6m-nmea";
    };
};
This modifications have to be done in the linux kernel bcm2710-rpi-3-b.dts and cross compiled using

make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- dtbs
and then scp the corresponding dtb file to /boot/firmware/

---

## Notes on robustness & "production-grade" details

- **Checksum verified** (`*XX`) before parsing; malformed or partial lines are discarded.
- **Back-pressure safe**: RX path only assembles one line and defers parsing to a **workqueue** to keep interrupt context short.
- **Empty fields** (no fix) do not overwrite prior values unless a field is present; we still mark `have_fix = 0` if GGA fix=0 or RMC status `V`.
- **Units** are SI and integer where possible to avoid FP in kernel:
  - lat/lon as **deg × 1e7** (`int32_t`).
  - altitude **mm**, speed **mm/s**.
- **Locking**: a `spinlock_t` guards the latest fix snapshot for sysfs/ioctl reads.
- **Baud** defaults to **9600 8N1** (NEO-6M default). Changeable in code if you reconfigure the module to 38400/115200 via u-blox CFG.
- **No policy in kernel**: we only parse and expose; consumers (EKF, logging, etc.) stay in user space or other kernel clients via ioctl.

---

## Doxygen

All public structs/functions in the driver and the exported `neo6m_gnss_ioctl.h` include **Doxygen** comments. To generate HTML:

```bash
doxygen -g  # creates Doxyfile
# Edit Doxyfile minimally:
#   PROJECT_NAME           = "NEO6M GNSS Driver"
#   INPUT                  = drivers/ include/
#   RECURSIVE              = YES
#   EXTRACT_ALL            = YES
doxygen
```


'''code
sudo systemctl stop dr_stack.service
sudo systemctl disable dr_stack.service
sudo journalctl -u dr_stack.service -f

sudo systemctl daemon-reload
sudo systemctl enable dr_stack.service
sudo systemctl start dr_stack.service

sudo journalctl -u dr_stack.service -f

'''
'''
|Col  |Header Name  |Variable in Code   |                     Description                                                   |
-----------------------------------------------------------------------------------------------------------------------------
|A    |time_s       |tlog               |Timestamp of the log entry (seconds since boot)                                    |
|B    |E            |C->ins.p.x         |Estimated Position East (meters) relative to the start point                       |
|C    |N            |C->ins.p.y         |Estimated Position North (meters).                                                 |
|D    |U            |C->ins.p.z         |Estimated Position Up (meters).                                                    |
|E    |V_E          |C->ins.v.x         |Estimated Velocity East (m/s).                                                     |        
|F    |V_N          |C->ins.v.y         |Estimated Velocity North (m/s).                                                    |
|G    |V_U          |C->ins.v.z         |Estimated Velocity Up (m/s).                                                       |
|H    |A_E          |C->last_aw.x       |World Acceleration East (m/s²). The raw accel rotated to world frame minus gravity |
|I    |A_N          |C->last_aw.y       |World Acceleration North (m/s²).                                                   |
|J    |A_U          |C->last_aw.z       |World Acceleration Up (m/s²)                                                       |
|K    |yaw_deg      |yaw_deg            |Estimated Yaw/Heading (degrees). calculated from the quaternion C->ins.q.          |
|L    |ba_x         |C->ins.ba.x        |Accel Bias X (m/s²). The filter's estimate of the sensor error.                    |
|M    |ba_y         |C->ins.ba.y        |Accel Bias Y (m/s²).                                                               |
|N    |ba_z         |C->ins.ba.z        |Accel Bias Z (m/s²).                                                               |
|O    |bg_x         |C->ins.bg.x        |Gyro Bias X (rad/s). The filter's estimate of gyro drift.                          |
|P    |bg_y         |C->ins.bg.y        |Gyro Bias Y (rad/s).                                                               |
|Q    |bg_z         |C->ins.bg.z        |Gyro Bias Z (rad/s).                                                               |
|R    |imu_ax       |imu_raw.ax         |Raw Accel X (counts). Direct data from MPU6050 register.                           |
|S    |imu_ay       |imu_raw.ay         |Raw Accel Y (counts).                                                              |
|T    |imu_az       |imu_raw.az         |Raw Accel Z (counts).                                                              |
|U    |imu_gx       |imu_raw.gx         |Raw Gyro X (counts).                                                               |
|V    |imu_gy       |imu_raw.gy         |Raw Gyro Y (counts).                                                               |
|W    |imu_gz       |imu_raw.gz         |Raw Gyro Z (counts).                                                               |
|X    |gnss_fix     |C->gnss_last_fix   |"Fix Status. 1 if GNSS has a fix, 0 if not."                                       |
|Y    |lat_deg      |gnss_last_lat      |GNSS Latitude (degrees). Last received value.                                      |
|Z    |lon_deg      |gnss_last_lon      |GNSS Longitude (degrees). Last received value.                                     |
|AA   |alt_m        |gnss_last_alt      |GNSS Altitude (meters).                                                            |
|AB   |speed_mps    |gnss_last_speed    |GNSS Speed (m/s).                                                                  |
|AC   |heading_deg  |gnss_last_head     |GNSS Heading (degrees). Course over ground from GPS.                               |
|AD   |hdop         |gnss_last_hdop     |HDOP. Horizontal Dilution of Precision (lower is better accuracy).                 |
|AE   |gnss_present |gnss_present       |Signal Presence. 1 if valid GNSS data arrived recently (within 2s).                |
|AF   |gnss_valid   |gnss_valid         |Data Acceptance. 1 if the filter accepted the last GNSS point.                     |
|AG   |gnss_used_pos|last_gnss_used_pos |Update Flag (Pos). 1 if a position update happened in this specific log step.      |
|AH   |gnss_used_vel|last_gnss_used_vel |Update Flag (Vel). 1 if a velocity update happened in this specific log step.      |
|AI   |reacq_active |reacq_active       |Reacquisition Mode. 1 if filter has relaxed tolerances to find GPS again.          |
|AJ   |snap_applied |snap_applied       |Snap Event. 1 if filter forced position to match GPS (hard reset).                 |
|AK   |outage_s     |C->outage_s        |Outage Duration (seconds). How long since the last valid GPS update.               |
|AL   |qscale       |C->qscale          |Uncertainty Scale. Multiplier for process noise (higher = less trusting of IMU).   |
|AM   |nis_pos      |C->last_nis_pos    |"NIS Position. ""Normalized Innovation Squared"". If > 16.2, update was rejected." |
|AN   |nis_vel      |C->last_nis_vel    |NIS Velocity. Quality metric for velocity updates.                                 |
|AO   |gnss_age_s   |gnss_meas_age_s    |Data Latency. How old the last GNSS measurement is (seconds).                      |
-----------------------------------------------------------------------------------------------------------------------------

## Key Columns for Troubleshooting
1. Heading Mismatch: Compare column K (yaw_deg) with column AC (heading_deg)
    > If K stays 0 while AC changes to 90 or 180, Initialization is failing
2. Filter Rejection: Check Column AM (nis_pos)
    > If this value is consistently > 16.2 (or red/high), the filter is rejecting GNSS data
3. Vertical Instability: Check column (V_U) and Column N (ba_z)
    > If G runs away to -10 or 10 m/s, your Z axis is dampening is insufficient.
'''

---

# IMU Migration: MPU6050 → ISM330 — Noise Parameter Update Guide

When switching the IMU from MPU6050 to ISM330 (ISM330DHCX/DLC), the following noise-related
parameters in `dr_dead_reckoning_app_ins15.c` must be reviewed and updated. The ISM330 is
roughly 5–8x better on noise density and bias stability.

## 1. EKF Process Noise (lines 984–988) — Most Critical

These directly drive filter performance. Lowering them tells the filter to trust the IMU more
and rely less on GNSS corrections.

| Parameter  | Current (MPU6050)    | Suggested (ISM330)    | Rationale                                                                 |
|------------|----------------------|-----------------------|---------------------------------------------------------------------------|
| `sigma_a`  | `0.50 m/s²`         | `~0.08–0.12 m/s²`    | ISM330 accel noise density ~60–80 µg/√Hz vs MPU6050 ~400 µg/√Hz          |
| `sigma_g`  | `0.010 rad/s`        | `~0.002–0.004 rad/s`  | ISM330 gyro noise density ~3.8 mdps/√Hz vs MPU6050 ~10 mdps/√Hz          |
| `sigma_ba` | `0.010 m/s²/√s`     | `~0.002–0.005 m/s²/√s`| ISM330 bias stability ~30 µg vs MPU6050 ~500 µg                           |
| `sigma_bg` | `0.005 rad/s/√s`    | `~0.001–0.002 rad/s/√s`| ISM330 gyro bias stability ~1–3 dps vs MPU6050 ~5–20 dps                 |

## 2. Initial Covariance P₀ (lines 876–880)

| Parameter | Current | Suggested (ISM330) | Notes                                                    |
|-----------|---------|---------------------|----------------------------------------------------------|
| `ba0`     | `0.5 m/s²` | `~0.05–0.10 m/s²` | Reflects tighter accel bias bounds                       |
| `bg0`     | `0.01 rad/s`| `~0.003–0.005 rad/s`| Reflects tighter gyro bias bounds                       |
| `p0`      | `5.0 m`    | no change           | Geometry/application-dependent, not sensor-dependent     |
| `v0`      | `1.0 m/s`  | no change           | Geometry/application-dependent, not sensor-dependent     |
| `th0`     | `5.0 deg`  | no change           | Geometry/application-dependent, not sensor-dependent     |

## 3. Stillness Detection Thresholds (lines 93–99)

The ISM330's lower noise floor allows tightening these for better ZUPT/ZARU detection.

| Parameter          | Current              | Suggested (ISM330)    | Notes                                     |
|--------------------|----------------------|-----------------------|-------------------------------------------|
| `YAW_DEADBAND_RAD` | `0.03 rad/s (~1.7°/s)` | `~0.01 rad/s (~0.6°/s)` | Tighter deadband feasible with lower noise |
| `ACC_STILL_TOL`    | `0.10g (~0.98 m/s²)` | `~0.03–0.05g`         | Fewer false positives during gentle driving |
| `GYRO_STILL_TOL_RAD`| `3.0°/s`            | `~1.0°/s`             | Better stillness discrimination            |
| `ZUPT_ACC_THR`     | `0.40 m/s²`          | `~0.15–0.20 m/s²`    | Tighter zero-velocity detection            |
| `ZUPT_GYRO_THR`    | `5.0°/s`             | `~2.0°/s`             | Tighter zero-velocity detection            |

## 4. Gyro Scale Constant (line 138)

```c
#define GYRO_LSB_PER_DPS    65.5f    // MPU6050 @ ±500 dps
```

**Must** change to match the ISM330 full-scale setting:

| ISM330 FS   | LSB/dps |
|-------------|---------|
| ±125 dps    | 228.571 |
| ±250 dps    | 114.286 |
| ±500 dps    | 57.143  |
| ±1000 dps   | 28.571  |
| ±2000 dps   | 14.286  |

If using the same ±500 dps, the value stays 65.5. Otherwise update accordingly.

## 5. Outage-Tiered Q Inflation (lines 110–113)

With a better IMU the filter drifts less during GNSS outages, so these multipliers can be reduced.
Best validated with real outage test data on the ISM330.

| Parameter | Current | Suggested (ISM330) |
|-----------|---------|--------------------|
| `QSCL_A`  | 2.0     | 1.5                |
| `QSCL_B`  | 5.0     | 3.0                |
| `QSCL_C`  | 10.0    | 5.0–7.0            |
| `QSCL_D`  | 20.0    | 10.0–15.0          |

## Priority Order for Tuning

1. **`sigma_a`, `sigma_g`, `sigma_ba`, `sigma_bg`** (lines 985–988) — largest impact on filter behavior
2. **`GYRO_LSB_PER_DPS`** (line 138) — must match ISM330 full-scale config or raw conversion is wrong
3. **Stillness thresholds** (lines 93–99) — tighten for better ZUPT/ZARU
4. **Initial P₀ bias terms** (lines 879–880) — reflect tighter bias bounds
5. **Q inflation tiers** (lines 110–113) — optional, tune empirically