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
- Without the DTS/overlay, Linux won’t create the IIO node and the test apps won’t find iio:deviceN.

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
- If buffered app reports “Missing scan enable” or “Permission denied,” run with sudo or adjust udev rules for /sys/bus/iio/devices/** and /dev/iio:device*.
- If IRQ doesn’t fire:
  ```sh
  grep -i mpu6050 /proc/interrupts
  dmesg | tail -n 100
  ```

## Common gotchas

- Wrong address: If i2cdetect shows 0x69, set AD0 → 3V3 and use reg = <0x69> in the overlay.
- No I²C devices: Ensure dtparam=i2c_arm=on and probe bus 1 (`i2cdetect -y 1`).
- INT noise: Keep wire short, use rising edge (0x8). If spurious interrupts occur, try level-high (0x4) and/or debounce in the driver.
- Power: Some breakout boards add regulators/level shifters—ensure they’re 3.3 V-safe on SDA/SCL and INT.

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

---

## Notes on robustness & “production-grade” details

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

