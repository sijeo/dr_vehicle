/**************** RPi3 Model B: Device Tree Overlay (mpu6050-rpi3-overlay.dts) ****************
$(MAKE) -C $(KDIR) M=$(PWD) clean
EOF


# Build and load
$ make -j$(nproc)
$ sudo insmod mpu6050_iio_driver.ko # or copy to /lib/modules/... and depmod
# If built-in via kernel tree, it will autoload via DT; as a module you may need:
$ sudo modprobe industrialio
$ sudo modprobe industrialio-triggered-buffer


# Verify device
$ dmesg | tail -n 50
$ ls -l /sys/bus/iio/devices/
$ iio_info | sed -n '/Device.*mpu6050/,$p'


# Raw reads
$ for f in in_accel_x_raw in_accel_y_raw in_accel_z_raw in_anglvel_x_raw in_temp_raw; do
echo "$f: $(cat /sys/bus/iio/devices/iio:device*/$f)";
done


# Buffered stream (timestamped)
$ cd /sys/bus/iio/devices/iio:device*
$ echo 1 | sudo tee buffer/enable
$ sudo iio_readdev -b 512 . | hexdump -C | head
$ echo 0 | sudo tee buffer/enable


/**************** Notes for Raspberry Pi I2C performance ****************
- Use 400 kHz bus for lower read latency:
Add to /boot/config.txt: dtparam=i2c_arm_baudrate=400000
- Keep INT line short; if noisy, try IRQ_TYPE_LEVEL_HIGH (0x4) and enable deglitch
in software by discarding repeated IRQs without data-ready status.
- If you need sustained >400 Hz, prefer enabling the MPU FIFO and burst-reading
from FIFO_R_W in the IRQ thread. Ask and we’ll drop in that code.


/**************** Troubleshooting ****************
- i2cdetect shows nothing at 0x68/0x69: check power (3.3V), pull-ups (Pi has 1k8),
wiring (SDA/SCL swapped?), and ensure AD0 pin state matches reg address.
- Driver fails WHO_AM_I: many clones still report 0x68; if not, scope SCL/SDA.
- IRQ never triggers: verify GPIO17 wiring; try `grep . /proc/interrupts | grep mpu6050`.
- Scale looks off by ~2x/4x: confirm DT full-scale properties match your use.

To generate docs on your Pi (or dev box):

sudo apt-get install doxygen graphviz   # graphviz optional
cd /path/to/driver
cat > Doxyfile <<'EOF'
# (copy the Doxyfile block from the canvas)
EOF
doxygen Doxyfile
xdg-open docs/html/index.html


## For Test Applications
### Device Tree Changes for test_mpu_iio.c
Use a tiny overlay that places the device on I²C1 and wires INT → GPIO17. Save as mpu6050-rpi3-overlay.dts:

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

                // Optional defaults (match your driver’s parsing)
                invensense,accel-fsr-g = <4>;     // 2/4/8/16 g
                invensense,gyro-fsr-dps = <500>;  // 250/500/1000/2000 dps
                invensense,odr-hz = <200>;        // effective ODR with DLPF=on
            };
        };
    };
};

Build & install the overlay
# 1) Enable I2C if not already:
sudo raspi-config        # Interface Options -> I2C -> Enable
# or ensure /boot/config.txt contains:
#   dtparam=i2c_arm=on

# (Optional) Faster I2C for lower latency:
#   dtparam=i2c_arm_baudrate=400000

# 2) Compile overlay
dtc -@ -I dts -O dtb -o mpu6050-rpi3-overlay.dtbo mpu6050-rpi3-overlay.dts

# 3) Install overlay
sudo cp mpu6050-rpi3-overlay.dtbo /boot/overlays/

# 4) Tell firmware to load it
echo 'dtoverlay=mpu6050-rpi3-overlay' | sudo tee -a /boot/config.txt

# 5) Reboot
sudo reboot

Post-reboot sanity checks
# Detect the IMU address (should show 68 or 69)
sudo apt-get update && sudo apt-get install -y i2c-tools
sudo i2cdetect -y 1

# Load your IIO driver module if it isn’t built-in
sudo insmod ./mpu6050_iio_driver.ko || sudo modprobe mpu6050-iio

# Check that the IIO device exists and is named mpu6050
iio_info | sed -n '/Device.*mpu6050/,$p'
ls -l /sys/bus/iio/devices/

3) Why this overlay is needed for your test apps

Both test_mpu6050_iio.c and test_mpu6050_iio_buffered.c look for iio:deviceN/name == "mpu6050" and read attributes/character device from there. The overlay:

Instantiates the MPU-6050 on I2C1 at 0x68 (or 0x69 if AD0=1).

Plumbs the interrupt to GPIO17, enabling low-jitter buffered capture for the buffered test app.

Optionally sets defaults (FSR and ODR) that your driver reads as DT properties.

Without this DTS/overlay, Linux won’t create the IIO device node, and the apps won’t find iio:deviceN.

4) Running the test apps

Build the two apps you already have:

# Polled sysfs reader (100 ms)
gcc -O2 -Wall -Wextra -o test_mpu6050_iio test_mpu6050_iio.c
./test_mpu6050_iio

# Buffered IIO character-device reader
gcc -O2 -Wall -Wextra -o test_mpu6050_iio_buffered test_mpu6050_iio_buffered.c
sudo ./test_mpu6050_iio_buffered    # buffer/enable typically requires root


Tips:

If test_mpu6050_iio_buffered says “Missing scan enable” or “Permission denied,” run with sudo or adjust udev rules for /sys/bus/iio/devices/** and /dev/iio:device*.

If IRQ doesn’t fire: recheck INT wiring to GPIO17; verify with

grep -i mpu6050 /proc/interrupts
dmesg | tail -n 100

5) Common gotchas

Wrong address: If i2cdetect shows 0x69, set AD0 → 3V3 and change reg = <0x69> in the overlay.

No I²C devices shown: Make sure dtparam=i2c_arm=on and you are probing -y 1 (I2C1).

Noise on INT: Keep INT wire short, use rising edge (0x8). If spurious interrupts occur, try level-high (0x4) and/or add simple deglitching in the driver ISR path.

Power: Some cheap breakout boards have onboard regulators/level shifters. If so, make sure they’re truly 3.3 V-safe on SDA/SCL and INT.

1) Physical/Electrical connections (MPU-6050 ↔ RPi 3B)

Voltage: 3.3 V only. Do not use 5 V.

MPU-6050 pin |	Raspberry Pi 3B pin	               |Signal	        
VCC	         |  Pin 1 (3V3) or Pin 17 (3V3)	       |3.3 V power	
GND	         |  Any GND (e.g., Pin 6)	           |Ground	
SDA	         |  Pin 3 (GPIO2 / SDA1)	           |I2C data	
SCL	         |  Pin 5 (GPIO3 / SCL1)	           |I2C clock	
INT	         |  Pin 11 (GPIO17)	                   |Data-ready interrupt	
AD0	         |  GND (default 0x68) or 3V3 (0x69)   |I2C address select	

If your board labels the INT pin as “INT/INTA/DRDY,” that’s the one. MPU-6050 INT is push-pull, active-high by default—connect directly to GPIO17 (no level shifter).

Optional: if you need higher throughput, set the I²C baud rate to 400 kHz.