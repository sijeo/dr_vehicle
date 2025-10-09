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
