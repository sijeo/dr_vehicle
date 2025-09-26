# Raspberry Pi 3B Implementation

This directory contains the complete dead reckoning implementation for Raspberry Pi 3B with MPU6050 and Ublox NEO6M.

## Hardware Setup

### Required Components
- **Raspberry Pi 3B** (or newer)
- **MPU6050** 6-axis IMU sensor
- **Ublox NEO6M** GPS module
- **MicroSD card** (16GB+)
- **Power supply** (5V/2.5A)

### Wiring Connections

#### MPU6050 (I2C)
| MPU6050 Pin | Raspberry Pi Pin | Function |
|-------------|------------------|----------|
| VCC         | 3.3V (Pin 1)     | Power    |
| GND         | GND (Pin 6)      | Ground   |
| SCL         | GPIO 3 (Pin 5)   | I2C Clock |
| SDA         | GPIO 2 (Pin 3)   | I2C Data  |

#### Ublox NEO6M (UART)
| NEO6M Pin   | Raspberry Pi Pin | Function |
|-------------|------------------|----------|
| VCC         | 5V (Pin 2)       | Power    |
| GND         | GND (Pin 6)      | Ground   |
| TX          | GPIO 15 (Pin 10) | UART RX  |
| RX          | GPIO 14 (Pin 8)  | UART TX  |

## Software Installation

### 1. Raspberry Pi OS Setup
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Enable I2C and Serial
sudo raspi-config
# Navigate to Interface Options -> I2C -> Enable
# Navigate to Interface Options -> Serial -> Enable
# Reboot when prompted

# Install Python dependencies
sudo apt install python3-pip python3-dev git -y
```

### 2. Install Project Dependencies
```bash
# Clone repository
git clone https://github.com/sijeo/dr_vehicle.git
cd dr_vehicle/platforms/raspberry_pi

# Install Python packages
pip3 install -r requirements.txt
```

### 3. Hardware Verification
```bash
# Check I2C devices (should show 0x68 for MPU6050)
sudo i2cdetect -y 1

# Check serial port
ls -la /dev/ttyAMA0
```

## Configuration

Edit `config.json` to customize system parameters:

```json
{
  "i2c_bus": 1,
  "mpu6050_address": 104,
  "gps_serial_port": "/dev/ttyAMA0",
  "gps_baud_rate": 9600,
  "process_noise": {
    "position": 0.1,
    "velocity": 0.1,
    "heading": 0.01,
    "heading_rate": 0.01
  },
  "measurement_noise": {
    "gps_position": 5.0,
    "imu_accel": 0.1,
    "imu_gyro": 0.01
  }
}
```

## Running the System

### Basic Usage
```bash
# Run the dead reckoning system
python3 main.py

# Run with debug output
python3 main.py --debug

# Run with custom config
python3 main.py --config custom_config.json
```

### Expected Output
```
Dead Reckoning System for Raspberry Pi 3B
Hardware: MPU6050 + Ublox NEO6M
==================================================
Dead Reckoning System initialized
IMU: MPU6050 on I2C bus 1, address 0x68
GPS: /dev/ttyAMA0 at 9600 baud
Starting dead reckoning system...
Calibrating IMU... (keep device stationary)
IMU calibration completed successfully
Dead reckoning system started successfully

=== Dead Reckoning Status (Uptime: 5.2s) ===
Position: [0.00, 0.00] m
Velocity: [0.00, 0.00] m/s (Speed: 0.00 m/s)
Heading:  0.000 rad (0.0°)
Uncertainty: 5.00 m
Data Age: GPS 1.2s, IMU 0.010s
```

## Testing Without Hardware

The system includes simulation mode for testing without actual hardware:

```bash
# The drivers automatically detect missing hardware and enter simulation mode
python3 main.py
# Output: "MPU6050: Running in simulation mode"
# Output: "GPS: Running in simulation mode"
```

## Troubleshooting

### I2C Issues
```bash
# Check if I2C is enabled
sudo raspi-config

# Install I2C tools
sudo apt install i2c-tools

# Scan for devices
sudo i2cdetect -y 1
```

### GPS Issues
```bash
# Check serial port permissions
sudo usermod -a -G dialout $USER
# Logout and login again

# Test GPS manually
sudo cat /dev/ttyAMA0
# Should see NMEA sentences like $GPGGA,$GPRMC, etc.
```

### Permission Issues
```bash
# Add user to required groups
sudo usermod -a -G gpio,i2c,spi,dialout $USER

# Logout and login again
```

## Performance Tuning

### IMU Sample Rate
- Default: 100 Hz
- Increase for better motion tracking
- Decrease to save CPU/power

### GPS Update Rate
- Default: 1 Hz (hardware limitation)
- Some modules support higher rates
- Configure in GPS module directly

### EKF Parameters
- Tune process/measurement noise in `config.json`
- Lower values = trust sensors more
- Higher values = trust model more

## API Interface

The system provides programmatic access:

```python
from main import DeadReckoningSystem

# Create and start system
dr_system = DeadReckoningSystem("config.json")
dr_system.start()

# Get current position
position = dr_system.get_current_position()
print(f"Current position: {position}")

# Stop system
dr_system.stop()
```

## Log Files

System logs are saved to `dr_vehicle.log`:
- Startup/shutdown events
- GPS fixes and quality
- IMU calibration results
- System errors and warnings

## Auto-Start at Boot

To run automatically on startup:

```bash
# Create systemd service
sudo nano /etc/systemd/system/dr-vehicle.service
```

```ini
[Unit]
Description=Dead Reckoning Vehicle System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/dr_vehicle/platforms/raspberry_pi
ExecStart=/usr/bin/python3 main.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# Enable and start service
sudo systemctl enable dr-vehicle.service
sudo systemctl start dr-vehicle.service

# Check status
sudo systemctl status dr-vehicle.service
```

## Performance Specifications

- **Update Rate**: 100 Hz (IMU), 1 Hz (GPS)
- **Position Accuracy**: <5m RMS with GPS, <50m after 60s GPS outage
- **Power Consumption**: ~500mA @ 5V (Raspberry Pi + sensors)
- **CPU Usage**: <10% on Raspberry Pi 3B
- **Memory Usage**: <50MB RAM

## Next Steps

1. Mount system in vehicle
2. Calibrate for specific installation orientation
3. Tune EKF parameters for vehicle dynamics
4. Add data logging and analysis
5. Integrate with vehicle systems (CAN bus, etc.)