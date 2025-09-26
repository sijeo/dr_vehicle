# Multi-Platform Dead Reckoning Solution

This repository implements a multi-platform dead reckoning algorithm using Extended Kalman Filter (EKF) for sensor fusion on ground vehicles.

## Overview

The system uses sensor fusion to combine data from:
- **IMU (MPU6050)**: Accelerometer + Gyroscope data
- **GPS (Ublox NEO6M)**: NMEA format position data (1Hz)

## Supported Platforms

1. **Raspberry Pi 3B** - Primary development platform
2. **Cortex-M4F (Renesas)** - Embedded controller implementation  
3. **OpenCPU LTE Module** - Cellular IoT implementation

## Architecture

```
dr_vehicle/
├── core/                    # Platform-independent algorithms
│   ├── ekf/                 # Extended Kalman Filter implementation
│   ├── sensors/             # Sensor data processing
│   └── math/                # Mathematical utilities
├── platforms/               # Platform-specific implementations
│   ├── raspberry_pi/        # Raspberry Pi 3B implementation
│   ├── renesas_m4f/         # Cortex-M4F Renesas implementation
│   └── opencpu_lte/         # OpenCPU LTE implementation
├── tests/                   # Unit tests
└── examples/                # Usage examples
```

## Features

- **Extended Kalman Filter**: For optimal sensor fusion
- **IMU Processing**: Real-time accelerometer and gyroscope data processing
- **GPS Integration**: NMEA sentence parsing and position estimation
- **Dead Reckoning**: Position estimation during GPS outages
- **Multi-Platform**: Portable codebase for different hardware platforms

## Getting Started

### Raspberry Pi 3B Setup

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install python3-pip python3-dev
pip3 install -r platforms/raspberry_pi/requirements.txt

# Enable I2C for MPU6050
sudo raspi-config
# Navigate to Interface Options -> I2C -> Enable

# Run the application
cd platforms/raspberry_pi
python3 main.py
```

### Building for Other Platforms

- **Renesas M4F**: See `platforms/renesas_m4f/README.md`
- **OpenCPU LTE**: See `platforms/opencpu_lte/README.md`

## Algorithm Details

The dead reckoning system uses a 6-DOF Extended Kalman Filter with the following state vector:
- Position (x, y)
- Velocity (vx, vy)  
- Heading (θ)
- Heading rate (ω)

### Sensor Integration

- **IMU Data**: Used for motion prediction and heading estimation
- **GPS Data**: Used for position correction when available
- **Dead Reckoning**: Maintains position estimate during GPS outages

## License

MIT License - see LICENSE file for details.
