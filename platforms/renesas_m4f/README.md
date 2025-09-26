# Cortex-M4F Renesas Implementation

This directory contains the dead reckoning implementation for Cortex-M4F based Renesas controllers.

## Hardware Requirements

- **Renesas Cortex-M4F Controller** (e.g., RX65N, RX72N, or similar)
- **MPU6050** IMU sensor (I2C interface)
- **Ublox NEO6M** GPS module (UART interface)
- **Flash Memory**: Minimum 512KB for code and data
- **RAM**: Minimum 64KB for EKF calculations and buffers

## Development Environment

- **IDE**: Renesas e² studio or IAR Embedded Workbench
- **Compiler**: GCC for Renesas or IAR C/C++ Compiler
- **SDK**: Renesas Flexible Software Package (FSP)

## Architecture

```
renesas_m4f/
├── src/                    # Source code
│   ├── main.c             # Main application
│   ├── ekf_port.c         # EKF ported to fixed-point
│   ├── sensors/           # Sensor drivers
│   └── hal/               # Hardware abstraction layer
├── inc/                   # Header files
├── config/                # Configuration files
├── build/                 # Build outputs
└── scripts/               # Build and flash scripts
```

## Key Features

- **Fixed-Point Math**: EKF calculations optimized for Cortex-M4F DSP instructions
- **RTOS Integration**: FreeRTOS or Renesas kernel support
- **Low Power**: Sleep modes for battery operation
- **Flash Logging**: Data logging to external flash memory
- **CAN Interface**: Vehicle bus integration (optional)

## Porting Notes

The core algorithms from the `core/` directory need to be ported from Python/NumPy to C with the following considerations:

1. **Matrix Operations**: Use ARM CMSIS-DSP library for optimized matrix math
2. **Memory Management**: Static allocation for all data structures
3. **Timing**: Use hardware timers for precise timing
4. **Interrupt Handling**: IMU and GPS data collection in ISRs
5. **Calibration**: EEPROM storage for calibration parameters

## Build Instructions

```bash
# Set up Renesas toolchain
export RENESAS_TOOLCHAIN_PATH=/path/to/renesas/toolchain

# Configure for target device
make config DEVICE=RX65N

# Build firmware
make all

# Flash to device
make flash
```

## Configuration

Key configuration parameters in `config/dr_config.h`:

```c
// EKF Configuration
#define EKF_STATE_SIZE          6
#define EKF_GPS_MEAS_SIZE       2
#define EKF_IMU_MEAS_SIZE       3

// Timing Configuration
#define IMU_SAMPLE_RATE_HZ      100
#define GPS_UPDATE_RATE_HZ      1
#define EKF_UPDATE_RATE_HZ      100

// Hardware Configuration
#define MPU6050_I2C_CHANNEL    0
#define GPS_UART_CHANNEL       1
#define GPS_BAUD_RATE          9600

// Memory Configuration
#define DATA_LOG_BUFFER_SIZE   4096
#define SENSOR_BUFFER_SIZE     256
```

## Performance Targets

- **EKF Update Rate**: 100 Hz
- **GPS Integration**: 1 Hz
- **Power Consumption**: <50mA @ 3.3V (active), <1mA (sleep)
- **Position Accuracy**: <5m RMS with GPS, <50m after 60s GPS outage
- **Memory Usage**: <200KB Flash, <32KB RAM

## Status

🚧 **Under Development**

- [ ] Port core EKF algorithm to fixed-point C
- [ ] Implement hardware drivers for Renesas peripherals
- [ ] Add RTOS integration
- [ ] Optimize for power consumption
- [ ] Add CAN bus interface
- [ ] Implement data logging
- [ ] Add calibration routines
- [ ] Performance testing and validation

## Getting Started

1. Install Renesas development tools
2. Clone this repository
3. Set up hardware connections
4. Build and flash firmware
5. Use serial interface for monitoring

For detailed implementation, see the Python reference in `../raspberry_pi/`.