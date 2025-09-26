# OpenCPU LTE Module Implementation

This directory contains the dead reckoning implementation for OpenCPU LTE modules with cellular connectivity.

## Hardware Requirements

- **OpenCPU LTE Module** (e.g., Quectel EC25, BC95, or similar)
- **MPU6050** IMU sensor (I2C interface)
- **Built-in GNSS** (most LTE modules have integrated GPS/GNSS)
- **SIM Card** for cellular connectivity
- **External antenna** for LTE and GNSS

## Supported Modules

| Module | CPU | RAM | Flash | GNSS | Notes |
|--------|-----|-----|-------|------|-------|
| EC25-E | ARM Cortex-A7 | 256MB | 256MB | Yes | Cat 4 LTE |
| BC95-G | ARM Cortex-M3 | 128KB | 256KB | Yes | NB-IoT |
| BG96 | ARM Cortex-A7 | 256MB | 256MB | Yes | Cat M1/NB1 |

## Development Environment

- **SDK**: Quectel OpenCPU SDK or vendor-specific SDK
- **IDE**: Eclipse or vendor IDE
- **Compiler**: ARM GCC
- **Tools**: Module flashing utilities

## Architecture

```
opencpu_lte/
├── src/                    # Source code
│   ├── main.c             # Main application
│   ├── dr_system.c        # Dead reckoning system
│   ├── cellular.c         # Cellular communication
│   ├── sensors/           # Sensor drivers
│   └── at_commands/       # AT command interface
├── inc/                   # Header files
├── config/                # Module configurations
├── tools/                 # Development tools
└── examples/              # Example applications
```

## Key Features

- **Cellular Connectivity**: Real-time data upload via LTE/NB-IoT
- **Cloud Integration**: MQTT/CoAP/HTTP protocols
- **Remote Configuration**: Over-the-air parameter updates
- **Integrated GNSS**: High-accuracy positioning
- **Low Power**: Deep sleep modes for battery operation
- **Asset Tracking**: Complete vehicle tracking solution

## Communication Protocols

### MQTT Integration
```c
// Example MQTT message format
{
  "device_id": "DR001",
  "timestamp": 1640995200,
  "position": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 15.2,
    "accuracy": 3.5
  },
  "velocity": {
    "speed": 12.5,
    "heading": 45.2
  },
  "sensors": {
    "gps_satellites": 8,
    "gps_fix": 1,
    "imu_temperature": 23.4
  },
  "battery": 85
}
```

### AT Commands for Configuration
```
AT+DRCONFIG="update_rate",10        // Set update rate to 10 seconds
AT+DRCONFIG="accuracy_mode",1       // Enable high accuracy mode
AT+DRSTATUS?                        // Query system status
AT+DRPOSITION?                      // Get current position
AT+DRCALIBRATE                      // Start IMU calibration
```

## Power Management

| Mode | Current | Description | Wake Sources |
|------|---------|-------------|--------------|
| Active | 100-200mA | Full operation | N/A |
| Idle | 10-20mA | GPS tracking only | Motion, Timer |
| Sleep | 1-5mA | Periodic wake | Timer, SMS |
| Deep Sleep | <1mA | Emergency only | External interrupt |

## Cloud Platform Integration

### Supported Platforms
- **AWS IoT Core**: MQTT over TLS
- **Azure IoT Hub**: AMQP/MQTT
- **Google Cloud IoT**: MQTT
- **ThingSpeak**: HTTP REST API
- **Custom servers**: HTTP/MQTT/CoAP

### Data Flow
```
Vehicle → IMU/GPS → EKF → OpenCPU → Cellular → Cloud → Dashboard
```

## Configuration

Module configuration in `config/module_config.h`:

```c
// Cellular configuration
#define APN_NAME                "internet"
#define APN_USER                ""
#define APN_PASS                ""

// MQTT configuration
#define MQTT_BROKER_HOST       "mqtt.example.com"
#define MQTT_BROKER_PORT       1883
#define MQTT_CLIENT_ID         "dr_vehicle_001"
#define MQTT_TOPIC_TELEMETRY   "vehicles/dr001/telemetry"
#define MQTT_TOPIC_CONFIG      "vehicles/dr001/config"

// Update rates
#define POSITION_UPDATE_RATE_S  10
#define SENSOR_UPDATE_RATE_S    1
#define CLOUD_UPLOAD_RATE_S     30

// Power management
#define MOTION_THRESHOLD        0.5    // m/s² for motion detection
#define SLEEP_TIMEOUT_S         300    // Sleep after 5 min stationary
#define WAKEUP_INTERVAL_S       3600   // Wake every hour in sleep mode
```

## Application Examples

### 1. Vehicle Fleet Tracking
- Real-time position reporting
- Geofence monitoring
- Driver behavior analysis
- Maintenance scheduling

### 2. Asset Monitoring
- Container tracking
- Equipment location
- Theft prevention
- Usage monitoring

### 3. Emergency Response
- Automatic crash detection
- Emergency position reporting
- SOS button integration
- Two-way communication

## Build Instructions

```bash
# Set up OpenCPU SDK
export OPENCPU_SDK_PATH=/path/to/opencpu/sdk

# Configure for target module
make config MODULE=EC25

# Build firmware
make all

# Flash to module
make flash
```

## AT Command Interface

The system provides an AT command interface for configuration and monitoring:

```
AT+DRSTART                    // Start dead reckoning system
AT+DRSTOP                     // Stop system
AT+DRPOSITION?                // Get current position
AT+DRVELOCITY?                // Get current velocity
AT+DRSTATUS?                  // Get system status
AT+DRCONFIG=<param>,<value>   // Set configuration parameter
AT+DRCALIBRATE                // Calibrate IMU
AT+DRRESET                    // Reset system
```

## Status

🚧 **Under Development**

- [ ] Port core algorithms to OpenCPU environment
- [ ] Implement cellular connectivity layer
- [ ] Add MQTT/HTTP client libraries
- [ ] Implement power management
- [ ] Add cloud platform integrations
- [ ] Create AT command interface
- [ ] Add OTA update capability
- [ ] Field testing and validation

## Performance Targets

- **Position Update Rate**: 1-60 seconds (configurable)
- **Position Accuracy**: <10m RMS with GNSS, <100m after 5min outage
- **Battery Life**: >30 days (with 1 hour update rate)
- **Cloud Latency**: <5 seconds for position updates
- **Coverage**: Global (LTE/NB-IoT networks)

## Getting Started

1. Obtain OpenCPU development kit
2. Install SDK and development tools
3. Configure cellular network settings
4. Build and flash firmware
5. Set up cloud platform
6. Test with AT commands

For detailed implementation, see the Python reference in `../raspberry_pi/`.