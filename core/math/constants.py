"""
Mathematical and physical constants for dead reckoning.
"""

import math

# Mathematical constants
PI = math.pi
TWO_PI = 2 * math.pi
HALF_PI = math.pi / 2

# Earth parameters
EARTH_RADIUS_M = 6371000.0  # Earth radius in meters
GRAVITY_MS2 = 9.80665       # Standard gravity in m/s²

# Conversion factors
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

# GPS/NMEA constants
GPS_UPDATE_RATE_HZ = 1.0    # Ublox NEO6M update rate
GPS_UPDATE_PERIOD_S = 1.0   # GPS update period in seconds

# IMU constants (MPU6050)
MPU6050_ACCEL_SCALE_2G = 16384.0   # LSB/g for ±2g range
MPU6050_GYRO_SCALE_250DPS = 131.0  # LSB/°/s for ±250°/s range

# Default noise parameters (will be tuned experimentally)
# Process noise
Q_POSITION = 0.1      # Position process noise
Q_VELOCITY = 0.1      # Velocity process noise  
Q_HEADING = 0.01      # Heading process noise
Q_HEADING_RATE = 0.01 # Heading rate process noise

# Measurement noise
R_GPS_POSITION = 5.0  # GPS position measurement noise (meters)
R_IMU_ACCEL = 0.1     # IMU acceleration measurement noise
R_IMU_GYRO = 0.01     # IMU gyroscope measurement noise

# Kalman filter parameters
INITIAL_POSITION_UNCERTAINTY = 10.0    # Initial position uncertainty (meters)
INITIAL_VELOCITY_UNCERTAINTY = 1.0     # Initial velocity uncertainty (m/s)
INITIAL_HEADING_UNCERTAINTY = 0.1      # Initial heading uncertainty (radians)
INITIAL_HEADING_RATE_UNCERTAINTY = 0.1 # Initial heading rate uncertainty (rad/s)