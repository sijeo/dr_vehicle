"""
Core dead reckoning algorithms and utilities.

This module provides platform-independent implementations of:
- Extended Kalman Filter for sensor fusion
- Sensor data processing
- Mathematical utilities
"""

__version__ = "1.0.0"
__author__ = "DR Vehicle Team"

from .ekf import ExtendedKalmanFilter
from .sensors import IMUProcessor, GPSProcessor
from .math import rotation_matrix, normalize_angle

__all__ = [
    "ExtendedKalmanFilter",
    "IMUProcessor", 
    "GPSProcessor",
    "rotation_matrix",
    "normalize_angle"
]