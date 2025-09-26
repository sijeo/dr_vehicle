"""
IMU sensor data processing for MPU6050.
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple
from ..math.constants import *
from ..math.utils import normalize_angle

@dataclass
class IMUData:
    """Raw IMU sensor data."""
    
    # Accelerometer data (m/s²)
    accel_x: float
    accel_y: float  
    accel_z: float
    
    # Gyroscope data (rad/s)
    gyro_x: float
    gyro_y: float
    gyro_z: float
    
    # Temperature (°C)
    temperature: Optional[float] = None
    
    # Timestamp
    timestamp: Optional[float] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    @property
    def acceleration(self) -> np.ndarray:
        """Get acceleration as numpy array."""
        return np.array([self.accel_x, self.accel_y, self.accel_z])
    
    @property
    def angular_velocity(self) -> np.ndarray:
        """Get angular velocity as numpy array."""
        return np.array([self.gyro_x, self.gyro_y, self.gyro_z])

class IMUProcessor:
    """
    Processes IMU data from MPU6050 for dead reckoning.
    """
    
    def __init__(self, 
                 accel_scale: float = MPU6050_ACCEL_SCALE_2G,
                 gyro_scale: float = MPU6050_GYRO_SCALE_250DPS,
                 calibrate_on_init: bool = True):
        """
        Initialize IMU processor.
        
        Args:
            accel_scale: Accelerometer scale factor (LSB/g)
            gyro_scale: Gyroscope scale factor (LSB/°/s)
            calibrate_on_init: Whether to calibrate on initialization
        """
        self.accel_scale = accel_scale
        self.gyro_scale = gyro_scale
        
        # Calibration offsets
        self.accel_offset = np.zeros(3)
        self.gyro_offset = np.zeros(3)
        
        # Filtered values
        self.filtered_accel = np.zeros(3)
        self.filtered_gyro = np.zeros(3)
        
        # Low-pass filter coefficient (alpha = dt / (dt + tau))
        self.alpha_accel = 0.1  # More filtering for accelerometer
        self.alpha_gyro = 0.5   # Less filtering for gyroscope
        
        # Calibration data collection
        self.calibration_samples = []
        self.is_calibrated = False
        
        # Statistics
        self.sample_count = 0
        self.last_update_time = time.time()
    
    def convert_raw_data(self, raw_accel: np.ndarray, raw_gyro: np.ndarray) -> IMUData:
        """
        Convert raw sensor readings to physical units.
        
        Args:
            raw_accel: Raw accelerometer data [x, y, z] (LSB)
            raw_gyro: Raw gyroscope data [x, y, z] (LSB)
            
        Returns:
            IMUData with converted values
        """
        # Convert accelerometer (LSB to m/s²)
        accel_g = raw_accel / self.accel_scale  # Convert to g
        accel_ms2 = accel_g * GRAVITY_MS2       # Convert to m/s²
        
        # Convert gyroscope (LSB to rad/s)
        gyro_dps = raw_gyro / self.gyro_scale   # Convert to degrees/s
        gyro_rps = gyro_dps * DEG_TO_RAD        # Convert to rad/s
        
        return IMUData(
            accel_x=accel_ms2[0],
            accel_y=accel_ms2[1], 
            accel_z=accel_ms2[2],
            gyro_x=gyro_rps[0],
            gyro_y=gyro_rps[1],
            gyro_z=gyro_rps[2],
            timestamp=time.time()
        )
    
    def calibrate(self, calibration_data: List[IMUData], 
                  static_threshold: float = 0.5) -> bool:
        """
        Calibrate IMU using static data samples.
        
        Args:
            calibration_data: List of IMU data samples when stationary
            static_threshold: Threshold for detecting static condition (m/s²)
            
        Returns:
            True if calibration successful
        """
        if len(calibration_data) < 50:
            print("Warning: Need at least 50 samples for calibration")
            return False
        
        # Convert to numpy arrays
        accels = np.array([[d.accel_x, d.accel_y, d.accel_z] for d in calibration_data])
        gyros = np.array([[d.gyro_x, d.gyro_y, d.gyro_z] for d in calibration_data])
        
        # Check if data is from static condition
        accel_std = np.std(accels, axis=0)
        if np.max(accel_std) > static_threshold:
            print("Warning: Calibration data appears to be from moving condition")
            return False
        
        # Calculate offsets
        self.gyro_offset = np.mean(gyros, axis=0)
        
        # For accelerometer, assume Z-axis should read +1g when upright
        accel_mean = np.mean(accels, axis=0)
        self.accel_offset = accel_mean.copy()
        self.accel_offset[2] -= GRAVITY_MS2  # Remove gravity from Z-axis
        
        self.is_calibrated = True
        
        print(f"IMU Calibration complete:")
        print(f"  Accel offset: [{self.accel_offset[0]:.3f}, {self.accel_offset[1]:.3f}, {self.accel_offset[2]:.3f}] m/s²")
        print(f"  Gyro offset:  [{self.gyro_offset[0]:.3f}, {self.gyro_offset[1]:.3f}, {self.gyro_offset[2]:.3f}] rad/s")
        
        return True
    
    def apply_calibration(self, imu_data: IMUData) -> IMUData:
        """Apply calibration offsets to IMU data."""
        if not self.is_calibrated:
            return imu_data
        
        calibrated_data = IMUData(
            accel_x=imu_data.accel_x - self.accel_offset[0],
            accel_y=imu_data.accel_y - self.accel_offset[1],
            accel_z=imu_data.accel_z - self.accel_offset[2],
            gyro_x=imu_data.gyro_x - self.gyro_offset[0],
            gyro_y=imu_data.gyro_y - self.gyro_offset[1],
            gyro_z=imu_data.gyro_z - self.gyro_offset[2],
            temperature=imu_data.temperature,
            timestamp=imu_data.timestamp
        )
        
        return calibrated_data
    
    def apply_low_pass_filter(self, imu_data: IMUData) -> IMUData:
        """Apply low-pass filter to reduce noise."""
        current_accel = imu_data.acceleration
        current_gyro = imu_data.angular_velocity
        
        # Apply exponential moving average filter
        self.filtered_accel = (1 - self.alpha_accel) * self.filtered_accel + self.alpha_accel * current_accel
        self.filtered_gyro = (1 - self.alpha_gyro) * self.filtered_gyro + self.alpha_gyro * current_gyro
        
        return IMUData(
            accel_x=self.filtered_accel[0],
            accel_y=self.filtered_accel[1],
            accel_z=self.filtered_accel[2],
            gyro_x=self.filtered_gyro[0],
            gyro_y=self.filtered_gyro[1],
            gyro_z=self.filtered_gyro[2],
            temperature=imu_data.temperature,
            timestamp=imu_data.timestamp
        )
    
    def process_data(self, imu_data: IMUData, apply_filtering: bool = True) -> IMUData:
        """
        Process IMU data with calibration and filtering.
        
        Args:
            imu_data: Raw IMU data
            apply_filtering: Whether to apply low-pass filtering
            
        Returns:
            Processed IMU data
        """
        # Apply calibration
        processed_data = self.apply_calibration(imu_data)
        
        # Apply filtering if requested
        if apply_filtering:
            processed_data = self.apply_low_pass_filter(processed_data)
        
        # Update statistics
        self.sample_count += 1
        self.last_update_time = processed_data.timestamp
        
        return processed_data
    
    def get_measurement_for_ekf(self, imu_data: IMUData) -> np.ndarray:
        """
        Convert IMU data to measurement vector for EKF.
        
        For ground vehicles, we primarily use:
        - Lateral acceleration (after removing gravity)
        - Longitudinal acceleration (after removing gravity)  
        - Yaw rate (gyro_z)
        
        Args:
            imu_data: Processed IMU data
            
        Returns:
            Measurement vector [ax, ay, omega_z]
        """
        # For ground vehicle, assume Z-axis is vertical
        # Remove gravity from accelerometer readings
        ax = imu_data.accel_x
        ay = imu_data.accel_y
        
        # Yaw rate is directly gyro_z
        omega_z = imu_data.gyro_z
        
        return np.array([ax, ay, omega_z])
    
    def estimate_heading_change(self, imu_data: IMUData, dt: float) -> float:
        """
        Estimate heading change from gyroscope data.
        
        Args:
            imu_data: IMU data
            dt: Time step
            
        Returns:
            Heading change in radians
        """
        return imu_data.gyro_z * dt
    
    def get_statistics(self) -> dict:
        """Get processor statistics."""
        return {
            'sample_count': self.sample_count,
            'is_calibrated': self.is_calibrated,
            'last_update_time': self.last_update_time,
            'accel_offset': self.accel_offset.tolist(),
            'gyro_offset': self.gyro_offset.tolist()
        }