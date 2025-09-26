"""
MPU6050 IMU driver for Raspberry Pi using I2C.
"""

import time
import numpy as np
from typing import Optional, Tuple

try:
    import smbus2
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    print("Warning: smbus2 not available, MPU6050 driver will use simulation mode")

# MPU6050 Register Map
MPU6050_REG_PWR_MGMT_1 = 0x6B
MPU6050_REG_PWR_MGMT_2 = 0x6C
MPU6050_REG_CONFIG = 0x1A
MPU6050_REG_GYRO_CONFIG = 0x1B
MPU6050_REG_ACCEL_CONFIG = 0x1C
MPU6050_REG_ACCEL_XOUT_H = 0x3B
MPU6050_REG_TEMP_OUT_H = 0x41
MPU6050_REG_GYRO_XOUT_H = 0x43
MPU6050_REG_WHO_AM_I = 0x75

# Configuration values
MPU6050_CLOCK_PLL_XGYRO = 0x01
MPU6050_GYRO_FS_250 = 0x00
MPU6050_ACCEL_FS_2G = 0x00
MPU6050_DLPF_BW_42 = 0x03

class MPU6050Driver:
    """
    Driver for MPU6050 6-axis IMU sensor on Raspberry Pi.
    """
    
    def __init__(self, i2c_address: int = 0x68, i2c_bus: int = 1):
        """
        Initialize MPU6050 driver.
        
        Args:
            i2c_address: I2C address of MPU6050 (default 0x68)
            i2c_bus: I2C bus number (default 1 for Raspberry Pi)
        """
        self.i2c_address = i2c_address
        self.i2c_bus = i2c_bus
        self.bus = None
        self.initialized = False
        
        # Scale factors (will be set during initialization)
        self.accel_scale = 16384.0  # LSB/g for ±2g range
        self.gyro_scale = 131.0     # LSB/°/s for ±250°/s range
        
        # Simulation mode for testing without hardware
        self.simulation_mode = not I2C_AVAILABLE
        self.sim_time_start = time.time()
        
    def initialize(self) -> bool:
        """
        Initialize the MPU6050 sensor.
        
        Returns:
            True if initialization successful
        """
        if self.simulation_mode:
            print("MPU6050: Running in simulation mode")
            self.initialized = True
            return True
        
        try:
            # Initialize I2C bus
            self.bus = smbus2.SMBus(self.i2c_bus)
            
            # Wake up the MPU6050 (it starts in sleep mode)
            self.bus.write_byte_data(self.i2c_address, MPU6050_REG_PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            # Verify device is responding
            who_am_i = self.bus.read_byte_data(self.i2c_address, MPU6050_REG_WHO_AM_I)
            if who_am_i != 0x68:
                print(f"MPU6050: Wrong device ID {who_am_i:02X}, expected 0x68")
                return False
            
            # Configure the accelerometer (±2g)
            self.bus.write_byte_data(self.i2c_address, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G)
            
            # Configure the gyroscope (±250°/s)
            self.bus.write_byte_data(self.i2c_address, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250)
            
            # Configure DLPF (Digital Low Pass Filter)
            self.bus.write_byte_data(self.i2c_address, MPU6050_REG_CONFIG, MPU6050_DLPF_BW_42)
            
            # Set clock source to PLL with X-axis gyroscope reference
            self.bus.write_byte_data(self.i2c_address, MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO)
            
            self.initialized = True
            print("MPU6050: Initialized successfully")
            return True
            
        except Exception as e:
            print(f"MPU6050: Initialization failed: {e}")
            return False
    
    def read_raw_accel(self) -> Optional[np.ndarray]:
        """
        Read raw accelerometer data.
        
        Returns:
            [x, y, z] acceleration in LSB units, or None if error
        """
        if self.simulation_mode:
            # Simulate accelerometer data (mostly gravity on Z-axis with some noise)
            noise = np.random.normal(0, 100, 3)  # Small noise
            return np.array([noise[0], noise[1], 16384 + noise[2]])  # ~1g on Z-axis
        
        if not self.initialized:
            return None
        
        try:
            # Read 6 bytes starting from ACCEL_XOUT_H
            data = self.bus.read_i2c_block_data(self.i2c_address, MPU6050_REG_ACCEL_XOUT_H, 6)
            
            # Convert to signed 16-bit values
            accel_x = self._bytes_to_int16(data[0], data[1])
            accel_y = self._bytes_to_int16(data[2], data[3])
            accel_z = self._bytes_to_int16(data[4], data[5])
            
            return np.array([accel_x, accel_y, accel_z], dtype=np.float64)
            
        except Exception as e:
            print(f"MPU6050: Failed to read accelerometer: {e}")
            return None
    
    def read_raw_gyro(self) -> Optional[np.ndarray]:
        """
        Read raw gyroscope data.
        
        Returns:
            [x, y, z] angular velocity in LSB units, or None if error
        """
        if self.simulation_mode:
            # Simulate gyroscope data (small random drift)
            current_time = time.time() - self.sim_time_start
            drift = np.sin(current_time * 0.1) * 50  # Slow sinusoidal drift
            noise = np.random.normal(0, 20, 3)
            return np.array([noise[0], noise[1], drift + noise[2]])
        
        if not self.initialized:
            return None
        
        try:
            # Read 6 bytes starting from GYRO_XOUT_H
            data = self.bus.read_i2c_block_data(self.i2c_address, MPU6050_REG_GYRO_XOUT_H, 6)
            
            # Convert to signed 16-bit values
            gyro_x = self._bytes_to_int16(data[0], data[1])
            gyro_y = self._bytes_to_int16(data[2], data[3])
            gyro_z = self._bytes_to_int16(data[4], data[5])
            
            return np.array([gyro_x, gyro_y, gyro_z], dtype=np.float64)
            
        except Exception as e:
            print(f"MPU6050: Failed to read gyroscope: {e}")
            return None
    
    def read_temperature(self) -> Optional[float]:
        """
        Read temperature sensor.
        
        Returns:
            Temperature in Celsius, or None if error
        """
        if self.simulation_mode:
            # Simulate temperature (room temperature with slight variation)
            return 25.0 + np.random.normal(0, 0.5)
        
        if not self.initialized:
            return None
        
        try:
            # Read 2 bytes starting from TEMP_OUT_H
            data = self.bus.read_i2c_block_data(self.i2c_address, MPU6050_REG_TEMP_OUT_H, 2)
            
            # Convert to signed 16-bit value
            temp_raw = self._bytes_to_int16(data[0], data[1])
            
            # Convert to Celsius
            temperature = temp_raw / 340.0 + 36.53
            
            return temperature
            
        except Exception as e:
            print(f"MPU6050: Failed to read temperature: {e}")
            return None
    
    def read_sensors(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Read both accelerometer and gyroscope data.
        
        Returns:
            (accel, gyro) tuple with [x, y, z] arrays, or (None, None) if error
        """
        if self.simulation_mode:
            return self.read_raw_accel(), self.read_raw_gyro()
        
        if not self.initialized:
            return None, None
        
        try:
            # Read all sensor data in one transaction for better timing
            # ACCEL_XOUT_H to GYRO_ZOUT_L (14 bytes total)
            data = self.bus.read_i2c_block_data(self.i2c_address, MPU6050_REG_ACCEL_XOUT_H, 14)
            
            # Parse accelerometer data
            accel_x = self._bytes_to_int16(data[0], data[1])
            accel_y = self._bytes_to_int16(data[2], data[3])
            accel_z = self._bytes_to_int16(data[4], data[5])
            accel = np.array([accel_x, accel_y, accel_z], dtype=np.float64)
            
            # Skip temperature (bytes 6-7)
            
            # Parse gyroscope data
            gyro_x = self._bytes_to_int16(data[8], data[9])
            gyro_y = self._bytes_to_int16(data[10], data[11])
            gyro_z = self._bytes_to_int16(data[12], data[13])
            gyro = np.array([gyro_x, gyro_y, gyro_z], dtype=np.float64)
            
            return accel, gyro
            
        except Exception as e:
            print(f"MPU6050: Failed to read sensors: {e}")
            return None, None
    
    def _bytes_to_int16(self, high_byte: int, low_byte: int) -> int:
        """Convert two bytes to signed 16-bit integer."""
        value = (high_byte << 8) | low_byte
        # Convert to signed
        if value >= 32768:
            value -= 65536
        return value
    
    def self_test(self) -> bool:
        """
        Perform basic self-test.
        
        Returns:
            True if self-test passes
        """
        if not self.initialized:
            print("MPU6050: Not initialized")
            return False
        
        # Read several samples and check for reasonable values
        samples = []
        for _ in range(10):
            accel, gyro = self.read_sensors()
            if accel is not None and gyro is not None:
                samples.append((accel, gyro))
            time.sleep(0.01)
        
        if len(samples) < 5:
            print("MPU6050: Self-test failed - insufficient samples")
            return False
        
        # Check accelerometer (should see ~1g total magnitude)
        accel_magnitudes = [np.linalg.norm(sample[0]) / self.accel_scale for sample, _ in samples]
        avg_magnitude = np.mean(accel_magnitudes)
        
        if not (0.5 < avg_magnitude < 2.0):  # Should be around 1g
            print(f"MPU6050: Self-test failed - unexpected accelerometer magnitude: {avg_magnitude:.2f}g")
            return False
        
        # Check gyroscope (should be relatively stable)
        gyro_values = np.array([sample[1] for _, sample in samples])
        gyro_std = np.std(gyro_values, axis=0)
        max_std = np.max(gyro_std)
        
        if max_std > 1000:  # Arbitrary threshold for stability
            print(f"MPU6050: Self-test failed - gyroscope too noisy: {max_std:.1f} LSB std")
            return False
        
        print("MPU6050: Self-test passed")
        return True
    
    def cleanup(self):
        """Cleanup resources."""
        if self.bus:
            try:
                self.bus.close()
            except:
                pass
        self.initialized = False
        print("MPU6050: Cleanup completed")