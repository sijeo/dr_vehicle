"""
Configuration manager for Raspberry Pi dead reckoning system.
"""

import json
import os
from typing import Dict, Any

class Config:
    """Configuration manager for the dead reckoning system."""
    
    DEFAULT_CONFIG = {
        # Hardware configuration
        "i2c_bus": 1,
        "mpu6050_address": 0x68,
        "gps_serial_port": "/dev/ttyAMA0",
        "gps_baud_rate": 9600,
        
        # EKF process noise parameters
        "process_noise": {
            "position": 0.1,
            "velocity": 0.1,
            "heading": 0.01,
            "heading_rate": 0.01
        },
        
        # EKF measurement noise parameters
        "measurement_noise": {
            "gps_position": 5.0,
            "imu_accel": 0.1,
            "imu_gyro": 0.01
        },
        
        # Data logging
        "enable_logging": True,
        "log_file": "dr_vehicle.log",
        "log_level": "INFO",
        
        # Output configuration
        "output_rate_hz": 1.0,
        "enable_web_interface": False,
        "web_port": 8080,
        
        # Calibration
        "auto_calibrate_imu": True,
        "calibration_duration_s": 5,
        "save_calibration": True,
        "calibration_file": "imu_calibration.json"
    }
    
    def __init__(self, config_file: str = "config.json"):
        """
        Initialize configuration.
        
        Args:
            config_file: Path to configuration file
        """
        self.config_file = config_file
        self.config = self.DEFAULT_CONFIG.copy()
        
        # Load configuration from file if it exists
        if os.path.exists(config_file):
            self.load_config()
        else:
            print(f"Config file {config_file} not found, using defaults")
            self.save_config()  # Create default config file
    
    def load_config(self) -> bool:
        """
        Load configuration from file.
        
        Returns:
            True if loaded successfully
        """
        try:
            with open(self.config_file, 'r') as f:
                file_config = json.load(f)
            
            # Merge with defaults (file config overrides defaults)
            self._merge_config(self.config, file_config)
            
            print(f"Configuration loaded from {self.config_file}")
            return True
            
        except Exception as e:
            print(f"Failed to load config: {e}")
            return False
    
    def save_config(self) -> bool:
        """
        Save current configuration to file.
        
        Returns:
            True if saved successfully
        """
        try:
            with open(self.config_file, 'w') as f:
                json.dump(self.config, f, indent=2)
            
            print(f"Configuration saved to {self.config_file}")
            return True
            
        except Exception as e:
            print(f"Failed to save config: {e}")
            return False
    
    def _merge_config(self, base: Dict[str, Any], override: Dict[str, Any]):
        """Recursively merge configuration dictionaries."""
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._merge_config(base[key], value)
            else:
                base[key] = value
    
    def get(self, key: str, default=None):
        """Get configuration value with optional default."""
        keys = key.split('.')
        value = self.config
        
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        
        return value
    
    def set(self, key: str, value: Any):
        """Set configuration value."""
        keys = key.split('.')
        config = self.config
        
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]
        
        config[keys[-1]] = value
    
    # Property accessors for common configuration values
    @property
    def i2c_bus(self) -> int:
        return self.config["i2c_bus"]
    
    @property
    def mpu6050_address(self) -> int:
        return self.config["mpu6050_address"]
    
    @property
    def gps_serial_port(self) -> str:
        return self.config["gps_serial_port"]
    
    @property
    def gps_baud_rate(self) -> int:
        return self.config["gps_baud_rate"]
    
    @property
    def process_noise(self) -> Dict[str, float]:
        return self.config["process_noise"]
    
    @property
    def measurement_noise(self) -> Dict[str, float]:
        return self.config["measurement_noise"]
    
    @property
    def enable_logging(self) -> bool:
        return self.config["enable_logging"]
    
    @property
    def log_file(self) -> str:
        return self.config["log_file"]
    
    @property
    def output_rate_hz(self) -> float:
        return self.config["output_rate_hz"]
    
    def print_config(self):
        """Print current configuration."""
        print("=== Dead Reckoning Configuration ===")
        print(json.dumps(self.config, indent=2))