#!/usr/bin/env python3
"""
Dead Reckoning Application for Raspberry Pi 3B
Hardware: MPU6050 (I2C) + Ublox NEO6M (UART)
"""

import sys
import os
import time
import threading
import signal
import json
from typing import Optional

# Add core modules to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from core.ekf import ExtendedKalmanFilter, VehicleState
from core.sensors import IMUProcessor, GPSProcessor, IMUData, GPSData
from hardware.mpu6050_driver import MPU6050Driver
from hardware.gps_driver import GPSDriver
from config import Config

class DeadReckoningSystem:
    """Main dead reckoning system for Raspberry Pi."""
    
    def __init__(self, config_file: str = "config.json"):
        """Initialize the dead reckoning system."""
        
        # Load configuration
        self.config = Config(config_file)
        
        # Initialize hardware drivers
        self.mpu6050 = MPU6050Driver(
            i2c_address=self.config.mpu6050_address,
            i2c_bus=self.config.i2c_bus
        )
        
        self.gps_driver = GPSDriver(
            serial_port=self.config.gps_serial_port,
            baud_rate=self.config.gps_baud_rate
        )
        
        # Initialize sensor processors
        self.imu_processor = IMUProcessor()
        self.gps_processor = GPSProcessor()
        
        # Initialize EKF
        initial_state = VehicleState()
        self.ekf = ExtendedKalmanFilter(
            initial_state=initial_state,
            process_noise=self.config.process_noise,
            measurement_noise=self.config.measurement_noise
        )
        
        # Threading control
        self.running = False
        self.imu_thread = None
        self.gps_thread = None
        self.output_thread = None
        
        # Data storage
        self.current_state = initial_state
        self.last_gps_time = 0
        self.last_imu_time = 0
        
        # Statistics
        self.start_time = time.time()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("Dead Reckoning System initialized")
        print(f"IMU: MPU6050 on I2C bus {self.config.i2c_bus}, address 0x{self.config.mpu6050_address:02X}")
        print(f"GPS: {self.config.gps_serial_port} at {self.config.gps_baud_rate} baud")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        print("\nShutdown signal received, stopping system...")
        self.stop()
        sys.exit(0)
    
    def start(self):
        """Start the dead reckoning system."""
        if self.running:
            print("System already running")
            return
        
        print("Starting dead reckoning system...")
        
        # Initialize hardware
        if not self.mpu6050.initialize():
            print("ERROR: Failed to initialize MPU6050")
            return False
        
        if not self.gps_driver.initialize():
            print("ERROR: Failed to initialize GPS")
            return False
        
        # Calibrate IMU
        print("Calibrating IMU... (keep device stationary)")
        if not self._calibrate_imu():
            print("WARNING: IMU calibration failed, continuing with default values")
        
        self.running = True
        
        # Start threads
        self.imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self.gps_thread = threading.Thread(target=self._gps_loop, daemon=True)
        self.output_thread = threading.Thread(target=self._output_loop, daemon=True)
        
        self.imu_thread.start()
        self.gps_thread.start()
        self.output_thread.start()
        
        print("Dead reckoning system started successfully")
        return True
    
    def stop(self):
        """Stop the dead reckoning system."""
        if not self.running:
            return
        
        print("Stopping dead reckoning system...")
        
        self.running = False
        
        # Wait for threads to finish
        if self.imu_thread and self.imu_thread.is_alive():
            self.imu_thread.join(timeout=2.0)
        
        if self.gps_thread and self.gps_thread.is_alive():
            self.gps_thread.join(timeout=2.0)
        
        if self.output_thread and self.output_thread.is_alive():
            self.output_thread.join(timeout=2.0)
        
        # Cleanup hardware
        self.mpu6050.cleanup()
        self.gps_driver.cleanup()
        
        print("Dead reckoning system stopped")
    
    def _calibrate_imu(self) -> bool:
        """Calibrate the IMU sensor."""
        calibration_samples = []
        calibration_time = 5  # seconds
        
        print(f"Collecting calibration data for {calibration_time} seconds...")
        
        start_time = time.time()
        while time.time() - start_time < calibration_time:
            raw_accel, raw_gyro = self.mpu6050.read_sensors()
            if raw_accel is not None and raw_gyro is not None:
                imu_data = self.imu_processor.convert_raw_data(raw_accel, raw_gyro)
                calibration_samples.append(imu_data)
            
            time.sleep(0.01)  # 100 Hz sampling
        
        if len(calibration_samples) < 100:
            print(f"WARNING: Only collected {len(calibration_samples)} samples")
            return False
        
        success = self.imu_processor.calibrate(calibration_samples)
        if success:
            print("IMU calibration completed successfully")
        
        return success
    
    def _imu_loop(self):
        """IMU data processing loop."""
        last_time = time.time()
        
        while self.running:
            try:
                # Read IMU data
                raw_accel, raw_gyro = self.mpu6050.read_sensors()
                
                if raw_accel is not None and raw_gyro is not None:
                    # Convert and process IMU data
                    imu_data = self.imu_processor.convert_raw_data(raw_accel, raw_gyro)
                    processed_imu = self.imu_processor.process_data(imu_data)
                    
                    # Update EKF with prediction step
                    current_time = time.time()
                    dt = current_time - last_time
                    
                    if dt > 0:
                        self.current_state = self.ekf.predict(dt)
                        
                        # Update with IMU measurement
                        imu_measurement = self.imu_processor.get_measurement_for_ekf(processed_imu)
                        self.current_state = self.ekf.update_imu(imu_measurement, dt)
                        
                        self.last_imu_time = current_time
                    
                    last_time = current_time
                
                # Sleep for target rate (100 Hz)
                time.sleep(0.01)
                
            except Exception as e:
                print(f"IMU loop error: {e}")
                time.sleep(0.1)
    
    def _gps_loop(self):
        """GPS data processing loop."""
        while self.running:
            try:
                # Read GPS data
                nmea_sentence = self.gps_driver.read_sentence()
                
                if nmea_sentence:
                    # Process NMEA sentence
                    gps_data = self.gps_processor.process_nmea_sentence(nmea_sentence)
                    
                    if gps_data and gps_data.is_valid:
                        # Update EKF with GPS measurement
                        gps_measurement = self.gps_processor.get_measurement_for_ekf(gps_data)
                        
                        if gps_measurement is not None:
                            self.current_state = self.ekf.update_gps(gps_measurement)
                            self.last_gps_time = time.time()
                            
                            print(f"GPS Fix: {gps_data.latitude:.6f}, {gps_data.longitude:.6f} "
                                  f"(Quality: {gps_data.fix_quality}, Sats: {gps_data.satellites})")
                
                # GPS typically updates at 1 Hz
                time.sleep(0.1)
                
            except Exception as e:
                print(f"GPS loop error: {e}")
                time.sleep(1.0)
    
    def _output_loop(self):
        """Output and logging loop."""
        last_output_time = time.time()
        output_interval = 1.0  # 1 Hz output
        
        while self.running:
            try:
                current_time = time.time()
                
                if current_time - last_output_time >= output_interval:
                    self._print_status()
                    last_output_time = current_time
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"Output loop error: {e}")
                time.sleep(1.0)
    
    def _print_status(self):
        """Print current system status."""
        current_time = time.time()
        uptime = current_time - self.start_time
        
        # Get current state
        state = self.current_state
        uncertainty = self.ekf.get_position_uncertainty()
        
        # Get GPS age
        gps_age = current_time - self.last_gps_time if self.last_gps_time > 0 else float('inf')
        imu_age = current_time - self.last_imu_time if self.last_imu_time > 0 else float('inf')
        
        print(f"\n=== Dead Reckoning Status (Uptime: {uptime:.1f}s) ===")
        print(f"Position: [{state.x:.2f}, {state.y:.2f}] m")
        print(f"Velocity: [{state.vx:.2f}, {state.vy:.2f}] m/s (Speed: {state.speed:.2f} m/s)")
        print(f"Heading:  {state.heading:.3f} rad ({state.heading * 180 / 3.14159:.1f}°)")
        print(f"Uncertainty: {uncertainty:.2f} m")
        print(f"Data Age: GPS {gps_age:.1f}s, IMU {imu_age:.3f}s")
        
        # Get statistics
        ekf_stats = self.ekf.get_statistics()
        imu_stats = self.imu_processor.get_statistics()
        gps_stats = self.gps_processor.get_statistics()
        
        print(f"EKF: {ekf_stats['predictions']} predictions, "
              f"{ekf_stats['gps_updates']} GPS updates, "
              f"{ekf_stats['imu_updates']} IMU updates")
        
        print(f"IMU: {imu_stats['sample_count']} samples, Calibrated: {imu_stats['is_calibrated']}")
        print(f"GPS: {gps_stats['fix_count']} fixes, Accuracy: {gps_stats['estimated_accuracy']:.1f}m")
    
    def get_current_position(self) -> dict:
        """Get current position for external API."""
        state = self.current_state
        uncertainty = self.ekf.get_position_uncertainty()
        
        # Convert local coordinates back to GPS if possible
        gps_pos = None
        if (self.gps_processor.reference_lat is not None and 
            self.gps_processor.reference_lon is not None):
            try:
                lat, lon = self.gps_processor.local_to_global(state.x, state.y)
                gps_pos = {'latitude': lat, 'longitude': lon}
            except:
                pass
        
        return {
            'timestamp': time.time(),
            'local_position': {'x': state.x, 'y': state.y},
            'global_position': gps_pos,
            'velocity': {'vx': state.vx, 'vy': state.vy, 'speed': state.speed},
            'heading': {'radians': state.heading, 'degrees': state.heading * 180 / 3.14159},
            'uncertainty': uncertainty,
            'gps_age': time.time() - self.last_gps_time if self.last_gps_time > 0 else None,
            'imu_age': time.time() - self.last_imu_time if self.last_imu_time > 0 else None
        }

def main():
    """Main entry point."""
    print("Dead Reckoning System for Raspberry Pi 3B")
    print("Hardware: MPU6050 + Ublox NEO6M")
    print("=" * 50)
    
    # Create and start system
    dr_system = DeadReckoningSystem()
    
    if not dr_system.start():
        print("Failed to start system")
        return 1
    
    try:
        # Keep main thread alive
        while dr_system.running:
            time.sleep(1.0)
    
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    
    finally:
        dr_system.stop()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())