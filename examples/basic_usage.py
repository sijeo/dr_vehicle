#!/usr/bin/env python3
"""
Basic usage example of the dead reckoning system.

This example demonstrates how to use the core dead reckoning algorithms
without specific hardware dependencies.
"""

import sys
import os
import time
import numpy as np

# Add core modules to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.ekf import ExtendedKalmanFilter, VehicleState
from core.sensors import IMUProcessor, GPSProcessor, IMUData, GPSData

def simulate_vehicle_motion(duration=60, dt=0.1):
    """
    Simulate a vehicle moving in a simple pattern for testing.
    
    Args:
        duration: Simulation duration in seconds
        dt: Time step in seconds
        
    Yields:
        (timestamp, imu_data, gps_data) tuples
    """
    # Vehicle motion parameters
    speed = 10.0  # m/s
    turn_radius = 50.0  # meters
    angular_velocity = speed / turn_radius  # rad/s
    
    # Starting position (San Francisco)
    start_lat = 37.7749
    start_lon = -122.4194
    
    # Noise parameters
    accel_noise = 0.1  # m/s²
    gyro_noise = 0.01  # rad/s
    gps_noise = 0.00001  # degrees (~1m)
    
    t = 0
    while t < duration:
        timestamp = time.time() + t
        
        # Calculate vehicle position (circular motion)
        x = turn_radius * np.sin(angular_velocity * t)
        y = turn_radius * (1 - np.cos(angular_velocity * t))
        
        # Calculate vehicle dynamics
        vx = speed * np.cos(angular_velocity * t)
        vy = speed * np.sin(angular_velocity * t)
        ax = -speed * angular_velocity * np.sin(angular_velocity * t)
        ay = speed * angular_velocity * np.cos(angular_velocity * t)
        
        # Convert to lat/lon (approximate)
        earth_radius = 6371000  # meters
        lat_offset = y / earth_radius * 180 / np.pi
        lon_offset = x / (earth_radius * np.cos(np.radians(start_lat))) * 180 / np.pi
        
        # Add noise and create sensor data
        imu_data = IMUData(
            accel_x=ax + np.random.normal(0, accel_noise),
            accel_y=ay + np.random.normal(0, accel_noise),
            accel_z=9.81 + np.random.normal(0, accel_noise),  # gravity + noise
            gyro_x=np.random.normal(0, gyro_noise),
            gyro_y=np.random.normal(0, gyro_noise),
            gyro_z=angular_velocity + np.random.normal(0, gyro_noise),
            timestamp=timestamp
        )
        
        # Generate GPS data (every 1 second)
        gps_data = None
        if t % 1.0 < dt:  # GPS updates at 1 Hz
            gps_data = GPSData(
                latitude=start_lat + lat_offset + np.random.normal(0, gps_noise),
                longitude=start_lon + lon_offset + np.random.normal(0, gps_noise),
                altitude=50.0,
                fix_quality=1,
                satellites=8,
                timestamp=timestamp
            )
        
        yield timestamp, imu_data, gps_data
        
        t += dt

def main():
    """Main example function."""
    print("Dead Reckoning System - Basic Usage Example")
    print("=" * 50)
    
    # Initialize processors
    imu_processor = IMUProcessor()
    gps_processor = GPSProcessor()
    
    # Initialize EKF with default state
    initial_state = VehicleState(
        x=0.0, y=0.0,           # Start at origin
        vx=0.0, vy=0.0,         # Start stationary
        heading=0.0,            # Facing north
        heading_rate=0.0        # No initial rotation
    )
    
    ekf = ExtendedKalmanFilter(initial_state)
    
    print("Initialized dead reckoning system")
    print(f"Initial state: {initial_state}")
    print()
    
    # Simulate vehicle motion
    print("Starting simulation (circular motion, 60 seconds)...")
    
    last_print_time = 0
    print_interval = 5.0  # Print status every 5 seconds
    
    for timestamp, imu_data, gps_data in simulate_vehicle_motion(duration=60, dt=0.1):
        
        # Process IMU data
        processed_imu = imu_processor.process_data(imu_data, apply_filtering=True)
        
        # Prediction step
        predicted_state = ekf.predict(dt=0.1)
        
        # IMU update
        imu_measurement = imu_processor.get_measurement_for_ekf(processed_imu)
        updated_state = ekf.update_imu(imu_measurement, dt=0.1)
        
        # GPS update (when available)
        if gps_data:
            # Set reference point for first GPS fix
            if gps_processor.reference_lat is None:
                gps_processor.set_reference_point(gps_data.latitude, gps_data.longitude)
            
            # Convert GPS to local coordinates
            x, y = gps_processor.global_to_local(gps_data.latitude, gps_data.longitude)
            gps_data.x = x
            gps_data.y = y
            
            # GPS measurement update
            gps_measurement = gps_processor.get_measurement_for_ekf(gps_data)
            if gps_measurement is not None:
                updated_state = ekf.update_gps(gps_measurement)
        
        # Print status periodically
        if timestamp - last_print_time >= print_interval:
            print_status(updated_state, ekf, timestamp)
            last_print_time = timestamp
    
    print("\nSimulation completed!")
    
    # Final statistics
    final_stats = ekf.get_statistics()
    print("\n=== Final Statistics ===")
    print(f"EKF Predictions: {final_stats['predictions']}")
    print(f"GPS Updates: {final_stats['gps_updates']}")
    print(f"IMU Updates: {final_stats['imu_updates']}")
    print(f"Final Position Uncertainty: {final_stats['position_uncertainty']:.2f} m")

def print_status(state: VehicleState, ekf: ExtendedKalmanFilter, timestamp: float):
    """Print current system status."""
    uncertainty = ekf.get_position_uncertainty()
    
    print(f"Time: {timestamp:.1f}s")
    print(f"  Position: [{state.x:6.2f}, {state.y:6.2f}] m")
    print(f"  Velocity: [{state.vx:5.2f}, {state.vy:5.2f}] m/s (Speed: {state.speed:5.2f} m/s)")
    print(f"  Heading:  {state.heading:6.3f} rad ({np.degrees(state.heading):6.1f}°)")
    print(f"  Uncertainty: {uncertainty:5.2f} m")
    print()

if __name__ == "__main__":
    main()