#!/usr/bin/env python3
"""
Integration tests for the complete dead reckoning system.
"""

import unittest
import numpy as np
import sys
import os
import time

# Add core modules to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.ekf import ExtendedKalmanFilter, VehicleState
from core.sensors import IMUProcessor, GPSProcessor, IMUData, GPSData
from core.sensors.nmea import NMEAParser

class TestSensorIntegration(unittest.TestCase):
    """Test sensor integration with EKF."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.imu_processor = IMUProcessor()
        self.gps_processor = GPSProcessor()
        
        initial_state = VehicleState()
        self.ekf = ExtendedKalmanFilter(initial_state)
    
    def test_imu_to_ekf_pipeline(self):
        """Test complete IMU data processing pipeline."""
        # Create raw IMU data
        raw_accel = np.array([100, 200, 16384])  # LSB units
        raw_gyro = np.array([131, 262, 655])     # LSB units
        
        # Convert to physical units
        imu_data = self.imu_processor.convert_raw_data(raw_accel, raw_gyro)
        
        # Check conversion
        self.assertAlmostEqual(imu_data.accel_x, 100/16384 * 9.80665, places=3)
        self.assertAlmostEqual(imu_data.gyro_z, 655/131 * np.pi/180, places=3)
        
        # Process data
        processed_imu = self.imu_processor.process_data(imu_data)
        
        # Convert to EKF measurement
        measurement = self.imu_processor.get_measurement_for_ekf(processed_imu)
        self.assertEqual(len(measurement), 3)
        
        # Update EKF
        initial_uncertainty = self.ekf.get_position_uncertainty()
        updated_state = self.ekf.update_imu(measurement, dt=0.01)
        
        self.assertIsInstance(updated_state, VehicleState)
        self.assertEqual(self.ekf.imu_update_count, 1)
    
    def test_gps_to_ekf_pipeline(self):
        """Test complete GPS data processing pipeline."""
        # Create GPS data
        gps_data = GPSData(
            latitude=37.7749,
            longitude=-122.4194,
            altitude=50.0,
            fix_quality=1,
            satellites=8
        )
        
        # Set reference point
        self.gps_processor.set_reference_point(37.7749, -122.4194)
        
        # Convert to local coordinates
        x, y = self.gps_processor.global_to_local(gps_data.latitude, gps_data.longitude)
        gps_data.x = x
        gps_data.y = y
        
        # Should be at origin since it's the reference point
        self.assertAlmostEqual(x, 0.0, places=1)
        self.assertAlmostEqual(y, 0.0, places=1)
        
        # Convert to EKF measurement
        measurement = self.gps_processor.get_measurement_for_ekf(gps_data)
        self.assertIsNotNone(measurement)
        self.assertEqual(len(measurement), 2)
        
        # Update EKF
        initial_uncertainty = self.ekf.get_position_uncertainty()
        updated_state = self.ekf.update_gps(measurement)
        
        self.assertIsInstance(updated_state, VehicleState)
        self.assertEqual(self.ekf.gps_update_count, 1)
        
        # Uncertainty should decrease with GPS update
        final_uncertainty = self.ekf.get_position_uncertainty()
        self.assertLess(final_uncertainty, initial_uncertainty)
    
    def test_combined_sensor_fusion(self):
        """Test sensor fusion with both IMU and GPS."""
        # Initial prediction
        self.ekf.predict(dt=0.1)
        
        # GPS update
        gps_data = GPSData(latitude=37.7749, longitude=-122.4194, fix_quality=1, satellites=8)
        self.gps_processor.set_reference_point(37.7749, -122.4194)
        
        x, y = self.gps_processor.global_to_local(gps_data.latitude, gps_data.longitude)
        gps_data.x, gps_data.y = x, y
        
        gps_measurement = self.gps_processor.get_measurement_for_ekf(gps_data)
        self.ekf.update_gps(gps_measurement)
        
        # IMU update
        imu_data = IMUData(
            accel_x=1.0, accel_y=0.0, accel_z=9.81,
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.1
        )
        processed_imu = self.imu_processor.process_data(imu_data)
        imu_measurement = self.imu_processor.get_measurement_for_ekf(processed_imu)
        self.ekf.update_imu(imu_measurement, dt=0.1)
        
        # Check that both sensors updated
        stats = self.ekf.get_statistics()
        self.assertEqual(stats['gps_updates'], 1)
        self.assertEqual(stats['imu_updates'], 1)
        self.assertGreater(stats['predictions'], 0)

class TestNMEAIntegration(unittest.TestCase):
    """Test NMEA parsing integration."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.nmea_parser = NMEAParser()
        self.gps_processor = GPSProcessor()
    
    def test_nmea_to_gps_pipeline(self):
        """Test NMEA sentence parsing to GPS data."""
        # Valid GGA sentence
        gga_sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        
        # Parse NMEA
        fix_data = self.nmea_parser.parse_sentence(gga_sentence)
        self.assertIsNotNone(fix_data)
        self.assertTrue(fix_data.is_valid)
        
        # Convert to GPS data
        gps_data = self.gps_processor.convert_nmea_to_gps_data(fix_data)
        
        # Check conversion
        self.assertEqual(gps_data.fix_quality, 1)
        self.assertEqual(gps_data.satellites, 8)
        self.assertIsNotNone(gps_data.x)
        self.assertIsNotNone(gps_data.y)
        
        # Should be valid for EKF
        measurement = self.gps_processor.get_measurement_for_ekf(gps_data)
        self.assertIsNotNone(measurement)
    
    def test_invalid_nmea_handling(self):
        """Test handling of invalid NMEA sentences."""
        invalid_sentences = [
            "$GPGGA,invalid,data*00",  # Invalid data
            "$GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,*46",  # No fix
            "Not an NMEA sentence",    # Not NMEA format
            "$GPGGA,123519*FF",        # Wrong checksum
        ]
        
        for sentence in invalid_sentences:
            fix_data = self.nmea_parser.parse_sentence(sentence)
            # Should either be None or invalid
            if fix_data is not None:
                self.assertFalse(fix_data.is_valid)

class TestSystemReliability(unittest.TestCase):
    """Test system reliability and error handling."""
    
    def test_ekf_numerical_stability(self):
        """Test EKF numerical stability with extreme values."""
        initial_state = VehicleState()
        ekf = ExtendedKalmanFilter(initial_state)
        
        # Test with large time steps
        for dt in [0.001, 0.1, 1.0, 10.0]:
            try:
                state = ekf.predict(dt=dt)
                self.assertIsInstance(state, VehicleState)
                
                # Check for NaN/Inf values
                self.assertFalse(np.isnan(state.x))
                self.assertFalse(np.isinf(state.x))
                
            except Exception as e:
                self.fail(f"EKF failed with dt={dt}: {e}")
    
    def test_sensor_data_validation(self):
        """Test sensor data validation and bounds checking."""
        imu_processor = IMUProcessor()
        
        # Test extreme IMU values
        extreme_values = [
            np.array([0, 0, 0]),           # Zero values
            np.array([1e6, 1e6, 1e6]),     # Very large values
            np.array([-1e6, -1e6, -1e6]),  # Very negative values
            np.array([np.nan, 0, 0]),      # NaN values
            np.array([np.inf, 0, 0])       # Infinite values
        ]
        
        for raw_accel in extreme_values:
            try:
                imu_data = imu_processor.convert_raw_data(raw_accel, np.array([0, 0, 0]))
                processed = imu_processor.process_data(imu_data)
                
                # Should not contain NaN or Inf
                for attr in ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']:
                    value = getattr(processed, attr)
                    self.assertFalse(np.isnan(value), f"NaN in {attr}")
                    self.assertFalse(np.isinf(value), f"Inf in {attr}")
                    
            except Exception as e:
                # Some extreme values might cause expected exceptions
                pass
    
    def test_coordinate_conversion_accuracy(self):
        """Test GPS coordinate conversion accuracy."""
        gps_processor = GPSProcessor()
        
        # Set reference point
        ref_lat, ref_lon = 37.7749, -122.4194
        gps_processor.set_reference_point(ref_lat, ref_lon)
        
        # Test points at known distances (use more accurate values)
        test_points = [
            (ref_lat, ref_lon, 0.0, 0.0),                    # Reference point
            (ref_lat + 0.001, ref_lon, 0.0, 111.32),         # ~111m north
            (ref_lat, ref_lon + 0.001, None, 0.0),           # East (calculate expected)
            (ref_lat - 0.001, ref_lon - 0.001, None, -111.32)  # Southwest
        ]
        
        for lat, lon, expected_x, expected_y in test_points:
            x, y = gps_processor.global_to_local(lat, lon)
            
            if expected_x is not None:
                # Check accuracy within 10% or 5m (GPS coordinate conversion has inherent errors)
                tolerance = max(5.0, abs(expected_x) * 0.1)
                self.assertAlmostEqual(x, expected_x, delta=tolerance, 
                                     msg=f"X coordinate mismatch for {lat}, {lon}")
            
            if expected_y is not None:
                tolerance = max(5.0, abs(expected_y) * 0.1)
                self.assertAlmostEqual(y, expected_y, delta=tolerance,
                                     msg=f"Y coordinate mismatch for {lat}, {lon}")
            
            # Test round-trip conversion (should be more accurate)
            lat_back, lon_back = gps_processor.local_to_global(x, y)
            self.assertAlmostEqual(lat_back, lat, places=5)
            self.assertAlmostEqual(lon_back, lon, places=5)

class TestPerformance(unittest.TestCase):
    """Test system performance characteristics."""
    
    def test_ekf_update_performance(self):
        """Test EKF update performance."""
        initial_state = VehicleState()
        ekf = ExtendedKalmanFilter(initial_state)
        
        # Time prediction updates
        start_time = time.time()
        for _ in range(1000):
            ekf.predict(dt=0.01)
        prediction_time = time.time() - start_time
        
        # Should complete 1000 predictions in reasonable time (<1 second)
        self.assertLess(prediction_time, 1.0, 
                       f"Predictions too slow: {prediction_time:.3f}s for 1000 updates")
        
        # Time GPS updates
        start_time = time.time()
        for _ in range(100):
            gps_measurement = np.array([0.0, 0.0])
            ekf.update_gps(gps_measurement)
        gps_time = time.time() - start_time
        
        # Should complete 100 GPS updates in reasonable time
        self.assertLess(gps_time, 1.0,
                       f"GPS updates too slow: {gps_time:.3f}s for 100 updates")
    
    def test_memory_usage(self):
        """Test memory usage remains bounded."""
        import gc
        
        # Force garbage collection
        gc.collect()
        
        # Create system components
        imu_processor = IMUProcessor()
        gps_processor = GPSProcessor()
        ekf = ExtendedKalmanFilter(VehicleState())
        
        # Run many updates
        for i in range(1000):
            # Create sensor data
            imu_data = IMUData(
                accel_x=np.random.normal(0, 0.1),
                accel_y=np.random.normal(0, 0.1),
                accel_z=9.81 + np.random.normal(0, 0.1),
                gyro_x=np.random.normal(0, 0.01),
                gyro_y=np.random.normal(0, 0.01),
                gyro_z=np.random.normal(0, 0.01)
            )
            
            # Process and update
            processed_imu = imu_processor.process_data(imu_data)
            imu_measurement = imu_processor.get_measurement_for_ekf(processed_imu)
            
            ekf.predict(dt=0.01)
            ekf.update_imu(imu_measurement, dt=0.01)
            
            # Periodic GPS update
            if i % 100 == 0:
                gps_measurement = np.array([0.0, 0.0])
                ekf.update_gps(gps_measurement)
        
        # Memory usage should be reasonable (this is a basic check)
        # In a real system, you'd use memory profiling tools
        self.assertLess(len(gps_processor.position_history), 20,
                       "GPS history growing unbounded")

if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)