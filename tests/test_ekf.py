#!/usr/bin/env python3
"""
Unit tests for Extended Kalman Filter implementation.
"""

import unittest
import numpy as np
import sys
import os

# Add core modules to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.ekf import ExtendedKalmanFilter, VehicleState
from core.ekf.models import MotionModel, MeasurementModel

class TestVehicleState(unittest.TestCase):
    """Test VehicleState class."""
    
    def test_initialization(self):
        """Test state initialization."""
        state = VehicleState(x=1.0, y=2.0, vx=3.0, vy=4.0, heading=0.5, heading_rate=0.1)
        
        self.assertEqual(state.x, 1.0)
        self.assertEqual(state.y, 2.0)
        self.assertEqual(state.vx, 3.0)
        self.assertEqual(state.vy, 4.0)
        self.assertEqual(state.heading, 0.5)
        self.assertEqual(state.heading_rate, 0.1)
        self.assertIsNotNone(state.timestamp)
    
    def test_state_vector_property(self):
        """Test state vector conversion."""
        state = VehicleState(x=1.0, y=2.0, vx=3.0, vy=4.0, heading=0.5, heading_rate=0.1)
        
        expected_vector = np.array([1.0, 2.0, 3.0, 4.0, 0.5, 0.1])
        np.testing.assert_array_equal(state.state_vector, expected_vector)
        
        # Test setter
        new_vector = np.array([10.0, 20.0, 30.0, 40.0, 1.5, 0.2])
        state.state_vector = new_vector
        
        self.assertEqual(state.x, 10.0)
        self.assertEqual(state.y, 20.0)
        self.assertEqual(state.vx, 30.0)
        self.assertEqual(state.vy, 40.0)
        self.assertEqual(state.heading, 1.5)
        self.assertEqual(state.heading_rate, 0.2)
    
    def test_speed_property(self):
        """Test speed calculation."""
        state = VehicleState(vx=3.0, vy=4.0)
        self.assertAlmostEqual(state.speed, 5.0, places=6)
    
    def test_copy(self):
        """Test state copying."""
        original = VehicleState(x=1.0, y=2.0, vx=3.0, vy=4.0, heading=0.5, heading_rate=0.1)
        copy = original.copy()
        
        self.assertEqual(copy.x, original.x)
        self.assertEqual(copy.y, original.y)
        self.assertEqual(copy.vx, original.vx)
        self.assertEqual(copy.vy, original.vy)
        self.assertEqual(copy.heading, original.heading)
        self.assertEqual(copy.heading_rate, original.heading_rate)
        
        # Ensure it's a different object
        copy.x = 100.0
        self.assertNotEqual(copy.x, original.x)

class TestMotionModel(unittest.TestCase):
    """Test MotionModel class."""
    
    def test_predict_state_constant_velocity(self):
        """Test state prediction with constant velocity."""
        state = np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.0])  # Moving at 5 m/s in x direction
        dt = 1.0
        
        predicted = MotionModel.predict_state(state, dt)
        
        # Position should update: x = x + vx * dt = 0 + 5 * 1 = 5
        self.assertAlmostEqual(predicted[0], 5.0)
        self.assertAlmostEqual(predicted[1], 0.0)
        
        # Velocity should remain constant
        self.assertAlmostEqual(predicted[2], 5.0)
        self.assertAlmostEqual(predicted[3], 0.0)
    
    def test_predict_state_with_acceleration(self):
        """Test state prediction with acceleration."""
        state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Starting from rest
        dt = 2.0
        control_input = np.array([1.0, 0.0, 0.0])  # 1 m/s² acceleration in x
        
        predicted = MotionModel.predict_state(state, dt, control_input)
        
        # Position: x = 0.5 * ax * dt² = 0.5 * 1.0 * 4 = 2.0
        self.assertAlmostEqual(predicted[0], 2.0)
        
        # Velocity: vx = ax * dt = 1.0 * 2 = 2.0
        self.assertAlmostEqual(predicted[2], 2.0)
    
    def test_jacobian_F(self):
        """Test Jacobian matrix calculation."""
        state = np.array([1.0, 2.0, 3.0, 4.0, 0.5, 0.1])
        dt = 0.1
        
        F = MotionModel.jacobian_F(state, dt)
        
        # Check dimensions
        self.assertEqual(F.shape, (6, 6))
        
        # Check diagonal elements (should be 1.0)
        for i in range(6):
            self.assertAlmostEqual(F[i, i], 1.0)
        
        # Check position-velocity coupling
        self.assertAlmostEqual(F[0, 2], dt)  # dx/dvx
        self.assertAlmostEqual(F[1, 3], dt)  # dy/dvy
        
        # Check heading-heading_rate coupling
        self.assertAlmostEqual(F[4, 5], dt)  # dheading/dheading_rate
    
    def test_process_noise_matrix(self):
        """Test process noise matrix generation."""
        Q_params = {
            'position': 0.1,
            'velocity': 0.2,
            'heading': 0.05,
            'heading_rate': 0.02
        }
        dt = 0.1
        
        Q = MotionModel.process_noise_matrix(Q_params, dt)
        
        # Check dimensions
        self.assertEqual(Q.shape, (6, 6))
        
        # Check that it's diagonal
        for i in range(6):
            for j in range(6):
                if i != j:
                    self.assertAlmostEqual(Q[i, j], 0.0)
        
        # Check that noise increases with time
        self.assertGreater(Q[0, 0], 0)  # Position noise
        self.assertGreater(Q[2, 2], 0)  # Velocity noise

class TestMeasurementModel(unittest.TestCase):
    """Test MeasurementModel class."""
    
    def test_gps_measurement(self):
        """Test GPS measurement model."""
        state = np.array([10.0, 20.0, 1.0, 2.0, 0.5, 0.1])
        
        measurement = MeasurementModel.gps_measurement(state)
        
        # GPS should directly observe position
        np.testing.assert_array_equal(measurement, [10.0, 20.0])
    
    def test_gps_jacobian_H(self):
        """Test GPS measurement Jacobian."""
        state = np.array([10.0, 20.0, 1.0, 2.0, 0.5, 0.1])
        
        H = MeasurementModel.gps_jacobian_H(state)
        
        # Check dimensions
        self.assertEqual(H.shape, (2, 6))
        
        # GPS directly observes x and y positions
        self.assertAlmostEqual(H[0, 0], 1.0)  # dx/dx
        self.assertAlmostEqual(H[1, 1], 1.0)  # dy/dy
        
        # All other elements should be zero
        for i in range(2):
            for j in range(2, 6):
                self.assertAlmostEqual(H[i, j], 0.0)
    
    def test_imu_measurement(self):
        """Test IMU measurement model."""
        state = np.array([10.0, 20.0, 1.0, 2.0, 0.5, 0.1])
        dt = 0.1
        
        measurement = MeasurementModel.imu_measurement(state, dt)
        
        # Should return [ax, ay, omega_z]
        self.assertEqual(len(measurement), 3)
        
        # Heading rate should be directly observed
        self.assertAlmostEqual(measurement[2], 0.1)
    
    def test_measurement_noise_matrix(self):
        """Test measurement noise matrix generation."""
        R_params = {
            'gps_position': 5.0,
            'imu_accel': 0.1,
            'imu_gyro': 0.01
        }
        
        # Test GPS noise matrix
        R_gps = MeasurementModel.measurement_noise_matrix(R_params, 'gps')
        self.assertEqual(R_gps.shape, (2, 2))
        self.assertAlmostEqual(R_gps[0, 0], 5.0)
        self.assertAlmostEqual(R_gps[1, 1], 5.0)
        
        # Test IMU noise matrix
        R_imu = MeasurementModel.measurement_noise_matrix(R_params, 'imu')
        self.assertEqual(R_imu.shape, (3, 3))
        self.assertAlmostEqual(R_imu[0, 0], 0.1)
        self.assertAlmostEqual(R_imu[1, 1], 0.1)
        self.assertAlmostEqual(R_imu[2, 2], 0.01)

class TestExtendedKalmanFilter(unittest.TestCase):
    """Test ExtendedKalmanFilter class."""
    
    def setUp(self):
        """Set up test fixtures."""
        initial_state = VehicleState()
        self.ekf = ExtendedKalmanFilter(initial_state)
    
    def test_initialization(self):
        """Test EKF initialization."""
        self.assertEqual(len(self.ekf.state), 6)
        self.assertEqual(self.ekf.P.shape, (6, 6))
        self.assertEqual(self.ekf.prediction_count, 0)  # No predictions yet
    
    def test_predict(self):
        """Test prediction step."""
        initial_state_vector = self.ekf.state.copy()
        
        # Predict with small time step
        predicted_state = self.ekf.predict(dt=0.1)
        
        # State should be updated
        self.assertIsInstance(predicted_state, VehicleState)
        
        # Prediction count should increase
        self.assertEqual(self.ekf.prediction_count, 1)
    
    def test_gps_update(self):
        """Test GPS measurement update."""
        initial_uncertainty = self.ekf.get_position_uncertainty()
        
        # GPS measurement at origin
        gps_measurement = np.array([0.0, 0.0])
        
        updated_state = self.ekf.update_gps(gps_measurement)
        
        # Update count should increase
        self.assertGreater(self.ekf.gps_update_count, 0)
        
        # Uncertainty should decrease (GPS provides position information)
        final_uncertainty = self.ekf.get_position_uncertainty()
        self.assertLess(final_uncertainty, initial_uncertainty)
    
    def test_imu_update(self):
        """Test IMU measurement update."""
        initial_count = self.ekf.imu_update_count
        
        # IMU measurement (acceleration and angular velocity)
        imu_measurement = np.array([0.0, 0.0, 0.0])
        
        updated_state = self.ekf.update_imu(imu_measurement, dt=0.1)
        
        # Update count should increase
        self.assertGreater(self.ekf.imu_update_count, initial_count)
    
    def test_get_statistics(self):
        """Test statistics retrieval."""
        stats = self.ekf.get_statistics()
        
        required_keys = ['predictions', 'gps_updates', 'imu_updates', 
                        'position_uncertainty', 'state_uncertainty']
        
        for key in required_keys:
            self.assertIn(key, stats)
        
        self.assertIsInstance(stats['position_uncertainty'], float)
        self.assertIsInstance(stats['state_uncertainty'], list)
        self.assertEqual(len(stats['state_uncertainty']), 6)
    
    def test_reset(self):
        """Test filter reset."""
        # Do some operations first
        self.ekf.predict(dt=0.1)
        self.ekf.update_gps(np.array([1.0, 1.0]))
        
        # Reset with new state
        new_state = VehicleState(x=10.0, y=20.0)
        self.ekf.reset(new_state)
        
        # Counters should be reset
        self.assertEqual(self.ekf.prediction_count, 0)
        self.assertEqual(self.ekf.gps_update_count, 0)
        self.assertEqual(self.ekf.imu_update_count, 0)
        
        # State should be updated
        current_state = self.ekf.get_current_state()
        self.assertAlmostEqual(current_state.x, 10.0)
        self.assertAlmostEqual(current_state.y, 20.0)

if __name__ == '__main__':
    unittest.main()