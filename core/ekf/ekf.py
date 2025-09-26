"""
Extended Kalman Filter implementation for vehicle dead reckoning.
"""

import numpy as np
import time
from typing import Optional, Dict, Any, Tuple
from .state import VehicleState
from .models import MotionModel, MeasurementModel
from ..math.constants import *

class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for vehicle state estimation using IMU and GPS.
    """
    
    def __init__(self, initial_state: VehicleState, 
                 process_noise: Dict[str, float] = None,
                 measurement_noise: Dict[str, float] = None):
        """
        Initialize the Extended Kalman Filter.
        
        Args:
            initial_state: Initial vehicle state
            process_noise: Process noise parameters
            measurement_noise: Measurement noise parameters
        """
        # State vector and covariance
        self.state = initial_state.state_vector
        self.P = self._initialize_covariance()
        
        # Models
        self.motion_model = MotionModel()
        self.measurement_model = MeasurementModel()
        
        # Noise parameters
        self.Q_params = process_noise or {
            'position': Q_POSITION,
            'velocity': Q_VELOCITY,
            'heading': Q_HEADING,
            'heading_rate': Q_HEADING_RATE
        }
        
        self.R_params = measurement_noise or {
            'gps_position': R_GPS_POSITION,
            'imu_accel': R_IMU_ACCEL,
            'imu_gyro': R_IMU_GYRO
        }
        
        # Timing
        self.last_predict_time = initial_state.timestamp or time.time()
        self.last_state = self.state.copy()
        
        # Statistics
        self.prediction_count = 0
        self.gps_update_count = 0
        self.imu_update_count = 0
    
    def _initialize_covariance(self) -> np.ndarray:
        """Initialize state covariance matrix."""
        return np.diag([
            INITIAL_POSITION_UNCERTAINTY**2,        # x variance
            INITIAL_POSITION_UNCERTAINTY**2,        # y variance
            INITIAL_VELOCITY_UNCERTAINTY**2,        # vx variance
            INITIAL_VELOCITY_UNCERTAINTY**2,        # vy variance
            INITIAL_HEADING_UNCERTAINTY**2,         # heading variance
            INITIAL_HEADING_RATE_UNCERTAINTY**2     # heading_rate variance
        ])
    
    def predict(self, dt: Optional[float] = None, control_input: np.ndarray = None) -> VehicleState:
        """
        Prediction step of the Kalman filter.
        
        Args:
            dt: Time step in seconds (if None, computed from timestamps)
            control_input: Optional control input [ax, ay, alpha]
            
        Returns:
            Predicted vehicle state
        """
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_predict_time
            self.last_predict_time = current_time
        
        # Store previous state for IMU measurement model
        self.last_state = self.state.copy()
        
        # Predict state using motion model
        self.state = self.motion_model.predict_state(self.state, dt, control_input)
        
        # Compute Jacobian and process noise
        F = self.motion_model.jacobian_F(self.state, dt)
        Q = self.motion_model.process_noise_matrix(self.Q_params, dt)
        
        # Update covariance: P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + Q
        
        self.prediction_count += 1
        
        # Return predicted state
        predicted_state = VehicleState()
        predicted_state.state_vector = self.state
        predicted_state.timestamp = self.last_predict_time
        
        return predicted_state
    
    def update_gps(self, gps_measurement: np.ndarray) -> VehicleState:
        """
        Update step with GPS measurement.
        
        Args:
            gps_measurement: GPS position measurement [x, y]
            
        Returns:
            Updated vehicle state
        """
        # Predicted measurement
        z_pred = self.measurement_model.gps_measurement(self.state)
        
        # Innovation (measurement residual)
        y = gps_measurement - z_pred
        
        # Measurement Jacobian
        H = self.measurement_model.gps_jacobian_H(self.state)
        
        # Measurement noise covariance
        R = self.measurement_model.measurement_noise_matrix(self.R_params, 'gps')
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.state = self.state + K @ y
        I = np.eye(len(self.state))
        self.P = (I - K @ H) @ self.P
        
        self.gps_update_count += 1
        
        # Return updated state
        updated_state = VehicleState()
        updated_state.state_vector = self.state
        updated_state.timestamp = time.time()
        
        return updated_state
    
    def update_imu(self, imu_measurement: np.ndarray, dt: float) -> VehicleState:
        """
        Update step with IMU measurement.
        
        Args:
            imu_measurement: IMU measurement [ax, ay, omega_z]
            dt: Time step since last measurement
            
        Returns:
            Updated vehicle state
        """
        # Predicted measurement
        z_pred = self.measurement_model.imu_measurement(self.state, dt, self.last_state)
        
        # Innovation (measurement residual)
        y = imu_measurement - z_pred
        
        # Measurement Jacobian
        H = self.measurement_model.imu_jacobian_H(self.state, dt)
        
        # Measurement noise covariance
        R = self.measurement_model.measurement_noise_matrix(self.R_params, 'imu')
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain (handle potential singularity)
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # If S is singular, use pseudo-inverse
            K = self.P @ H.T @ np.linalg.pinv(S)
        
        # Update state and covariance
        self.state = self.state + K @ y
        I = np.eye(len(self.state))
        self.P = (I - K @ H) @ self.P
        
        self.imu_update_count += 1
        
        # Return updated state
        updated_state = VehicleState()
        updated_state.state_vector = self.state
        updated_state.timestamp = time.time()
        
        return updated_state
    
    def get_current_state(self) -> VehicleState:
        """Get current estimated state."""
        current_state = VehicleState()
        current_state.state_vector = self.state
        current_state.timestamp = time.time()
        return current_state
    
    def get_uncertainty(self) -> np.ndarray:
        """Get current state uncertainty (diagonal of covariance matrix)."""
        return np.sqrt(np.diag(self.P))
    
    def get_position_uncertainty(self) -> float:
        """Get position uncertainty (2D RMS error)."""
        pos_var = self.P[0, 0] + self.P[1, 1]
        return np.sqrt(pos_var)
    
    def reset(self, new_state: VehicleState):
        """Reset filter with new initial state."""
        self.state = new_state.state_vector
        self.P = self._initialize_covariance()
        self.last_predict_time = new_state.timestamp or time.time()
        self.last_state = self.state.copy()
        
        # Reset counters
        self.prediction_count = 0
        self.gps_update_count = 0
        self.imu_update_count = 0
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get filter statistics."""
        return {
            'predictions': self.prediction_count,
            'gps_updates': self.gps_update_count,
            'imu_updates': self.imu_update_count,
            'position_uncertainty': self.get_position_uncertainty(),
            'state_uncertainty': self.get_uncertainty().tolist()
        }