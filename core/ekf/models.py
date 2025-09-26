"""
Motion and measurement models for the Extended Kalman Filter.
"""

import numpy as np
import math
from typing import Tuple
from ..math.utils import normalize_angle

class MotionModel:
    """
    Constant velocity motion model with heading dynamics.
    
    State: [x, y, vx, vy, heading, heading_rate]
    """
    
    @staticmethod
    def predict_state(state: np.ndarray, dt: float, control_input: np.ndarray = None) -> np.ndarray:
        """
        Predict next state using motion model.
        
        Args:
            state: Current state [x, y, vx, vy, heading, heading_rate]
            dt: Time step in seconds
            control_input: Optional control input [ax, ay, alpha] (acceleration and angular acceleration)
            
        Returns:
            Predicted state vector
        """
        x, y, vx, vy, heading, heading_rate = state
        
        # Default control input (no acceleration)
        if control_input is None:
            ax, ay, alpha = 0.0, 0.0, 0.0
        else:
            ax, ay, alpha = control_input
        
        # Position update (constant velocity + acceleration)
        x_new = x + vx * dt + 0.5 * ax * dt**2
        y_new = y + vy * dt + 0.5 * ay * dt**2
        
        # Velocity update
        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        
        # Heading update (constant angular velocity + angular acceleration)
        heading_new = normalize_angle(heading + heading_rate * dt + 0.5 * alpha * dt**2)
        heading_rate_new = heading_rate + alpha * dt
        
        return np.array([x_new, y_new, vx_new, vy_new, heading_new, heading_rate_new])
    
    @staticmethod
    def jacobian_F(state: np.ndarray, dt: float) -> np.ndarray:
        """
        Compute Jacobian of motion model with respect to state.
        
        Args:
            state: Current state vector
            dt: Time step
            
        Returns:
            6x6 Jacobian matrix F
        """
        F = np.eye(6)
        
        # Position derivatives
        F[0, 2] = dt  # dx/dvx
        F[1, 3] = dt  # dy/dvy
        
        # Heading derivatives  
        F[4, 5] = dt  # dheading/dheading_rate
        
        return F
    
    @staticmethod
    def process_noise_matrix(Q_params: dict, dt: float) -> np.ndarray:
        """
        Compute process noise covariance matrix.
        
        Args:
            Q_params: Dictionary with noise parameters
            dt: Time step
            
        Returns:
            6x6 process noise covariance matrix Q
        """
        # Extract noise parameters
        q_pos = Q_params.get('position', 0.1)
        q_vel = Q_params.get('velocity', 0.1)
        q_heading = Q_params.get('heading', 0.01)
        q_heading_rate = Q_params.get('heading_rate', 0.01)
        
        # Process noise increases with time
        Q = np.diag([
            q_pos * dt**2,      # x position noise
            q_pos * dt**2,      # y position noise
            q_vel * dt,         # vx velocity noise
            q_vel * dt,         # vy velocity noise
            q_heading * dt**2,  # heading noise
            q_heading_rate * dt # heading rate noise
        ])
        
        return Q

class MeasurementModel:
    """
    Measurement model for GPS and IMU sensors.
    """
    
    @staticmethod
    def gps_measurement(state: np.ndarray) -> np.ndarray:
        """
        GPS measurement model - directly observes position.
        
        Args:
            state: Current state vector
            
        Returns:
            Expected GPS measurement [x, y]
        """
        x, y = state[0], state[1]
        return np.array([x, y])
    
    @staticmethod
    def gps_jacobian_H(state: np.ndarray) -> np.ndarray:
        """
        Jacobian of GPS measurement model.
        
        Args:
            state: Current state vector
            
        Returns:
            2x6 Jacobian matrix H for GPS
        """
        H = np.zeros((2, 6))
        H[0, 0] = 1.0  # dx/dx
        H[1, 1] = 1.0  # dy/dy
        return H
    
    @staticmethod
    def imu_measurement(state: np.ndarray, dt: float, prev_state: np.ndarray = None) -> np.ndarray:
        """
        IMU measurement model - observes acceleration and angular velocity.
        
        Args:
            state: Current state vector
            dt: Time step
            prev_state: Previous state for velocity differentiation
            
        Returns:
            Expected IMU measurement [ax, ay, omega_z]
        """
        if prev_state is None:
            # If no previous state, assume zero acceleration
            ax, ay = 0.0, 0.0
        else:
            # Compute acceleration from velocity difference
            dvx = state[2] - prev_state[2]
            dvy = state[3] - prev_state[3]
            ax = dvx / dt if dt > 0 else 0.0
            ay = dvy / dt if dt > 0 else 0.0
        
        # Angular velocity is directly the heading rate
        omega_z = state[5]
        
        return np.array([ax, ay, omega_z])
    
    @staticmethod
    def imu_jacobian_H(state: np.ndarray, dt: float) -> np.ndarray:
        """
        Jacobian of IMU measurement model.
        
        Args:
            state: Current state vector
            dt: Time step
            
        Returns:
            3x6 Jacobian matrix H for IMU
        """
        H = np.zeros((3, 6))
        
        # For this simplified model:
        # ax and ay are not directly observable from current state
        # omega_z directly observes heading_rate
        H[2, 5] = 1.0  # domega_z/dheading_rate
        
        return H
    
    @staticmethod
    def measurement_noise_matrix(R_params: dict, measurement_type: str) -> np.ndarray:
        """
        Get measurement noise covariance matrix.
        
        Args:
            R_params: Dictionary with noise parameters
            measurement_type: 'gps' or 'imu'
            
        Returns:
            Measurement noise covariance matrix R
        """
        if measurement_type == 'gps':
            r_gps = R_params.get('gps_position', 5.0)
            return np.diag([r_gps, r_gps])
        
        elif measurement_type == 'imu':
            r_accel = R_params.get('imu_accel', 0.1)
            r_gyro = R_params.get('imu_gyro', 0.01)
            return np.diag([r_accel, r_accel, r_gyro])
        
        else:
            raise ValueError(f"Unknown measurement type: {measurement_type}")