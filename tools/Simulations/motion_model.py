"""
motion_model.py

This module defines a simple kinematic bycycle style model used to 
update the "ground truth" state of the stimulated vehicle. The 
model integrates user defined acceleration and yaw rate commands into
a tragectory on flat plane (z = 0). It also provides helper functions for 
converting yaw angles to quaternions.

Classes:
VehicleState: 
    Holds the true position, heading(yaw), and speed of the vehicle
    and updates the state based on control inputs.

Functions:
yaw_to_quaternion(yaw: float) -> np.ndarray:
    Converts a yaw angle (in radians) to a quaternion representation.
    represented about world z axis.
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass

def yaw_to_quaternion(yaw: float) -> np.ndarray:
    """
    Convert a yaw angle into a quaternion representation. (w, x, y, z)
    
    Yaw is a rotation about the Z axis in the ENU (East, North, Up) frame.
    This helper produces a unit quaternion representing that rotation.

    Args:
        yaw (float): Yaw angle in radians. Positive yaw is a counter-clockwise
                     rotation in the ENU frame.
    Returns:
        np.ndarray: A numpy array representing the quaternion (w, x, y, z).
    """

    return np.array([
        np.cos(yaw / 2),  # w
        0,                 # x
        0,                 # y
        np.sin(yaw / 2)    # z
    ])

@dataclass
class VehicleState:
    """Ground truth state for a planar vehicle.
    Attributes:
        x (float): X position in meters.
        y (float): Y position in meters.
        yaw (float): Heading angle in radians.
        v(float): Speed in meters per second.
        
        
        """
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0

    def update(self, acc_cmd: float, yaw_rate_cmd: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Advance the vehicle state using commanded acceleration and yaw rate
        
        The update integrates forward speed and heading using simple kinematics.
        The acceleration commands are clamped to keep speed non-negative. A Trapezoidal
        integration scheme is used for position to provide better accuracy than a simple
        Euler step.

        Args:
            acc_cmd (float): Commanded acceleration in meters per second squared.
            yaw_rate_cmd (float): Commanded yaw rate in radians per second.
            dt (float): Time step for the update in seconds.

        Returns:
            accel_world (np.ndarray): Acceleration vector in world frame (ax, ay, az).
                Linear acceleration of the vehicle in world coordinates. This excludes
                gravity (i.e. translational acceleration only).
            gyro_world (np.ndarray): Angular velocity vector in world frame (wx, wy, wz).
                For planar motion only the z component is non-zero.

            
        """

        # Save initial velocity components
        vx_old = self.v * np.cos(self.yaw)
        vy_old = self.v * np.sin(self.yaw)
        # Integrate speed with commanded acceleration
        self.v += acc_cmd * dt
        if self.v < 0:
            self.v = 0.0
        # Integrate yaw
        self.yaw += yaw_rate_cmd * dt
        # Compute new velocity components
        vx_new = self.v * np.cos(self.yaw)
        vy_new = self.v * np.sin(self.yaw)
        # Trapezoidal integration for position
        self.x += 0.5 * (vx_old + vx_new) * dt
        self.y += 0.5 * (vy_old + vy_new) * dt
        # For a flat plane z remains constant at Zero
        self.z = 0.0
        # Linear acceleration in world coordinates (difference of velocity vectors)
        ax_world = (vx_new - vx_old) / dt
        ay_world = (vy_new - vy_old) / dt
        az_world = 0.0
        accel_world = np.array([ax_world, ay_world, az_world])
        # Angular velocity: yaw_rate about Z in world coordinates
        gyro_world = np.array([0.0, 0.0, yaw_rate_cmd])
        return accel_world, gyro_world
    
    