"""
sensor_simulator.py

This module defines a simple sensor simulation fo rthe MPU6050 IMU and 
NEO6M GNSS used in this example. It converts ground truth motion into
synthetic IMU and GNSS measurements by adding configurable biases, 
white noise and random walks. The accelerometer outputs specific force
in body frame( i.e. translational acceleration minus gravity) and the 
gyroscope reports angular velocity in body coordinates. GNSS outputs 
absolute position at a user specified frequency.

Classes:
SensorSimulator:
    Simulates IMU and GNSS sensors with biases, noise and random walk dynamics.

"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple

from .motion_model import yaw_to_quaternion
from .ekf import quat_rotate

@dataclass
class SensorSimulator:
    """Simulate IMU and GPS measurements from ground truth motion
    
    Parameters:
        accel_bias (np.ndarray): Initial accelerometer bias in body coordinates (3,).
        gyro_bias (np.ndarray): Initial gyroscope bias in body coordinates (3,).
        accel_noise_std (float): Standard deviation of accelerometer white noise (m/s^2).
        gyro_noise_std (float): Standard deviation of gyroscope white noise (rad/s).
        accel_bias_rw (float): Accelerometer bias random walk standard deviation (m/s^2/sqrt(s)).
        gyro_bias_rw (float): Gyroscope bias random walk standard deviation (rad/s/sqrt(s)).
        gps_noise_std (float): Standard deviation of GPS position noise (m) on each axis.
        gps_rate (float): GPS measurement rate in Hz, if Zero, no GPS measurements are generated.
        random_state(Optional[np.random.Generator]): Random number generator for reproducibility. If 
            None, a new default generator is created.
    """
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel_noise_std: float = 0.004 # m/s^2
    gyro_noise_std: float = 0.000087 # rad/s
    accel_bias_rw: float = 1e-4 # m/s^2/sqrt(s)
    gyro_bias_rw: float = 1e-5 # rad/s/sqrt(s)
    gps_noise_std: float = 3.0 # m
    gps_rate: float = 1.0 # Hz
    random_state: Optional[np.random.Generator] = field(default=None)

    def __post_init__(self):
        # Convert ilsts to numpy arrays
        self.accel_bias = np.asarray(self.accel_bias, dtype=float)
        self.gyro_bias = np.asarray(self.gyro_bias, dtype=float)
        # RNG for reproducibility
        if self.random_state is None:
            self.rng = np.random.default_rng()
        else:
            self.rng = self.random_state
        # Compute GPS interval from rate
        self.gps_interval: float = 1.0 / self.gps_rate if self.gps_rate > 0 else float('inf')
        self._gps_timer: float = 0.0

    def _update_biases(self, dt: float) -> None:
        """ Evolve sensor biases using a random walk over time dt.
        """
        # Random walk increments with variance = rw^2 * dt
        self.accel_bias += self.rng.normal(scale=self.accel_bias_rw * np.sqrt(dt), size=3)
        self.gyro_bias += self.rng.normal(scale=self.gyro_bias_rw * np.sqrt(dt), size=3)

    def measure_imu( self, acc_world: np.ndarray, gyro_world: np.ndarray, orientation: np.ndarray, 
                    dt: float) -> Tuple[np.ndarray, np.ndarray]:
        
        """ Generate synthetic IMU measurements.
        
        Args:
            acc_world (np.ndarray): Linear acceleration in world frame (3,).
            gyro_world (np.ndarray): Angular velocity in world frame (3,).
            orientation (np.ndarray): True body->world quaternion for rotating into body frame (4,).
            dt (float): Time step since last measurement in seconds.

        Returns:
            accel_meas (np.ndarray): Simulated accelerometer measurement in body frame (3,).
            gyro_meas (np.ndarray): Simulated gyroscope measurement in body frame (3,).
        """
        # Update biases via Random Walk
        self._update_biases(dt)
        acc_world = np.asarray(acc_world, dtype=float)
        gyro_world = np.asarray(gyro_world, dtype=float)
        orientation = np.asarray(orientation, dtype=float)
        # Compute specific force: rotate (acc_world - gravity) into body frame
        gravity = np.array([0, 0, -9.80665])
        specific_force_world = acc_world - gravity
        # Body specific force via quaternion rotation (world -> body)
        # Use quaternion conjugate to rotate from world to body
        q_conj = np.array([orientation[0], -orientation[1], -orientation[2], -orientation[3]])
        # Use quat_rotate from ekf module (body->world). To go world->body, use q_conj
        specific_force_body = quat_rotate(q_conj, specific_force_world)
        # Add bias and noise
        accel_meas = (specific_force_body + self.accel_bias +
                      self.rng.normal(scale=self.accel_noise_std, size=3))
        # Gyro measurement: convert world angular velocity to body (for planar
        # motion world and body align, but we do full transform for generality)
        # Angular velocity is already expressed in world; to express in body rotate by q_conj
        gyro_body = quat_rotate(q_conj, gyro_world)
        gyro_meas = (gyro_body + self.gyro_bias +
                     self.rng.normal(scale=self.gyro_noise_std, size=3))
        return accel_meas, gyro_meas
    
    def measure_gps(self, pos: np.ndarray, dt: float) -> Optional[np.ndarray]:
        """ Generate a GPS measurement if enough time is passed.
        GPS measurements are produced at fixed rate. The function accumulates dt
        internally and  returns a noisy position when the accumulated time exceeds the 
        GPS interval, Otherwise it returns None.

        Args:
            pos (np.ndarray): True position in world frame (3,).
            dt (float): Time step since last measurement in seconds.

        Returns:
            gps_meas (Optional[np.ndarray]): Simulated GPS position measurement (3,).
                Returns None if no measurement is produced at this time step.
    """
        self._gps_timer += dt
        if self._gps_timer >= self.gps_interval:
            # Time to produce a GPS measurement
            self._gps_timer -= self.gps_interval
            pos = np.asarray(pos, dtype=float)
            gps_meas = pos + self.rng.normal(scale=self.gps_noise_std, size=3)
            return gps_meas
        else:
            return None
        
    
