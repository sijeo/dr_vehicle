"""
plotter.py

This module provides a simple helper for real time visualization of 
vehicle tracking simulations. The Plotter class encapsulates the creation
and update of matplotlib figure with multiple subplots, including 
velocity/acceleration, yaw angle, trajectory and position error. 
The figure is updated incrementally as the simulation runs.

Use of this module is optional: if Matplotlib cannot be imported or 
interactive plotting is not possible,( e.g. in a headless environment), 
the class will gracefully do nothing when update()  is called. 

Classes:
Plotter:
    Maintains a figure with subplots and provides an update() method to 
    append new data points.
"""

from __future__ import annotations

import numpy as np
import warnings
from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class Plotter:
    """Real time plot manager for vehicle tracking simulations
    
    Parameters:
    enable : bool, optional
        If False, the plotter will not attempt to create plots. Default is True
        if matplotlib can be imported.
    """

    enable : bool = field(default=True)
    # Data buffers
    time_log: List[float] = field(default_factory=list)
    speed_true: List[float] = field(default_factory=list)
    speed_est: List[float] = field(default_factory=list)
    accel_cmd: List[float] = field(default_factory=list)
    yaw_true: List[float] = field(default_factory=list)
    yaw_est: List[float] = field(default_factory=list)
    pos_err: List[float] = field(default_factory=list)
    x_true: List[float] = field(default_factory=list)
    y_true: List[float] = field(default_factory=list)
    x_est: List[float] = field(default_factory=list)
    y_est: List[float] = field(default_factory=list)

    def __post_init__(self):
        # Try to import matplotlib for plotting
        self._fig = None
        self._lines = {}
        if self.enable:
            try:
                import matplotlib.pyplot as plt
                plt.ion()  # Enable interactive mode
                self._fig, axes = plt.subplots(2, 2, figsize=(10, 8))
                ax_speed = axes[0, 0]
                ax_yaw = axes[0, 1]
                ax_traj = axes[1, 0]
                ax_err = axes[1, 1]
                # Setup plots
                ax_speed.set_title("Velocity / Acceleration")
                ax_speed.set_xlabel("Time (s)")
                ax_speed.set_ylabel("Speed (m/s) ")
                # Speed and Acceleration lines
                l1, = ax_speed.plot([], [], label="True Speed")
                l2, = ax_speed.plot([], [], label="Estimated Speed", linestyle='--')
                l3, = ax_speed.plot([], [], label="Accel Command", linestyle=':')
                ax_speed.legend()
                ax_yaw.set_title("Yaw Angle")
                ax_yaw.set_xlabel("Time (s)")
                ax_yaw.set_ylabel("Yaw (deg)")
                l4, = ax_yaw.plot([], [], label="True Yaw")
                l5, = ax_yaw.plot([], [], label="Estimated Yaw", linestyle='--')
                ax_yaw.legend()
                ax_traj.set_title("Trajectory")
                ax_traj.set_xlabel("East (m)")
                ax_traj.set_ylabel("North (m)")
                ax_traj.set_aspect('equal', 'box')
                l6, = ax_traj.plot([], [], label="True Position")
                l7, = ax_traj.plot([], [], label="Estimated Position", linestyle='--')
                ax_traj.legend()
                ax_err.set_title("Position Error Norm")
                ax_err.set_xlabel("Time (s)")
                ax_err.set_ylabel("Position Error (m)")
                l8, = ax_err.plot([], [], label="Position Error")
                ax_err.legend()
                plt.tight_layout()
                # Store line handles
                self._axes = {
                    'speed': ax_speed,
                    'yaw': ax_yaw,
                    'traj': ax_traj,
                    'err': ax_err
                }
                self._lines = {
                    'speed_true': l1,
                    'speed_est': l2,
                    'accel_cmd': l3,
                    'yaw_true': l4,
                    'yaw_est': l5,
                    'traj_true': l6,
                    'traj_est': l7,
                    'pos_err': l8
                }
                self._plt = plt
            except Exception as e:
                warnings.warn(f"Plotter disabled: could not initialize matplotlib plots. {e}")
                self.enable = False
    def update(self, t: float,
               true_speed: float, 
               est_speed: float,
               accel_cmd: float,
                true_yaw: float,
                est_yaw: float,
                pos_err: float,
                true_pos: np.ndarray,
                est_pos: np.ndarray ) -> None:
        """ Append new data and update the figures.
        
        Parameters:
        t : float
            Current simulation time in seconds.
        true_speed : float
            True vehicle speed in m/s.
        est_speed : float
            Estimated vehicle speed in m/s.
        accel_cmd : float
            Commanded acceleration in m/s^2.
        true_yaw : float
            True vehicle yaw angle in radians.
        est_yaw : float
            Estimated vehicle yaw angle in radians.
        pos_err : float
            Euclidean norm of position estimation error in meters.
        true_pos : np.ndarray
            True vehicle position as (x, y) in meters in world coordinates.
        est_pos : np.ndarray
            Estimated vehicle position as (x, y) in meters in world coordinates.
        """

        # Append to logs
        self.time_log.append(t)
        self.speed_true.append(true_speed)
        self.speed_est.append(est_speed)
        self.accel_cmd.append(accel_cmd)
        self.yaw_true.append(np.degrees(true_yaw))
        self.yaw_est.append(np.degrees(est_yaw))
        self.pos_err.append(pos_err)
        self.x_true.append(true_pos[0])
        self.y_true.append(true_pos[1])
        self.x_est.append(est_pos[0])
        self.y_est.append(est_pos[1])
        # Update plots if enabled
        if not self.enable or self._fig is None:
            return
        # Update line data
        self._lines['speed_true'].set_data(self.time_log, self.speed_true)
        self._lines['speed_est'].set_data(self.time_log, self.speed_est)
        self._lines['accel_cmd'].set_data(self.time_log, self.accel_cmd)
        self._lines['yaw_true'].set_data(self.time_log, self.yaw_true)
        self._lines['yaw_est'].set_data(self.time_log, self.yaw_est)
        self._lines['traj_true'].set_data(self.x_true, self.y_true)
        self._lines['traj_est'].set_data(self.x_est, self.y_est)
        self._lines['pos_err'].set_data(self.time_log, self.pos_err)
        # Rescale axes
        # Speed and Acceleration share x axis
        ax_speed = self._axes['speed']
        ax_speed.set_xlim(0, max(5, self.time_log[-1]))
        ax_speed.relim()
        ax_speed.autoscale_view()
        # Yaw
        ax_yaw = self._axes['yaw']
        ax_yaw.set_xlim(0, max(5, self.time_log[-1]))
        ax_yaw.relim()
        ax_yaw.autoscale_view()
        # Trajectory
        ax_traj = self._axes['traj']
        ax_traj.relim()
        ax_traj.autoscale_view()
        # Position Error
        ax_err = self._axes['err']
        ax_err.set_xlim(0, max(5, self.time_log[-1]))
        ax_err.relim()
        ax_err.autoscale_view()
        self._plt.pause(0.001)


        
