"""
main.py

Executable entry point for the ground vehicle simulation with an EKF
The script ties together the motion model, sensor simulator, controller,
EKF and plotter into a complete application. The simulation runs either 
interactively (using Pygame for control and Matplotlib for Visualization)
or headless using pre-determined control inputs.

Usage:
    Run the script as a module to start the simulation::

    python -m vehicle_tracking.main

Optional arguments can be provided to set the simulation duration, time step,
or to disable interactively control or plotting. Use the --help flag for details.

"""

from __future__ import annotations
import argparse
import time
import numpy as np

from ekf import EKFConfig, EKF15State
from motion_model import VehicleState, yaw_to_quaternion
from sensor_simulator import SensorSimulator
from controls import KeyboardController
from plotter import Plotter

def quaternion_to_yaw(q: np.ndarray) -> float:
    """Convert a quaternion into a yaw angle (radians).
    
    Args:
        q (np.ndarray): Quaternion as a numpy array (w, x, y, z).
        
    Returns:
        float: Yaw angle in radians.
    """
    w, x, y, z = q
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw

def run_simulation( dt: float,
                    duration: float,
                    interactive: bool = True,
                    plotting: bool = True) -> None:
    """Execute the vehicle simulation for a given duration.

    Args:
        dt (float): Simulation time step in seconds.
        duration (float): Total simulation duration in seconds, Use 
        negative value to run indefinitely until user quits.
        interactive (bool): If True, enable keyboard control via Pygame
        for keyboard control; otherwise a zero control input is used.
        plotting (bool): If True, enable real-time plotting via Matplotlib
        if False the plotting is disabled.
    """
    # Initialize Subsystems
    ekf_cfg = EKFConfig()
    ekf = EKF15State(ekf_cfg)
    vehicle = VehicleState()
    # Sensor Simulation: Start with some biases as an example.
    sensor = SensorSimulator(
        accel_bias=np.array([0.1, -0.05, 0.0]),
        gyro_bias=np.array([0.005, -0.005, 0.002]),
        accel_noise_std=ekf_cfg.sigma_a * np.sqrt(dt),
        gyro_noise_std=ekf_cfg.sigma_g * np.sqrt(dt),
        accel_bias_rw=ekf_cfg.sigma_ba,
        gyro_bias_rw=ekf_cfg.sigma_bg,
        gps_noise_std=ekf_cfg.sigma_gps,
        gps_rate=1.0 # 1 Hz GPS
    )
    plotter = Plotter(enable=plotting)
    controller = KeyboardController() 
    # Simulation Clock
    sim_time = 0.0
    start_time_wall = time.time()
    while True:
        # Determine Control Inputs
        if interactive and controller.running:
            acc_cmd, yaw_rate_cmd = controller.poll()
        else:
            acc_cmd, yaw_rate_cmd = 0.0, 0.0
            if not interactive:
                # Could provide scipted control here (e.g. constant speed)
                pass
        # Update True Vehicle State
        accel_world, gyro_world = vehicle.update(acc_cmd, yaw_rate_cmd, dt)
        # Obtain True Orientation Quaternion
        true_q = yaw_to_quaternion(vehicle.yaw)
        # Generate Synthetic Sensor readings
        accel_meas, gyro_meas = sensor.measure_imu(accel_world, gyro_world, true_q, dt)
        # EKF Prediction Step
        ekf.predict(gyro_meas, accel_meas, dt)
        # EKF Update Step (GPS)
        gps_meas = sensor.measure_gps(np.array([vehicle.x, vehicle.y, vehicle.z]), dt)
        if gps_meas is not None:
            ekf.update_gps(gps_meas)
        # Compute estimation metrics
        pos_error = np.linalg.norm(ekf.p - np.array([vehicle.x, vehicle.y, vehicle.z]))
        est_speed = np.linalg.norm(ekf.v[0:2])
        # Extract yaw from estimated quaternion
        est_yaw = quaternion_to_yaw(ekf.q)
        # Update Plots
        if plotting:
            plotter.update(
                t=sim_time,
                true_speed=vehicle.v,
                est_speed=est_speed,
                accel_cmd=acc_cmd,
                true_yaw=vehicle.yaw,
                est_yaw=est_yaw,
                pos_err=pos_error,
                true_pos=np.array([vehicle.x, vehicle.y, vehicle.z]),
                est_pos=ekf.p.copy(),
            )
        # Check termination conditions
        sim_time += dt
        if duration > 0 and sim_time >= duration:
            break
        if interactive and not controller.running:
            break
        # Avoid busy waiting by sleeping a small fraction of dt
        # (optional, ensures wall clock roughly matches sim time if interactive)
        elapsed_wall = time.time() - start_time_wall
        if interactive and elapsed_wall < sim_time:
            time.sleep(min(dt, sim_time - elapsed_wall))

def main() -> None:
    """ Entry point when running this module as a script."""
    parser = argparse.ArgumentParser(description="Ground Vehicle Simulation with EKF")
    parser.add_argument('--dt', type=float, default=0.01, help='Simulation time step in seconds (default: 0.01s)')
    parser.add_argument('--duration', type=float, default=60.0, help='Total simulation duration in seconds (default: 60s). Use negative for indefinite.')
    parser.add_argument('--no-interactive', action='store_true', help='Disable interactive keyboard control.')
    parser.add_argument('--no-plotting', action='store_true', help='Disable real-time plotting.')
    args = parser.parse_args()
    interactive = not args.no_interactive
    plotting = not args.no_plotting
    run_simulation(dt=args.dt, duration=args.duration, interactive=interactive, plotting=plotting)

if __name__ == "__main__":
    main()


