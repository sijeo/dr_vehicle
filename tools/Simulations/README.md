Ground Vehicle Tracking Simulation with EKF

This project simulates sensor fusion for a ground vehicle using an Extended Kalman Filter (EKF). It fuses data from:

MPU6050 IMU (accelerometer + gyroscope)

NEO-6M GNSS (GPS receiver)

The simulation emulates motion, applies realistic sensor noise and biases, and performs EKF-based tracking. Real-time plots display orientation, estimated position, velocity, and errors.

📁 Project Structure
vehicle_tracking/
│
├── __init__.py
├── ekf.py               # Extended Kalman Filter implementation
├── motion_model.py      # Vehicle dynamics and true motion
├── sensor_simulator.py  # IMU and GPS simulation with noise
├── controls.py          # Keyboard/mouse-based control interface
├── plotter.py           # Live Matplotlib visualization
└── main.py              # Simulation entry point

⚙️ Requirements

Python 3.7 or newer
Recommended installation via virtual environment.

Dependencies

Install with:

pip install -r requirements.txt


If requirements.txt is not provided, install manually:

pip install numpy matplotlib pygame

▶️ Running the Simulation
Basic Run (60 seconds, 100Hz update rate)
python -m vehicle_tracking.main --dt 0.01 --duration 60

Interactive Mode

Use arrow keys or mouse to steer and control acceleration.

Esc or window close to exit early.

CLI Options
--dt <timestep>         # Time step in seconds (default: 0.01)
--duration <seconds>    # Total simulation time
--no-plot               # Disable graphical plotting
--no-interactive        # Disable user control (uses default straight-line motion)

🧪 Features

15-state EKF with position, velocity, orientation, and sensor biases

Configurable sensor noise and biases

Real-time plotting:

Position error (true vs estimated)

Velocity, acceleration

Orientation (Euler angles)

Modular design for easy extension (e.g., new sensors or different filters)

📝 Notes

The IMU and GNSS data generation is based on simulated ground vehicle motion.

Orientation is managed via quaternions and converted to Euler angles for visualization.

🛠️ Customize

You can edit parameters in:

main.py: to adjust initial conditions or scenario setup

sensor_simulator.py: to inject different noise/bias profiles

ekf.py: to swap with different state models or update strategies