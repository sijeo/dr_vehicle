# Ground Vehicle Tracking Simulation with EKF

This project simulates sensor fusion for a ground vehicle using an Extended Kalman Filter (EKF) that fuses data from an MPU6050 IMU (accelerometer + gyroscope) and a NEO-6M GNSS (GPS receiver). The simulation emulates motion, applies realistic sensor noise and biases, and performs EKF-based tracking with real-time plots of orientation, estimated position, velocity, and errors.

## 📁 Project Structure

```
vehicle_tracking/
├── __init__.py
├── ekf.py               # Extended Kalman Filter implementation
├── motion_model.py      # Vehicle dynamics and true motion
├── sensor_simulator.py  # IMU and GPS simulation with noise
├── controls.py          # Keyboard/mouse-based control interface
├── plotter.py           # Live Matplotlib visualization
└── main.py              # Simulation entry point
```

## ⚙️ Requirements

- Python 3.7+
- Recommended: virtual environment

Install dependencies:

```bash
pip install -r requirements.txt
```

If `requirements.txt` is unavailable:

```bash
pip install numpy matplotlib pygame
```

## ▶️ Running the Simulation

Run a 60 s simulation at 100 Hz:

```bash
python -m vehicle_tracking.main --dt 0.01 --duration 60
```

Interactive control:

- Arrow keys or mouse for steering/acceleration
- `Esc` or window close to exit

### CLI Options

- `--dt <timestep>` – simulation time step (default 0.01 s)
- `--duration <seconds>` – total run time (negative for indefinite)
- `--no-plot` – disable plotting
- `--no-interactive` – disable user control

## 🧪 Features

- 15-state EKF (position, velocity, orientation, IMU biases)
- Configurable sensor noise and bias profiles
- Real-time plots:
  - Position error (true vs. estimated)
  - Velocity and acceleration
  - Orientation (Euler angles)
- Modular design for extending sensors or filters

## 📝 Notes

- IMU and GNSS data are generated from the simulated vehicle motion.
- Orientation uses quaternions, converted to Euler angles for display.

## 🛠️ Customization

- `main.py` – adjust initial conditions or scenario setup
- `sensor_simulator.py` – modify noise/bias characteristics
- `ekf.py` – replace or tweak state/update models