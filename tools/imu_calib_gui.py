# imu_calib_gui.py
#
# This connects to PI Calibration sever, guides you
# through 6 orientations, Computes A, b, then C and o

from doctest import master
import socket
import tkinter as tk
from tkinter import messagebox
import numpy as np
import json
import time

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


G = 9.80665  # Gravity constant
DEG2RAD = np.pi / 180.0


def quat_normalize(q):
    norm = np.linalg.norm(q)
    if norm <= 0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / norm

def quat_derivative(q, omega_body):
    """
    Quaternion derivative from angular velocity
    q = [W, x, y, z]
    omega_body = [wx, wy, wz] in rad/s
    dq/dt = 0.5 * q * omega_quat
    where omega_quat = [0, wx, wy, wz]
    """

    w, x, y, z = q
    wx, wy, wz = omega_body
    dqdt = 0.5 * np.array([
        -x * wx - y * wy - z * wz,
         w * wx + y * wz - z * wy,
         w * wy - x * wz + z * wx,
         w * wz + x * wy - y * wx
    ], dtype=float)

    return dqdt

def quat_to_rotation_matrix(q):
    """ Convert quaternion to rotation matrix """
    w, x, y, z = q
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),   1 - 2*(x**2 + y**2)]
    ], dtype=float)
    return R

def gravity_body_from_quat(q):
    """ 
    Given quaternion q (body->world), compute expected gravity direction
    in body-frame (unit-vector). We assume gravity in world frame is [0, 0, 1].
    g_body = R(q)^T * [0, 0, 1]
    """
    R = quat_to_rotation_matrix(q)
    g_body = R.T @ np.array([0.0, 0.0, 1.0], dtype=float)
    n = np.linalg.norm(g_body)
    if n <= 0.0:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return g_body / n

def quat_to_euler(q):
    """
    Convert quaternion to Euler angles (yaw, pitch, roll) in radians.
    
    Convention: ZYX (yaw-pitch-roll)
    yaw = rotation about Z
    pitch = rotation about Y
    roll = rotation about X
    """

    q = quat_normalize(q)
    w, x, y, z = q

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.pi / 2 * np.sign(sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw, pitch, roll


class IMUCalibApp:
    def __init__(self, master):
        self.master = master
        master.title("MPU6050 Accelerometer Calibration ")

        # Connection Parameters
        self.host_var = tk.StringVar(value='10.225.104.7')
        self.port_var = tk.IntVar(value=5005)
        self.samples_var = tk.IntVar(value=2000)
        self.gyro_scale_var = tk.DoubleVar(value=65.5)  # LSB per rad/s for gyro at +/- 250 dps

        self.sock = None
        self.streaming = False
        

        # Storage for 6 orientations means
        self.orientations = ["+X_Up", "-X_Up", "+Y_Up", "-Y_Up", "+Z_Up", "-Z_Up"]
        self.raw_means = {o : None for o in self.orientations}
        self.gyro_bias = None

        self.C = None
        self.o = None

        # For yaw/pitch/roll integration
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.last_stream_time = None

        # History for plotting (time series)
        self.max_points = 500  # max points to keep in history
        self.acc_hist = {
            't': [],
            'ax': [],
            'ay': [],
            'az': []
        }

        self.ang_hist = {
            't': [],
            'yaw': [],
            'pitch': [],
            'roll': []
        }


        # GUI Elements
        row = 0
        tk.Label(master, text="Pi IP:").grid(row=row, column=0, sticky='e')
        tk.Entry(master, textvariable=self.host_var, width=15).grid(row=row, column=1)
        tk.Label(master, text="Port:").grid(row=row, column=2, sticky='e')
        tk.Entry(master, textvariable=self.port_var, width=6).grid(row=row, column=3)
        tk.Label(master, text="Samples:").grid(row=row, column=4, sticky='e')
        tk.Entry(master, textvariable=self.samples_var, width=6).grid(row=row, column=5)
        tk.Button(master, text="Connect", command=self.connect).grid(row=row, column=6, padx=5)
        row += 1

        # Gyro Scale factor field
        tk.Label(master, text="Gyro Scale (LSB/rad/s):").grid(row=row, column=0, sticky='e')
        tk.Entry(master, textvariable=self.gyro_scale_var, width=10).grid(row=row, column=1, sticky='w')
        tk.Label(master, text="(default 65.5 for +/-500 dps)").grid(row=row, column=2, columnspan=5, sticky='w')
        row += 1

        self.status_var = tk.StringVar(value="Not connected")
        tk.Label(master, textvariable=self.status_var, fg="blue").grid(row=row, column=0, columnspan=7, sticky='w')
        row += 1

        # Orientation Buttons
        self.orientation_labels = {}
        for o in self.orientations:
            tk.Button(master, text=f"Capture {o}", command=lambda o=o: self.capture_orientation(o)).grid(row=row, column=0, padx=5, pady=4, sticky='w')
            lbl = tk.Label(master, text=f"{o} Mean: (not captured)", width=60, anchor='w')
            lbl.grid(row=row, column=1, columnspan=6, sticky='w', padx=5)
            self.orientation_labels[o] = lbl 
            row += 1

        tk.Button(master, text="Capture Gyro Bias", command=self.capture_gyro_bias).grid(row=row, column=0, sticky='w', padx=5, pady=5)
        tk.Label(master, text="(keep sensor very still)", anchor='w').grid(row=row, column=1, columnspan=6, sticky='w')
        row += 1

        tk.Button(master, text="Compute Calibration", command=self.compute_calibration).grid(row=row, column=0, sticky='w', padx=5, pady=10)
        row += 1

        # Streaming Buttons
        tk.Button(master, text="Start Calibrated Stream", command=self.start_stream).grid(row=row, column=0, sticky='w', padx=5, pady=5)
        tk.Button(master, text="Stop Stream", command=self.stop_stream).grid(row=row, column=1, sticky='w', padx=5, pady=5)
        row += 1

        # ---------------------------------------------------------------------
        # Matplotlib Figure embedded in Tkinter for live plots
        # ---------------------------------------------------------------------
        main_frame = tk.Frame(master)
        main_frame.grid(row=row, column=0, columnspan=7, padx=5, pady=5, sticky='nsew')
        row += 1

        main_frame.columnconfigure(0, weight=3)
        main_frame.columnconfigure(1, weight=2)

        plot_frame = tk.Frame(main_frame)
        plot_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        self.fig = Figure(figsize=(8, 6), dpi=100)

        self.ax_acc = self.fig.add_subplot(2, 1, 1)
        self.ax_acc.set_title("Calibrated Accelerometer Data (m/s^2)")
        self.ax_acc.set_xlabel("Time (s)")
        self.ax_acc.set_ylabel("Acceleration (m/s^2)")

        self.line_ax, = self.ax_acc.plot([], [], label='Ax', color='r')
        self.line_ay, = self.ax_acc.plot([], [], label='Ay', color='g')
        self.line_az, = self.ax_acc.plot([], [], label='Az', color='b')
        self.ax_acc.legend(loc='upper right')

        self.ax_ang = self.fig.add_subplot(2, 1, 2)
        self.ax_ang.set_title("Estimated Angles from Gyro Integration (degrees)")
        self.ax_ang.set_xlabel("Time (s)")
        self.ax_ang.set_ylabel("Angle (degrees)")
        self.ang_line_yaw, = self.ax_ang.plot([], [], label='Yaw', color='m')
        self.ang_line_pitch, = self.ax_ang.plot([], [], label='Pitch', color='c')
        self.ang_line_roll, = self.ax_ang.plot([], [], label='Roll', color='y')

        self.ax_ang.legend(loc='upper right')

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=1)

        # Output Text Box
        log_frame = tk.Frame(main_frame)
        log_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        self.output_text = tk.Text(log_frame, wrap='word', height=30, width=50)
        self.output_text.pack(fill=tk.BOTH, expand=1)

    def log(self, message):
        self.output_text.insert(tk.END, message + "\n")
        self.output_text.see(tk.END)
        print(message)

    def connect(self):
        host = self.host_var.get()
        port = self.port_var.get()
        try:
            self.sock = socket.create_connection((host, port))
            self.status_var.set(f"Connected to {host}:{port}")
            self.log(f"Connected to {host}:{port}")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect to {host}:{port}\n{e}")
            self.status_var.set("Not connected")
            self.sock = None

    def send_sample_command(self, N):
        """Send SAMPLE command, return raw accel+gyro as numpy arrays"""
        if self.sock is None:
            messagebox.showerror("Error", "Not connected to server")
            return None
        
        cmd = f"SAMPLE {N}\n".encode()
        try:
            self.sock.sendall(cmd)
            data = b""
            while not data.endswith(b"\n"):
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise RuntimeError("Disconnected")
                data += chunk
            line = data.decode().strip()
            if line.startswith("ERR"):
                self.log(f"Error from server: {line}")
                return None
            self.log(f"Received line: {line}")
            vals = list(map(float, line.split()))
            if len(vals) != 6:
                self.log(f"Bad data from server: {line}")
                return None
            
            acc = np.array(vals[0:3], dtype=float)
            gyro = np.array(vals[3:], dtype=float)
            parts = line.split()
            return acc, gyro
        
        except Exception as e:
            messagebox.showerror("Socket Error",str(e))
            if self.sock:
                self.sock.close()
            self.sock = None
            self.status_var.set("Not connected")
            return None
        

        
    def capture_orientation(self, orientation):
        if self.sock is None:
            return 
        

        N = self.samples_var.get()
        messagebox.showinfo("Orientation", f"Place the IMU in the '{orientation}' orientation and click OK\n"
                            f"Keep it perfectly still while {N} samples are collected.")
        result = self.send_sample_command(N)
        if result is None:
            return
        acc_raw, _ignored = result
        self.raw_means[orientation] = acc_raw
        self.orientation_labels[orientation].config(text=f"{orientation} Mean: [{acc_raw[0]:.2f}, {acc_raw[1]:.2f}, {acc_raw[2]:.2f}] raw")
        self.log(f"{orientation} raw mean: {acc_raw}")

    def capture_gyro_bias(self):
        if self.sock is None:   
            return
        
        N = self.samples_var.get()
        messagebox.showinfo("Gyro Bias", f"Keep the IMU perfectly still and click OK\n"
                            f"{N} samples will be collected to compute gyro bias.")
        result = self.send_sample_command(N)
        if result is None:
            return
        _acc, gyro = result
        self.gyro_bias = gyro
        self.log(f"Gyro bias (raw): {gyro}")

    def compute_calibration(self):
        # check all orientations captured
        for o in self.orientations:
            if self.raw_means[o] is None:
                messagebox.showerror("Missing Data", f"Orientation {o} not captured")
                return 
            
        #Build least squares System r = A * a_true + b
        # Unknowns: 12 -> [A11, A12, A13, A21, A22, A23, A31, A32, A33, b1, b2, b3]
        H_rows = []
        y_rows = []

        # Map orientation to true acceleration vector in m/s^2
        true_map = {
            "+X_Up" : np.array([+G, 0.0, 0.0]),
            "-X_Up" : np.array([-G, 0.0, 0.0]),
            "+Y_Up" : np.array([0.0, +G, 0.0]),
            "-Y_Up" : np.array([0.0, -G, 0.0]),
            "+Z_Up" : np.array([0.0, 0.0, +G]),
            "-Z_Up" : np.array([0.0, 0.0, -G])
        }

        for o in self.orientations:
            r = self.raw_means[o]
            at = true_map[o]

            ax_t, ay_t, az_t = at
            rx, ry, rz = r

            # Equation for rx: rx = A11*ax_t + A12*ay_t + A13*az_t + b1
            H_rows.append([ax_t, ay_t, az_t, 0, 0, 0, 0, 0, 0, 1, 0, 0])
            y_rows.append(rx)
            # Equation for ry: ry = A21*ax_t + A22*ay_t + A23*az_t + b2
            H_rows.append([0, 0, 0, ax_t, ay_t, az_t, 0, 0, 0, 0, 1, 0])
            y_rows.append(ry)
            # Equation for rz: rz = A31*ax_t + A32*ay_t + A33*az_t + b3
            H_rows.append([0, 0, 0, 0, 0, 0, ax_t, ay_t, az_t, 0, 0, 1])
            y_rows.append(rz)

        H = np.array(H_rows) # shape (18, 12)
        y = np.array(y_rows) # shape (18,)

        # Solve least squares Hp = y
        p, *_ = np.linalg.lstsq(H, y, rcond=None)
        #unpack p
        A = np.array([[p[0], p[1], p[2]],
                      [p[3], p[4], p[5]],
                      [p[6], p[7], p[8]]])
        b = np.array([p[9], p[10], p[11]])

        # Invert A to get C and offset vector o
        self.C = np.linalg.inv(A)
        self.o = -self.C @ b

        self.log("=== Calibration Results ===")
        self.log(f"A (raw = A * a_true + b):\n{A}")
        self.log(f"b (raw bias): {b}")
        self.log(f"C (a_true = C * (raw + o)):\n{self.C}")
        self.log(f"o (offset in m/s^2): {self.o}")

        if self.gyro_bias is not None:
            self.log(f"Gyro Bias (raw counts): {self.gyro_bias}")
        else:
            self.log("Gyro Bias not captured.")
        
        # Emit C/o in C code format
        self.log("\n---C code snippet (accel calibration)---")
        self.log("/* Multiply this 3x3 matrix X by raw counts, then add o to get m/s^2 */")
        for row in self.C:
            self.log("     {%.9ff, %.9ff, %.9ff}," % tuple(row))
        self.log("};")

        self.log("static const float accel_o[3] = { %.9ff, %.9ff, %.9ff };" % tuple(self.o))
        if self.gyro_bias is not None:
            gb = self.gyro_bias
            self.log("\n/* Gyro bias in raw counts (substract below scaling to rad/s)")
            self.log("static const float gyro_bias[3] = { %.3ff, %.3ff, %.3ff };"%(gb[0], gb[1], gb[2]))

        # Save to JSON
        calib_data = {
            "A": A.tolist(),
            "b": b.tolist(),
            "C": self.C.tolist(),
            "o": self.o.tolist(),
            "gyro_bias": self.gyro_bias.tolist() if self.gyro_bias is not None else None,
            "gyro_scale": self.gyro_scale_var.get()
        }
        with open("imu_calibration.json", "w") as f:
            json.dump(calib_data, f, indent=4)
        self.log("\nCalibration data saved to imu_calibration.json")
        self.log("You can now start streaming calibrated data.")

    def ekf_predict(self, gyro_rads, dt):
        """ EKF predict step using gyro readings (rad/s) and time delta dt (s) """
        q = self.ekf_q
        p = self.ekf_p

        dqdt = quat_derivative(q, gyro_rads)
        q_pred = quat_normalize(q + dqdt * dt)

        # very simple model F ~ I add small process noise
        F = np.eye(4)
        q_process = 1e-6
        Q = q_process * np.eye(4)

        P_pred = F @ p @ F.T + Q

        self.ekf_q = q_pred
        self.ekf_p = P_pred

    def ekf_update(self, acc_cal):
        """
        EKF update with accelerometer as gravity direction measurement

        Measurement: z = acc_cal / ||acc_cal|| (unit vector)
        Model: h(q) = gravity_body_from_quat(q)
        """
        acc_norm = np.linalg.norm(acc_cal)
        if acc_norm < 0.5 * G or acc_norm > 1.5 * G:
            # invalid measurement
            return  
        
        z = acc_cal / acc_norm
        q = self.ekf_q
        p = self.ekf_p

        # Predicted measurement
        h_q = gravity_body_from_quat(q)

        #NUmerical Jacobian H (3x4) wrt quaternion states
        eps = 1e-6
        H = np.zeros((3, 4), dtype=float)
        for i in range(4):
            dq = np.zeros(4, dtype=float)
            dq[i] = eps
            q_plus = quat_normalize(q + dq)
            q_minus = quat_normalize(q - dq)
            h_plus = gravity_body_from_quat(q_plus)
            h_minus = gravity_body_from_quat(q_minus)
            H[:, i] = (h_plus - h_minus) / (2 * eps)
        # Measurement noise
        R = (0.02 ** 2) * np.eye(3)  # assume 0.02 noise in each axis

        y = z - h_q  # innovation
        S = H @ p @ H.T + R  # innovation covariance
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return  # cannot invert, skip update
        
        K = p @ H.T @ S_inv  # Kalman gain
        q_upd = quat_normalize(q + K @ y)
        P_upd = (np.eye(4) - K @ H) @ p

        self.ekf_q = q_upd
        self.ekf_p = P_upd


       

    def start_stream(self):
        """ Start calibrated streaming using SAMPLE 1 polling."""
        if self.C is None or self.o is None:
            messagebox.showerror("Error", "Calibration not computed yet.")
            return
        if self.sock is None:
            messagebox.showerror("Error", "Not connected to server")
            return
        
        self.ekf_q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # initial quaternion
        self.ekf_p = 1e-3 * np.eye(4)  # initial covariance
        self.last_stream_time = None
        
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.last_stream_time = None

        self.acc_hist = {
            't': [],
            'ax': [],
            'ay': [],
            'az': []
        }

        self.ang_hist = {
            't': [],
            'yaw': [],
            'pitch': [],
            'roll': []
        }

        self.streaming = True
        self.log("Starting calibrated data stream...")
        self.stream_loop()  
        
    def stop_stream(self):
        """ Stop streaming """
        if self.sock is None:
            return
        self.streaming = False
        self.log("Streaming stopped.")

    def stream_loop(self):
        """ Streaming loop """
        if not self.streaming:
            return
        
        result = self.send_sample_command(1)
        if result is None:
            self.streaming = False
            self.log("Streaming stopped due to error.")
            return
        
        now = time.time()
        if self.last_stream_time is None:
            dt = 0.02  # assume 50 Hz first time
        else:
            dt = now - self.last_stream_time
            #clamp dt sanity
            if dt <= 0.001:
                dt = 0.001
            elif dt > 0.1:
                dt = 0.1
        self.last_stream_time = now
        
        acc_raw, gyro_raw = result

        #accel calibration
        acc_cal = self.C @ acc_raw + self.o

        #gyro calibration
        if self.gyro_bias is not None:
            gyro_corr = gyro_raw - self.gyro_bias
        else:
            gyro_corr = gyro_raw
            
        scale = self.gyro_scale_var.get()
        gyro_cal = gyro_corr / scale  # rad/s
        gyro_rads = gyro_cal * DEG2RAD  # convert to rad/s

        # 2. Detect if IMU is stationary
        acc_norm = np.linalg.norm(acc_cal)
        near_stationary = abs(acc_norm - G) < 0.1 * G  # within 10% of G

        # 3. Small deadband on yaw rotation.
        yaw_deadband = 0.03  # rad/s
        if abs(gyro_rads[2]) < yaw_deadband and near_stationary:
            gyro_rads[2] = 0.0

        #4. Slowly update gyro bias when stationary
        if near_stationary:
            self.gyro_bias[2] = 0.999 * self.gyro_bias[2] + 0.001 * gyro_raw[2]

        #5. Freeze yaw integration if still
        if near_stationary and gyro_rads[2] == 0.0:
            yaw_rate = 0.0
        else:
            yaw_rate = gyro_rads[2]

        # Replace yaw rate in gyro_rads with the possibly modified yaw_rate
        gyro_rads[2] = yaw_rate

        # EKF predict/update
        self.ekf_predict(gyro_rads, dt)
        self.ekf_update(acc_cal)

        # Extract Euler angles
        yaw_rad, pitch_rad, roll_rad = quat_to_euler(self.ekf_q)
        yaw_deg = np.degrees(yaw_rad)
        pitch_deg = np.degrees(pitch_rad)
        roll_deg = np.degrees(roll_rad)



        self.log(f"Calibrated Accel (m/s^2): [{acc_cal[0]:7.3f}, {acc_cal[1]:7.3f}, {acc_cal[2]:7.3f}] | "
                 f"Calibrated Gyro (rad/s): [{gyro_rads[0]:7.3f}, {gyro_rads[1]:7.3f}, {gyro_rads[2]:7.3f} | "
                 f"Yaw: {yaw_deg:7.2f}°, Pitch: {pitch_deg:7.2f}°, Roll: {roll_deg:7.2f}°")
        # Update history
        self.append_history(now, acc_cal, yaw_deg, pitch_deg, roll_deg)
        self.update_plots()
        self.master.after(20, self.stream_loop)  # schedule next call

    def append_history(self, t, acc_cal, yaw, pitch, roll):
        self.acc_hist['t'].append(t)
        self.acc_hist['ax'].append(acc_cal[0])
        self.acc_hist['ay'].append(acc_cal[1])
        self.acc_hist['az'].append(acc_cal[2])

        self.ang_hist['t'].append(t)
        self.ang_hist['yaw'].append(yaw)
        self.ang_hist['pitch'].append(pitch)
        self.ang_hist['roll'].append(roll)

        # Trim history if too long
        for key in self.acc_hist:
            if len(self.acc_hist[key]) > self.max_points:
                self.acc_hist[key] = self.acc_hist[key][-self.max_points:]
        for key in self.ang_hist:
            if len(self.ang_hist[key]) > self.max_points:
                self.ang_hist[key] = self.ang_hist[key][-self.max_points:]
    
    def update_plots(self):
        # Update matplotlib with latest history
        if len(self.acc_hist['t']) == 0:
            return
        
        # Use relative time (seconds since first sample )
        t0 = self.acc_hist['t'][0]
        t_rel_acc = [ti - t0 for ti in self.acc_hist['t']]
        t_rel_ang = [ti - t0 for ti in self.ang_hist['t']]

        # update accel lines
        self.line_ax.set_data(t_rel_acc, self.acc_hist['ax'])
        self.line_ay.set_data(t_rel_acc, self.acc_hist['ay'])
        self.line_az.set_data(t_rel_acc, self.acc_hist['az'])

        # Auto x-limits: show the last few senconds or full range
        if len(t_rel_acc) > 1:
            xmin = max(0, t_rel_acc[-1] - 5)
            xmax = t_rel_acc[-1] if t_rel_acc[-1] > 1.0 else 1.0
        else:
            xmin = 0
            xmax = 1.0

        self.ax_acc.set_xlim(xmin, xmax)

        # Auto y-limits
        all_acc = self.acc_hist['ax'] + self.acc_hist['ay'] + self.acc_hist['az']
        ymin = min(all_acc) 
        ymax = max(all_acc)
        if ymin == ymax:
            ymin -= 1.0
            ymax += 1.0
        self.ax_acc.set_ylim(ymin, ymax)

        # update angle lines
        self.ang_line_yaw.set_data(t_rel_ang, self.ang_hist['yaw'])
        self.ang_line_pitch.set_data(t_rel_ang, self.ang_hist['pitch'])
        self.ang_line_roll.set_data(t_rel_ang, self.ang_hist['roll'])

        #x limits reused
        self.ax_ang.set_xlim(xmin, xmax)
        # Auto y-limits
        all_ang = self.ang_hist['yaw'] + self.ang_hist['pitch'] + self.ang_hist['roll']
        ymin = min(all_ang)
        ymax = max(all_ang)
        if ymin == ymax:
            ymin -= 5.0
            ymax += 5.0
        else:
            margin = (ymax - ymin) * 0.1
            ymin -= margin
            ymax += margin
        self.ax_ang.set_ylim(ymin, ymax)
        self.canvas.draw_idle()




#----------------------------------------------------------------------------
def main():
    root = tk.Tk()
    app = IMUCalibApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()

    