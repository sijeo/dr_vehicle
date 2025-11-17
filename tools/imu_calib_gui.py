# imu_calib_gui.py
#
# This connects to PI Calibration sever, guides you
# through 6 orientations, Computes A, b, then C and o

import socket
import tkinter as tk
from tkinter import messagebox
import numpy as np
import json

G = 9.80665  # Gravity constant
DEG2RAD = np.pi / 180.0


class IMUCalibApp:
    def __init__(self, master):
        self.master = master
        master.title("MPU6050 Accelerometer Calibration ")

        # Connection Parameters
        self.host_var = tk.StringVar(value='192.168.1.100')
        self.port_var = tk.IntVar(value=5005)
        self.samples_var = tk.IntVar(value=2000)
        self.gyro_scale_var = tk.DoubleVar(value=131.0)  # LSB per rad/s for gyro at +/- 250 dps

        self.sock = None
        self.streaming = False
        

        # Storage for 6 orientations means
        self.orientations = ["+X Up", "-X Up", "+Y Up", "-Y Up", "+Z Up", "-Z Up"]
        self.raw_means = {o : None for o in self.orientations}
        self.gyro_bias = None

        self.C = None
        self.o = None


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
        tk.Label(master, text="(default 131.0 for +/-250 dps)").grid(row=row, column=2, columnspan=5, sticky='w')
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


        self.output_text = tk.Text(master, height=20, width=100)
        self.output_text.grid(row=row, column=0, columnspan=7, padx=5, pady=5)

    def log(self, message):
        self.output_text.insert(tk.END, message + "\n")
        self.output_text.see(tk.END)
        print(message)

    def connect(self):
        host = self.host_var.get()
        port = self.port_var.get()
        try:
            self.sock = socket.create_connection((host, port), timeout=5.0)
            self.sock.settimeout(5.0)
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
        C = np.linalg.inv(A)
        o = -C @ b

        self.log("=== Calibration Results ===")
        self.log(f"A (raw = A * a_true + b):\n{A}")
        self.log(f"b (raw bias): {b}")
        self.log(f"C (a_true = C * (raw + o)):\n{C}")
        self.log(f"o (offset in m/s^2): {o}")

        if self.gyro_bias is not None:
            self.log(f"Gyro Bias (raw counts): {self.gyro_bias}")
        else:
            self.log("Gyro Bias not captured.")
        
        # Emit C/o in C code format
        self.log("\n---C code snippet (accel calibration)---")
        self.log("/* Multiply this 3x3 matrix X by raw counts, then add o to get m/s^2 */")
        for row in C:
            self.log("     {%.9ff, %.9ff, %.9ff}," % tuple(row))
        self.log("};")

        self.log("static const float accel_o[3] = { %.9ff, %.9ff, %.9ff };" % tuple(o))

        if self.gyro_bias is not None:
            gb = self.gyro_bias
            self.log("\n/* Gyro bias in raw counts (substract below scaling to rad/s)")
            self.log("static const float gyro_bias[3] = { %.3ff, %.3ff, %.3ff };"%(gb[0], gb[1], gb[2]))

        # Save to JSON
        calib_data = {
            "A": A.tolist(),
            "b": b.tolist(),
            "C": C.tolist(),
            "o": o.tolist(),
            "gyro_bias": self.gyro_bias.tolist() if self.gyro_bias is not None else None,
            "gyro_scale": self.gyro_scale_var.get()
        }
        with open("imu_calibration.json", "w") as f:
            json.dump(calib_data, f, indent=4)
        self.log("\nCalibration data saved to imu_calibration.json")

    def start_stream(self):
        """ Start calibrated streaming using SAMPLE 1 polling."""
        if self.C is None or self.o is None:
            messagebox.showerror("Error", "Calibration not computed yet.")
            return
        if self.sock is None:
            messagebox.showerror("Error", "Not connected to server")
            return

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
            return
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

        self.log(f"Calibrated Accel (m/s^2): [{acc_cal[0]:7.3f}, {acc_cal[1]:7.3f}, {acc_cal[2]:7.3f}] | "
                 f"Calibrated Gyro (rad/s): [{gyro_rads[0]:7.3f}, {gyro_rads[1]:7.3f}, {gyro_rads[2]:7.3f}]")
        self.master.after(20, self.stream_loop)  # schedule next call

def main():
    root = tk.Tk()
    app = IMUCalibApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()

    