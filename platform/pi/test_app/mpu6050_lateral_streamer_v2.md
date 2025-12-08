📘 Appendix — Quaternion EKF (Markdown-Compatible)

This appendix documents the lightweight 4-state quaternion EKF used in
mpu6050_lateral_streamer_v2.c, rewritten in plain Markdown so it renders
correctly on GitHub (no MathJax or LaTeX needed).

🧩 1. EKF State Definition

We estimate only the orientation quaternion:

State x = [ q0, q1, q2, q3 ]ᵀ
Constraint: ||q|| = 1


Gyroscope input (rad/s):

ω = [ wx, wy, wz ]ᵀ


Covariance matrix:

P ∈ ℝ^(4×4)

⚙️ 2. Process Model (Prediction Step)

Quaternion differential equation:

dq/dt = 0.5 * Ω(ω) * q


Quaternion multiplication matrix:

Ω(ω) =
[  0   -wx  -wy  -wz ]
[  wx   0    wz  -wy ]
[  wy  -wz   0    wx ]
[  wz   wy  -wx   0  ]


Discrete update:

q_pred = normalize( q + dq/dt * dt )


State transition Jacobian (approximate):

F ≈ I₄


Covariance prediction:

P_pred = P + Q
Q = q_process * I₄
q_process ≈ 1e−6

📡 3. Measurement Model (Accelerometer → Gravity Direction)

Accelerometer after LS calibration:

acc = [ ax, ay, az ]ᵀ  (m/s²)


Converted to a unit vector:

z = acc / ||acc||


Predicted measurement from quaternion:

h(q) = R(q)ᵀ * [0, 0, 1]   (gravity direction in body frame)


Innovation:

y = z − h(q)


Measurement noise:

R = (0.02)² * I₃

🧮 4. Numerical Jacobian (3×4)

Instead of analytic derivatives:

H(:,i) ≈ ( h(q + εeᵢ) − h(q − εeᵢ) ) / (2ε)
ε = 1e−6


This matches the Python GUI calibration tool.

🔁 5. EKF Update Equations

Innovation covariance:

S = H * P * Hᵀ + R


Kalman gain:

K = P * Hᵀ * inv(S)


State update:

q_new = normalize( q + K*y )


Covariance update:

P_new = ( I − K*H ) * P


Same implementation in:

ekf4_predict()

ekf4_update()

🚶‍♂️ 6. Linear Acceleration, ZUPT & Velocity Decay
6.1 Linear acceleration in world frame
a_world = R(q) * acc_body
lin_raw = a_world + [0,0,G]

6.2 Zero Velocity Update (ZUPT)

We consider IMU still if:

|lin_raw| ≈ G
|gyro| < threshold


After N frames of stillness:

velocity = 0

6.3 Velocity decay (fighting drift)
if |lin| < accel_threshold:
    vel *= 0.98
    if |vel| < epsilon → vel = 0

🎛 7. Tunable Parameters (Reference Table)
Parameter	Default	Meaning	Effect
ACC_ZUPT_THRESH	0.25 m/s²	How close accel must be to 1g for ZUPT	Lower = stricter
GYRO_ZUPT_THRESH	5°/s	Allowable rotation for stillness	Lower = stricter
ZUPT_COUNT_REQUIRED	5	Frames before vel reset	Higher = less sensitive
LIN_ACC_ALPHA	0.90	LPF smoothing factor	Higher = smoother, slower
VEL_DECAY_NEAR_ZERO	0.98	Velocity damping	Lower = faster decay
VEL_EPSILON	1e-3	Min velocity threshold	Prevents tiny drift
POS_CLAMP_MAX	5 m	Max allowed position magnitude	Safety clamp
ACC_STILL_TOL	0.1g	How close accel magnitude must be to g	Governs yaw drift logic
YAW_DEADBAND_RAD	0.03 rad/s	GyroZ threshold to suppress yaw drift	Useful when stationary
q_process	1e-6	EKF process noise	Higher = trust gyro less
R	(0.02)²	Measurement noise	Higher = trust accel less
🔍 8. Tuning Suggestions
Indoor low-drift operation
ACC_ZUPT_THRESH = 0.15
ZUPT_COUNT_REQUIRED = 8–12
VEL_DECAY_NEAR_ZERO = 0.96

Handheld motion
ACC_ZUPT_THRESH = 0.25–0.30
VEL_DECAY_NEAR_ZERO = 0.98–0.99
ACC_STILL_TOL = 0.15g

Vehicle-mounted IMU
GYRO_ZUPT_THRESH = higher value
Disable yaw deadband

📚 9. EKF Data Flow Diagram (ASCII)
                 +-----------------------------+
 raw_acc ------> |   LS Calibration            |
 raw_gyro -----> |   ACCEL_C + bias removal    |
                 +--------------+--------------+
                                |
                                v
                        +---------------+
                        | EKF Predict   | <--- gyro (rad/s)
                        +---------------+
                                |
                                v
                        +---------------+
                        | EKF Update    | <--- accel (gravity dir)
                        +---------------+
                                |
                                v
                +----------------------------------+
                | Rotate accel → world frame        |
                | lin = a_world + gravity           |
                +----------------------------------+
                                |
                                v
             +------------------------------------------------+
             | ZUPT + Velocity Decay + Velocity/Position Int. |
             +------------------------------------------------+
                                |
                                v
                   JSON Stream → imu_lateral_viewer.py

🧾 10. Summary

This Markdown appendix documents the full math behind the orientation EKF used in
mpu6050_lateral_streamer_v2.c and provides a unified reference for:

Calibration → EKF → ZUPT pipeline

Tuning parameters

Update equations

Recommended settings