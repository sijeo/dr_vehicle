"""
ekf.py

This module implements an 15 State Extended Kalman Filter (EKF) for 
inertial navigation system with GNSS position updates. The filter follows
the error state formulation in the accompanying documentation:
the nominal state contains position, velocity, orientation (as quaternion),
and IMU biases, while the error state contains small perturbations to each 
quantity. The EKF propogates the state and its covariance using inertial 
measurements and corrects the drift when GNSS position measurement are available.


Functions:
quat_from_rotvec(rv)
    Convert a rotation vector to a quaternion.

quat_mult(q, r)
    Multiply two quaternions using the Hamilton product.

quat_rotate(q, v)
    Rotate a 3D vector v from body to world frame using quaternion q.

Classes
EKF15State
    Encapsulates the normal state and covariance and provides predict()
    and update_gps() methods for EKF.
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass, field

def quat_from_rotvec(rv: np.ndarray) -> np.ndarray:
    """Convert a rotation vector to a quaternion.

    Args:
        rv: 3D rotation vector (axis * angle) in radians. The norm of rv
        represents the rotation angle.

    Returns:
        A quaternion [qw, qx, qy, qz] representing the same rotation in the form [w, x, y, z].
    """
    rv = np.asarray(rv, dtype=np.float64)
    angle = np.linalg.norm(rv)
    if angle < 1e-8:
        return np.array([1.0, 0.5*rv[0], 0.5*rv[1], 0.5*rv[2]])
    axis = rv / angle
    half_angle = 0.5 * angle
    qw = np.cos(half_angle)
    qx = axis[0] * np.sin(half_angle)
    qy = axis[1] * np.sin(half_angle)
    qz = axis[2] * np.sin(half_angle)
    return np.array([qw, qx, qy, qz])


def quat_mult(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Compute the Hamilton product of two quaternions q and r.
    Args:
        q: First quaternion [qw, qx, qy, qz].
        r: Second quaternion [rw, rx, ry, rz].
    Returns:
        The product quaternion [pw, px, py, pz] = q * r.
        """
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = r
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a 3D vector v from body to world cordinates using quaternion q.
    
    The rotation uses the quaternion product q * [0, v] * conj(q).
    
    Args:
        q: Quaternion [qw, qx, qy, qz] representing rotation from body to world frame
        v: 3D vector in body frame to be rotated.
    Returns:
        v_rot: The rotated 3D vector in world frame.
        """
    # Convert vector to numpy arrays
    q = np.asarray(q, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    # Quaternion representation of vector
    v_q = np.array([0.0, v[0], v[1], v[2]])
    # Conjugate of quaternion q
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    # Rotate vector: q * v_q * conj(q)
    v_rot_q = quat_mult(quat_mult(q, v_q), q_conj)
    return v_rot_q[1:]  # Return only the vector part

@dataclass
class EKFConfig:
    """ Configuration parameters for the EKF
    
    Attributes:
    p0_pos: float
        Initial position standard deviation (m).
    p0_vel: float
        Initial velocity standard deviation (m/s).
    p0_ang: float
        Initial attitude standard deviation (rad).
    p0_bg: float
        Initial gyroscope bias standard deviation (rad/s).
    p0_ba: float
        Initial accelerometer bias standard deviation (m/s^2).
    sigma_g: float
        Gyro noise density (rad/s/sqrt(Hz)).
    sigma_a: float
        Accelerometer noise density (m/s^2/sqrt(Hz)).
    sigma_bg: float
        Gyro bias random walk (rad/s^2/sqrt(Hz)).
    sigma_ba: float
        Accelerometer bias random walk (m/s^3/sqrt(Hz)).
    sigma_gps: float
        GNSS position measurement noise standard deviation (m).
    gravity: tuple[float, float, float]
        Gravity vector in world frame (m/s^2). By default ENU: [0, 0, -9.81]
    """
    p0_pos: float = 5.0
    p0_vel: float = 1.0
    p0_ang: float = np.deg2rad(3.0)
    p0_bg: float = 0.01
    p0_ba: float = 0.1
    sigma_g: float = 8.7e-5
    sigma_a: float = 3.9e-3
    sigma_bg: float = 1e-5
    sigma_ba: float = 1e-4
    sigma_gps: float = 3.0
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)

@dataclass

class EKF15State:
    """ Extended Kalman Filter for 15 state Inertial Navigation system . 
    The filter maintains the nominal state [position, velocity, orientation (quaternion), biases]
    and the 15 x 15 error convariance matrix. It provides predict() and update_gps() methods
    for state propogation and measurement updates.
    
    Parameters:
    cfg: EKFConfig
        Configurations containing noise parameters and initial uncertainties.

    """
    cfg: EKFConfig = field(default_factory=EKFConfig)
    # Nominal state
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Position (m)
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Velocity (m/s)
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # Orientation (quaternion)
    bg: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Gyro biases (rad/s)
    ba: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Accel biases (m/s^2)

    # Error state covariance
    P: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        # Initialize Covariance matrix with diagonal enteries from cfg
        P0 = np.zeros((15, 15))
        diag_vals = ([self.cfg.p0_pos**2]*3 + [self.cfg.p0_vel**2]*3 + [self.cfg.p0_ang**2]*3 +
                        [self.cfg.p0_bg**2]*3 + [self.cfg.p0_ba**2]*3)
        np.fill_diagonal(P0, diag_vals)
        self.P = P0

    def predict(self, gyro_meas: np.ndarray, accel_meas: np.ndarray, dt: float) -> None:
        """ Propogate the nominal state and covariance using IMU measurements. 

        Args:
            gyro_meas: Measured angular velocity from gyroscope (rad/s), shape(3,).
            accel_meas: Measured linear acceleration from accelerometer (m/s^2), shape(3,).
            dt: Time step for propogation (s).
        """
        gyro_meas = np.asarray(gyro_meas, dtype=np.float64)
        accel_meas = np.asarray(accel_meas, dtype=np.float64)
        # Remove current biase estimates
        omega_b = gyro_meas - self.bg
        f_b = accel_meas - self.ba
        # Update orientation quaternion
        dq = quat_from_rotvec(omega_b * dt)
        self.q = quat_mult(self.q, dq)
        # Normalize to avoid drift
        self.q /= np.linalg.norm(self.q)
        # Rotate specific forrc to world frame and add gravity
        a_world = quat_rotate(self.q, f_b) + np.array(self.cfg.gravity)
        # Update velocity and position using the Simple Euler integration
        self.v += a_world * dt
        self.p += self.v * dt + 0.5 * a_world * dt**2
        # Linearize dynamics for covariance propogation
        F = np.zeros((15, 15))
        F[0:3, 3:6] = np.eye(3)
        # Extract rotation matrix from quaternion
        w, x, y, z = self.q
        Rwb = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])
        # skew matrices for specific force and angular rate
        fx, fy, fz = f_b
        acc_skew = np.array([
            [0, -fz, fy],
            [fz, 0, -fx],
            [-fy, fx, 0]
        ])
        wx, wy, wz = omega_b
        omega_skew = np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ])
        # Sublocks of F
        F[3:6, 6:9] = -Rwb.dot(acc_skew) #dv/d(theta) = -Rwb*[f]_x
        F[3:6, 12:15] = -Rwb        #dv/d(ba) = -Rwb    
        F[6:9, 6:9] = -omega_skew   #d(theta)/d(theta) = -[omega]_x
        F[6:9, 9:12] = -np.eye(3)   #d(theta)/d(bg) = -I
        # Build G (noise Gain) matrix (15x12)
        G = np.zeros((15, 12))
        # Gyro noise -> orientation error
        G[6:9, 0:3] = -np.eye(3)
        # Accel noise -> velocity error in world frame
        G[3:6, 3:6] = Rwb
        # Gyro bias noise -> gyro bias random walk
        G[9:12, 6:9] = np.eye(3)
        # Accel bias noise -> accel bias random walk
        G[12:15, 9:12] = np.eye(3)
        # Continuous noise covariance
        sg = self.cfg.sigma_g
        sa = self.cfg.sigma_a
        sbg = self.cfg.sigma_bg
        sba = self.cfg.sigma_ba 
        Qc = np.diag([
            sg**2, sg**2, sg**2,
            sa**2, sa**2, sa**2,
            sbg**2, sbg**2, sbg**2,
            sba**2, sba**2, sba**2
        ])
        # Compute time derivative of Covariance
        Pdot = F @ self.P + self.P @ F.T + G @ Qc @ G.T
        # Discretize using Euler integration
        self.P += Pdot * dt

    def update_gps(self, gps_pos: np.ndarray) -> None:
        """ Correct the state using a GNSS position measurement.
        
        Args:
            gps_pos: Measured GNSS position in world frame (m), shape(3,).
        """
        gps_pos = np.asarray(gps_pos, dtype=np.float64)
        # Measurement Model extracts position from nominal state.
        H = np.zeros((3, 15))
        H[:, 0:3] = np.eye(3)
        R = np.eye(3) * (self.cfg.sigma_gps ** 2 )
        # Innovation (measurement residual)
        y = gps_pos - self.p
        # Innovation covariance
        S = H @ self.P @ H.T + R
        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y  # Error state correction
        # Update nominal state with error state
        dp = dx[0:3]
        dv = dx[3:6]
        dtheta = dx[6:9]
        dbg = dx[9:12]
        dba = dx[12:15]
        self.p += dp
        self.v += dv
        dq = quat_from_rotvec(dtheta)
        self.q = quat_mult(self.q, dq)
        self.q /= np.linalg.norm(self.q)  # Normalize quaternion
        self.bg += dbg
        self.ba += dba
        # Update covariance using Joseph form for numerical stability
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

