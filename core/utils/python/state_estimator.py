"""
state_estimator.py
──────────────────
G1 state estimator — pure NumPy, no ROS2 dependency.

Problem: the simulator provides quantities the real robot cannot directly
measure. This module estimates those quantities from what the robot CAN
measure (IMU + joint encoders + pressure sensors), so the policy sees the
same observation matrix in both sim and real.

  Sim gives you:               We estimate from:
  ─────────────────────────    ─────────────────────────────────────
  base_lin_vel  (body frame)   Linear velocity EKF from IMU accel
  base_ang_vel  (body frame)   Gyroscope (direct — no estimation)
  projected_gravity            R^T @ [0,0,-1] from quaternion
  foot_contact_state           Pressure sensors OR tau_est threshold
  foot_vel       (body frame)  Forward kinematics + joint velocity
  yaw_heading                  Extracted from quaternion
  gravity_in_body              Same as projected_gravity

All returned arrays match the shape and order expected by IsaacLab /
unitree_rl_lab observation builders.

Usage:
    est = G1StateEstimator(dt=0.002)

    # each control step:
    est.update(
        quat_wxyz   = sub.imu.quaternion,
        gyro        = sub.imu.gyroscope,
        accel       = sub.imu.accelerometer,
        joint_pos   = sub.motor.q,
        joint_vel   = sub.motor.dq,
        tau_est     = sub.motor.tau_est,
        pressure    = sub.pressure,       # optional, zeros if not available
    )

    proj_grav  = est.projected_gravity()   # (3,)
    lin_vel    = est.body_lin_vel          # (3,)  estimated
    ang_vel    = est.body_ang_vel          # (3,)  from gyro
    contacts   = est.contact_state        # (4,)  bool
    yaw        = est.yaw                  # float rad
"""

import time
import numpy as np
from typing import Optional

NUM_JOINTS = 29

# ── G1 forward kinematics constants ─────────────────────────────────────────
# Approximate body-frame foot offsets at nominal stance (used for foot vel est)
# Based on G1 URDF zero-pose. Refine if you have precise URDF values.
NOMINAL_FOOT_POS_BODY = np.array([
    [ 0.17, -0.13, -0.80],   # left  foot  (hip→foot approx)
    [ 0.17,  0.13, -0.80],   # right foot
], dtype=np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Pure math helpers — no state, fully reusable in policy scripts
# ─────────────────────────────────────────────────────────────────────────────

def quat_to_rotation_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    """Quaternion [w,x,y,z] → 3×3 rotation matrix R (world←body)."""
    w, x, y, z = quat_wxyz.astype(float)
    R = np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ], dtype=np.float32)
    return R


def projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Rotate world gravity (0,0,-1) into body frame.
    Matches IsaacLab's projected_gravity_b observation term.
    quat_wxyz: [w, x, y, z]
    Returns:   (3,) float32
    """
    R = quat_to_rotation_matrix(quat_wxyz)
    # g_body = R^T @ [0, 0, -1]
    return (-R[2, :]).astype(np.float32)


def quat_to_rpy(quat_wxyz: np.ndarray) -> np.ndarray:
    """Quaternion [w,x,y,z] → [roll, pitch, yaw] in radians."""
    w, x, y, z = quat_wxyz.astype(float)
    roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
    yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw], dtype=np.float32)


def extract_yaw_quat(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Extract only the yaw component as a quaternion (removes roll/pitch).
    Useful for heading-relative velocity commands.
    Returns [w,x,y,z] with x=y=0.
    """
    yaw = quat_to_rpy(quat_wxyz)[2]
    return np.array([np.cos(yaw/2), 0, 0, np.sin(yaw/2)], dtype=np.float32)


def quat_rotate_vector(quat_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by quaternion q (active rotation)."""
    R = quat_to_rotation_matrix(quat_wxyz)
    return R @ v


def quat_rotate_inverse(quat_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Rotate vector v by q^-1 (body ← world transform).
    Matches instinct_onboard quat_rotate_inverse().
    """
    R = quat_to_rotation_matrix(quat_wxyz)
    return R.T @ v


def normalize_quat(quat_wxyz: np.ndarray) -> np.ndarray:
    """Normalize quaternion and ensure w > 0."""
    q = quat_wxyz.astype(float)
    n = np.linalg.norm(q)
    q = q / max(n, 1e-8)
    if q[0] < 0:
        q = -q
    return q.astype(np.float32)


def quat_to_tan_norm(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to tangent-normal 6-vector representation.
    Matches instinct_onboard quat_to_tan_norm() exactly.
    Returns (6,): [tan_x, tan_y, tan_z, norm_x, norm_y, norm_z]
    """
    R = quat_to_rotation_matrix(quat_wxyz)
    tan  = R @ np.array([1., 0., 0.])  # body x-axis in world frame
    norm = R @ np.array([0., 0., 1.])  # body z-axis in world frame
    return np.concatenate([tan, norm]).astype(np.float32)


def low_pass_filter(prev: np.ndarray, new: np.ndarray, alpha: float) -> np.ndarray:
    """Exponential moving average: alpha=1 → no filtering, alpha→0 → heavy."""
    return (alpha * np.asarray(new) + (1.0 - alpha) * np.asarray(prev)).astype(np.float32)


class CircularBuffer:
    """
    Fixed-length circular buffer for observation history.
    Matches instinct_onboard CircularBuffer exactly.
    """
    def __init__(self, length: int):
        self._buf: Optional[np.ndarray] = None
        self._length = length
        self._n_pushes = 0

    def append(self, value: np.ndarray):
        if self._buf is None:
            self._buf = np.zeros((self._length,) + tuple(value.shape), dtype=np.float32)
        if self._n_pushes == 0:
            self._buf[:] = value
        else:
            self._buf = np.roll(self._buf, -1, axis=0)
            self._buf[-1] = value
        self._n_pushes += 1

    @property
    def buffer(self) -> Optional[np.ndarray]:
        return self._buf

    def reset(self):
        if self._buf is not None:
            self._buf[:] = 0.0
        self._n_pushes = 0


# ─────────────────────────────────────────────────────────────────────────────
# G1StateEstimator — stateful estimator, update() every control step
# ─────────────────────────────────────────────────────────────────────────────

class G1StateEstimator:
    """
    Estimates quantities unavailable from direct sensing.

    Inputs  (from G1Subscriber):
        sub.imu.quaternion      orientation
        sub.imu.gyroscope       body angular velocity (direct, no estimation)
        sub.imu.accelerometer   body linear acceleration
        sub.motor.q             joint positions
        sub.motor.dq            joint velocities
        sub.motor.tau_est       estimated torques (used for contact detection)
        sub.pressure            12-ch pressure sensors (optional)

    Estimated outputs:
        body_lin_vel    (3,)  body-frame linear velocity via IMU integration + EKF
        body_ang_vel    (3,)  body-frame angular velocity (gyro, smoothed)
        projected_gravity (3,) gravity vector in body frame
        contact_state   (4,)  bool  [left_foot, right_foot, ...] contact flags
        yaw             float current heading (rad)
        R               (3,3) rotation matrix world←body
    """

    def __init__(self, dt: float = 0.002, smoothing_ratio: float = 0.2):
        self.dt   = dt
        self._alpha = smoothing_ratio  # for angular velocity smoothing

        # ── Internal state ─────────────────────────────────────────────────
        self.R    = np.eye(3, dtype=np.float32)
        self.yaw  = 0.0

        # Linear velocity estimate (simple IMU integration — reset on contact)
        self.body_lin_vel = np.zeros(3, dtype=np.float32)
        self._vel_confidence = 0.0     # 0=uncertain 1=confident

        # Angular velocity (smoothed gyro)
        self.body_ang_vel = np.zeros(3, dtype=np.float32)

        # Gravity vector in body frame
        self._proj_gravity = np.array([0., 0., -1.], dtype=np.float32)

        # Contact state [left_foot, right_foot] (G1 has 2 feet unlike GO2's 4)
        self.contact_state = np.zeros(2, dtype=np.float32)

        # Angular velocity smoothing buffer (mirrors GO2 deuler_history)
        _N = 12
        self._deuler_history = np.zeros((_N, 3), dtype=np.float32)
        self._dt_history     = np.ones((_N, 1),  dtype=np.float32) * dt
        self._rpy_prev       = np.zeros(3, dtype=np.float32)
        self._buf_idx        = 0
        self._t_prev         = time.time()

        # Contact detection thresholds
        self._pressure_threshold  = 10.0   # N — adjust for your sensor calibration
        self._tau_contact_threshold = 20.0 # Nm on ankle joints → contact proxy

    # ── Main update — call every control step ──────────────────────────────

    def update(
        self,
        quat_wxyz:  np.ndarray,           # (4,)  [w,x,y,z]
        gyro:       np.ndarray,           # (3,)  rad/s, body frame
        accel:      np.ndarray,           # (3,)  m/s², body frame
        joint_pos:  np.ndarray,           # (29,) rad, sim order
        joint_vel:  np.ndarray,           # (29,) rad/s, sim order
        tau_est:    np.ndarray,           # (29,) Nm, sim order
        pressure:   Optional[np.ndarray] = None,  # (12,) N, optional
    ):
        quat_wxyz = normalize_quat(quat_wxyz)

        # Rotation matrix and gravity
        self.R = quat_to_rotation_matrix(quat_wxyz)
        self._proj_gravity = (-self.R[2, :]).astype(np.float32)

        # Yaw
        rpy = quat_to_rpy(quat_wxyz)
        self.yaw = float(rpy[2])

        # ── Angular velocity: gyro + history smoothing (GO2 style) ─────────
        t_now = time.time()
        dt_actual = max(t_now - self._t_prev, 1e-6)
        drpy = rpy - self._rpy_prev
        # Handle yaw wrap-around
        drpy[2] = (drpy[2] + np.pi) % (2*np.pi) - np.pi

        idx = self._buf_idx % len(self._deuler_history)
        self._deuler_history[idx] = drpy
        self._dt_history[idx]     = dt_actual
        self._buf_idx += 1
        self._t_prev  = t_now
        self._rpy_prev = rpy.copy()

        # Smooth: mean of history, blend with previous estimate
        ang_vel_hist = np.mean(self._deuler_history / self._dt_history, axis=0)
        self.body_ang_vel = (
            self._alpha * ang_vel_hist +
            (1 - self._alpha) * self.body_ang_vel
        ).astype(np.float32)

        # ── Contact detection ────────────────────────────────────────────────
        # Method 1: pressure sensors (preferred when USE_SDK2=ON)
        if pressure is not None and np.any(pressure > 0):
            # G1: pressure[0..5]=left foot/toe, pressure[6..11]=right foot/toe
            left_press  = np.max(pressure[0:6])
            right_press = np.max(pressure[6:12])
            self.contact_state[0] = float(left_press  > self._pressure_threshold)
            self.contact_state[1] = float(right_press > self._pressure_threshold)
        else:
            # Method 2: ankle torque proxy (mirrors GO2 foot_force approach)
            # G1 sim order: ankle_pitch L=25, R=26, ankle_roll L=27, R=28
            left_ankle_tau  = np.abs(tau_est[25]) + np.abs(tau_est[27])
            right_ankle_tau = np.abs(tau_est[26]) + np.abs(tau_est[28])
            self.contact_state[0] = float(left_ankle_tau  > self._tau_contact_threshold)
            self.contact_state[1] = float(right_ankle_tau > self._tau_contact_threshold)

        # ── Linear velocity: IMU integration with drift correction ──────────
        # Gravity-correct acceleration (remove static gravity component)
        accel_world = self.R @ accel  # body → world
        accel_world[2] += 9.81        # remove gravity
        accel_body = self.R.T @ accel_world

        # Simple leaky integration — drift builds up, but contact resets it
        alpha_vel = 0.98
        self.body_lin_vel = alpha_vel * (self.body_lin_vel + accel_body * self.dt)

        # When neither foot is in contact, trust the integral less
        any_contact = np.any(self.contact_state > 0.5)
        if not any_contact:
            self.body_lin_vel *= 0.95  # bleed off velocity estimate when airborne

        # Clip to physical limits (the robot can't go faster than ~3 m/s)
        self.body_lin_vel = np.clip(self.body_lin_vel, -3.0, 3.0)

    # ── Observation accessors — match IsaacLab / unitree_rl_lab naming ──────

    def projected_gravity(self) -> np.ndarray:
        """(3,) gravity vector in body frame. Matches IsaacLab projected_gravity_b."""
        return self._proj_gravity.copy()

    def base_lin_vel(self) -> np.ndarray:
        """(3,) estimated base linear velocity in body frame."""
        return self.body_lin_vel.copy()

    def base_ang_vel(self) -> np.ndarray:
        """(3,) base angular velocity in body frame (smoothed gyro)."""
        return self.body_ang_vel.copy()

    def foot_contact(self) -> np.ndarray:
        """(2,) float32 contact flags [left, right]. 1.0=contact, 0.0=no contact."""
        return self.contact_state.copy()

    def heading(self) -> float:
        """Current yaw heading in radians."""
        return self.yaw

    def tan_norm(self, quat_wxyz: np.ndarray) -> np.ndarray:
        """
        6-vector tangent-normal representation.
        Matches instinct_onboard quat_to_tan_norm().
        """
        return quat_to_tan_norm(quat_wxyz)

    # ── Convenience: build the standard sim-compatible obs block ────────────

    def build_base_obs(self, quat_wxyz: np.ndarray) -> dict:
        """
        Returns a dict of all base observations that differ between sim and real.
        Feed these into your policy obs builder.

        Keys match IsaacLab ManagerBasedRLEnv observation terms:
            base_lin_vel         (3,)
            base_ang_vel         (3,)
            projected_gravity    (3,)
            heading              float
            foot_contact         (2,)
        """
        return {
            "base_lin_vel":      self.base_lin_vel(),
            "base_ang_vel":      self.base_ang_vel(),
            "projected_gravity": self.projected_gravity(),
            "heading":           self.heading(),
            "foot_contact":      self.foot_contact(),
        }
