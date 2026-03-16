"""
ros2_subscriber.py
──────────────────
The Python IO interface — mirrors the GO2 python_middleware.py pattern.

Provides:
  G1Subscriber  — receives all mirror topics from the C++ middleware
  G1Commander   — sends action commands to /g1/cmd_action

Usage (mirrors GO2 Go2Controller pattern):
    rclpy.init()
    sub = G1Subscriber()
    cmd = G1Commander(sub)
    threading.Thread(target=rclpy.spin, args=(sub,), daemon=True).start()

    while running:
        q   = sub.motor.q          # float32[29] joint positions
        tau = sub.motor.tau_est    # float32[29] estimated torques
        imu = sub.imu.quaternion   # float32[4]  body orientation [w,x,y,z]
        cmd_vel = sub.rc.linear    # float32[3]  joystick axes
        soc = sub.bms.soc          # float32     battery %

        # Send action (sim order, same as instinct_onboard send_action)
        cmd.send(q_des, dq_des, tau_des, kp, kd)
"""

import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import (Float32, Float32MultiArray, UInt32,
                          UInt16MultiArray, UInt32MultiArray, String)

from ..bridge.ros2_msg_types.topics import (
    MOTOR_STATE, MOTOR_DDQ, MOTOR_TEMPERATURE, MOTOR_VOLTAGE, MOTOR_STATUS,
    IMU_OUT, IMU_RPY, RC_CMD_OUT, HW_TICK,
    BMS_SOC, BMS_CURRENT, BMS_VOLTAGE, BMS_TEMPERATURE, BMS_CELL_VOLTAGE,
    BOARD_FAN, BOARD_TEMPERATURE, PRESS_SENSORS,
    CMD_ACTION,
)

NUM_JOINTS = 29
_SQ  = QoSPresetProfiles.SENSOR_DATA.value
_Q10 = rclpy.qos.QoSProfile(depth=10)


# ─────────────────────────────────────────────────────────────────────────────
# Data containers — plain Python objects, accessed as sub.motor.q etc.
# ─────────────────────────────────────────────────────────────────────────────

class MotorState:
    """All per-joint fields from MotorState_ IDL — sim order."""
    def __init__(self):
        self.q           = np.zeros(NUM_JOINTS, np.float32)  # rad
        self.dq          = np.zeros(NUM_JOINTS, np.float32)  # rad/s
        self.tau_est     = np.zeros(NUM_JOINTS, np.float32)  # Nm
        self.ddq         = np.zeros(NUM_JOINTS, np.float32)  # rad/s²
        self.temperature = np.zeros(NUM_JOINTS, np.float32)  # °C
        self.voltage     = np.zeros(NUM_JOINTS, np.float32)  # V
        self.status      = np.zeros(NUM_JOINTS, np.uint32)   # fault flags

class ImuData:
    """Full IMU from IMUState_ IDL."""
    def __init__(self):
        self.quaternion    = np.array([1.,0.,0.,0.], np.float32)  # w,x,y,z
        self.gyroscope     = np.zeros(3, np.float32)              # rad/s
        self.accelerometer = np.zeros(3, np.float32)              # m/s²
        self.rpy           = np.zeros(3, np.float32)              # rad

class RcData:
    """Wireless controller / joystick."""
    def __init__(self):
        self.linear  = np.zeros(3, np.float32)   # x=fwd, y=strafe, z=0
        self.angular = np.zeros(3, np.float32)   # z=yaw
        self.mode    = 0                          # button-selected mode

class BmsData:
    """Battery — from BmsState_ IDL via SDK2 thread."""
    def __init__(self):
        self.soc          = 0.0
        self.current      = 0.0
        self.voltage      = 0.0
        self.temperature  = np.zeros(12, np.float32)
        self.cell_voltage = np.zeros(40, np.float32)

class BoardData:
    """Main board — from MainBoardState_ IDL via SDK2 thread."""
    def __init__(self):
        self.fan_speed   = np.zeros(6, np.uint16)
        self.temperature = np.zeros(6, np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# G1Subscriber — receives all mirror topics, exposes as Python attributes
# ─────────────────────────────────────────────────────────────────────────────

class G1Subscriber(Node):
    def __init__(self):
        super().__init__("g1_subscriber")

        self.motor    = MotorState()
        self.imu      = ImuData()
        self.rc       = RcData()
        self.bms      = BmsData()
        self.board    = BoardData()
        self.pressure = np.zeros(12, np.float32)
        self.tick     = 0
        self._lock    = threading.Lock()

        # ── Motor state ───────────────────────────────────────────────────
        self.create_subscription(JointState,         MOTOR_STATE,       self._on_motor_state,  _SQ)
        self.create_subscription(Float32MultiArray,  MOTOR_DDQ,         self._on_motor_ddq,    _SQ)
        self.create_subscription(Float32MultiArray,  MOTOR_TEMPERATURE, self._on_motor_temp,   _SQ)
        self.create_subscription(Float32MultiArray,  MOTOR_VOLTAGE,     self._on_motor_vol,    _SQ)
        self.create_subscription(UInt32MultiArray,   MOTOR_STATUS,      self._on_motor_status, _SQ)

        # ── IMU ───────────────────────────────────────────────────────────
        self.create_subscription(Imu,     IMU_OUT, self._on_imu, _SQ)
        self.create_subscription(Vector3, IMU_RPY, self._on_rpy, _SQ)

        # ── RC / system ───────────────────────────────────────────────────
        self.create_subscription(Twist,  RC_CMD_OUT, self._on_rc,   _Q10)
        self.create_subscription(UInt32, HW_TICK,    self._on_tick, _Q10)

        # ── BMS ───────────────────────────────────────────────────────────
        self.create_subscription(Float32,            BMS_SOC,         self._on_bms_soc,   _Q10)
        self.create_subscription(Float32,            BMS_CURRENT,     self._on_bms_cur,   _Q10)
        self.create_subscription(Float32,            BMS_VOLTAGE,     self._on_bms_vol,   _Q10)
        self.create_subscription(Float32MultiArray,  BMS_TEMPERATURE, self._on_bms_temp,  _Q10)
        self.create_subscription(Float32MultiArray,  BMS_CELL_VOLTAGE,self._on_bms_cells, _Q10)

        # ── Board + pressure ──────────────────────────────────────────────
        self.create_subscription(UInt16MultiArray,   BOARD_FAN,         self._on_board_fan,  _Q10)
        self.create_subscription(Float32MultiArray,  BOARD_TEMPERATURE, self._on_board_temp, _Q10)
        self.create_subscription(Float32MultiArray,  PRESS_SENSORS,     self._on_pressure,   _Q10)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_motor_state(self, msg: JointState):
        with self._lock:
            n = min(len(msg.position), NUM_JOINTS)
            self.motor.q[:n]       = msg.position[:n]
            self.motor.dq[:n]      = msg.velocity[:n]
            self.motor.tau_est[:n] = msg.effort[:n]

    def _on_motor_ddq(self, msg: Float32MultiArray):
        with self._lock:
            n = min(len(msg.data), NUM_JOINTS)
            self.motor.ddq[:n] = msg.data[:n]

    def _on_motor_temp(self, msg: Float32MultiArray):
        with self._lock:
            n = min(len(msg.data), NUM_JOINTS)
            self.motor.temperature[:n] = msg.data[:n]

    def _on_motor_vol(self, msg: Float32MultiArray):
        with self._lock:
            n = min(len(msg.data), NUM_JOINTS)
            self.motor.voltage[:n] = msg.data[:n]

    def _on_motor_status(self, msg: UInt32MultiArray):
        with self._lock:
            n = min(len(msg.data), NUM_JOINTS)
            self.motor.status[:n] = msg.data[:n]

    def _on_imu(self, msg: Imu):
        with self._lock:
            q = msg.orientation
            self.imu.quaternion[:] = [q.w, q.x, q.y, q.z]
            g = msg.angular_velocity
            self.imu.gyroscope[:]  = [g.x, g.y, g.z]
            a = msg.linear_acceleration
            self.imu.accelerometer[:] = [a.x, a.y, a.z]

    def _on_rpy(self, msg: Vector3):
        with self._lock:
            self.imu.rpy[:] = [msg.x, msg.y, msg.z]

    def _on_rc(self, msg: Twist):
        with self._lock:
            self.rc.linear[:]  = [msg.linear.x,  msg.linear.y,  msg.linear.z]
            self.rc.angular[:] = [msg.angular.x, msg.angular.y, msg.angular.z]

    def _on_tick(self, msg: UInt32):
        with self._lock:
            self.tick = msg.data

    def _on_bms_soc(self,  msg: Float32): 
        with self._lock: self.bms.soc     = msg.data
    def _on_bms_cur(self,  msg: Float32):
        with self._lock: self.bms.current = msg.data
    def _on_bms_vol(self,  msg: Float32):
        with self._lock: self.bms.voltage = msg.data
    def _on_bms_temp(self, msg: Float32MultiArray):
        with self._lock:
            n = min(len(msg.data), 12)
            self.bms.temperature[:n] = msg.data[:n]
    def _on_bms_cells(self, msg: Float32MultiArray):
        with self._lock:
            n = min(len(msg.data), 40)
            self.bms.cell_voltage[:n] = msg.data[:n]
    def _on_board_fan(self, msg: UInt16MultiArray):
        with self._lock:
            n = min(len(msg.data), 6)
            self.board.fan_speed[:n] = msg.data[:n]
    def _on_board_temp(self, msg: Float32MultiArray):
        with self._lock:
            n = min(len(msg.data), 6)
            self.board.temperature[:n] = msg.data[:n]
    def _on_pressure(self, msg: Float32MultiArray):
        with self._lock:
            n = min(len(msg.data), 12)
            self.pressure[:n] = msg.data[:n]


# ─────────────────────────────────────────────────────────────────────────────
# G1Commander — sends action commands to /g1/cmd_action
#
# The C++ middleware reads this topic in CmdActionHandler() and writes the
# result to /lowcmd at 500 Hz.
#
# send() format:  Float32MultiArray [145 floats, sim order]
#   [0  ..28]   q_des   joint position targets (rad)
#   [29 ..57]   dq_des  joint velocity targets (rad/s)
#   [58 ..86]   tau_des feedforward torques (Nm)
#   [87 ..115]  kp      position gains
#   [116..144]  kd      velocity gains
# ─────────────────────────────────────────────────────────────────────────────

class G1Commander:
    """
    Publishes action commands to the C++ middleware.

    Usage:
        cmd = G1Commander(sub_node)
        cmd.send(q_des, dq_des, tau_des, kp, kd)

        # Or use the helpers:
        cmd.send_pd_only(q_des, kp, kd)       # tau_des=0, dq_des=0
        cmd.send_torque_only(tau_des)          # kp=kd=0
        cmd.send_damping()                     # kd=5, everything else 0
    """

    def __init__(self, node: Node):
        self._pub = node.create_publisher(
            Float32MultiArray, CMD_ACTION, rclpy.qos.QoSProfile(depth=10))
        self._node = node
        self.timestamp_us = 0

    def send(self,
             q_des:   np.ndarray,
             dq_des:  np.ndarray,
             tau_des: np.ndarray,
             kp:      np.ndarray,
             kd:      np.ndarray):
        """Send a full PD + feedforward command. All arrays must be length 29."""
        msg = Float32MultiArray()
        msg.data = (
            list(np.asarray(q_des,   np.float32)) +
            list(np.asarray(dq_des,  np.float32)) +
            list(np.asarray(tau_des, np.float32)) +
            list(np.asarray(kp,      np.float32)) +
            list(np.asarray(kd,      np.float32))
        )
        self._pub.publish(msg)
        self.timestamp_us = int(time.time() * 1e6)

    def send_pd_only(self, q_des: np.ndarray, kp: np.ndarray, kd: np.ndarray):
        """Position control only (tau_des=0, dq_des=0)."""
        z = np.zeros(NUM_JOINTS, np.float32)
        self.send(q_des, z, z, kp, kd)

    def send_torque_only(self, tau_des: np.ndarray):
        """Pure torque control (kp=kd=q_des=dq_des=0)."""
        z = np.zeros(NUM_JOINTS, np.float32)
        self.send(z, z, tau_des, z, z)

    def send_damping(self, kd: float = 5.0):
        """Full damping — safe compliance mode, same as L2+B."""
        z  = np.zeros(NUM_JOINTS, np.float32)
        kd_arr = np.full(NUM_JOINTS, kd, np.float32)
        self.send(z, z, z, z, kd_arr)
