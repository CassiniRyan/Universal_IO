"""
isaaclab_adapter.py
────────────────────
Subscribes to the G1 mirror topics and re-publishes them in the format
Isaac Sim expects.  Also converts Isaac Sim joint_command back to /lowcmd
format so the C++ middleware can pick it up when running multi-target.

Run this alongside the middleware:
    ros2 run g1_io isaaclab_adapter

It is a pure ROS2 node — no direct SDK dependency.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from unitree_hg.msg import LowCmd  # type: ignore

from ..utils.python.state_estimator import projected_gravity
from ..bridge.ros2_msg_types.topics import (
    MOTOR_STATE, IMU_OUT, RC_CMD_OUT,
    ISAAC_JOINT_STATE, ISAAC_IMU,
    ISAAC_JOINT_CMD, LOWCMD,
)

# Joint names in simulation order (must match robot_config.hpp)
SIM_JOINT_NAMES = [
    "left_shoulder_pitch_joint",  "right_shoulder_pitch_joint", "waist_pitch_joint",
    "left_shoulder_roll_joint",   "right_shoulder_roll_joint",  "waist_roll_joint",
    "left_shoulder_yaw_joint",    "right_shoulder_yaw_joint",   "waist_yaw_joint",
    "left_elbow_joint",           "right_elbow_joint",
    "left_hip_pitch_joint",       "right_hip_pitch_joint",
    "left_wrist_roll_joint",      "right_wrist_roll_joint",
    "left_hip_roll_joint",        "right_hip_roll_joint",
    "left_wrist_pitch_joint",     "right_wrist_pitch_joint",
    "left_hip_yaw_joint",         "right_hip_yaw_joint",
    "left_wrist_yaw_joint",       "right_wrist_yaw_joint",
    "left_knee_joint",            "right_knee_joint",
    "left_ankle_pitch_joint",     "right_ankle_pitch_joint",
    "left_ankle_roll_joint",      "right_ankle_roll_joint",
]

NUM_JOINTS = 29


class IsaacLabAdapter(Node):
    """
    Mirror → Isaac Sim:
        /g1/motor_state  →  /isaac/joint_states
        /g1/imu          →  /isaac/imu

    Isaac Sim → /lowcmd:
        /isaac/joint_command  →  /lowcmd   (only when multi-target active)
    """

    def __init__(self):
        super().__init__("isaaclab_adapter")

        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value

        # ── Forward: G1 mirrors → Isaac Sim ──────────────────────────────
        self.isaac_js_pub  = self.create_publisher(JointState, ISAAC_JOINT_STATE, qos)
        self.isaac_imu_pub = self.create_publisher(Imu,        ISAAC_IMU, qos)

        self.create_subscription(JointState, MOTOR_STATE, self._fwd_joint, qos)
        self.create_subscription(Imu,        IMU_OUT,     self._fwd_imu,   qos)

        # ── Reverse: Isaac Sim joint_command → /lowcmd ───────────────────
        # Only used in multi-target mode (Isaac + real robot simultaneously).
        # When not needed, this subscriber just sits idle.
        self.lowcmd_pub = self.create_publisher(LowCmd, LOWCMD, 10)
        self.create_subscription(
            JointState, ISAAC_JOINT_CMD, self._reverse_cmd, 10)

        self.get_logger().info("IsaacLabAdapter ready.")

    # ── Forward ──────────────────────────────────────────────────────────────

    def _fwd_joint(self, msg: JointState):
        """Pass joint state straight through with Isaac-compatible names."""
        out = JointState()
        out.header = msg.header
        out.name     = SIM_JOINT_NAMES
        out.position = list(msg.position) if msg.position else [0.0] * NUM_JOINTS
        out.velocity = list(msg.velocity) if msg.velocity else [0.0] * NUM_JOINTS
        self.isaac_js_pub.publish(out)

    def _fwd_imu(self, msg: Imu):
        """Pass IMU straight through (already in sensor_msgs/Imu format)."""
        self.isaac_imu_pub.publish(msg)

    # ── Reverse ──────────────────────────────────────────────────────────────

    def _reverse_cmd(self, msg: JointState):
        """
        Convert an Isaac Sim JointState command back to unitree_hg/LowCmd.
        This runs only in multi-target mode.  The C++ middleware's LowCmdWrite()
        ignores this topic — it reads from its own internal buffer.
        This is published to /lowcmd so any secondary observer can see it.
        """
        from robot_config import JOINT_MAP, JOINT_SIGNS, NUM_JOINTS  # type: ignore
        cmd = LowCmd()
        cmd.mode_pr = 0
        name_to_sim = {n: i for i, n in enumerate(SIM_JOINT_NAMES)}
        for i, name in enumerate(msg.name):
            sim = name_to_sim.get(name)
            if sim is None:
                continue
            real = JOINT_MAP[sim]
            sign = JOINT_SIGNS[sim]
            cmd.motor_cmd[real].mode = 0x01
            cmd.motor_cmd[real].q    = msg.position[i] * sign if i < len(msg.position) else 0.0
            cmd.motor_cmd[real].dq   = msg.velocity[i] * sign if i < len(msg.velocity) else 0.0
            cmd.motor_cmd[real].kp   = 100.0
            cmd.motor_cmd[real].kd   = 3.0
        self.lowcmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = IsaacLabAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
