"""
mujoco_adapter.py
──────────────────
Mirror → MuJoCo (unitree_rl_mjlab):
    /g1/motor_state  →  /mujoco/joint_states
    /g1/imu          →  /mujoco/imu

MuJoCo → /lowcmd  (reverse, multi-target only):
    /mujoco/joint_command  →  /lowcmd

Run alongside the middleware:
    ros2 run g1_io mujoco_adapter
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from unitree_hg.msg import LowCmd  # type: ignore

from ..bridge.ros2_msg_types.topics import (
    MOTOR_STATE, IMU_OUT,
    MUJOCO_JOINT_STATE, MUJOCO_IMU,
    MUJOCO_JOINT_CMD, LOWCMD,
)

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


class MujocoAdapter(Node):
    def __init__(self):
        super().__init__("mujoco_adapter")
        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value

        self.mj_js_pub  = self.create_publisher(JointState, MUJOCO_JOINT_STATE, qos)
        self.mj_imu_pub = self.create_publisher(Imu,        MUJOCO_IMU, qos)
        self.lowcmd_pub = self.create_publisher(LowCmd, LOWCMD, 10)

        self.create_subscription(JointState, MOTOR_STATE, self._fwd_joint, qos)
        self.create_subscription(Imu,        IMU_OUT,     self._fwd_imu,   qos)
        self.create_subscription(JointState, MUJOCO_JOINT_CMD, self._reverse_cmd, 10)

        self.get_logger().info("MujocoAdapter ready.")

    def _fwd_joint(self, msg: JointState):
        out = JointState()
        out.header   = msg.header
        out.name     = SIM_JOINT_NAMES
        out.position = list(msg.position) if msg.position else [0.0] * NUM_JOINTS
        out.velocity = list(msg.velocity) if msg.velocity else [0.0] * NUM_JOINTS
        self.mj_js_pub.publish(out)

    def _fwd_imu(self, msg: Imu):
        self.mj_imu_pub.publish(msg)

    def _reverse_cmd(self, msg: JointState):
        from robot_config import JOINT_MAP, JOINT_SIGNS  # type: ignore
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
    node = MujocoAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
