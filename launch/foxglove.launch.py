"""Foxglove WebSocket bridge — read-only debug observer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port",    default_value="8765"),
        DeclareLaunchArgument("address", default_value="0.0.0.0"),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{
                "port":    LaunchConfiguration("port"),
                "address": LaunchConfiguration("address"),
                "topic_whitelist": [
                    # Motor
                    "/g1/motor_state", "/g1/motor_ddq",
                    "/g1/motor_temperature", "/g1/motor_voltage", "/g1/motor_status",
                    # IMU
                    "/g1/imu", "/g1/imu_rpy",
                    # RC + tick + debug
                    "/g1/rc_cmd", "/g1/tick", "/g1/debug",
                    # Battery
                    "/g1/bms/soc", "/g1/bms/current", "/g1/bms/voltage",
                    "/g1/bms/temperature", "/g1/bms/cell_voltage", "/g1/bms/state",
                    # Board + pressure
                    "/g1/board/fan", "/g1/board/temperature", "/g1/pressure",
                    # Camera
                    "/camera/depth/image_rect_raw",
                    "/camera/depth/camera_info",
                    "/camera/depth/points",
                    # TF
                    "/tf", "/tf_static",
                    # Sim mirrors
                    "/isaac/joint_states", "/mujoco/joint_states",
                ],
            }],
            output="screen",
        ),
    ])
