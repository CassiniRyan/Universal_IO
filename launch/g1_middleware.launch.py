"""Launch G1 control middleware, or recorder-only mode with record_log:=True."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("targets", default_value="real"),
        DeclareLaunchArgument("dryrun", default_value="False"),
        DeclareLaunchArgument("record_log", default_value="False"),
        DeclareLaunchArgument(
            "log_dir",
            default_value=EnvironmentVariable("G1_IO_LOG_DIR", default_value=""),
        ),
        Node(
            package="g1_io",
            executable="cpp_middleware",
            name="g1_middleware",
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("record_log")),
            arguments=["--targets", LaunchConfiguration("targets")],
            parameters=[{
                "dryrun": ParameterValue(LaunchConfiguration("dryrun"), value_type=bool),
            }],
        ),
        Node(
            package="g1_io",
            executable="lowlevel_csv_recorder",
            name="g1_lowlevel_csv_recorder",
            output="screen",
            condition=IfCondition(LaunchConfiguration("record_log")),
            parameters=[{
                "log_dir": LaunchConfiguration("log_dir"),
            }],
        ),
    ])
