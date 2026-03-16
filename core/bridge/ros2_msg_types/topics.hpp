#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// All ROS2 topic names used by g1_io.
// Single source of truth — edit here, everything picks it up.
// Python mirror: core/bridge/ros2_msg_types/topics.py
// ─────────────────────────────────────────────────────────────────────────────
namespace topics {

// ── Inputs: unitree_ros2 bridge → middleware (500 Hz control path) ─────────
constexpr const char* LOWSTATE          = "/lowstate";
constexpr const char* SECONDARY_IMU     = "/secondary_imu";
constexpr const char* WIRELESS_CTRL     = "/wirelesscontroller";

// ── Output: middleware → unitree_ros2 bridge  (SINGLE WRITER RULE) ─────────
constexpr const char* LOWCMD       = "/lowcmd";
// Command input from policy to middleware (Float32MultiArray, 145 floats, sim order)
constexpr const char* CMD_ACTION   = "/g1/cmd_action";

// ══════════════════════════════════════════════════════
// READ-ONLY MIRROR TOPICS  (50 Hz, never in ctrl loop)
// ══════════════════════════════════════════════════════

// Motor state — all fields from MotorState_ IDL
constexpr const char* MOTOR_STATE       = "/g1/motor_state";       // JointState: q + dq + tau_est
constexpr const char* MOTOR_DDQ         = "/g1/motor_ddq";         // Float32MultiArray[29]: joint acceleration
constexpr const char* MOTOR_TEMPERATURE = "/g1/motor_temperature"; // Float32MultiArray[29]: winding temp
constexpr const char* MOTOR_VOLTAGE     = "/g1/motor_voltage";     // Float32MultiArray[29]: motor voltage
constexpr const char* MOTOR_STATUS      = "/g1/motor_status";      // UInt32MultiArray[29]: fault flags

// IMU
constexpr const char* IMU_OUT           = "/g1/imu";               // sensor_msgs/Imu
constexpr const char* IMU_RPY           = "/g1/imu_rpy";           // geometry_msgs/Vector3

// RC / joystick
constexpr const char* RC_CMD_OUT        = "/g1/rc_cmd";            // geometry_msgs/Twist

// Hardware timestamp
constexpr const char* HW_TICK           = "/g1/tick";              // std_msgs/UInt32

// Debug
constexpr const char* DEBUG_LOG         = "/g1/debug";             // std_msgs/String

// Battery — from BmsState_ via unitree_sdk2 DDS (separate low-priority thread)
constexpr const char* BMS_SOC           = "/g1/bms/soc";           // std_msgs/Float32: %
constexpr const char* BMS_CURRENT       = "/g1/bms/current";       // std_msgs/Float32: A
constexpr const char* BMS_VOLTAGE       = "/g1/bms/voltage";       // std_msgs/Float32: V
constexpr const char* BMS_TEMPERATURE   = "/g1/bms/temperature";   // Float32MultiArray[12]
constexpr const char* BMS_CELL_VOLTAGE  = "/g1/bms/cell_voltage";  // Float32MultiArray[40]
constexpr const char* BMS_STATE         = "/g1/bms/state";         // std_msgs/String JSON

// Main board — from MainBoardState_ via unitree_sdk2 DDS
constexpr const char* BOARD_FAN         = "/g1/board/fan";         // UInt16MultiArray[6]
constexpr const char* BOARD_TEMPERATURE = "/g1/board/temperature"; // Float32MultiArray[6]

// Pressure sensors — from PressSensorState_ via unitree_sdk2 DDS
constexpr const char* PRESS_SENSORS     = "/g1/pressure";          // Float32MultiArray[12]

// Camera (realsense_node — independent process)
constexpr const char* DEPTH_IMAGE       = "/camera/depth/image_rect_raw";
constexpr const char* DEPTH_INFO        = "/camera/depth/camera_info";
constexpr const char* DEPTH_POINTS      = "/camera/depth/points";

// Sim adapter target topics
constexpr const char* ISAAC_JOINT_CMD   = "/isaac/joint_command";
constexpr const char* ISAAC_JOINT_STATE = "/isaac/joint_states";
constexpr const char* ISAAC_IMU         = "/isaac/imu";
constexpr const char* MUJOCO_JOINT_CMD  = "/mujoco/joint_command";
constexpr const char* MUJOCO_JOINT_STATE= "/mujoco/joint_states";
constexpr const char* MUJOCO_IMU        = "/mujoco/imu";

} // namespace topics
