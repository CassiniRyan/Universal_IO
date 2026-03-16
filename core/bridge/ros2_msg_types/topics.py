# topics.py  — Python mirror of topics.hpp
# Keep in sync when you rename topics.

# Control path (500 Hz)
LOWSTATE          = "/lowstate"
SECONDARY_IMU     = "/secondary_imu"
WIRELESS_CTRL     = "/wirelesscontroller"
LOWCMD            = "/lowcmd"

# Motor state mirrors
MOTOR_STATE       = "/g1/motor_state"
MOTOR_DDQ         = "/g1/motor_ddq"
MOTOR_TEMPERATURE = "/g1/motor_temperature"
MOTOR_VOLTAGE     = "/g1/motor_voltage"
MOTOR_STATUS      = "/g1/motor_status"

# IMU mirrors
IMU_OUT           = "/g1/imu"
IMU_RPY           = "/g1/imu_rpy"

# RC
RC_CMD_OUT        = "/g1/rc_cmd"

# Hardware tick
HW_TICK           = "/g1/tick"

# Debug
DEBUG_LOG         = "/g1/debug"

# Battery (from unitree_sdk2 DDS)
BMS_SOC           = "/g1/bms/soc"
BMS_CURRENT       = "/g1/bms/current"
BMS_VOLTAGE       = "/g1/bms/voltage"
BMS_TEMPERATURE   = "/g1/bms/temperature"
BMS_CELL_VOLTAGE  = "/g1/bms/cell_voltage"
BMS_STATE         = "/g1/bms/state"

# Main board
BOARD_FAN         = "/g1/board/fan"
BOARD_TEMPERATURE = "/g1/board/temperature"

# Pressure sensors
PRESS_SENSORS     = "/g1/pressure"

# Camera
DEPTH_IMAGE       = "/camera/depth/image_rect_raw"
DEPTH_INFO        = "/camera/depth/camera_info"
DEPTH_POINTS      = "/camera/depth/points"

# Sim adapters
ISAAC_JOINT_CMD   = "/isaac/joint_command"
ISAAC_JOINT_STATE = "/isaac/joint_states"
ISAAC_IMU         = "/isaac/imu"
MUJOCO_JOINT_CMD  = "/mujoco/joint_command"
MUJOCO_JOINT_STATE= "/mujoco/joint_states"
MUJOCO_IMU        = "/mujoco/imu"
-e 
# Policy → middleware command
CMD_ACTION = "/g1/cmd_action"
