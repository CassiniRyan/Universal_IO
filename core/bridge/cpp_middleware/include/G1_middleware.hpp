#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// G1_middleware.hpp
//
// Mirrors the structure of the GO2 Cpp_middleware.hpp/cpp exactly, adapted
// for G1 (29 DOF, unitree_hg IDL, ROS2 topics instead of LCM).
//
// Three threads mirror the GO2 design:
//   Thread 1 — ros2_publish_thread  : reads low_state_ → publishes all mirrors
//   Thread 2 — ros2_receive_thread  : spins ROS2 to receive /g1/cmd_action
//   Thread 3 — low_cmd_write_thread : runs at 500 Hz, writes /lowcmd
//
// Safety logic is in safety_filter_0/1.cpp — not here.
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <array>
#include <fstream>
#include <string>
#include <vector>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

// Unitree ROS2 messages (from unitree_ros2)
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_go/msg/wireless_controller.hpp"

// G1 config
#include "robot_config.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// ControlParams — filtered commands passed from policy to LowCmdWrite
// Protected by motor_cmd_mutex so thread 2 can write while thread 3 reads
// ─────────────────────────────────────────────────────────────────────────────
struct ControlParams {
    float q_des[NUM_JOINTS]   = {};  // target joint position (sim order, rad)
    float dq_des[NUM_JOINTS]  = {};  // target joint velocity (rad/s)
    float tau_des[NUM_JOINTS] = {};  // feedforward torque (Nm)
    float kp[NUM_JOINTS]      = {};  // position gain
    float kd[NUM_JOINTS]      = {};  // velocity gain
    std::mutex motor_cmd_mutex;
};

// ─────────────────────────────────────────────────────────────────────────────
// Multi-target descriptor — each entry gets its own /lowcmd publisher
// ─────────────────────────────────────────────────────────────────────────────
struct CmdTarget {
    std::string topic;
    std::string description;
};

// ─────────────────────────────────────────────────────────────────────────────
// G1Middleware — the main class
// ─────────────────────────────────────────────────────────────────────────────
class G1Middleware : public rclcpp::Node {
public:
    explicit G1Middleware(const std::vector<CmdTarget>& targets,
                          const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
    ~G1Middleware();

    void Init();    // wire up all publishers, subscribers, init cmd buffers
    void Loop();    // spawn the three named threads

private:
    // ── Raw hardware buffers (written only inside message handlers) ────────
    unitree_hg::msg::LowState         low_state_{};   // latest from /lowstate
    unitree_hg::msg::IMUState         torso_imu_{};   // latest from /secondary_imu
    unitree_go::msg::WirelessController joystick_{};  // latest from /wirelesscontroller
    unitree_hg::msg::LowCmd           low_cmd_{};     // built here, published to /lowcmd

    std::mutex state_mutex_;   // guards low_state_ / torso_imu_ / joystick_
    bool       secondary_imu_valid_{ false };
    bool       first_run_{ true };
    int        motiontime_{ 0 };
    int        mode_{ 0 };     // button-selected mode (A=0 B=1 X=2 Y=3 ...)

    float dt_{ 0.002f };       // 500 Hz

    // Key union — same as GO2
    typedef union {
        struct {
            uint8_t R1:1, L1:1, start:1, select:1;
            uint8_t R2:1, L2:1, F1:1, F2:1;
            uint8_t A:1, B:1, X:1, Y:1;
            uint8_t up:1, right:1, down:1, left:1;
        } components;
        uint16_t value;
    } KeySwitch;
    KeySwitch key_{};

    // ControlParams — the interface between policy and LowCmdWrite
    ControlParams ctrl_params_;

    // ── ROS2 publishers ────────────────────────────────────────────────────
    // Cmd (one per target, written at 500 Hz)
    std::vector<rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr> lowcmd_publishers_;

    // State mirrors (published at 500 Hz from ros2_publish_thread)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr         pub_motor_state_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      pub_motor_ddq_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      pub_motor_temp_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      pub_motor_vol_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr       pub_motor_status_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                pub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr          pub_imu_rpy_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            pub_rc_cmd_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr                pub_tick_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                pub_debug_;

    // SDK2 monitoring publishers (battery, board, pressure)
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr               pub_bms_soc_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr               pub_bms_current_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr               pub_bms_voltage_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      pub_bms_temp_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      pub_bms_cells_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                pub_bms_state_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr       pub_board_fan_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      pub_board_temp_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      pub_pressure_;

    // ── ROS2 subscribers ───────────────────────────────────────────────────
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr         sub_lowstate_;
    rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr         sub_imu_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr sub_joy_;
    // Policy sends action here → LowCmdWrite picks it up via ctrl_params_
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   sub_cmd_action_;

    // ── TF ────────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_bc_;

    // ── CSV recording ────────────────────────────────────────────────────
    bool record_log_{ false };
    std::ofstream log_file_;
    std::string log_path_;
    std::mutex log_mutex_;

    // ─────────────────────────────────────────────────────────────────────
    // Message handlers — each stores incoming data into member buffers
    // ─────────────────────────────────────────────────────────────────────
    void LowStateMessageHandler(unitree_hg::msg::LowState::SharedPtr msg);
    void ImuMessageHandler(unitree_hg::msg::IMUState::SharedPtr msg);
    void JoystickHandler(unitree_go::msg::WirelessController::SharedPtr msg);
    void CmdActionHandler(std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // ─────────────────────────────────────────────────────────────────────
    // Thread 1 — ros2_publish
    // Reads low_state_ → publishes all mirror topics at 500 Hz
    // Equivalent to GO2's lcm_send()
    // ─────────────────────────────────────────────────────────────────────
    void ROS2Publish();

    // ─────────────────────────────────────────────────────────────────────
    // Thread 2 — ros2_receive
    // Spins rclcpp to handle incoming /g1/cmd_action
    // Equivalent to GO2's lcm_receive()
    // ─────────────────────────────────────────────────────────────────────
    void ROS2Receive();

    // ─────────────────────────────────────────────────────────────────────
    // Thread 3 — low_cmd_write
    // Builds LowCmd from ctrl_params_, calls safety filters, writes /lowcmd
    // Equivalent to GO2's LowCmdWrite()
    // ─────────────────────────────────────────────────────────────────────
    void LowCmdWrite();

    // Helpers
    void InitLowCmd();
    void SetDamping();
    void PublishStaticTF();
    void LogInfo(const std::string& s);
    void InitCsvLogger();
    void WriteCsvHeader();
    void RecordLogRow(const unitree_hg::msg::LowCmd& cmd,
                      const unitree_hg::msg::LowState& state,
                      const unitree_hg::msg::IMUState& secondary_imu,
                      bool has_secondary_imu);
    void PublishLowCmd();
    uint32_t CRC32(uint32_t* ptr, uint32_t len);
};
