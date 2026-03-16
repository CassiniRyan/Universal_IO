// ─────────────────────────────────────────────────────────────────────────────
// G1_middleware.cpp
//
// Concrete implementation — written in the same style as GO2 Cpp_middleware.cpp.
// Every handler, thread function, and Init/Loop is fully implemented here.
//
// Three threads (mirrors GO2 design, ROS2 topics replace LCM):
//   Thread 1  ROS2Publish()    — reads low_state_ → publishes mirror topics
//   Thread 2  ROS2Receive()    — spins rclcpp → feeds ctrl_params_ from policy
//   Thread 3  LowCmdWrite()    — 500 Hz, builds LowCmd, safety check, writes /lowcmd
//
// Safety logic: safety_filter_0.cpp / safety_filter_1.cpp  (not this file)
// Topic names:  topics.hpp
// Joint config: robot_config.hpp
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <random>

#include "G1_middleware.hpp"
#include "safety_filter_0.hpp"
#include "safety_filter_1.hpp"
#include "topics.hpp"

using namespace std::chrono_literals;

// ═════════════════════════════════════════════════════════════════════════════
// CRC32 — Unitree standard, do not edit
// ═════════════════════════════════════════════════════════════════════════════
uint32_t G1Middleware::CRC32(uint32_t* ptr, uint32_t len) {
    unsigned int xbit = 0, data = 0;
    unsigned int crc  = 0xFFFFFFFF;
    const unsigned int poly = 0x04c11db7;
    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (crc & 0x80000000) { crc <<= 1; crc ^= poly; } else { crc <<= 1; }
            if (data & xbit) crc ^= poly;
            xbit >>= 1;
        }
    }
    return crc;
}

// ═════════════════════════════════════════════════════════════════════════════
// Constructor — declare ROS2 parameters only; actual wiring happens in Init()
// ═════════════════════════════════════════════════════════════════════════════
G1Middleware::G1Middleware(const std::vector<CmdTarget>& targets,
                           const rclcpp::NodeOptions& opts)
: rclcpp::Node("g1_middleware", opts)
{
    declare_parameter("dryrun",   false);
    declare_parameter("use_sdk2", true);

    bool dryrun = get_parameter("dryrun").as_bool();

    auto sq   = rclcpp::SensorDataQoS();
    auto q10  = rclcpp::QoS(10);

    // ── /lowcmd publishers — one per CmdTarget ────────────────────────────
    for (const auto& t : targets) {
        std::string topic = dryrun
            ? t.topic + "_dryrun_" + std::to_string(
                std::mt19937(std::random_device{}())() % 65536)
            : t.topic;
        lowcmd_publishers_.push_back(
            create_publisher<unitree_hg::msg::LowCmd>(topic, 10));
        std::cout << "[G1Middleware] CMD target [" << t.description
                  << "] → " << topic << (dryrun ? "  (DRYRUN)" : "") << std::endl;
    }

    // ── Mirror publishers ──────────────────────────────────────────────────
    pub_motor_state_  = create_publisher<sensor_msgs::msg::JointState>(topics::MOTOR_STATE, sq);
    pub_motor_ddq_    = create_publisher<std_msgs::msg::Float32MultiArray>(topics::MOTOR_DDQ, sq);
    pub_motor_temp_   = create_publisher<std_msgs::msg::Float32MultiArray>(topics::MOTOR_TEMPERATURE, sq);
    pub_motor_vol_    = create_publisher<std_msgs::msg::Float32MultiArray>(topics::MOTOR_VOLTAGE, sq);
    pub_motor_status_ = create_publisher<std_msgs::msg::UInt32MultiArray>(topics::MOTOR_STATUS, sq);
    pub_imu_          = create_publisher<sensor_msgs::msg::Imu>(topics::IMU_OUT, sq);
    pub_imu_rpy_      = create_publisher<geometry_msgs::msg::Vector3>(topics::IMU_RPY, sq);
    pub_rc_cmd_       = create_publisher<geometry_msgs::msg::Twist>(topics::RC_CMD_OUT, q10);
    pub_tick_         = create_publisher<std_msgs::msg::UInt32>(topics::HW_TICK, q10);
    pub_debug_        = create_publisher<std_msgs::msg::String>(topics::DEBUG_LOG, q10);

    // ── SDK2 monitoring publishers ─────────────────────────────────────────
    pub_bms_soc_     = create_publisher<std_msgs::msg::Float32>(topics::BMS_SOC, q10);
    pub_bms_current_ = create_publisher<std_msgs::msg::Float32>(topics::BMS_CURRENT, q10);
    pub_bms_voltage_ = create_publisher<std_msgs::msg::Float32>(topics::BMS_VOLTAGE, q10);
    pub_bms_temp_    = create_publisher<std_msgs::msg::Float32MultiArray>(topics::BMS_TEMPERATURE, q10);
    pub_bms_cells_   = create_publisher<std_msgs::msg::Float32MultiArray>(topics::BMS_CELL_VOLTAGE, q10);
    pub_bms_state_   = create_publisher<std_msgs::msg::String>(topics::BMS_STATE, q10);
    pub_board_fan_   = create_publisher<std_msgs::msg::UInt16MultiArray>(topics::BOARD_FAN, q10);
    pub_board_temp_  = create_publisher<std_msgs::msg::Float32MultiArray>(topics::BOARD_TEMPERATURE, q10);
    pub_pressure_    = create_publisher<std_msgs::msg::Float32MultiArray>(topics::PRESS_SENSORS, q10);

    std::cout << "[G1Middleware] Node constructed." << std::endl;
}

G1Middleware::~G1Middleware() {}

// ═════════════════════════════════════════════════════════════════════════════
// Init — wire subscribers, init cmd buffer, broadcast static TF
// Called once before Loop()
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::Init() {
    first_run_ = true;
    InitLowCmd();

    auto sq  = rclcpp::SensorDataQoS();
    auto q10 = rclcpp::QoS(10);

    // ── Subscribers ────────────────────────────────────────────────────────
    sub_lowstate_ = create_subscription<unitree_hg::msg::LowState>(
        topics::LOWSTATE, sq,
        [this](unitree_hg::msg::LowState::SharedPtr msg) {
            LowStateMessageHandler(msg);
        });

    sub_imu_ = create_subscription<unitree_hg::msg::IMUState>(
        topics::SECONDARY_IMU, sq,
        [this](unitree_hg::msg::IMUState::SharedPtr msg) {
            ImuMessageHandler(msg);
        });

    sub_joy_ = create_subscription<unitree_go::msg::WirelessController>(
        topics::WIRELESS_CTRL, q10,
        [this](unitree_go::msg::WirelessController::SharedPtr msg) {
            JoystickHandler(msg);
        });

    // Policy sends action command here (Float32MultiArray, layout below)
    // Format: [q_des×29 | dq_des×29 | tau_des×29 | kp×29 | kd×29] = 145 floats
    sub_cmd_action_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        topics::CMD_ACTION, q10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            CmdActionHandler(msg);
        });

    // Static TF: torso_link → realsense_depth_link
    static_tf_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    PublishStaticTF();

    // Wait until first LowState arrives
    std::cout << "[G1Middleware] Init complete. Waiting for first /lowstate..." << std::endl;
    while (rclcpp::ok()) {
        rclcpp::spin_some(get_node_base_interface());
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (low_state_.tick != 0) break;
        std::this_thread::sleep_for(10ms);
    }
    std::cout << "[G1Middleware] /lowstate received. Ready." << std::endl;
    std::cout << "[G1Middleware] ROS2 <<<<------>>>> Unitree G1" << std::endl;
    std::cout << "[G1Middleware] -----------------------------------" << std::endl;
    std::cout << "[G1Middleware] Press L2+B for emergency damping." << std::endl;
}

// ═════════════════════════════════════════════════════════════════════════════
// Loop — spawn the three named threads (mirrors GO2 CreateRecurrentThreadEx)
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::Loop() {
    // Thread 1 — publish mirror topics at 500 Hz
    std::thread publish_thread([this]() {
        rclcpp::Rate rate(1.0 / dt_);
        while (rclcpp::ok()) {
            ROS2Publish();
            rate.sleep();
        }
    });
    publish_thread.detach();

    // Thread 2 — spin rclcpp to receive /g1/cmd_action
    std::thread receive_thread([this]() {
        ROS2Receive();
    });
    receive_thread.detach();

    // Thread 3 — 500 Hz LowCmd write (the control loop)
    std::thread cmd_thread([this]() {
        rclcpp::Rate rate(1.0 / dt_);
        while (rclcpp::ok()) {
            LowCmdWrite();
            rate.sleep();
        }
    });
    cmd_thread.detach();
}

// ═════════════════════════════════════════════════════════════════════════════
// Message Handler — /lowstate
// Stores latest hardware state. Called from Thread 2 (rclcpp spin).
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::LowStateMessageHandler(unitree_hg::msg::LowState::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    low_state_ = *msg;
    // Keep mode_machine in sync with cmd buffer
    low_cmd_.mode_machine = msg->mode_machine;
    RCLCPP_INFO_ONCE(get_logger(), "LowState received.");
}

// ═════════════════════════════════════════════════════════════════════════════
// Message Handler — /secondary_imu
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::ImuMessageHandler(unitree_hg::msg::IMUState::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    torso_imu_ = *msg;
    RCLCPP_INFO_ONCE(get_logger(), "Secondary IMU received.");
}

// ═════════════════════════════════════════════════════════════════════════════
// Message Handler — /wirelesscontroller
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::JoystickHandler(unitree_go::msg::WirelessController::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    joystick_    = *msg;
    key_.value   = static_cast<uint16_t>(msg->keys);

    // Map buttons to mode integer (same as GO2)
    if      (key_.components.A)     mode_ = 0;
    else if (key_.components.B)     mode_ = 1;
    else if (key_.components.X)     mode_ = 2;
    else if (key_.components.Y)     mode_ = 3;
    else if (key_.components.up)    mode_ = 4;
    else if (key_.components.right) mode_ = 5;
    else if (key_.components.down)  mode_ = 6;
    else if (key_.components.left)  mode_ = 7;
}

// ═════════════════════════════════════════════════════════════════════════════
// Message Handler — /g1/cmd_action (from policy script)
//
// Format (Float32MultiArray, 145 floats, sim order):
//   [0  .. 28]  q_des   joint position targets
//   [29 .. 57]  dq_des  joint velocity targets
//   [58 .. 86]  tau_des feedforward torques
//   [87 ..115]  kp      position gains
//   [116..144]  kd      velocity gains
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::CmdActionHandler(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < static_cast<size_t>(NUM_JOINTS * 5)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "CmdActionHandler: expected %d floats, got %zu — ignoring",
            NUM_JOINTS * 5, msg->data.size());
        return;
    }
    std::lock_guard<std::mutex> lk(ctrl_params_.motor_cmd_mutex);
    for (int i = 0; i < NUM_JOINTS; i++) {
        ctrl_params_.q_des[i]   = msg->data[i];
        ctrl_params_.dq_des[i]  = msg->data[NUM_JOINTS + i];
        ctrl_params_.tau_des[i] = msg->data[NUM_JOINTS * 2 + i];
        ctrl_params_.kp[i]      = msg->data[NUM_JOINTS * 3 + i];
        ctrl_params_.kd[i]      = msg->data[NUM_JOINTS * 4 + i];
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Thread 1 — ROS2Publish
// Reads low_state_, torso_imu_, joystick_ → publishes all mirror topics.
// Mirrors GO2's lcm_send().
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::ROS2Publish() {
    // Take a local copy so the lock is held only briefly
    unitree_hg::msg::LowState  ls;
    unitree_hg::msg::IMUState  imu;
    unitree_go::msg::WirelessController joy;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        ls  = low_state_;
        imu = torso_imu_;
        joy = joystick_;
    }

    auto stamp = now();

    // ── /g1/motor_state — JointState (q, dq, tau_est) ────────────────────
    {
        sensor_msgs::msg::JointState js;
        js.header.stamp = stamp;
        js.position.resize(NUM_JOINTS);
        js.velocity.resize(NUM_JOINTS);
        js.effort.resize(NUM_JOINTS);
        for (int sim = 0; sim < NUM_JOINTS; sim++) {
            int   real = JOINT_MAP[sim];
            float sign = JOINT_SIGNS[sim];
            js.position[sim] = ls.motor_state[real].q       * sign;
            js.velocity[sim] = ls.motor_state[real].dq      * sign;
            js.effort[sim]   = ls.motor_state[real].tau_est * sign;
        }
        pub_motor_state_->publish(js);
    }

    // ── /g1/motor_ddq — joint acceleration ────────────────────────────────
    {
        std_msgs::msg::Float32MultiArray ddq;
        ddq.data.resize(NUM_JOINTS);
        for (int sim = 0; sim < NUM_JOINTS; sim++)
            ddq.data[sim] = ls.motor_state[JOINT_MAP[sim]].ddq * JOINT_SIGNS[sim];
        pub_motor_ddq_->publish(ddq);
    }

    // ── /g1/motor_temperature — winding temperature (°C) ─────────────────
    {
        std_msgs::msg::Float32MultiArray temp;
        temp.data.resize(NUM_JOINTS);
        for (int sim = 0; sim < NUM_JOINTS; sim++)
            // temperature[0] = winding, temperature[1] = driver, stored as ×0.1 °C
            temp.data[sim] = ls.motor_state[JOINT_MAP[sim]].temperature[0] * 0.1f;
        pub_motor_temp_->publish(temp);
    }

    // ── /g1/motor_voltage — motor bus voltage ─────────────────────────────
    {
        std_msgs::msg::Float32MultiArray vol;
        vol.data.resize(NUM_JOINTS);
        for (int sim = 0; sim < NUM_JOINTS; sim++)
            vol.data[sim] = ls.motor_state[JOINT_MAP[sim]].vol;
        pub_motor_vol_->publish(vol);
    }

    // ── /g1/motor_status — fault / error flags ────────────────────────────
    {
        std_msgs::msg::UInt32MultiArray ms;
        ms.data.resize(NUM_JOINTS);
        for (int sim = 0; sim < NUM_JOINTS; sim++)
            ms.data[sim] = ls.motor_state[JOINT_MAP[sim]].motorstate;
        pub_motor_status_->publish(ms);
    }

    // ── /g1/imu — full IMU ────────────────────────────────────────────────
    {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp    = stamp;
        imu_msg.header.frame_id = "torso_link";
        // Use torso_imu if available, fall back to built-in imu_state
        const auto& q   = imu.quaternion[0] != 0 ? imu.quaternion
                                                  : ls.imu_state.quaternion;
        const auto& gyr = imu.quaternion[0] != 0 ? imu.gyroscope
                                                  : ls.imu_state.gyroscope;
        const auto& acc = imu.quaternion[0] != 0 ? imu.accelerometer
                                                  : ls.imu_state.accelerometer;
        imu_msg.orientation.w = q[0]; imu_msg.orientation.x = q[1];
        imu_msg.orientation.y = q[2]; imu_msg.orientation.z = q[3];
        imu_msg.angular_velocity.x = gyr[0];
        imu_msg.angular_velocity.y = gyr[1];
        imu_msg.angular_velocity.z = gyr[2];
        imu_msg.linear_acceleration.x = acc[0];
        imu_msg.linear_acceleration.y = acc[1];
        imu_msg.linear_acceleration.z = acc[2];
        pub_imu_->publish(imu_msg);
    }

    // ── /g1/imu_rpy — roll pitch yaw ──────────────────────────────────────
    {
        geometry_msgs::msg::Vector3 rpy_msg;
        const auto& r = imu.quaternion[0] != 0 ? imu.rpy : ls.imu_state.rpy;
        rpy_msg.x = r[0]; rpy_msg.y = r[1]; rpy_msg.z = r[2];
        pub_imu_rpy_->publish(rpy_msg);
    }

    // ── /g1/rc_cmd — joystick axes as Twist ──────────────────────────────
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = joy.ly;   // forward
        twist.linear.y  = joy.lx;   // strafe
        twist.linear.z  = 0.0f;
        twist.angular.x = 0.0f;
        twist.angular.y = joy.ry;
        twist.angular.z = joy.rx;   // yaw
        pub_rc_cmd_->publish(twist);
    }

    // ── /g1/tick — hardware timestamp counter ─────────────────────────────
    {
        std_msgs::msg::UInt32 tick_msg;
        tick_msg.data = ls.tick;
        pub_tick_->publish(tick_msg);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Thread 2 — ROS2Receive
// Spins rclcpp so all message handler callbacks fire.
// Equivalent to GO2's lcm_receive() — runs until shutdown.
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::ROS2Receive() {
    rclcpp::spin(this->shared_from_this());
}

// ═════════════════════════════════════════════════════════════════════════════
// Thread 3 — LowCmdWrite
// 500 Hz. Builds LowCmd from ctrl_params_, calls safety filters, writes /lowcmd.
// Mirrors GO2's LowCmdWrite() exactly.
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::LowCmdWrite() {
    motiontime_++;

    // ── First run: latch current position to prevent startup jerk ─────────
    if (first_run_) {
        std::lock_guard<std::mutex> ls_lk(state_mutex_);
        if (low_state_.motor_state[0].q != 0.0f) {
            std::lock_guard<std::mutex> cp_lk(ctrl_params_.motor_cmd_mutex);
            for (int sim = 0; sim < NUM_JOINTS; sim++) {
                int   real = JOINT_MAP[sim];
                float sign = JOINT_SIGNS[sim];
                ctrl_params_.q_des[sim]   = low_state_.motor_state[real].q * sign;
                ctrl_params_.dq_des[sim]  = 0.0f;
                ctrl_params_.tau_des[sim] = 0.0f;
                ctrl_params_.kp[sim]      = 0.0f;
                ctrl_params_.kd[sim]      = 5.0f;  // damping only on first run
            }
            first_run_ = false;
            std::cout << "[G1Middleware] First run: position latched." << std::endl;
        }
        return;
    }

    // ── Emergency stop: L2+B → damping mode ───────────────────────────────
    bool l2b_pressed;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        l2b_pressed = (key_.components.L2 == 1 && key_.components.B == 1);
    }
    if (l2b_pressed) {
        SetDamping();
        std::cout << "[G1Middleware] ===== L2+B: Damping mode =====" << std::endl;
        for (auto& pub : lowcmd_publishers_) pub->publish(low_cmd_);
        return;
    }

    // ── Copy ctrl_params_ into low_cmd_ ───────────────────────────────────
    {
        std::lock_guard<std::mutex> cp_lk(ctrl_params_.motor_cmd_mutex);
        for (int sim = 0; sim < NUM_JOINTS; sim++) {
            int   real = JOINT_MAP[sim];
            float sign = JOINT_SIGNS[sim];
            if (!get_parameter("dryrun").as_bool())
                low_cmd_.motor_cmd[real].mode = MOTOR_MODE_ON;
            low_cmd_.motor_cmd[real].q   = ctrl_params_.q_des[sim]   * sign;
            low_cmd_.motor_cmd[real].dq  = ctrl_params_.dq_des[sim]  * sign;
            low_cmd_.motor_cmd[real].tau = ctrl_params_.tau_des[sim] * sign;
            low_cmd_.motor_cmd[real].kp  = ctrl_params_.kp[sim];
            low_cmd_.motor_cmd[real].kd  = ctrl_params_.kd[sim];
        }
    }

    // ── Safety Filter 0: joint position / velocity guard ──────────────────
    {
        unitree_hg::msg::LowState ls_copy;
        { std::lock_guard<std::mutex> lk(state_mutex_); ls_copy = low_state_; }
        std::string reason;
        if (!safety_filter_0(low_cmd_, ls_copy, reason)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                "Safety_0: %s", reason.c_str());
            // low_cmd_ replaced with damping command by the filter
            for (auto& pub : lowcmd_publishers_) pub->publish(low_cmd_);
            return;
        }
    }

    // ── Safety Filter 1: NaN / torque / gain guard ────────────────────────
    {
        std::string reason;
        if (!safety_filter_1(low_cmd_, reason)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                "Safety_1: %s", reason.c_str());
            // low_cmd_ zeroed by the filter
            for (auto& pub : lowcmd_publishers_) pub->publish(low_cmd_);
            return;
        }
    }

    // ── CRC + publish to ALL registered targets ────────────────────────────
    low_cmd_.crc = CRC32(
        reinterpret_cast<uint32_t*>(&low_cmd_),
        (sizeof(unitree_hg::msg::LowCmd) >> 2) - 1);

    for (auto& pub : lowcmd_publishers_) {
        pub->publish(low_cmd_);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// InitLowCmd — set initial cmd buffer (mirrors GO2 InitLowCmd)
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::InitLowCmd() {
    low_cmd_.mode_pr = MODE_PR;
    for (int i = 0; i < NUM_JOINTS; i++) {
        low_cmd_.motor_cmd[i].mode = MOTOR_MODE_OFF;
        low_cmd_.motor_cmd[i].q   = 0.0f;
        low_cmd_.motor_cmd[i].dq  = 0.0f;
        low_cmd_.motor_cmd[i].tau = 0.0f;
        low_cmd_.motor_cmd[i].kp  = 0.0f;
        low_cmd_.motor_cmd[i].kd  = 0.0f;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// SetDamping — zero torques, zero position, kd=5 (safe compliance mode)
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::SetDamping() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        low_cmd_.motor_cmd[i].mode = MOTOR_MODE_ON;
        low_cmd_.motor_cmd[i].q   = 0.0f;
        low_cmd_.motor_cmd[i].dq  = 0.0f;
        low_cmd_.motor_cmd[i].tau = 0.0f;
        low_cmd_.motor_cmd[i].kp  = 0.0f;
        low_cmd_.motor_cmd[i].kd  = 5.0f;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// PublishStaticTF — torso_link → realsense_depth_link (G1 URDF calibration)
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::PublishStaticTF() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now();
    tf.header.frame_id = "torso_link";
    tf.child_frame_id  = "realsense_depth_link";
    tf.transform.translation.x =  0.0513;
    tf.transform.translation.y =  0.015;
    tf.transform.translation.z =  0.4571;
    tf.transform.rotation.w    =  0.9136;
    tf.transform.rotation.x    =  0.0044;
    tf.transform.rotation.y    =  0.4067;
    tf.transform.rotation.z    =  0.0;
    static_tf_bc_->sendTransform(tf);
}

// ═════════════════════════════════════════════════════════════════════════════
// LogInfo — publish to /g1/debug and console
// ═════════════════════════════════════════════════════════════════════════════
void G1Middleware::LogInfo(const std::string& s) {
    std_msgs::msg::String m; m.data = s;
    pub_debug_->publish(m);
    RCLCPP_INFO(get_logger(), "%s", s.c_str());
}

// ═════════════════════════════════════════════════════════════════════════════
// main — entry point, mirrors GO2 main()
// Usage:
//   ros2 run g1_io cpp_middleware                         # real robot
//   ros2 run g1_io cpp_middleware --targets real isaac    # real + Isaac
//   ros2 run g1_io cpp_middleware --targets isaac mujoco  # Isaac + MuJoCo
// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char* argv[]) {
    std::cout << "G1 IO Middleware" << std::endl;
    std::cout << "Communication level: LOW-LEVEL (500 Hz)" << std::endl;
    std::cout << "WARNING: Make sure the robot is suspended or in a safe state." << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    rclcpp::init(argc, argv);

    // Parse --targets from argv
    std::vector<std::string> target_names;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--targets") {
            for (++i; i < argc && argv[i][0] != '-'; i++)
                target_names.push_back(argv[i]);
        }
    }
    if (target_names.empty()) target_names.push_back("real");

    std::vector<CmdTarget> targets;
    for (const auto& n : target_names) {
        if      (n == "real")   targets.push_back({topics::LOWCMD, "real robot"});
        else if (n == "isaac")  targets.push_back({std::string(topics::LOWCMD) + "_isaac",  "Isaac Sim"});
        else if (n == "mujoco") targets.push_back({std::string(topics::LOWCMD) + "_mujoco", "MuJoCo"});
        else                    targets.push_back({n, n});
    }

    auto node = std::make_shared<G1Middleware>(targets);
    node->Init();

    std::cout << "[G1Middleware] Communication ready." << std::endl;
    std::cout << "[G1Middleware] ROS2 <<<<------>>>> Unitree G1" << std::endl;
    std::cout << "[G1Middleware] -----------------------------------" << std::endl;
    std::cout << "[G1Middleware] Press L2+B for emergency damping." << std::endl;

    node->Loop();

    // Main thread keeps spinning
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    rclcpp::shutdown();
    return 0;
}
