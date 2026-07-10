// Record-only low-level logger.
//
// This node never publishes /lowcmd and never runs a control loop. It records
// observed firmware/control commands separately from sensed low-level state.

#include <array>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include "topics.hpp"

class LowLevelCsvRecorder : public rclcpp::Node {
public:
    LowLevelCsvRecorder()
    : rclcpp::Node("g1_lowlevel_csv_recorder") {
        declare_parameter("log_dir", "");

        InitCsvLoggers();

        auto qos = rclcpp::SensorDataQoS();
        pub_motor_state_ = create_publisher<sensor_msgs::msg::JointState>(topics::MOTOR_STATE, qos);
        pub_motor_ddq_ = create_publisher<std_msgs::msg::Float32MultiArray>(topics::MOTOR_DDQ, qos);
        pub_motor_temperature_ = create_publisher<std_msgs::msg::Float32MultiArray>(topics::MOTOR_TEMPERATURE, qos);
        pub_motor_voltage_ = create_publisher<std_msgs::msg::Float32MultiArray>(topics::MOTOR_VOLTAGE, qos);
        pub_motor_status_ = create_publisher<std_msgs::msg::UInt32MultiArray>(topics::MOTOR_STATUS, qos);
        pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(topics::IMU_OUT, qos);
        pub_imu_rpy_ = create_publisher<geometry_msgs::msg::Vector3>(topics::IMU_RPY, qos);
        pub_tick_ = create_publisher<std_msgs::msg::UInt32>(topics::HW_TICK, qos);

        sub_lowcmd_ = create_subscription<unitree_hg::msg::LowCmd>(
            topics::LOWCMD, qos,
            [this](unitree_hg::msg::LowCmd::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(log_mutex_);
                WriteLowCmdRecord(*msg);
                WriteMotorCmdRecord(*msg);
            });

        sub_secondary_imu_ = create_subscription<unitree_hg::msg::IMUState>(
            topics::SECONDARY_IMU, qos,
            [this](unitree_hg::msg::IMUState::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(log_mutex_);
                WriteImuRecord("secondary_imu", *msg, 0, false);
            });

        sub_lowstate_ = create_subscription<unitree_hg::msg::LowState>(
            topics::LOWSTATE, qos,
            [this](unitree_hg::msg::LowState::SharedPtr msg) {
                PublishMirrors(*msg);
                std::lock_guard<std::mutex> lk(log_mutex_);
                WriteLowStateRecord(*msg);
                WriteMotorStateRecord(*msg);
                WriteImuRecord("lowstate_imu", msg->imu_state, msg->tick, true);
            });

        RCLCPP_INFO(get_logger(),
            "Record-only logger active. Subscribing to %s, %s, and %s. No control commands are published.",
            topics::LOWSTATE, topics::SECONDARY_IMU, topics::LOWCMD);
    }

private:
    static constexpr size_t kActiveJoints = 29;
    static constexpr std::array<const char*, kActiveJoints> kJointNamesModePr0 = {
        "L_LEG_HIP_PITCH", "L_LEG_HIP_ROLL", "L_LEG_HIP_YAW", "L_LEG_KNEE",
        "L_LEG_ANKLE_PITCH", "L_LEG_ANKLE_ROLL",
        "R_LEG_HIP_PITCH", "R_LEG_HIP_ROLL", "R_LEG_HIP_YAW", "R_LEG_KNEE",
        "R_LEG_ANKLE_PITCH", "R_LEG_ANKLE_ROLL",
        "WAIST_YAW", "WAIST_ROLL", "WAIST_PITCH",
        "L_SHOULDER_PITCH", "L_SHOULDER_ROLL", "L_SHOULDER_YAW", "L_ELBOW",
        "L_WRIST_ROLL", "L_WRIST_PITCH", "L_WRIST_YAW",
        "R_SHOULDER_PITCH", "R_SHOULDER_ROLL", "R_SHOULDER_YAW", "R_ELBOW",
        "R_WRIST_ROLL", "R_WRIST_PITCH", "R_WRIST_YAW",
    };
    static constexpr std::array<const char*, kActiveJoints> kJointNamesModePr1 = {
        "L_LEG_HIP_PITCH", "L_LEG_HIP_ROLL", "L_LEG_HIP_YAW", "L_LEG_KNEE",
        "L_LEG_ANKLE_B", "L_LEG_ANKLE_A",
        "R_LEG_HIP_PITCH", "R_LEG_HIP_ROLL", "R_LEG_HIP_YAW", "R_LEG_KNEE",
        "R_LEG_ANKLE_B", "R_LEG_ANKLE_A",
        "WAIST_YAW", "WAIST_A", "WAIST_B",
        "L_SHOULDER_PITCH", "L_SHOULDER_ROLL", "L_SHOULDER_YAW", "L_ELBOW",
        "L_WRIST_ROLL", "L_WRIST_PITCH", "L_WRIST_YAW",
        "R_SHOULDER_PITCH", "R_SHOULDER_ROLL", "R_SHOULDER_YAW", "R_ELBOW",
        "R_WRIST_ROLL", "R_WRIST_PITCH", "R_WRIST_YAW",
    };

    rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr sub_lowcmd_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr sub_lowstate_;
    rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr sub_secondary_imu_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_motor_state_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_ddq_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_temperature_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motor_voltage_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr pub_motor_status_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_imu_rpy_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_tick_;

    std::mutex log_mutex_;
    std::filesystem::path session_dir_;
    std::ofstream imu_file_;
    std::ofstream lowcmd_file_;
    std::ofstream lowstate_file_;
    std::ofstream motor_cmd_file_;
    std::ofstream motor_state_file_;

    void InitCsvLoggers() {
        std::string log_dir = get_parameter("log_dir").as_string();
        if (log_dir.empty()) {
            const char* env_log_dir = std::getenv("G1_IO_LOG_DIR");
            log_dir = env_log_dir ? std::string(env_log_dir)
                                  : (std::filesystem::current_path() / "g1_logs").string();
        }

        std::filesystem::create_directories(log_dir);
        session_dir_ = std::filesystem::path(log_dir) / MakeSessionDirName();
        std::filesystem::create_directories(session_dir_);

        OpenCsv(imu_file_, "imu_state_record.csv");
        OpenCsv(lowcmd_file_, "lowcmd_record.csv");
        OpenCsv(lowstate_file_, "lowstate_record.csv");
        OpenCsv(motor_cmd_file_, "motor_cmd_record.csv");
        OpenCsv(motor_state_file_, "motor_state_record.csv");

        WriteHeaders();
        RCLCPP_INFO(get_logger(), "Recording session directory: %s", session_dir_.c_str());
    }

    std::string MakeSessionDirName() const {
        auto now_time = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now_time.time_since_epoch()) % 1000;
        std::time_t tt = std::chrono::system_clock::to_time_t(now_time);
        std::tm tm{};
        localtime_r(&tt, &tm);

        std::ostringstream name;
        name << "record_" << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S")
             << "_" << std::setw(3) << std::setfill('0') << ms.count();
        return name.str();
    }

    void OpenCsv(std::ofstream& file, const std::string& file_name) {
        const auto path = session_dir_ / file_name;
        file.open(path, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open CSV log: " + path.string());
        }
    }

    void WriteHeaders() {
        imu_file_ << "ros_time_ns,source,has_lowstate_tick,lowstate_tick,"
                  << "qw,qx,qy,qz,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,"
                  << "roll,pitch,yaw,temperature\n";

        lowcmd_file_ << "ros_time_ns,mode_pr,mode_machine,"
                     << "reserve0,reserve1,reserve2,reserve3,crc\n";

        lowstate_file_ << "ros_time_ns,version0,version1,mode_pr,mode_machine,tick,"
                       << "wireless_remote_hex,reserve0,reserve1,reserve2,reserve3,crc\n";

        motor_cmd_file_ << "ros_time_ns,mode_pr,mode_machine,joint_index,"
                        << "joint_name_mode_pr0,joint_name_mode_pr1,"
                        << "mode,q,dq,tau,kp,kd,reserve,lowcmd_crc\n";

        motor_state_file_ << "ros_time_ns,mode_pr,mode_machine,tick,joint_index,"
                          << "joint_name_mode_pr0,joint_name_mode_pr1,"
                          << "mode,q,dq,ddq,tau_est,temperature0,temperature1,vol,"
                          << "sensor0,sensor1,motorstate,reserve0,reserve1,reserve2,reserve3,"
                          << "lowstate_crc\n";
    }

    uint64_t NowNs() const {
        return static_cast<uint64_t>(now().nanoseconds());
    }

    std::string WirelessRemoteHex(const unitree_hg::msg::LowState& state) const {
        std::ostringstream out;
        out << std::hex << std::setfill('0');
        for (const auto byte : state.wireless_remote) {
            out << std::setw(2) << static_cast<unsigned int>(byte);
        }
        return out.str();
    }

    void WriteLowCmdRecord(const unitree_hg::msg::LowCmd& cmd) {
        lowcmd_file_ << NowNs()
                     << "," << static_cast<unsigned int>(cmd.mode_pr)
                     << "," << static_cast<unsigned int>(cmd.mode_machine);
        for (const auto reserve : cmd.reserve) {
            lowcmd_file_ << "," << reserve;
        }
        lowcmd_file_ << "," << cmd.crc << "\n";
    }

    void WriteMotorCmdRecord(const unitree_hg::msg::LowCmd& cmd) {
        const uint64_t stamp = NowNs();
        for (size_t i = 0; i < kActiveJoints; ++i) {
            const auto& motor = cmd.motor_cmd[i];
            motor_cmd_file_ << stamp
                            << "," << static_cast<unsigned int>(cmd.mode_pr)
                            << "," << static_cast<unsigned int>(cmd.mode_machine)
                            << "," << i
                            << "," << kJointNamesModePr0[i]
                            << "," << kJointNamesModePr1[i]
                            << "," << static_cast<unsigned int>(motor.mode)
                            << "," << motor.q
                            << "," << motor.dq
                            << "," << motor.tau
                            << "," << motor.kp
                            << "," << motor.kd
                            << "," << motor.reserve
                            << "," << cmd.crc
                            << "\n";
        }
    }

    void WriteLowStateRecord(const unitree_hg::msg::LowState& state) {
        lowstate_file_ << NowNs()
                       << "," << state.version[0]
                       << "," << state.version[1]
                       << "," << static_cast<unsigned int>(state.mode_pr)
                       << "," << static_cast<unsigned int>(state.mode_machine)
                       << "," << state.tick
                       << "," << WirelessRemoteHex(state);
        for (const auto reserve : state.reserve) {
            lowstate_file_ << "," << reserve;
        }
        lowstate_file_ << "," << state.crc << "\n";
    }

    void WriteMotorStateRecord(const unitree_hg::msg::LowState& state) {
        const uint64_t stamp = NowNs();
        for (size_t i = 0; i < kActiveJoints; ++i) {
            const auto& motor = state.motor_state[i];
            motor_state_file_ << stamp
                              << "," << static_cast<unsigned int>(state.mode_pr)
                              << "," << static_cast<unsigned int>(state.mode_machine)
                              << "," << state.tick
                              << "," << i
                              << "," << kJointNamesModePr0[i]
                              << "," << kJointNamesModePr1[i]
                              << "," << static_cast<unsigned int>(motor.mode)
                              << "," << motor.q
                              << "," << motor.dq
                              << "," << motor.ddq
                              << "," << motor.tau_est
                              << "," << motor.temperature[0]
                              << "," << motor.temperature[1]
                              << "," << motor.vol
                              << "," << motor.sensor[0]
                              << "," << motor.sensor[1]
                              << "," << motor.motorstate;
            for (const auto reserve : motor.reserve) {
                motor_state_file_ << "," << reserve;
            }
            motor_state_file_ << "," << state.crc << "\n";
        }
    }

    void WriteImuRecord(const std::string& source,
                        const unitree_hg::msg::IMUState& imu,
                        uint32_t lowstate_tick,
                        bool has_lowstate_tick) {
        imu_file_ << NowNs()
                  << "," << source
                  << "," << (has_lowstate_tick ? 1 : 0)
                  << "," << lowstate_tick
                  << "," << imu.quaternion[0]
                  << "," << imu.quaternion[1]
                  << "," << imu.quaternion[2]
                  << "," << imu.quaternion[3]
                  << "," << imu.gyroscope[0]
                  << "," << imu.gyroscope[1]
                  << "," << imu.gyroscope[2]
                  << "," << imu.accelerometer[0]
                  << "," << imu.accelerometer[1]
                  << "," << imu.accelerometer[2]
                  << "," << imu.rpy[0]
                  << "," << imu.rpy[1]
                  << "," << imu.rpy[2]
                  << "," << imu.temperature
                  << "\n";
    }

    void PublishMirrors(const unitree_hg::msg::LowState& state) {
        auto stamp = now();

        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = stamp;
        joint_state.name.resize(kActiveJoints);
        joint_state.position.resize(kActiveJoints);
        joint_state.velocity.resize(kActiveJoints);
        joint_state.effort.resize(kActiveJoints);

        std_msgs::msg::Float32MultiArray ddq;
        std_msgs::msg::Float32MultiArray temperature;
        std_msgs::msg::Float32MultiArray voltage;
        std_msgs::msg::UInt32MultiArray status;
        ddq.data.resize(kActiveJoints);
        temperature.data.resize(kActiveJoints * 2);
        voltage.data.resize(kActiveJoints);
        status.data.resize(kActiveJoints);

        const auto& joint_names = state.mode_pr == 1 ? kJointNamesModePr1 : kJointNamesModePr0;
        for (size_t i = 0; i < kActiveJoints; ++i) {
            const auto& motor = state.motor_state[i];
            joint_state.name[i] = joint_names[i];
            joint_state.position[i] = motor.q;
            joint_state.velocity[i] = motor.dq;
            joint_state.effort[i] = motor.tau_est;
            ddq.data[i] = motor.ddq;
            temperature.data[i * 2] = static_cast<float>(motor.temperature[0]);
            temperature.data[i * 2 + 1] = static_cast<float>(motor.temperature[1]);
            voltage.data[i] = motor.vol;
            status.data[i] = motor.motorstate;
        }

        pub_motor_state_->publish(joint_state);
        pub_motor_ddq_->publish(ddq);
        pub_motor_temperature_->publish(temperature);
        pub_motor_voltage_->publish(voltage);
        pub_motor_status_->publish(status);

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = "lowstate_imu";
        imu_msg.orientation.w = state.imu_state.quaternion[0];
        imu_msg.orientation.x = state.imu_state.quaternion[1];
        imu_msg.orientation.y = state.imu_state.quaternion[2];
        imu_msg.orientation.z = state.imu_state.quaternion[3];
        imu_msg.angular_velocity.x = state.imu_state.gyroscope[0];
        imu_msg.angular_velocity.y = state.imu_state.gyroscope[1];
        imu_msg.angular_velocity.z = state.imu_state.gyroscope[2];
        imu_msg.linear_acceleration.x = state.imu_state.accelerometer[0];
        imu_msg.linear_acceleration.y = state.imu_state.accelerometer[1];
        imu_msg.linear_acceleration.z = state.imu_state.accelerometer[2];
        pub_imu_->publish(imu_msg);

        geometry_msgs::msg::Vector3 rpy;
        rpy.x = state.imu_state.rpy[0];
        rpy.y = state.imu_state.rpy[1];
        rpy.z = state.imu_state.rpy[2];
        pub_imu_rpy_->publish(rpy);

        std_msgs::msg::UInt32 tick;
        tick.data = state.tick;
        pub_tick_->publish(tick);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowLevelCsvRecorder>());
    rclcpp::shutdown();
    return 0;
}
