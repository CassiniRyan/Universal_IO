#pragma once
#include <array>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// G1 29-DOF hardware configuration
// All arrays are indexed in SIMULATION ORDER (IsaacLab / MuJoCo order).
// Use JOINT_MAP[sim_idx] to get the Unitree SDK motor index.
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int NUM_JOINTS  = 29;
static constexpr int NUM_ACTIONS = 29;

// sim_idx → unitree SDK motor index
static constexpr std::array<int, NUM_JOINTS> JOINT_MAP = {
    15, 22,  // shoulder_pitch  L, R
    14,      // waist_pitch
    16, 23,  // shoulder_roll   L, R
    13,      // waist_roll
    17, 24,  // shoulder_yaw    L, R
    12,      // waist_yaw
    18, 25,  // elbow           L, R
     0,  6,  // hip_pitch       L, R
    19, 26,  // wrist_roll      L, R
     1,  7,  // hip_roll        L, R
    20, 27,  // wrist_pitch     L, R
     2,  8,  // hip_yaw         L, R
    21, 28,  // wrist_yaw       L, R
     3,  9,  // knee            L, R
     4, 10,  // ankle_pitch     L, R
     5, 11,  // ankle_roll      L, R
};

// +1 same direction as Unitree motor, -1 reversed
static constexpr std::array<float, NUM_JOINTS> JOINT_SIGNS = {
     1,  1, -1,
     1,  1, -1,
     1,  1, -1,
     1,  1,
     1,  1,
     1,  1,
     1,  1,
     1,  1,
     1,  1,
     1,  1,
     1,  1,
     1,  1,
     1,
};

static constexpr std::array<float, NUM_JOINTS> JOINT_LIMITS_HIGH = {
     2.6704f,  2.6704f,  0.5200f,
     2.2515f,  1.5882f,  0.5200f,
     2.6180f,  2.6180f,  2.6180f,
     2.0944f,  2.0944f,
     2.8798f,  2.8798f,
     1.9722f,  1.9722f,
     2.9671f,  0.5236f,
     1.6144f,  1.6144f,
     2.7576f,  2.7576f,
     1.6144f,  1.6144f,
     2.8798f,  2.8798f,
     0.5236f,  0.5236f,
     0.2618f,  0.2618f,
};

static constexpr std::array<float, NUM_JOINTS> JOINT_LIMITS_LOW = {
    -3.0892f, -3.0892f, -0.5200f,
    -1.5882f, -2.2515f, -0.5200f,
    -2.6180f, -2.6180f, -2.6180f,
    -1.0472f, -1.0472f,
    -2.5307f, -2.5307f,
    -1.9722f, -1.9722f,
    -0.5236f, -2.9671f,
    -1.6144f, -1.6144f,
    -2.7576f, -2.7576f,
    -1.6144f, -1.6144f,
    -0.0873f, -0.0873f,
    -0.8727f, -0.8727f,
    -0.2618f, -0.2618f,
};

// Per-joint torque limits (Nm), sim order
static constexpr std::array<float, NUM_JOINTS> TORQUE_LIMITS = {
     25.f,  25.f,  50.f,
     25.f,  25.f,  50.f,
     25.f,  25.f,  88.f,
     25.f,  25.f,
     88.f,  88.f,
     25.f,  25.f,
     88.f,  88.f,
      5.f,   5.f,
     88.f,  88.f,
      5.f,   5.f,
    139.f, 139.f,
     50.f,  50.f,
     50.f,  50.f,
};

static constexpr float JOINT_PROTECT_RATIO = 1.5f;   // guard band beyond limits
static constexpr float SAFETY_THRESHOLD    = 0.97f;  // soft-warn at 97%
static constexpr float MAX_VELOCITY        = 30.0f;  // rad/s
static constexpr float MAX_TORQUE_GLOBAL   = 150.0f; // Nm hard cap

static constexpr uint8_t MOTOR_MODE_ON  = 0x01;
static constexpr uint8_t MOTOR_MODE_OFF = 0x00;
static constexpr uint8_t MODE_PR        = 0;     // G1 LowCmd mode_pr field

// Wireless controller button bitmask (Unitree standard)
namespace btn {
    static constexpr uint32_t R1     = 1u <<  0;
    static constexpr uint32_t L1     = 1u <<  1;
    static constexpr uint32_t START  = 1u <<  2;
    static constexpr uint32_t SELECT = 1u <<  3;
    static constexpr uint32_t R2     = 1u <<  4;
    static constexpr uint32_t L2     = 1u <<  5;
    static constexpr uint32_t F1     = 1u <<  6;
    static constexpr uint32_t F2     = 1u <<  7;
    static constexpr uint32_t A      = 1u <<  8;
    static constexpr uint32_t B      = 1u <<  9;
    static constexpr uint32_t X      = 1u << 10;
    static constexpr uint32_t Y      = 1u << 11;
    static constexpr uint32_t UP     = 1u << 12;
    static constexpr uint32_t RIGHT  = 1u << 13;
    static constexpr uint32_t DOWN   = 1u << 14;
    static constexpr uint32_t LEFT   = 1u << 15;
} // namespace btn
