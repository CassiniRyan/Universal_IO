#include "safety_filter_0.hpp"
#include <cmath>
#include <sstream>
#include <iostream>

// ─────────────────────────────────────────────────────────────────────────────
// Tunable limits — edit these without touching G1_middleware.cpp
// ─────────────────────────────────────────────────────────────────────────────

// Per-joint position protect range = nominal_limit * PROTECT_RATIO
static constexpr float PROTECT_RATIO    = JOINT_PROTECT_RATIO;  // from robot_config.hpp
static constexpr float VEL_LIMIT        = MAX_VELOCITY;         // rad/s

// ─────────────────────────────────────────────────────────────────────────────

static void apply_damping(unitree_hg::msg::LowCmd& cmd) {
    for (int real = 0; real < NUM_JOINTS; ++real) {
        cmd.motor_cmd[real].mode = MOTOR_MODE_ON;
        cmd.motor_cmd[real].q    = 0.f;
        cmd.motor_cmd[real].dq   = 0.f;
        cmd.motor_cmd[real].tau  = 0.f;
        cmd.motor_cmd[real].kp   = 0.f;
        cmd.motor_cmd[real].kd   = 5.f;  // damping only
    }
}

bool safety_filter_0(
    unitree_hg::msg::LowCmd&          cmd,
    const unitree_hg::msg::LowState&  state,
    std::string&                      reason_out)
{
    for (int sim = 0; sim < NUM_JOINTS; ++sim) {
        int   real = JOINT_MAP[sim];
        float sign = JOINT_SIGNS[sim];

        // Current joint position in sim convention
        float q_current = state.motor_state[real].q * sign;

        // Protected range (wider than hard joint limits)
        float mid   = (JOINT_LIMITS_HIGH[sim] + JOINT_LIMITS_LOW[sim]) * 0.5f;
        float range = (JOINT_LIMITS_HIGH[sim] - JOINT_LIMITS_LOW[sim]) * 0.5f;
        float q_hi  = mid + range * PROTECT_RATIO;
        float q_lo  = mid - range * PROTECT_RATIO;

        // Check current state
        if (q_current > q_hi || q_current < q_lo) {
            std::ostringstream oss;
            oss << "SF0: joint " << sim << " (real " << real
                << ") pos=" << q_current
                << " outside [" << q_lo << ", " << q_hi << "]";
            reason_out = oss.str();
            std::cerr << "[SAFETY_0] " << reason_out << " → damping\n";
            apply_damping(cmd);
            return false;
        }

        // Check commanded position (convert back to sim convention for checking)
        float q_cmd = cmd.motor_cmd[real].q * sign;
        if (q_cmd > q_hi || q_cmd < q_lo) {
            std::ostringstream oss;
            oss << "SF0: joint " << sim << " (real " << real
                << ") cmd_q=" << q_cmd
                << " outside [" << q_lo << ", " << q_hi << "]";
            reason_out = oss.str();
            std::cerr << "[SAFETY_0] " << reason_out << " → damping\n";
            apply_damping(cmd);
            return false;
        }

        // Check commanded velocity
        float dq_cmd = std::abs(cmd.motor_cmd[real].dq);
        if (dq_cmd > VEL_LIMIT) {
            std::ostringstream oss;
            oss << "SF0: joint " << sim << " cmd_dq=" << dq_cmd
                << " exceeds " << VEL_LIMIT;
            reason_out = oss.str();
            std::cerr << "[SAFETY_0] " << reason_out << " → damping\n";
            apply_damping(cmd);
            return false;
        }
    }
    return true;
}
