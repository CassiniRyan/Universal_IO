#include "safety_filter_1.hpp"
#include <cmath>
#include <sstream>
#include <iostream>

// ─────────────────────────────────────────────────────────────────────────────
// Tunable limits — edit without touching G1_middleware.cpp
// ─────────────────────────────────────────────────────────────────────────────

// Global hard torque ceiling applied on top of per-joint TORQUE_LIMITS
static constexpr float GLOBAL_TAU_CAP = MAX_TORQUE_GLOBAL;  // from robot_config.hpp

// ─────────────────────────────────────────────────────────────────────────────

static void apply_zero(unitree_hg::msg::LowCmd& cmd) {
    for (int real = 0; real < NUM_JOINTS; ++real) {
        cmd.motor_cmd[real].mode = MOTOR_MODE_OFF;
        cmd.motor_cmd[real].q    = 0.f;
        cmd.motor_cmd[real].dq   = 0.f;
        cmd.motor_cmd[real].tau  = 0.f;
        cmd.motor_cmd[real].kp   = 0.f;
        cmd.motor_cmd[real].kd   = 0.f;
    }
}

bool safety_filter_1(unitree_hg::msg::LowCmd& cmd, std::string& reason_out) {
    for (int sim = 0; sim < NUM_JOINTS; ++sim) {
        int real = JOINT_MAP[sim];

        float q   = cmd.motor_cmd[real].q;
        float dq  = cmd.motor_cmd[real].dq;
        float tau = cmd.motor_cmd[real].tau;
        float kp  = cmd.motor_cmd[real].kp;
        float kd  = cmd.motor_cmd[real].kd;

        // NaN / Inf check
        if (!std::isfinite(q) || !std::isfinite(dq) ||
            !std::isfinite(tau) || !std::isfinite(kp) || !std::isfinite(kd)) {
            std::ostringstream oss;
            oss << "SF1: NaN/Inf in joint " << sim << " (real " << real << ")";
            reason_out = oss.str();
            std::cerr << "[SAFETY_1] " << reason_out << " → zero gains\n";
            apply_zero(cmd);
            return false;
        }

        // Per-joint torque ceiling
        if (std::abs(tau) > TORQUE_LIMITS[sim]) {
            std::ostringstream oss;
            oss << "SF1: joint " << sim << " tau=" << tau
                << " exceeds limit " << TORQUE_LIMITS[sim];
            reason_out = oss.str();
            std::cerr << "[SAFETY_1] " << reason_out << " → zero gains\n";
            apply_zero(cmd);
            return false;
        }

        // Global hard cap
        if (std::abs(tau) > GLOBAL_TAU_CAP) {
            reason_out = "SF1: global torque cap exceeded at joint " + std::to_string(sim);
            std::cerr << "[SAFETY_1] " << reason_out << " → zero gains\n";
            apply_zero(cmd);
            return false;
        }

        // Gain sanity bounds
        if (kp < 0.f || kp > 500.f || kd < 0.f || kd > 5.f) {
            std::ostringstream oss;
            oss << "SF1: joint " << sim << " gains out of range kp=" << kp << " kd=" << kd;
            reason_out = oss.str();
            std::cerr << "[SAFETY_1] " << reason_out << " → zero gains\n";
            apply_zero(cmd);
            return false;
        }
    }
    return true;
}
