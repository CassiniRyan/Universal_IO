#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// safety_filter_1 — NaN / Inf, torque ceiling, and command-age watchdog
//
// Called inside LowCmdWrite() AFTER safety_filter_0.
// If a violation is detected the command is zeroed (zero gains, zero torques)
// and the function returns false.
//
// EDIT THIS FILE to tune limits without touching the comms loop.
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <string>
#include "robot_config.hpp"
#include "unitree_hg/msg/low_cmd.hpp"

// Returns true  → command is safe.
// Returns false → violation; cmd zeroed; caller should write the zeroed cmd.
bool safety_filter_1(
    unitree_hg::msg::LowCmd& cmd,
    std::string&             reason_out
);
