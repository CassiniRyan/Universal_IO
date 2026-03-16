#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// safety_filter_0 — joint position and velocity guard
//
// Called inside LowCmdWrite() BEFORE every write.
// If a violation is detected the command is replaced with a damping command
// and the function returns false (caller must NOT send the original command).
//
// EDIT THIS FILE to tune limits without touching the comms loop.
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <string>
#include "robot_config.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"

// Returns true  → command is safe, write it.
// Returns false → violation detected; cmd has been overwritten with a damping
//                 command; caller should still write it (safe shutdown).
bool safety_filter_0(
    unitree_hg::msg::LowCmd&              cmd,        // in/out — may be overwritten
    const unitree_hg::msg::LowState&      state,
    std::string&                          reason_out  // human-readable reason if false
);
