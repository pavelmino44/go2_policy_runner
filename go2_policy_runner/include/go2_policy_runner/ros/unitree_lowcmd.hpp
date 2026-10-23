// Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
//
// Contains code derived from unitree_ros2 (https://github.com/unitreerobotics/unitree_ros2) (BSD 3-Clause License, see THIRD_PARTY_NOTICES.md):
//   example/src/include/common/motor_crc.h
//   example/src/src/common/motor_crc.cpp
// Changes by unitree-go2-rl contributors (2026):
//   - deterministic padding, static layout checks
//   - LowCmd initialization and LowState conversion helpers
//
// Robot-API specific helpers for unitree_go/LowCmd: initialization and CRC.
// The real robot rejects LowCmd messages with a wrong CRC, the simulator ignores it.

#pragma once

#include <cstdint>

#include "go2_policy_runner/core/types.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace go2_policy_runner::ros
{

/// "Stop" values for unused motors (motor ignores the position / velocity loop).
constexpr float kPosStop = 2.146e9f;
constexpr float kVelStop = 16000.0f;

/// Fill header fields and put every motor into a safe, passive command.
void init_lowcmd(unitree_go::msg::LowCmd & msg, std::size_t num_motors);

/// Write a joint command into the message (motors [0, cmd.size()) ).
void write_lowcmd(const JointCommand & cmd, unitree_go::msg::LowCmd & msg);

/// Compute and store msg.crc.
void update_lowcmd_crc(unitree_go::msg::LowCmd & msg);

/// CRC32 (poly 0x04C11DB7, MSB first) over `len` 32-bit words, as used by Unitree.
std::uint32_t unitree_crc32(const std::uint32_t * data, std::uint32_t len);

/// Convert unitree_go/LowState into the core RobotState (first `state.q.size()` motors).
void read_lowstate(const unitree_go::msg::LowState & msg, RobotState & state);

}  // namespace go2_policy_runner::ros
