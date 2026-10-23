// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/unitree_joystick_dsl.hpp
// Changes by unitree-go2-rl contributors (2026):
//   - see remote_dsl.cpp
//
// Small expression language for remote-controller conditions, compatible with unitree_rl_lab
// deploy, so FSM transitions can be copied from its config.yaml unchanged.
//
//   "A"                    A is held
//   "A.on_pressed"         rising edge of A (one control cycle)
//   "A.on_released"        falling edge of A
//   "LT(2s)"               LT held for at least 2 s   (also "LT(0.5s)", "LT(2)")
//   "LT + B.on_pressed"    AND
//   "X | Y"                OR
//   "!A", "!(A + B)"       NOT, grouping
//
// Key names (case-insensitive): back/select start LB/L1 RB/R1 LT/L2 RT/R2 A B X Y
//                               up down left right F1 F2 lx ly rx ry

#pragma once

#include <functional>
#include <string>

#include "go2_policy_runner/remote/unitree_remote.hpp"

namespace go2_policy_runner
{

using RemoteCondition = std::function<bool (const UnitreeRemote &)>;

/// Parse and compile an expression. Throws std::invalid_argument with a readable message on error.
RemoteCondition compile_remote_condition(const std::string & expression);

}  // namespace go2_policy_runner
