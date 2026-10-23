// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/FSM/BaseState.h
//   deploy/include/FSM/FSMState.h
// Changes by unitree-go2-rl contributors (2026):
//   - modes are pure functions of (state, command, time) -> joint command, no DDS/ROS inside
//   - transitions stored per mode with compiled remote conditions
//
// Controller modes (FSM states). A mode owns the joint command while it is active.

#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "go2_policy_runner/core/types.hpp"
#include "go2_policy_runner/remote/remote_dsl.hpp"

namespace go2_policy_runner::fsm
{

/// Everything a mode may look at during one control cycle.
struct ControlInput
{
  const RobotState & state;
  const VelocityCommand & command;
  double time;  ///< monotonic time [s]
};

class Mode
{
public:
  struct Transition
  {
    std::string target;
    std::string expression;     ///< remote DSL expression, empty = only via service
    RemoteCondition condition;  ///< compiled expression (may be empty)
  };

  Mode(std::string name, std::size_t num_motors)
  : name_(std::move(name)), num_motors_(num_motors) {}
  virtual ~Mode() = default;

  const std::string & name() const {return name_;}
  std::size_t num_motors() const {return num_motors_;}
  virtual std::string type() const = 0;

  /// Called once when the mode becomes active. Should set gains in `cmd`.
  virtual void enter(const ControlInput & /*in*/, JointCommand & /*cmd*/) {}
  /// Called every control cycle while active (also right after enter()).
  virtual void run(const ControlInput & in, JointCommand & cmd) = 0;
  /// Called once when the mode is left.
  virtual void exit() {}
  /// Mode-specific safety check; a returned reason aborts the mode (switch to the fallback mode).
  virtual std::optional<std::string> check_safety(const ControlInput & /*in*/) const {return {};}

  /// Add an allowed transition. `expression` may be empty (service-only transition).
  void add_transition(const std::string & target, const std::string & expression)
  {
    transitions_.push_back(
      {target, expression, expression.empty() ? RemoteCondition{} :
        compile_remote_condition(expression)});
  }
  const std::vector<Transition> & transitions() const {return transitions_;}
  bool allows(const std::string & target) const
  {
    for (const auto & t : transitions_) {
      if (t.target == target) {return true;}
    }
    return false;
  }

protected:
  std::string name_;
  std::size_t num_motors_;
  std::vector<Transition> transitions_;
};

}  // namespace go2_policy_runner::fsm
