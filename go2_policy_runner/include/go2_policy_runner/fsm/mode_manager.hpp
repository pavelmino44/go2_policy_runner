// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/FSM/CtrlFSM.h
// Changes by unitree-go2-rl contributors (2026):
//   - no internal thread: update() is driven by the caller
//   - fault input, thread-safe external transition requests, exception fallback
//
// The controller FSM. Owns the modes and the remote-controller state and produces one joint
// command per control cycle. Thread-safety: update() must be called from a single (control)
// thread; request()/current_mode()/allowed_targets() may be called from any thread.

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "go2_policy_runner/fsm/mode.hpp"
#include "go2_policy_runner/remote/unitree_remote.hpp"

namespace go2_policy_runner::fsm
{

class ModeManager
{
public:
  /// @param modes     all modes (names must be unique, transition targets must exist)
  /// @param initial   mode entered on the first update()
  /// @param fallback  safe mode (passive); always reachable, used on any fault
  ModeManager(
    std::vector<std::unique_ptr<Mode>> modes, const std::string & initial,
    const std::string & fallback);

  /// Run one control cycle: update remote, evaluate transitions, run the active mode.
  /// @param fault  if set, the FSM switches to the fallback mode with this reason
  ///               (and stays there while the fault persists)
  void update(const ControlInput & in, JointCommand & cmd,
    const std::optional<std::string> & fault = std::nullopt);

  /// External transition request (e.g. ROS service). Validated against the transition graph
  /// of the current mode; the fallback mode is always allowed. Applied on the next update().
  /// @return empty string on success, otherwise the reason of rejection
  std::string request(const std::string & target, const std::string & reason);

  std::string current_mode() const;
  std::vector<std::string> mode_names() const;
  std::vector<std::string> allowed_targets() const;
  std::string last_transition_reason() const;
  const UnitreeRemote & remote() const {return remote_;}
  Mode * find(const std::string & name);

  /// Called (from the control thread) after every transition: (from, to, reason).
  std::function<void(const std::string &, const std::string &, const std::string &)> on_transition;

private:
  void switch_to(Mode * next, const std::string & reason, const ControlInput & in, JointCommand & cmd);

  std::vector<std::unique_ptr<Mode>> modes_;
  Mode * initial_{nullptr};
  Mode * fallback_{nullptr};
  Mode * current_{nullptr};
  UnitreeRemote remote_;

  mutable std::mutex mutex_;  // guards the fields below
  std::string current_name_;
  std::vector<std::string> current_targets_;
  std::string last_reason_;
  std::optional<std::pair<std::string, std::string>> pending_;  // (target, reason)
};

}  // namespace go2_policy_runner::fsm
