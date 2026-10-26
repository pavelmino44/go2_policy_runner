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
//   - modes are told which mode was active before them

#include "go2_policy_runner/fsm/mode_manager.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>

namespace go2_policy_runner::fsm
{

ModeManager::ModeManager(
  std::vector<std::unique_ptr<Mode>> modes, const std::string & initial,
  const std::string & fallback)
: modes_(std::move(modes))
{
  std::set<std::string> names;
  for (const auto & m : modes_) {
    if (!names.insert(m->name()).second) {
      throw std::invalid_argument("duplicate mode name '" + m->name() + "'");
    }
  }
  for (const auto & m : modes_) {
    for (const auto & t : m->transitions()) {
      if (!names.count(t.target)) {
        throw std::invalid_argument(
                "mode '" + m->name() + "' has a transition to unknown mode '" + t.target + "'");
      }
    }
  }
  initial_ = find(initial);
  fallback_ = find(fallback);
  if (!initial_) {throw std::invalid_argument("initial mode '" + initial + "' does not exist");}
  if (!fallback_) {throw std::invalid_argument("fallback mode '" + fallback + "' does not exist");}
  if (fallback_->type() != "passive") {
    throw std::invalid_argument("fallback mode '" + fallback + "' must be of type passive");
  }
}

Mode * ModeManager::find(const std::string & name)
{
  for (auto & m : modes_) {
    if (m->name() == name) {return m.get();}
  }
  return nullptr;
}

void ModeManager::switch_to(
  Mode * next, const std::string & reason, const ControlInput & in, JointCommand & cmd)
{
  const std::string from = current_ ? current_->name() : "";
  const ControlInput enter_in{in.state, in.command, in.time, from};
  if (current_) {current_->exit();}
  current_ = next;
  try {
    current_->enter(enter_in, cmd);
  } catch (const std::exception & e) {
    if (current_ == fallback_) {throw;}  // nothing safer left
    current_ = fallback_;
    current_->enter(enter_in, cmd);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_reason_ = "entering '" + next->name() + "' failed: " + e.what();
      current_name_ = current_->name();
      current_targets_.clear();
      for (const auto & t : current_->transitions()) {current_targets_.push_back(t.target);}
    }
    if (on_transition) {on_transition(from, current_->name(), last_transition_reason());}
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_reason_ = reason;
    current_name_ = current_->name();
    current_targets_.clear();
    for (const auto & t : current_->transitions()) {current_targets_.push_back(t.target);}
  }
  if (on_transition) {on_transition(from, current_->name(), reason);}
}

void ModeManager::update(
  const ControlInput & in, JointCommand & cmd, const std::optional<std::string> & fault)
{
  remote_.update(in.state.wireless_remote, in.time);

  if (!current_) {
    switch_to(fault ? fallback_ : initial_, fault ? *fault : "startup", in, cmd);
  }

  // --- decide the next mode (priority: fault > mode safety > external request > remote) ----
  Mode * next = nullptr;
  std::string reason;
  std::optional<std::pair<std::string, std::string>> pending;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending.swap(pending_);
  }

  if (fault) {
    if (current_ != fallback_) {next = fallback_; reason = *fault;}
  } else if (auto unsafe = current_->check_safety(in)) {
    if (current_ != fallback_) {next = fallback_; reason = *unsafe;}
  } else if (pending) {
    Mode * target = find(pending->first);
    // re-validate: the mode may have changed since the request was accepted
    if (target && target != current_ && (target == fallback_ || current_->allows(target->name()))) {
      next = target;
      reason = pending->second;
    }
  } else {
    for (const auto & t : current_->transitions()) {
      if (t.condition && t.condition(remote_)) {
        next = find(t.target);
        reason = "remote: " + t.expression;
        break;
      }
    }
  }

  if (next && next != current_) {switch_to(next, reason, in, cmd);}

  try {
    current_->run(in, cmd);
  } catch (const std::exception & e) {
    if (current_ == fallback_) {throw;}
    switch_to(fallback_, std::string("mode '") + current_->name() + "' failed: " + e.what(), in, cmd);
    current_->run(in, cmd);
  }
}

std::string ModeManager::request(const std::string & target, const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool exists = false;
  for (const auto & m : modes_) {exists = exists || m->name() == target;}
  if (!exists) {return "unknown mode '" + target + "'";}
  if (target == current_name_) {return "";}  // already there
  const bool allowed = target == fallback_->name() ||
    std::find(current_targets_.begin(), current_targets_.end(), target) != current_targets_.end();
  if (!allowed) {
    std::string msg = "transition '" + current_name_ + "' -> '" + target + "' is not allowed; allowed: [";
    for (std::size_t i = 0; i < current_targets_.size(); ++i) {
      msg += (i ? ", " : "") + current_targets_[i];
    }
    return msg + "]";
  }
  pending_ = std::make_pair(target, reason);
  return "";
}

std::string ModeManager::current_mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_name_;
}

std::vector<std::string> ModeManager::mode_names() const
{
  std::vector<std::string> names;
  for (const auto & m : modes_) {names.push_back(m->name());}
  return names;
}

std::vector<std::string> ModeManager::allowed_targets() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_targets_;
}

std::string ModeManager::last_transition_reason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_reason_;
}

}  // namespace go2_policy_runner::fsm
