// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/FSM/State_Passive.h
//   deploy/include/FSM/State_FixStand.h
//   deploy/include/FSM/State_RLBase.h
// Changes by unitree-go2-rl contributors (2026):
//   - see modes.cpp
//
// Built-in modes:
//   passive    - joint damping only (kp = 0). Safe state, also the fallback for every error.
//   fix_stand  - interpolate from the current pose through a list of key poses (stand up / sit down).
//   rl_policy  - run an exported IsaacLab policy (deploy.yaml + policy.onnx).

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "go2_policy_runner/fsm/mode.hpp"
#include "go2_policy_runner/policy/policy_env.hpp"

namespace go2_policy_runner::fsm
{

// ---------------------------------------------------------------------------------------------
class PassiveMode : public Mode
{
public:
  /// @param kd damping per motor (SDK order)
  PassiveMode(std::string name, std::size_t num_motors, std::vector<float> kd);

  std::string type() const override {return "passive";}
  void enter(const ControlInput & in, JointCommand & cmd) override;
  void run(const ControlInput & in, JointCommand & cmd) override;

private:
  std::vector<float> kd_;
};

// ---------------------------------------------------------------------------------------------
class FixStandMode : public Mode
{
public:
  enum class Interpolation { kLinear, kSmooth };

  /// @param poses      key poses (SDK order), reached one after another
  /// @param durations  time to reach each pose from the previous one [s] (same size as poses)
  FixStandMode(
    std::string name, std::size_t num_motors, std::vector<float> kp, std::vector<float> kd,
    std::vector<std::vector<float>> poses, std::vector<double> durations,
    Interpolation interpolation = Interpolation::kSmooth);

  std::string type() const override {return "fix_stand";}
  void enter(const ControlInput & in, JointCommand & cmd) override;
  void run(const ControlInput & in, JointCommand & cmd) override;

  /// True once the last pose has been reached.
  bool finished(double time) const;

private:
  std::vector<float> kp_, kd_;
  std::vector<std::vector<float>> poses_;
  std::vector<double> durations_;
  Interpolation interpolation_;
  std::vector<float> start_pose_;
  double t0_{0.0};
};

// ---------------------------------------------------------------------------------------------
/// Data published after every policy step (for logging / sim-to-real comparison).
struct PolicyStepInfo
{
  std::string mode;
  long step{0};
  const std::map<std::string, std::vector<float>> * observations{nullptr};
  const std::vector<float> * action{nullptr};             ///< raw network output
  const std::vector<float> * position_targets{nullptr};   ///< SDK order
  VelocityCommand command;
  double inference_ms{0.0};
};

class RlPolicyMode : public Mode
{
public:
  /// @param env           loaded policy environment
  /// @param control_rate  rate at which run() is called [Hz]; policy runs every
  ///                      round(step_dt * control_rate) cycles
  /// @param max_tilt      abort to fallback if the base tilts more than this [rad] (<=0 disables)
  RlPolicyMode(
    std::string name, std::size_t num_motors, std::unique_ptr<policy::PolicyEnv> env,
    double control_rate, double max_tilt);

  std::string type() const override {return "rl_policy";}
  void enter(const ControlInput & in, JointCommand & cmd) override;
  void run(const ControlInput & in, JointCommand & cmd) override;
  std::optional<std::string> check_safety(const ControlInput & in) const override;

  int decimation() const {return decimation_;}
  const policy::PolicyEnv & env() const {return *env_;}

  /// Optional observer called after every policy step (from the control thread).
  std::function<void(const PolicyStepInfo &)> on_policy_step;

private:
  void policy_step(const ControlInput & in);
  void write_command(JointCommand & cmd) const;

  std::unique_ptr<policy::PolicyEnv> env_;
  int decimation_{1};
  double max_tilt_;
  long cycle_{0};
  std::vector<float> kp_, kd_;  ///< SDK order, size num_motors
  std::vector<float> hold_pose_;  ///< pose held by motors the policy does not drive
};

/// Tilt of the base (angle between body z axis and world z axis) [rad].
double base_tilt(const RobotState & state);

}  // namespace go2_policy_runner::fsm
