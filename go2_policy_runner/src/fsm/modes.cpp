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
//   deploy/include/LinearInterpolator.h
//   deploy/include/isaaclab/envs/mdp/terminations.h
// Changes by unitree-go2-rl contributors (2026):
//   - policy inference runs in the control cycle (decimation) instead of a separate thread
//   - smoothstep interpolation option, parameter validation
//   - fix_stand: pose sequence selected by the previous mode
//   - tilt safety check evaluated on every control cycle

#include "go2_policy_runner/fsm/modes.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace go2_policy_runner::fsm
{

namespace
{
void require_size(const std::vector<float> & v, std::size_t n, const std::string & what)
{
  if (v.size() != n) {
    throw std::invalid_argument(what + " must have " + std::to_string(n) + " values, got " +
            std::to_string(v.size()));
  }
}
}  // namespace

double base_tilt(const RobotState & s)
{
  Eigen::Quaternionf q(s.quaternion[0], s.quaternion[1], s.quaternion[2], s.quaternion[3]);
  if (q.norm() < 1e-6f) {return 0.0;}
  q.normalize();
  const Eigen::Vector3f g = q.conjugate() * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
  return std::acos(std::clamp(-static_cast<double>(g.z()), -1.0, 1.0));
}

// ---------------------------------------------------------------------------------------------
// PassiveMode
// ---------------------------------------------------------------------------------------------
PassiveMode::PassiveMode(std::string name, std::size_t num_motors, std::vector<float> kd)
: Mode(std::move(name), num_motors), kd_(std::move(kd))
{
  require_size(kd_, num_motors_, "passive '" + name_ + "': kd");
}

void PassiveMode::enter(const ControlInput & in, JointCommand & cmd)
{
  run(in, cmd);
}

void PassiveMode::run(const ControlInput & in, JointCommand & cmd)
{
  for (std::size_t i = 0; i < num_motors_; ++i) {
    cmd[i].q = in.state.q[i];  // irrelevant with kp = 0, but keeps the command sane
    cmd[i].dq = 0.0f;
    cmd[i].kp = 0.0f;
    cmd[i].kd = kd_[i];
    cmd[i].tau = 0.0f;
  }
}

// ---------------------------------------------------------------------------------------------
// FixStandMode
// ---------------------------------------------------------------------------------------------
FixStandMode::FixStandMode(
  std::string name, std::size_t num_motors, std::vector<float> kp, std::vector<float> kd,
  std::vector<std::vector<float>> poses, std::vector<double> durations,
  Interpolation interpolation)
: Mode(std::move(name), num_motors), kp_(std::move(kp)), kd_(std::move(kd)),
  default_{std::move(poses), std::move(durations)}, interpolation_(interpolation)
{
  require_size(kp_, num_motors_, "fix_stand '" + name_ + "': kp");
  require_size(kd_, num_motors_, "fix_stand '" + name_ + "': kd");
  validate(default_, "fix_stand '" + name_ + "'");
  start_pose_.assign(num_motors_, 0.0f);
}

void FixStandMode::validate(const Sequence & s, const std::string & what) const
{
  if (s.poses.empty()) {throw std::invalid_argument(what + ": no poses");}
  if (s.durations.size() != s.poses.size()) {
    throw std::invalid_argument(what + ": durations must match poses");
  }
  for (std::size_t k = 0; k < s.poses.size(); ++k) {
    require_size(s.poses[k], num_motors_, what + ": pose " + std::to_string(k));
    if (s.durations[k] < 0.0) {throw std::invalid_argument(what + ": negative duration");}
  }
}

void FixStandMode::add_sequence_from(
  const std::string & previous_mode, std::vector<std::vector<float>> poses,
  std::vector<double> durations)
{
  Sequence s{std::move(poses), std::move(durations)};
  validate(s, "fix_stand '" + name_ + "' from '" + previous_mode + "'");
  by_source_[previous_mode] = std::move(s);
}

std::vector<std::string> FixStandMode::sequence_sources() const
{
  std::vector<std::string> out;
  for (const auto & kv : by_source_) {out.push_back(kv.first);}
  return out;
}

void FixStandMode::enter(const ControlInput & in, JointCommand & cmd)
{
  const auto it = by_source_.find(in.previous_mode);
  active_ = it == by_source_.end() ? &default_ : &it->second;
  start_pose_ = in.state.q;
  t0_ = in.time;
  for (std::size_t i = 0; i < num_motors_; ++i) {
    cmd[i].kp = kp_[i];
    cmd[i].kd = kd_[i];
    cmd[i].dq = 0.0f;
    cmd[i].tau = 0.0f;
  }
}

bool FixStandMode::finished(double time) const
{
  double total = 0.0;
  for (double d : active_->durations) {total += d;}
  return time - t0_ >= total;
}

void FixStandMode::run(const ControlInput & in, JointCommand & cmd)
{
  const auto & poses = active_->poses;
  const auto & durations = active_->durations;
  double t = in.time - t0_;
  const std::vector<float> * from = &start_pose_;
  const std::vector<float> * to = &poses.back();
  double alpha = 1.0;
  for (std::size_t k = 0; k < poses.size(); ++k) {
    to = &poses[k];
    if (t < durations[k]) {
      alpha = durations[k] > 0.0 ? t / durations[k] : 1.0;
      break;
    }
    t -= durations[k];
    from = &poses[k];
    alpha = 1.0;
  }
  if (interpolation_ == Interpolation::kSmooth) {
    alpha = alpha * alpha * (3.0 - 2.0 * alpha);  // smoothstep: zero velocity at both ends
  }
  for (std::size_t i = 0; i < num_motors_; ++i) {
    cmd[i].q = static_cast<float>((1.0 - alpha) * (*from)[i] + alpha * (*to)[i]);
    cmd[i].dq = 0.0f;
    cmd[i].kp = kp_[i];
    cmd[i].kd = kd_[i];
    cmd[i].tau = 0.0f;
  }
}

// ---------------------------------------------------------------------------------------------
// RlPolicyMode
// ---------------------------------------------------------------------------------------------
RlPolicyMode::RlPolicyMode(
  std::string name, std::size_t num_motors, std::unique_ptr<policy::PolicyEnv> env,
  double control_rate, double max_tilt)
: Mode(std::move(name), num_motors), env_(std::move(env)), max_tilt_(max_tilt)
{
  if (!env_) {throw std::invalid_argument("rl_policy '" + name_ + "': env is null");}
  if (control_rate <= 0.0) {throw std::invalid_argument("rl_policy: control_rate must be > 0");}
  const double ratio = env_->step_dt() * control_rate;
  decimation_ = std::max(1, static_cast<int>(std::lround(ratio)));
  if (std::fabs(ratio - decimation_) > 1e-3) {
    throw std::invalid_argument(
            "rl_policy '" + name_ + "': step_dt (" + std::to_string(env_->step_dt()) +
            " s) is not a multiple of the control period (1/" + std::to_string(control_rate) +
            " s); choose control_rate so that step_dt * control_rate is an integer");
  }
  kp_.assign(num_motors_, 0.0f);
  kd_.assign(num_motors_, 0.0f);
  const auto & d = env_->data();
  for (std::size_t i = 0; i < d.joint_stiffness.size() && i < num_motors_; ++i) {
    kp_[i] = d.joint_stiffness[i];
    kd_[i] = d.joint_damping[i];
  }
  hold_pose_.assign(num_motors_, 0.0f);
}

void RlPolicyMode::enter(const ControlInput & in, JointCommand & /*cmd*/)
{
  hold_pose_ = in.state.q;
  env_->reset(in.state, in.command);
  cycle_ = 0;  // run() is called in the same cycle -> first policy step happens immediately
}

void RlPolicyMode::run(const ControlInput & in, JointCommand & cmd)
{
  if (cycle_ % decimation_ == 0) {
    policy_step(in);
  }
  ++cycle_;
  write_command(cmd);
}

void RlPolicyMode::policy_step(const ControlInput & in)
{
  const auto t0 = std::chrono::steady_clock::now();
  env_->step(in.state, in.command);
  const auto t1 = std::chrono::steady_clock::now();
  if (on_policy_step) {
    PolicyStepInfo info;
    info.mode = name_;
    info.step = env_->episode_length();
    info.observations = &env_->observation_manager().last();
    info.action = &env_->action_manager().action();
    info.position_targets = &env_->position_targets_sdk();
    info.command = env_->data().command;
    info.inference_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    on_policy_step(info);
  }
}

void RlPolicyMode::write_command(JointCommand & cmd) const
{
  const auto & pos = env_->position_targets_sdk();
  const auto & vel = env_->velocity_targets_sdk();
  const auto & driven = env_->driven_sdk();
  for (std::size_t i = 0; i < num_motors_; ++i) {
    cmd[i].q = driven[i] ? pos[i] : hold_pose_[i];
    cmd[i].dq = driven[i] ? vel[i] : 0.0f;
    cmd[i].kp = kp_[i];
    cmd[i].kd = kd_[i];
    cmd[i].tau = 0.0f;
  }
}

std::optional<std::string> RlPolicyMode::check_safety(const ControlInput & in) const
{
  if (max_tilt_ > 0.0) {
    const double tilt = base_tilt(in.state);
    if (tilt > max_tilt_) {
      return "base tilt " + std::to_string(tilt) + " rad exceeds max_tilt " +
             std::to_string(max_tilt_) + " rad";
    }
  }
  return std::nullopt;
}

}  // namespace go2_policy_runner::fsm
