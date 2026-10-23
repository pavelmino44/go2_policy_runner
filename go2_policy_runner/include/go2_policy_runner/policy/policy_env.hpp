// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/isaaclab/envs/manager_based_rl_env.h
//   deploy/include/param.h
// Changes by unitree-go2-rl contributors (2026):
//   - rewritten without global state; explicit validation of deploy.yaml against the ONNX model
//   - joint targets are computed in SDK order for all action terms
//   - policy directory resolution moved here from param.h
//
// Minimal re-implementation of IsaacLab's ManagerBasedRLEnv for deployment, driven by the
// params/deploy.yaml that unitree_rl_lab exports next to exported/policy.onnx.

#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "go2_policy_runner/core/types.hpp"
#include "go2_policy_runner/policy/action_manager.hpp"
#include "go2_policy_runner/policy/articulation.hpp"
#include "go2_policy_runner/policy/observation_manager.hpp"
#include "go2_policy_runner/policy/policy_model.hpp"

namespace go2_policy_runner::policy
{

class PolicyEnv
{
public:
  /// @param cfg         parsed deploy.yaml
  /// @param model       inference backend
  /// @param num_motors  number of motors in RobotState (12 for Go2)
  PolicyEnv(const YAML::Node & cfg, std::unique_ptr<PolicyModel> model, std::size_t num_motors);

  /// Start an episode from the current state (fills observation history, zeroes last action).
  void reset(const RobotState & state, const VelocityCommand & cmd);
  /// One policy step: update data -> observations -> inference -> process actions.
  void step(const RobotState & state, const VelocityCommand & cmd);

  /// Joint targets in SDK motor order (size num_motors). Joints not driven by any action term
  /// keep their default position / zero velocity.
  const std::vector<float> & position_targets_sdk() const {return position_targets_sdk_;}
  const std::vector<float> & velocity_targets_sdk() const {return velocity_targets_sdk_;}
  /// Which SDK motors are driven by the policy (true) - others should be held by the caller.
  const std::vector<bool> & driven_sdk() const {return driven_sdk_;}

  float step_dt() const {return step_dt_;}
  long episode_length() const {return episode_length_;}

  const YAML::Node & cfg() const {return cfg_;}
  ArticulationData & data() {return data_;}
  const ArticulationData & data() const {return data_;}
  ObservationManager & observation_manager() {return *observation_manager_;}
  ActionManager & action_manager() {return *action_manager_;}
  const ActionManager & action_manager() const {return *action_manager_;}

  /// Phase used by the gait_phase observation term.
  float global_phase{0.0f};

private:
  void update_targets();

  YAML::Node cfg_;
  float step_dt_{0.02f};
  long episode_length_{0};
  std::size_t num_motors_;
  ArticulationData data_;
  std::unique_ptr<ActionManager> action_manager_;
  std::unique_ptr<ObservationManager> observation_manager_;
  std::unique_ptr<PolicyModel> model_;
  std::vector<float> position_targets_sdk_, velocity_targets_sdk_;
  std::vector<bool> driven_sdk_;
};

/// Result of resolving a policy directory (see resolve_policy_dir).
struct PolicyFiles
{
  std::filesystem::path dir;
  std::filesystem::path deploy_yaml;  ///< <dir>/params/deploy.yaml
  std::filesystem::path onnx;         ///< <dir>/exported/policy.onnx
};

/// Resolve a policy directory the same way unitree_rl_lab deploy does: if `dir` has no
/// `exported/` sub-folder, pick the lexicographically last sub-directory that has one
/// (i.e. the latest run in logs/rsl_rl/<experiment>/). Throws if nothing is found.
PolicyFiles resolve_policy_dir(const std::filesystem::path & dir);

/// Convenience: resolve + load deploy.yaml + ONNX model.
std::unique_ptr<PolicyEnv> load_policy_env(
  const std::filesystem::path & dir, std::size_t num_motors, const OnnxOptions & options = {});

}  // namespace go2_policy_runner::policy
