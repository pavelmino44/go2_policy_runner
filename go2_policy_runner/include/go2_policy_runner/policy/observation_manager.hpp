// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/isaaclab/manager/observation_manager.h
//   deploy/include/isaaclab/manager/manager_term_cfg.h
// Changes by unitree-go2-rl contributors (2026):
//   - ordered groups, explicit term registry instead of static registrars
//   - size checks with readable errors
//
// Observation manager reproducing IsaacLab semantics (clip -> scale, per-term history,
// optional gym-style history interleaving).

#pragma once

#include <yaml-cpp/yaml.h>

#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace go2_policy_runner::policy
{

class PolicyEnv;

/// Observation term function: (env, term params from deploy.yaml) -> values.
using ObservationFunc = std::function<std::vector<float>(PolicyEnv &, const YAML::Node &)>;

/// Register a custom observation term (call before constructing a PolicyEnv).
void register_observation(const std::string & name, ObservationFunc func);
/// Names of all registered observation terms.
std::vector<std::string> registered_observations();

struct ObservationTerm
{
  std::string name;
  YAML::Node params;
  ObservationFunc func;
  std::vector<float> clip;   ///< empty or {min, max}
  std::vector<float> scale;  ///< empty or one value per element
  int history_length{1};
  bool scale_first{false};

  void reset(const std::vector<float> & obs);
  void add(std::vector<float> obs);
  /// History entry n (0 = oldest).
  const std::vector<float> & get(std::size_t n) const {return buffer_[n];}
  /// All history entries, oldest first, concatenated.
  std::vector<float> flattened() const;
  std::size_t dim() const {return dim_;}

private:
  std::deque<std::vector<float>> buffer_;
  std::size_t dim_{0};
};

class ObservationManager
{
public:
  ObservationManager(const YAML::Node & cfg, PolicyEnv & env);

  void reset();
  /// Compute all groups (adds one history entry per term).
  const std::map<std::string, std::vector<float>> & compute();
  /// Result of the last compute().
  const std::map<std::string, std::vector<float>> & last() const {return last_;}
  /// Flattened size of each group.
  std::map<std::string, std::size_t> group_dims() const;

private:
  std::vector<float> compute_group(std::vector<ObservationTerm> & terms);
  std::vector<ObservationTerm> prepare_group(const YAML::Node & group_cfg);

  PolicyEnv & env_;
  bool use_gym_history_{false};
  std::map<std::string, std::vector<ObservationTerm>> groups_;
  std::map<std::string, std::vector<float>> last_;
};

}  // namespace go2_policy_runner::policy
