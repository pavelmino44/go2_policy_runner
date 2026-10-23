// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/isaaclab/manager/action_manager.h
//   deploy/include/isaaclab/envs/mdp/actions/joint_actions.h
// Changes by unitree-go2-rl contributors (2026):
//   - joint_ids support, size validation, reset() produces the offset pose
//
// Action terms (JointPositionAction / JointVelocityAction) with IsaacLab semantics:
//   processed = clip(raw * scale + offset).

#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

namespace go2_policy_runner::policy
{

class PolicyEnv;

class ActionTerm
{
public:
  enum class Target { kPosition, kVelocity };

  ActionTerm(const std::string & name, const YAML::Node & cfg, std::size_t num_joints, Target target);

  std::size_t dim() const {return joint_ids_.size();}
  Target target() const {return target_;}
  const std::string & name() const {return name_;}
  /// IsaacLab joint indices this term drives.
  const std::vector<int> & joint_ids() const {return joint_ids_;}

  void process(const float * raw);
  void reset();
  const std::vector<float> & raw() const {return raw_;}
  const std::vector<float> & processed() const {return processed_;}

private:
  std::string name_;
  Target target_;
  std::vector<int> joint_ids_;
  std::vector<float> scale_, offset_;
  std::vector<std::vector<float>> clip_;
  std::vector<float> raw_, processed_;
};

class ActionManager
{
public:
  ActionManager(const YAML::Node & cfg, std::size_t num_joints);

  std::size_t total_dim() const {return total_dim_;}
  void reset();
  void process(const std::vector<float> & action);
  /// Raw policy output of the last step (what IsaacLab calls "last_action").
  const std::vector<float> & action() const {return action_;}
  const std::vector<std::unique_ptr<ActionTerm>> & terms() const {return terms_;}

private:
  std::vector<std::unique_ptr<ActionTerm>> terms_;
  std::vector<float> action_;
  std::size_t total_dim_{0};
};

}  // namespace go2_policy_runner::policy
