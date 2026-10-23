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

#include "go2_policy_runner/policy/action_manager.hpp"

#include <algorithm>
#include <stdexcept>

namespace go2_policy_runner::policy
{

namespace
{
std::vector<float> optional_floats(const YAML::Node & n)
{
  if (!n || n.IsNull()) {return {};}
  return n.as<std::vector<float>>();
}
}  // namespace

ActionTerm::ActionTerm(
  const std::string & name, const YAML::Node & cfg, std::size_t num_joints, Target target)
: name_(name), target_(target)
{
  if (cfg["joint_ids"] && !cfg["joint_ids"].IsNull()) {
    joint_ids_ = cfg["joint_ids"].as<std::vector<int>>();
    for (int id : joint_ids_) {
      if (id < 0 || static_cast<std::size_t>(id) >= num_joints) {
        throw std::runtime_error("action '" + name + "': joint id " + std::to_string(id) +
                " out of range");
      }
    }
  } else {
    joint_ids_.resize(num_joints);
    for (std::size_t i = 0; i < num_joints; ++i) {joint_ids_[i] = static_cast<int>(i);}
  }
  scale_ = optional_floats(cfg["scale"]);
  offset_ = optional_floats(cfg["offset"]);
  if (cfg["clip"] && !cfg["clip"].IsNull()) {
    clip_ = cfg["clip"].as<std::vector<std::vector<float>>>();
  }
  const auto n = joint_ids_.size();
  if ((!scale_.empty() && scale_.size() != n) || (!offset_.empty() && offset_.size() != n) ||
    (!clip_.empty() && clip_.size() != n))
  {
    throw std::runtime_error("action '" + name + "': scale/offset/clip size mismatch (expected " +
            std::to_string(n) + ")");
  }
  for (const auto & c : clip_) {
    if (c.size() != 2) {throw std::runtime_error("action '" + name + "': clip must be [min, max]");}
  }
  raw_.assign(n, 0.0f);
  processed_.assign(n, 0.0f);
  reset();
}

void ActionTerm::process(const float * raw)
{
  for (std::size_t i = 0; i < raw_.size(); ++i) {
    raw_[i] = raw[i];
    float v = scale_.empty() ? raw_[i] : raw_[i] * scale_[i];
    if (!offset_.empty()) {v += offset_[i];}
    if (!clip_.empty()) {v = std::clamp(v, clip_[i][0], clip_[i][1]);}
    processed_[i] = v;
  }
}

void ActionTerm::reset()
{
  // processed action of a zero raw action = offset (default pose for position actions)
  std::vector<float> zeros(raw_.size(), 0.0f);
  process(zeros.data());
}

ActionManager::ActionManager(const YAML::Node & cfg, std::size_t num_joints)
{
  if (!cfg || !cfg.IsMap() || cfg.size() == 0) {
    throw std::runtime_error("deploy.yaml: 'actions' is missing or empty");
  }
  for (auto it = cfg.begin(); it != cfg.end(); ++it) {
    const auto name = it->first.as<std::string>();
    ActionTerm::Target target;
    if (name == "JointPositionAction") {
      target = ActionTerm::Target::kPosition;
    } else if (name == "JointVelocityAction") {
      target = ActionTerm::Target::kVelocity;
    } else {
      throw std::runtime_error("action term '" + name + "' is not supported "
              "(supported: JointPositionAction, JointVelocityAction)");
    }
    terms_.push_back(std::make_unique<ActionTerm>(name, it->second, num_joints, target));
    total_dim_ += terms_.back()->dim();
  }
  action_.assign(total_dim_, 0.0f);
}

void ActionManager::reset()
{
  action_.assign(total_dim_, 0.0f);
  for (auto & t : terms_) {t->reset();}
}

void ActionManager::process(const std::vector<float> & action)
{
  if (action.size() != total_dim_) {
    throw std::runtime_error("policy produced " + std::to_string(action.size()) +
            " actions, action terms expect " + std::to_string(total_dim_));
  }
  action_ = action;
  std::size_t idx = 0;
  for (auto & t : terms_) {
    t->process(action_.data() + idx);
    idx += t->dim();
  }
}

}  // namespace go2_policy_runner::policy
