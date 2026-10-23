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

#include "go2_policy_runner/policy/policy_env.hpp"

#include <algorithm>
#include <set>
#include <sstream>
#include <stdexcept>

namespace go2_policy_runner::policy
{

namespace
{
template<typename T>
std::vector<T> required_vector(const YAML::Node & cfg, const std::string & key)
{
  if (!cfg[key] || !cfg[key].IsSequence()) {
    throw std::runtime_error("deploy.yaml: '" + key + "' is missing");
  }
  return cfg[key].as<std::vector<T>>();
}
}  // namespace

PolicyEnv::PolicyEnv(
  const YAML::Node & cfg, std::unique_ptr<PolicyModel> model, std::size_t num_motors)
: cfg_(cfg), num_motors_(num_motors), model_(std::move(model))
{
  if (!model_) {throw std::invalid_argument("PolicyEnv: model is null");}
  if (!cfg_["step_dt"]) {throw std::runtime_error("deploy.yaml: 'step_dt' is missing");}
  step_dt_ = cfg_["step_dt"].as<float>();
  if (step_dt_ <= 0.0f) {throw std::runtime_error("deploy.yaml: step_dt must be > 0");}

  // --- joint mapping & gains --------------------------------------------------------------
  // joint_ids_map may be written as ints or floats depending on the exporter version.
  for (float v : required_vector<float>(cfg_, "joint_ids_map")) {
    data_.joint_ids_map.push_back(static_cast<int>(v));
  }
  const std::size_t n = data_.joint_ids_map.size();
  std::set<int> unique_ids;
  for (int id : data_.joint_ids_map) {
    if (id < 0 || static_cast<std::size_t>(id) >= num_motors_) {
      throw std::runtime_error("deploy.yaml: joint_ids_map contains motor index " +
              std::to_string(id) + ", robot has " + std::to_string(num_motors_) + " motors");
    }
    unique_ids.insert(id);
  }
  if (unique_ids.size() != n) {throw std::runtime_error("deploy.yaml: joint_ids_map has duplicates");}

  data_.joint_stiffness = required_vector<float>(cfg_, "stiffness");
  data_.joint_damping = required_vector<float>(cfg_, "damping");
  if (data_.joint_stiffness.size() != n || data_.joint_damping.size() != n) {
    throw std::runtime_error("deploy.yaml: stiffness/damping must have " + std::to_string(n) +
            " entries (one per joint, SDK order)");
  }
  const auto default_pos = required_vector<float>(cfg_, "default_joint_pos");
  if (default_pos.size() != n) {
    throw std::runtime_error("deploy.yaml: default_joint_pos must have " + std::to_string(n) +
            " entries");
  }
  data_.default_joint_pos = Eigen::Map<const Eigen::VectorXf>(default_pos.data(), n);
  data_.joint_pos = data_.default_joint_pos;
  data_.joint_vel = Eigen::VectorXf::Zero(n);

  position_targets_sdk_.assign(num_motors_, 0.0f);
  velocity_targets_sdk_.assign(num_motors_, 0.0f);
  driven_sdk_.assign(num_motors_, false);

  // --- managers (actions first: last_action observation needs it) --------------------------
  action_manager_ = std::make_unique<ActionManager>(cfg_["actions"], n);
  observation_manager_ = std::make_unique<ObservationManager>(cfg_["observations"], *this);

  // --- consistency between deploy.yaml and the model ----------------------------------------
  const auto dims = observation_manager_->group_dims();
  std::ostringstream err;
  for (const auto & in : model_->inputs()) {
    auto it = dims.find(in.first);
    if (it == dims.end()) {
      err << "model input '" << in.first << "' has no observation group in deploy.yaml; ";
    } else if (it->second != in.second) {
      err << "observation group '" << in.first << "' has " << it->second
          << " values, model expects " << in.second << "; ";
    }
  }
  if (model_->output_size() != action_manager_->total_dim()) {
    err << "model outputs " << model_->output_size() << " actions, deploy.yaml actions expect "
        << action_manager_->total_dim() << "; ";
  }
  if (!err.str().empty()) {throw std::runtime_error("policy/deploy.yaml mismatch: " + err.str());}

  update_targets();
}

void PolicyEnv::reset(const RobotState & state, const VelocityCommand & cmd)
{
  global_phase = 0.0f;
  episode_length_ = 0;
  data_.update(state, cmd);
  action_manager_->reset();
  observation_manager_->reset();
  update_targets();
}

void PolicyEnv::step(const RobotState & state, const VelocityCommand & cmd)
{
  ++episode_length_;
  data_.update(state, cmd);
  const auto & obs = observation_manager_->compute();
  action_manager_->process(model_->act(obs));
  update_targets();
}

void PolicyEnv::update_targets()
{
  // defaults: hold default pose, zero velocity
  std::fill(driven_sdk_.begin(), driven_sdk_.end(), false);
  for (std::size_t i = 0; i < data_.joint_ids_map.size(); ++i) {
    position_targets_sdk_[data_.joint_ids_map[i]] = data_.default_joint_pos[i];
    velocity_targets_sdk_[data_.joint_ids_map[i]] = 0.0f;
  }
  for (const auto & term : action_manager_->terms()) {
    const auto & ids = term->joint_ids();
    const auto & p = term->processed();
    for (std::size_t k = 0; k < ids.size(); ++k) {
      const int sdk = data_.joint_ids_map[ids[k]];
      driven_sdk_[sdk] = true;
      if (term->target() == ActionTerm::Target::kPosition) {
        position_targets_sdk_[sdk] = p[k];
      } else {
        velocity_targets_sdk_[sdk] = p[k];
      }
    }
  }
}

PolicyFiles resolve_policy_dir(const std::filesystem::path & dir_in)
{
  namespace fs = std::filesystem;
  if (!fs::is_directory(dir_in)) {
    throw std::runtime_error("policy directory '" + dir_in.string() + "' does not exist");
  }
  fs::path dir = dir_in;
  if (!fs::exists(dir / "exported")) {
    std::vector<fs::path> candidates;
    for (const auto & e : fs::directory_iterator(dir)) {
      if (e.is_directory() && fs::exists(e.path() / "exported")) {candidates.push_back(e.path());}
    }
    if (candidates.empty()) {
      throw std::runtime_error("'" + dir.string() + "' contains neither exported/ nor run "
              "sub-directories with exported/");
    }
    std::sort(candidates.begin(), candidates.end());
    dir = candidates.back();
  }
  PolicyFiles files{dir, dir / "params" / "deploy.yaml", dir / "exported" / "policy.onnx"};
  if (!fs::exists(files.deploy_yaml)) {
    throw std::runtime_error("missing " + files.deploy_yaml.string());
  }
  if (!fs::exists(files.onnx)) {throw std::runtime_error("missing " + files.onnx.string());}
  return files;
}

std::unique_ptr<PolicyEnv> load_policy_env(
  const std::filesystem::path & dir, std::size_t num_motors, const OnnxOptions & options)
{
  const auto files = resolve_policy_dir(dir);
  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(files.deploy_yaml.string());
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("failed to parse " + files.deploy_yaml.string() + ": " + e.what());
  }
  return std::make_unique<PolicyEnv>(cfg, load_onnx_policy(files.onnx.string(), options), num_motors);
}

}  // namespace go2_policy_runner::policy
