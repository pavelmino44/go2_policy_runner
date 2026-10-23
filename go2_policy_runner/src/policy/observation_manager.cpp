// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/isaaclab/manager/observation_manager.h
//   deploy/include/isaaclab/manager/manager_term_cfg.h
//   deploy/include/isaaclab/envs/mdp/observations/observations.h
// Changes by unitree-go2-rl contributors (2026):
//   - observation terms registered explicitly (no static initializers)
//   - velocity_commands read from the velocity command (cmd_vel) instead of the joystick
//   - group detection and size checks

#include "go2_policy_runner/policy/observation_manager.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

#include "go2_policy_runner/policy/policy_env.hpp"

namespace go2_policy_runner::policy
{

// ---------------------------------------------------------------------------------------------
// Registry with built-in terms. Built-ins are registered explicitly (no static-initializer
// tricks), so they can never be dropped by the linker.
// ---------------------------------------------------------------------------------------------
namespace
{

std::vector<int> joint_ids_param(const YAML::Node & params)
{
  if (params && params.IsMap() && params["asset_cfg"] && params["asset_cfg"]["joint_ids"] &&
    params["asset_cfg"]["joint_ids"].IsSequence())
  {
    return params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
  }
  return {};
}

std::vector<float> select(const Eigen::VectorXf & v, const std::vector<int> & ids)
{
  if (ids.empty()) {return std::vector<float>(v.data(), v.data() + v.size());}
  std::vector<float> out(ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i) {out[i] = v[ids[i]];}
  return out;
}

std::vector<float> velocity_commands(PolicyEnv & env, const YAML::Node &)
{
  const auto & c = env.data().command;
  std::vector<float> obs{c.vx, c.vy, c.wz};
  const YAML::Node ranges = env.cfg()["commands"]["base_velocity"]["ranges"];
  if (ranges && ranges.IsMap()) {
    const char * keys[3] = {"lin_vel_x", "lin_vel_y", "ang_vel_z"};
    for (int i = 0; i < 3; ++i) {
      const YAML::Node r = ranges[keys[i]];
      if (r && r.IsSequence() && r.size() == 2) {
        obs[i] = std::clamp(obs[i], r[0].as<float>(), r[1].as<float>());
      }
    }
  }
  return obs;
}

std::unordered_map<std::string, ObservationFunc> & registry()
{
  static std::unordered_map<std::string, ObservationFunc> map = {
    {"base_ang_vel", [](PolicyEnv & env, const YAML::Node &) {
        const auto & v = env.data().root_ang_vel_b;
        return std::vector<float>(v.data(), v.data() + 3);
      }},
    {"projected_gravity", [](PolicyEnv & env, const YAML::Node &) {
        const auto & v = env.data().projected_gravity_b;
        return std::vector<float>(v.data(), v.data() + 3);
      }},
    {"joint_pos", [](PolicyEnv & env, const YAML::Node & p) {
        return select(env.data().joint_pos, joint_ids_param(p));
      }},
    {"joint_pos_rel", [](PolicyEnv & env, const YAML::Node & p) {
        Eigen::VectorXf rel = env.data().joint_pos - env.data().default_joint_pos;
        return select(rel, joint_ids_param(p));
      }},
    {"joint_vel", [](PolicyEnv & env, const YAML::Node & p) {
        return select(env.data().joint_vel, joint_ids_param(p));
      }},
    // default joint velocity is zero for all Unitree assets
    {"joint_vel_rel", [](PolicyEnv & env, const YAML::Node & p) {
        return select(env.data().joint_vel, joint_ids_param(p));
      }},
    {"last_action", [](PolicyEnv & env, const YAML::Node &) {
        return env.action_manager().action();
      }},
    {"velocity_commands", velocity_commands},
    {"generated_commands", velocity_commands},
    {"gait_phase", [](PolicyEnv & env, const YAML::Node & p) {
        const float period = p["period"].as<float>();
        env.global_phase = std::fmod(env.global_phase + env.step_dt() / period, 1.0f);
        const float a = env.global_phase * 2.0f * static_cast<float>(M_PI);
        return std::vector<float>{std::sin(a), std::cos(a)};
      }},
  };
  return map;
}

std::mutex & registry_mutex()
{
  static std::mutex m;
  return m;
}

std::vector<float> optional_floats(const YAML::Node & n)
{
  if (!n || n.IsNull()) {return {};}
  return n.as<std::vector<float>>();
}

bool is_term_cfg(const YAML::Node & n)
{
  return n.IsMap() && (n["params"] || n["scale"] || n["clip"] || n["history_length"]);
}

}  // namespace

void register_observation(const std::string & name, ObservationFunc func)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  registry()[name] = std::move(func);
}

std::vector<std::string> registered_observations()
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  std::vector<std::string> names;
  for (const auto & kv : registry()) {names.push_back(kv.first);}
  std::sort(names.begin(), names.end());
  return names;
}

// ---------------------------------------------------------------------------------------------
// ObservationTerm
// ---------------------------------------------------------------------------------------------
void ObservationTerm::reset(const std::vector<float> & obs)
{
  buffer_.clear();
  for (int i = 0; i < history_length; ++i) {add(obs);}
}

void ObservationTerm::add(std::vector<float> obs)
{
  if (!scale.empty() && scale.size() != obs.size()) {
    throw std::runtime_error(
            "observation '" + name + "': scale has " + std::to_string(scale.size()) +
            " elements, term produces " + std::to_string(obs.size()));
  }
  for (std::size_t j = 0; j < obs.size(); ++j) {
    // IsaacLab order: clip, then scale (unless scale_first is set in deploy.yaml)
    if (scale_first && !scale.empty()) {obs[j] *= scale[j];}
    if (clip.size() == 2) {obs[j] = std::clamp(obs[j], clip[0], clip[1]);}
    if (!scale_first && !scale.empty()) {obs[j] *= scale[j];}
  }
  dim_ = obs.size();
  buffer_.push_back(std::move(obs));
  while (buffer_.size() > static_cast<std::size_t>(history_length)) {buffer_.pop_front();}
}

std::vector<float> ObservationTerm::flattened() const
{
  std::vector<float> out;
  out.reserve(dim_ * buffer_.size());
  for (const auto & e : buffer_) {out.insert(out.end(), e.begin(), e.end());}
  return out;
}

// ---------------------------------------------------------------------------------------------
// ObservationManager
// ---------------------------------------------------------------------------------------------
ObservationManager::ObservationManager(const YAML::Node & cfg, PolicyEnv & env)
: env_(env)
{
  if (!cfg || !cfg.IsMap() || cfg.size() == 0) {
    throw std::runtime_error("deploy.yaml: 'observations' is missing or empty");
  }
  // Either a single group (terms directly under `observations`, ONNX input "obs")
  // or several named groups (one ONNX input per group).
  bool single_group = false;
  for (auto it = cfg.begin(); it != cfg.end(); ++it) {
    if (is_term_cfg(it->second)) {single_group = true; break;}
  }
  if (single_group) {
    groups_["obs"] = prepare_group(cfg);
  } else {
    for (auto it = cfg.begin(); it != cfg.end(); ++it) {
      groups_[it->first.as<std::string>()] = prepare_group(it->second);
    }
  }
}

std::vector<ObservationTerm> ObservationManager::prepare_group(const YAML::Node & group_cfg)
{
  std::vector<ObservationTerm> terms;
  bool scale_first = false;
  for (auto it = group_cfg.begin(); it != group_cfg.end(); ++it) {
    const auto key = it->first.as<std::string>();
    if (key == "scale_first") {scale_first = it->second.as<bool>(); continue;}
    if (key == "use_gym_history") {use_gym_history_ = it->second.as<bool>(); continue;}

    const YAML::Node term_cfg = it->second;
    ObservationTerm term;
    term.name = key;
    term.params = term_cfg["params"];
    term.scale_first = scale_first;
    term.history_length = std::max(1, term_cfg["history_length"].as<int>(1));
    term.scale = optional_floats(term_cfg["scale"]);
    term.clip = optional_floats(term_cfg["clip"]);
    {
      std::lock_guard<std::mutex> lock(registry_mutex());
      auto f = registry().find(key);
      if (f == registry().end()) {
        throw std::runtime_error(
                "observation term '" + key + "' is not supported by go2_policy_runner "
                "(register it with register_observation())");
      }
      term.func = f->second;
    }
    term.reset(term.func(env_, term.params));
    terms.push_back(std::move(term));
  }
  return terms;
}

void ObservationManager::reset()
{
  for (auto & g : groups_) {
    for (auto & term : g.second) {term.reset(term.func(env_, term.params));}
  }
}

const std::map<std::string, std::vector<float>> & ObservationManager::compute()
{
  for (auto & g : groups_) {last_[g.first] = compute_group(g.second);}
  return last_;
}

std::vector<float> ObservationManager::compute_group(std::vector<ObservationTerm> & terms)
{
  for (auto & term : terms) {term.add(term.func(env_, term.params));}

  std::vector<float> obs;
  if (use_gym_history_ && !terms.empty()) {
    // [t-H+1: all terms], ..., [t: all terms]
    for (int h = 0; h < terms[0].history_length; ++h) {
      for (const auto & term : terms) {
        const auto & e = term.get(static_cast<std::size_t>(h));
        obs.insert(obs.end(), e.begin(), e.end());
      }
    }
  } else {
    // IsaacLab default: per term [oldest ... newest], terms concatenated
    for (const auto & term : terms) {
      const auto e = term.flattened();
      obs.insert(obs.end(), e.begin(), e.end());
    }
  }
  return obs;
}

std::map<std::string, std::size_t> ObservationManager::group_dims() const
{
  std::map<std::string, std::size_t> dims;
  for (const auto & g : groups_) {
    std::size_t d = 0;
    for (const auto & t : g.second) {d += t.dim() * static_cast<std::size_t>(t.history_length);}
    dims[g.first] = d;
  }
  return dims;
}

}  // namespace go2_policy_runner::policy
