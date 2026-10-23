// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/isaaclab/assets/articulation/articulation.h
//   deploy/include/unitree_articulation.h
// Changes by unitree-go2-rl contributors (2026):
//   - decoupled from unitree_sdk2: filled from a plain RobotState snapshot
//   - integer joint map, quaternion normalization, velocity command field
//
// Robot data as seen by the policy, in IsaacLab joint order.

#pragma once

#include <Eigen/Dense>
#include <vector>

#include "go2_policy_runner/core/types.hpp"

namespace go2_policy_runner::policy
{

struct ArticulationData
{
  const Eigen::Vector3f gravity_vec_w{0.0f, 0.0f, -1.0f};

  /// joint_ids_map[isaac_index] = sdk_index (as exported by unitree_rl_lab export_deploy_cfg.py).
  std::vector<int> joint_ids_map;

  /// PD gains in SDK order (as exported in deploy.yaml).
  std::vector<float> joint_stiffness;
  std::vector<float> joint_damping;

  // --- IsaacLab order -----------------------------------------------------------------------
  Eigen::VectorXf joint_pos;
  Eigen::VectorXf joint_vel;
  Eigen::VectorXf default_joint_pos;

  // --- base -------------------------------------------------------------------------------
  Eigen::Vector3f root_ang_vel_b{Eigen::Vector3f::Zero()};
  Eigen::Vector3f projected_gravity_b{0.0f, 0.0f, -1.0f};
  Eigen::Quaternionf root_quat_w{Eigen::Quaternionf::Identity()};

  /// Commanded base velocity (already in the body frame).
  VelocityCommand command;

  std::size_t num_joints() const {return joint_ids_map.size();}

  /// Fill from a low-level state snapshot (SDK order -> IsaacLab order).
  void update(const RobotState & state, const VelocityCommand & cmd)
  {
    for (int i = 0; i < 3; ++i) {
      root_ang_vel_b[i] = state.gyroscope[i];
    }
    root_quat_w = Eigen::Quaternionf(
      state.quaternion[0], state.quaternion[1], state.quaternion[2], state.quaternion[3]);
    root_quat_w.normalize();
    projected_gravity_b = root_quat_w.conjugate() * gravity_vec_w;
    for (std::size_t i = 0; i < joint_ids_map.size(); ++i) {
      joint_pos[i] = state.q[joint_ids_map[i]];
      joint_vel[i] = state.dq[joint_ids_map[i]];
    }
    command = cmd;
  }
};

}  // namespace go2_policy_runner::policy
