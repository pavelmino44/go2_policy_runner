// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/isaaclab/algorithms/algorithms.h
// Changes by unitree-go2-rl contributors (2026):
//   - abstract model interface independent of ONNX Runtime
//
// Inference backends. OnnxPolicy is the default (exported/policy.onnx from rsl_rl / unitree_rl_lab).

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace go2_policy_runner::policy
{

class PolicyModel
{
public:
  virtual ~PolicyModel() = default;

  /// Model input names and their flattened sizes (batch dimension excluded).
  virtual const std::vector<std::pair<std::string, std::size_t>> & inputs() const = 0;
  /// Flattened output size.
  virtual std::size_t output_size() const = 0;
  /// Run inference. `obs` must contain every input name with the right size.
  virtual std::vector<float> act(const std::map<std::string, std::vector<float>> & obs) = 0;
};

struct OnnxOptions
{
  int intra_op_threads{1};  ///< 1 = no thread pool -> lowest jitter for small MLPs
};

/// Loads an ONNX model with ONNX Runtime (CPU execution provider).
std::unique_ptr<PolicyModel> load_onnx_policy(const std::string & path, const OnnxOptions & options = {});

}  // namespace go2_policy_runner::policy
