// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/isaaclab/algorithms/algorithms.h
// Changes by unitree-go2-rl contributors (2026):
//   - owned name strings (no leaks), dynamic (-1) dims treated as 1
//   - input/output size validation, configurable thread count

#include <onnxruntime_cxx_api.h>

#include <numeric>
#include <stdexcept>

#include "go2_policy_runner/policy/policy_model.hpp"

namespace go2_policy_runner::policy
{

namespace
{

std::vector<int64_t> concrete_shape(std::vector<int64_t> shape)
{
  for (auto & d : shape) {
    if (d < 1) {d = 1;}  // dynamic batch -> 1
  }
  return shape;
}

std::size_t numel(const std::vector<int64_t> & shape)
{
  return static_cast<std::size_t>(
    std::accumulate(shape.begin(), shape.end(), int64_t{1}, std::multiplies<int64_t>()));
}

class OnnxPolicy : public PolicyModel
{
public:
  OnnxPolicy(const std::string & path, const OnnxOptions & options)
  : env_(ORT_LOGGING_LEVEL_WARNING, "go2_policy_runner"),
    memory_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU))
  {
    Ort::SessionOptions so;
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    so.SetIntraOpNumThreads(options.intra_op_threads);
    so.SetInterOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(env_, path.c_str(), so);

    Ort::AllocatorWithDefaultOptions allocator;
    for (std::size_t i = 0; i < session_->GetInputCount(); ++i) {
      input_names_.emplace_back(session_->GetInputNameAllocated(i, allocator).get());
      auto shape = concrete_shape(
        session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      input_shapes_.push_back(shape);
      inputs_.emplace_back(input_names_.back(), numel(shape));
    }
    if (session_->GetOutputCount() < 1) {
      throw std::runtime_error("ONNX model '" + path + "' has no outputs");
    }
    output_names_.emplace_back(session_->GetOutputNameAllocated(0, allocator).get());
    output_size_ = numel(
      concrete_shape(session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape()));

    for (const auto & n : input_names_) {input_name_ptrs_.push_back(n.c_str());}
    for (const auto & n : output_names_) {output_name_ptrs_.push_back(n.c_str());}
  }

  const std::vector<std::pair<std::string, std::size_t>> & inputs() const override {return inputs_;}
  std::size_t output_size() const override {return output_size_;}

  std::vector<float> act(const std::map<std::string, std::vector<float>> & obs) override
  {
    std::vector<Ort::Value> tensors;
    tensors.reserve(inputs_.size());
    for (std::size_t i = 0; i < inputs_.size(); ++i) {
      auto it = obs.find(inputs_[i].first);
      if (it == obs.end()) {
        throw std::runtime_error("observation group '" + inputs_[i].first + "' is missing");
      }
      if (it->second.size() != inputs_[i].second) {
        throw std::runtime_error(
                "observation group '" + inputs_[i].first + "' has size " +
                std::to_string(it->second.size()) + ", model expects " +
                std::to_string(inputs_[i].second));
      }
      // ORT does not modify inputs; const_cast is required by the C API signature.
      tensors.push_back(Ort::Value::CreateTensor<float>(
          memory_info_, const_cast<float *>(it->second.data()), it->second.size(),
          input_shapes_[i].data(), input_shapes_[i].size()));
    }
    auto out = session_->Run(
      Ort::RunOptions{nullptr}, input_name_ptrs_.data(), tensors.data(), tensors.size(),
      output_name_ptrs_.data(), 1);
    const float * data = out.front().GetTensorData<float>();
    return std::vector<float>(data, data + output_size_);
  }

private:
  Ort::Env env_;
  Ort::MemoryInfo memory_info_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<std::string> input_names_, output_names_;
  std::vector<const char *> input_name_ptrs_, output_name_ptrs_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::pair<std::string, std::size_t>> inputs_;
  std::size_t output_size_{0};
};

}  // namespace

std::unique_ptr<PolicyModel> load_onnx_policy(const std::string & path, const OnnxOptions & options)
{
  try {
    return std::make_unique<OnnxPolicy>(path, options);
  } catch (const Ort::Exception & e) {
    throw std::runtime_error("failed to load ONNX model '" + path + "': " + e.what());
  }
}

}  // namespace go2_policy_runner::policy
