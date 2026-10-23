// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// Parameter helpers. The controller node is created with
// automatically_declare_parameters_from_overrides(true) so that the FSM description
// (fsm.<mode>.*) can have an arbitrary structure; these helpers give typed access with
// defaults and readable errors.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace go2_policy_runner::ros
{

inline double number(const rclcpp::Parameter & p)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {return p.as_double();}
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(p.as_int());
  }
  throw std::runtime_error("parameter '" + p.get_name() + "' must be a number");
}

/// Get a parameter, declaring it with `default_value` if it was not given.
/// Doubles also accept integer values ("control_rate: 500").
template<typename T>
T param(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, default_value);
  }
  const auto p = node.get_parameter(name);
  try {
    if constexpr (std::is_same_v<T, double>) {
      return number(p);
    } else {
      return p.get_value<T>();
    }
  } catch (const std::exception &) {
    throw std::runtime_error("parameter '" + name + "' has a wrong type (" +
            p.get_type_name() + ")");
  }
}

/// Get a required parameter (throws with a readable message if missing).
inline rclcpp::Parameter required_param(rclcpp::Node & node, const std::string & name)
{
  if (!node.has_parameter(name)) {
    throw std::runtime_error("required parameter '" + name + "' is missing");
  }
  return node.get_parameter(name);
}

/// Read a number array, accepting both double and integer arrays ("[3, 3, 3]" in YAML is
/// an integer array for ROS 2).
inline std::vector<float> float_array(const rclcpp::Parameter & p)
{
  std::vector<float> out;
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      for (double v : p.as_double_array()) {out.push_back(static_cast<float>(v));}
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      for (int64_t v : p.as_integer_array()) {out.push_back(static_cast<float>(v));}
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      out.push_back(static_cast<float>(p.as_double()));
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      out.push_back(static_cast<float>(p.as_int()));
      break;
    default:
      throw std::runtime_error("parameter '" + p.get_name() + "' must be a number array");
  }
  return out;
}

/// Per-motor array: either `n` values, or a single value broadcast to all motors.
inline std::vector<float> motor_array(const rclcpp::Parameter & p, std::size_t n)
{
  auto v = float_array(p);
  if (v.size() == 1) {v.assign(n, v[0]);}
  if (v.size() != n) {
    throw std::runtime_error("parameter '" + p.get_name() + "' must have 1 or " +
            std::to_string(n) + " values, got " + std::to_string(v.size()));
  }
  return v;
}

/// Names of the direct children of `prefix` (e.g. "fsm.stand.transitions" -> {"passive", ...}).
inline std::vector<std::string> child_names(rclcpp::Node & node, const std::string & prefix)
{
  const auto result = node.list_parameters({prefix}, 0 /* recursive */);
  std::vector<std::string> names;
  const std::string p = prefix + ".";
  for (const auto & full : result.names) {
    if (full.rfind(p, 0) != 0) {continue;}
    const auto rest = full.substr(p.size());
    const auto child = rest.substr(0, rest.find('.'));
    if (std::find(names.begin(), names.end(), child) == names.end()) {names.push_back(child);}
  }
  return names;
}

}  // namespace go2_policy_runner::ros
