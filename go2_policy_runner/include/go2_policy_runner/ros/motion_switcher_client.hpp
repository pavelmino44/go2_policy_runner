// Copyright (c) 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
//
// Contains code derived from unitree_ros2 (https://github.com/unitreerobotics/unitree_ros2) (BSD 3-Clause License, see THIRD_PARTY_NOTICES.md):
//   example/src/include/b2/b2_motion_switch_client.hpp
//   example/src/include/common/base_client.hpp
// Changes by unitree-go2-rl contributors (2026):
//   - single client object with pending-request map, best-effort response reader
//   - no nlohmann::json dependency, service discovery check
//
// Client for the Go2 "motion_switcher" service over plain ROS 2 topics
// (/api/motion_switcher/request, /api/motion_switcher/response; unitree_api/Request|Response).
// Low-level control is possible only after the built-in sport mode has been released.
//
// Calls block the calling thread; the response subscription must be served by an executor
// running in another thread.

#pragma once

#include <chrono>
#include <cstdint>
#include <future>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace go2_policy_runner::ros
{

class MotionSwitcherClient
{
public:
  static constexpr int32_t kApiCheckMode = 1001;
  static constexpr int32_t kApiSelectMode = 1002;
  static constexpr int32_t kApiReleaseMode = 1003;

  MotionSwitcherClient(
    rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group,
    const std::string & request_topic = "api/motion_switcher/request",
    const std::string & response_topic = "api/motion_switcher/response");

  /// True if the motion_switcher service listens on the request topic (i.e. we are on a robot).
  bool service_available() const;

  struct Result
  {
    int32_t code{-1};   ///< 0 = success, <0 = timeout/transport error, >0 = robot error code
    std::string data;   ///< JSON payload
    bool ok() const {return code == 0;}
  };

  Result call(int32_t api_id, const std::string & parameter, std::chrono::milliseconds timeout);

  /// Active motion mode name ("" if released), or nullopt on error.
  std::optional<std::string> check_mode(std::chrono::milliseconds timeout, int32_t * code = nullptr);
  Result release_mode(std::chrono::milliseconds timeout);
  Result select_mode(const std::string & name, std::chrono::milliseconds timeout);

private:
  void on_response(const unitree_api::msg::Response & msg);

  rclcpp::Node & node_;
  std::string request_topic_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr sub_;
  std::mutex mutex_;
  std::map<int64_t, std::promise<unitree_api::msg::Response>> pending_;
  int64_t next_id_;
};

/// Extract a string field from a flat JSON object, e.g. json_string_field("{\"name\":\"x\"}", "name").
std::optional<std::string> json_string_field(const std::string & json, const std::string & key);

}  // namespace go2_policy_runner::ros
