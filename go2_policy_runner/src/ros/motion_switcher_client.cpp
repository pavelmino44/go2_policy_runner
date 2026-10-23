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

#include "go2_policy_runner/ros/motion_switcher_client.hpp"

#include <regex>

namespace go2_policy_runner::ros
{

MotionSwitcherClient::MotionSwitcherClient(
  rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group,
  const std::string & request_topic, const std::string & response_topic)
: node_(node), request_topic_(request_topic),
  next_id_(std::chrono::steady_clock::now().time_since_epoch().count())
{
  // Reliable writer matches both reliable and best-effort readers;
  // best-effort reader matches any writer.
  pub_ = node_.create_publisher<unitree_api::msg::Request>(request_topic, rclcpp::QoS(10).reliable());
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  sub_ = node_.create_subscription<unitree_api::msg::Response>(
    response_topic, rclcpp::QoS(10).best_effort(),
    [this](const unitree_api::msg::Response::SharedPtr msg) {on_response(*msg);}, options);
}

bool MotionSwitcherClient::service_available() const
{
  // matched readers of our request writer (respects remapping, unlike count_subscribers)
  return pub_->get_subscription_count() > 0;
}

void MotionSwitcherClient::on_response(const unitree_api::msg::Response & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = pending_.find(msg.header.identity.id);
  if (it != pending_.end()) {
    it->second.set_value(msg);
    pending_.erase(it);
  }
}

MotionSwitcherClient::Result MotionSwitcherClient::call(
  int32_t api_id, const std::string & parameter, std::chrono::milliseconds timeout)
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = api_id;
  req.parameter = parameter;
  std::future<unitree_api::msg::Response> future;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    req.header.identity.id = ++next_id_;
    future = pending_[req.header.identity.id].get_future();
  }
  pub_->publish(req);

  Result result;
  if (future.wait_for(timeout) != std::future_status::ready) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.erase(req.header.identity.id);
    result.code = -1;  // timeout
    return result;
  }
  const auto resp = future.get();
  result.code = resp.header.status.code;
  result.data = resp.data;
  return result;
}

std::optional<std::string> MotionSwitcherClient::check_mode(
  std::chrono::milliseconds timeout, int32_t * code)
{
  const auto r = call(kApiCheckMode, "", timeout);
  if (code) {*code = r.code;}
  if (!r.ok()) {return std::nullopt;}
  return json_string_field(r.data, "name").value_or("");
}

MotionSwitcherClient::Result MotionSwitcherClient::release_mode(std::chrono::milliseconds timeout)
{
  return call(kApiReleaseMode, "", timeout);
}

MotionSwitcherClient::Result MotionSwitcherClient::select_mode(
  const std::string & name, std::chrono::milliseconds timeout)
{
  return call(kApiSelectMode, "{\"name\":\"" + name + "\"}", timeout);
}

std::optional<std::string> json_string_field(const std::string & json, const std::string & key)
{
  const std::regex re("\"" + key + "\"\\s*:\\s*\"((?:[^\"\\\\]|\\\\.)*)\"");
  std::smatch m;
  if (std::regex_search(json, m, re)) {return m[1].str();}
  return std::nullopt;
}

}  // namespace go2_policy_runner::ros
