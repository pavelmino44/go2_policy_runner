// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// remote_teleop: Unitree wireless remote (LowState.wireless_remote) -> geometry_msgs/Twist.
// Works identically on the robot and in unitree_mujoco (use_joystick: 1), because both put
// the remote frame into LowState. Disable it when another node publishes cmd_vel.
//
//   left stick  up/down    -> linear.x   (forward)
//   left stick  left/right -> linear.y   (left)
//   right stick left/right -> angular.z  (CCW)

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "go2_policy_runner/input/input_mapping.hpp"
#include "go2_policy_runner/remote/unitree_remote.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace go2_policy_runner::ros
{

class RemoteTeleopNode : public rclcpp::Node
{
public:
  explicit RemoteTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("remote_teleop", options)
  {
    limits_.max_vx = declare_parameter<double>("max_vx", 1.0);
    limits_.max_vy = declare_parameter<double>("max_vy", 0.5);
    limits_.max_wz = declare_parameter<double>("max_wz", 1.0);
    limits_.deadzone = declare_parameter<double>("deadzone", 0.05);
    publish_when_idle_ = declare_parameter<bool>("publish_when_idle", false);
    const double rate = declare_parameter<double>("rate", 50.0);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    sub_ = create_subscription<unitree_go::msg::LowState>(
      "lowstate", rclcpp::SensorDataQoS(),
      [this](const unitree_go::msg::LowState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::copy(msg->wireless_remote.begin(), msg->wireless_remote.end(), raw_.begin());
        have_data_ = true;
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      [this]() {tick();});
  }

private:
  void tick()
  {
    std::array<std::uint8_t, 40> raw;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_data_) {return;}
      raw = raw_;
    }
    const bool connected = !RemoteFrame::is_empty(raw);
    if (!connected && !publish_when_idle_) {
      if (was_connected_) {pub_->publish(geometry_msgs::msg::Twist());}  // stop once
      was_connected_ = false;
      return;
    }
    was_connected_ = connected;
    const auto v = input::velocity_from_frame(RemoteFrame::decode(raw), limits_);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v.vx;
    cmd.linear.y = v.vy;
    cmd.angular.z = v.wz;
    pub_->publish(cmd);
  }

  input::TeleopLimits limits_;
  bool publish_when_idle_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::array<std::uint8_t, 40> raw_{};
  bool have_data_{false};
  bool was_connected_{false};
};

}  // namespace go2_policy_runner::ros

RCLCPP_COMPONENTS_REGISTER_NODE(go2_policy_runner::ros::RemoteTeleopNode)
