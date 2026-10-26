// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// robot_monitor: read-only view of the controller and the robot.
//
//   subscribes  ~/state, ~/policy_step of the controller, lowstate, lowcmd, cmd_vel, remote
//   reads       controller parameters (control_rate, joint_limits.*, fsm.<mode>.max_tilt)
//   publishes   /diagnostics (diagnostic_msgs/DiagnosticArray), optional
//   calls       <controller>/set_mode {mode: passive} on request (the only action it can take)
//
// The terminal UI lives in monitor_main.cpp; this node only collects data.

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "go2_policy_runner/monitor/monitor_model.hpp"
#include "go2_policy_runner_interfaces/msg/controller_state.hpp"
#include "go2_policy_runner_interfaces/msg/policy_step.hpp"
#include "go2_policy_runner_interfaces/srv/set_mode.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/wireless_controller.hpp"

namespace go2_policy_runner::ros
{

class MonitorNode : public rclcpp::Node
{
public:
  explicit MonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Consistent copy of everything received so far (ages computed at call time).
  monitor::Snapshot snapshot();
  const monitor::Thresholds & thresholds() const {return thresholds_;}
  double refresh_rate() const {return refresh_rate_;}
  const std::string & controller_name() const {return controller_;}

  /// Ask the controller to switch to its fallback (damping) mode.
  void request_passive();
  /// Last message about a set_mode request (empty if none).
  std::string action_status();

private:
  using ControllerState = go2_policy_runner_interfaces::msg::ControllerState;
  using PolicyStep = go2_policy_runner_interfaces::msg::PolicyStep;
  using SetMode = go2_policy_runner_interfaces::srv::SetMode;

  void on_slow_timer();
  void fetch_controller_parameters();
  void publish_diagnostics();
  double now_s() const;

  // parameters
  std::string controller_;
  double refresh_rate_{10.0};
  double nominal_lowstate_rate_{0.0};
  bool publish_diagnostics_{true};
  monitor::Thresholds thresholds_;

  // ROS entities
  rclcpp::Subscription<ControllerState>::SharedPtr state_sub_;
  rclcpp::Subscription<PolicyStep>::SharedPtr policy_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr lowcmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr remote_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Client<SetMode>::SharedPtr set_mode_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  rclcpp::TimerBase::SharedPtr slow_timer_;

  // data (guarded by mutex_)
  std::mutex mutex_;
  monitor::Snapshot data_;
  double t_state_{-1}, t_lowstate_{-1}, t_cmd_vel_{-1}, t_policy_{-1}, t_remote_{-1};
  std::map<std::string, double> max_tilt_by_mode_;
  std::vector<float> lower_, upper_;
  double t_params_{-1e9};
  std::string action_status_;

  const std::chrono::steady_clock::time_point t0_{std::chrono::steady_clock::now()};
};

}  // namespace go2_policy_runner::ros
