// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// policy_controller: the single node that talks to the robot (real Go2 or unitree_mujoco).
//
//   subscribes  lowstate  (unitree_go/LowState)       robot state, incl. wireless remote
//               cmd_vel   (geometry_msgs/Twist)       base velocity command for rl_policy modes
//   publishes   lowcmd    (unitree_go/LowCmd)         motor commands (with CRC)
//               ~/state   (ControllerState)           FSM / health, latched
//               ~/policy_step (PolicyStep)            observations / actions of every policy step
//   services    ~/set_mode (SetMode)                  request an FSM transition
//
// The same binary and configuration are used in simulation and on the robot; only the DDS
// network interface differs (see launch/policy_runner.launch.py).

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "go2_policy_runner/core/types.hpp"
#include "go2_policy_runner/fsm/mode_manager.hpp"
#include "go2_policy_runner/fsm/modes.hpp"
#include "go2_policy_runner/ros/motion_switcher_client.hpp"
#include "go2_policy_runner_interfaces/msg/controller_state.hpp"
#include "go2_policy_runner_interfaces/msg/policy_step.hpp"
#include "go2_policy_runner_interfaces/srv/set_mode.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace go2_policy_runner::ros
{

class PolicyControllerNode : public rclcpp::Node
{
public:
  explicit PolicyControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PolicyControllerNode() override;

  /// Force the fallback (passive) mode and keep publishing its command for `duration`.
  /// Call before rclcpp::shutdown() so the robot is left in damping mode.
  void safe_stop(std::chrono::milliseconds duration);

private:
  using LowState = unitree_go::msg::LowState;
  using LowCmd = unitree_go::msg::LowCmd;
  using ControllerState = go2_policy_runner_interfaces::msg::ControllerState;
  using PolicyStep = go2_policy_runner_interfaces::msg::PolicyStep;
  using SetMode = go2_policy_runner_interfaces::srv::SetMode;

  // --- setup ---
  void load_parameters();
  std::unique_ptr<fsm::ModeManager> build_fsm();
  std::unique_ptr<fsm::Mode> build_mode(const std::string & name);
  std::vector<float> pose(const std::string & mode_prefix, const std::string & pose_name);
  std::string resolve_policy_path(const std::string & path) const;

  // --- runtime ---
  void startup_sequence();
  bool release_motion_switcher();
  bool check_other_controllers();
  void control_cycle();
  void publish_lowcmd();
  void publish_state();
  void publish_policy_step(const fsm::PolicyStepInfo & info);
  void on_lowstate(const LowState::SharedPtr msg);
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_set_mode(
    const std::shared_ptr<SetMode::Request> req, std::shared_ptr<SetMode::Response> res);
  double now_s() const;

  // --- parameters ---
  std::size_t num_motors_{12};
  double control_rate_{500.0};
  double lowstate_timeout_{0.1};
  double cmd_vel_timeout_{0.5};
  double state_rate_{10.0};
  bool publish_policy_step_{true};
  int onnx_threads_{1};
  std::string motion_switcher_mode_{"auto"};  // auto | release | skip
  double motion_switcher_discovery_timeout_{3.0};
  double motion_switcher_call_timeout_{5.0};
  double lowcmd_check_duration_{0.5};
  std::string policy_root_;
  std::string fallback_mode_{"passive"};
  std::vector<float> q_lower_, q_upper_;  // optional position target limits (SDK order)

  // --- ROS entities ---
  rclcpp::CallbackGroup::SharedPtr control_group_, io_group_;
  rclcpp::Subscription<LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Publisher<ControllerState>::SharedPtr state_pub_;
  rclcpp::Publisher<PolicyStep>::SharedPtr policy_step_pub_;
  rclcpp::Service<SetMode>::SharedPtr set_mode_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_, state_timer_;
  std::unique_ptr<MotionSwitcherClient> motion_switcher_;

  // --- state ---
  std::unique_ptr<fsm::ModeManager> fsm_;
  std::thread startup_thread_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> stopping_{false};
  std::string startup_status_{"waiting for lowstate"};

  mutable std::mutex state_mutex_;  // guards the block below
  RobotState robot_state_;
  bool have_lowstate_{false};
  double last_lowstate_time_{-1.0};
  double lowstate_rate_{0.0};
  int lowstate_count_{0};
  double lowstate_rate_window_start_{0.0};
  VelocityCommand cmd_vel_;
  double last_cmd_vel_time_{-1.0};

  std::mutex cmd_mutex_;  // guards lowcmd_msg_ / joint_cmd_ (control thread vs. safe_stop)
  JointCommand joint_cmd_;
  LowCmd lowcmd_msg_;

  // control loop statistics (control thread writes, state timer reads)
  std::mutex stats_mutex_;
  double max_cycle_ms_{0.0};
  int cycle_count_{0};
  double control_rate_measured_{0.0};
  double stats_window_start_{0.0};

  const std::chrono::steady_clock::time_point t_start_{std::chrono::steady_clock::now()};
};

}  // namespace go2_policy_runner::ros
