// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// Standalone executable for the policy controller. Differs from loading the component into a
// container in one important way: on SIGINT/SIGTERM the robot is switched to damping (passive)
// and that command is published for a short time *before* the ROS context is shut down.

#include <atomic>
#include <csignal>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "go2_policy_runner/ros/policy_controller_node.hpp"

namespace
{
std::atomic<bool> g_stop{false};
void on_signal(int) {g_stop = true;}
}  // namespace

int main(int argc, char ** argv)
{
  // Own signal handling: rclcpp must not invalidate the context before we sent damping commands.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::shared_ptr<go2_policy_runner::ros::PolicyControllerNode> node;
  try {
    node = std::make_shared<go2_policy_runner::ros::PolicyControllerNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("policy_controller"), "configuration error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // control timer + I/O callbacks + motion-switcher responses run in parallel
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  while (!g_stop && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  node->safe_stop(std::chrono::milliseconds(300));
  executor.cancel();
  spinner.join();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
