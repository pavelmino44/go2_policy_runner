// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// ROS-agnostic part of the robot monitor: everything the screen shows is collected into a
// Snapshot, and the alert rules are evaluated on it. Kept free of ROS / ncurses so the rules
// can be unit-tested.

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace go2_policy_runner::monitor
{

enum class Level { kOk = 0, kWarn = 1, kError = 2 };

/// Alert categories, also used as /diagnostics status names.
enum class Category { kController, kCommunication, kCommand, kBase, kJoints, kPolicy, kBattery };

const char * category_name(Category c);

struct Alert
{
  Level level;
  Category category;
  std::string text;
};

struct Thresholds
{
  double motor_temp_warn{65.0};      ///< [degC]
  double joint_limit_margin{0.05};   ///< fraction of the joint range
  double battery_warn{20.0};         ///< [%]
  double lowstate_rate_ratio{0.8};   ///< of the nominal rate
  double tilt_ratio{0.7};            ///< of max_tilt
  double inference_ratio{0.5};       ///< of the policy period
};

struct JointSnapshot
{
  std::string name;
  float q{0}, dq{0}, tau{0};
  float q_cmd{0}, kp{0}, kd{0};
  int temperature{0};
  bool has_limits{false};
  float lower{0}, upper{0};
};

/// Everything the monitor knows at one instant. Flags tell which sources are available.
struct Snapshot
{
  double now{0.0};

  // --- controller (~/state) ---
  bool have_state{false};
  double state_age{1e9};
  bool ready{false};
  std::string mode, mode_type, last_reason;
  std::vector<std::string> allowed;
  float mode_time{0}, lowstate_rate{0}, control_rate{0}, max_cycle_ms{0};
  bool lowstate_ok{false}, cmd_vel_ok{false};
  double nominal_lowstate_rate{0.0};  ///< 0 = unknown

  // --- lowstate / lowcmd ---
  bool have_lowstate{false};
  double lowstate_age{1e9};
  bool have_lowcmd{false};
  std::array<float, 4> quat{1, 0, 0, 0};
  std::array<float, 3> gyro{0, 0, 0}, acc{0, 0, 0};
  std::vector<JointSnapshot> joints;
  bool battery_valid{false};
  int battery_soc{0};
  float power_v{0}, power_a{0};
  std::array<int, 4> foot_force{0, 0, 0, 0};
  std::array<std::uint8_t, 40> wireless_remote{};

  // --- remote topic (input_teleop: DualShock 4 / keyboard) ---
  bool have_remote_topic{false};
  double remote_topic_age{1e9};
  std::array<std::uint8_t, 40> remote_topic{};  ///< encoded like LowState.wireless_remote

  // --- command ---
  bool have_cmd_vel{false};
  double cmd_vel_age{1e9};
  std::array<double, 3> cmd_vel{0, 0, 0};  ///< vx, vy, wz
  std::vector<std::string> cmd_vel_publishers;

  // --- policy (~/policy_step) ---
  bool have_policy{false};
  double policy_age{1e9};
  std::uint64_t policy_step{0};
  float inference_ms{0};
  double policy_period_ms{0};  ///< measured interval between policy steps
  float action_min{0}, action_max{0};
  std::array<double, 3> policy_cmd{0, 0, 0};

  // --- configuration (controller parameters) ---
  double max_tilt{0.0};  ///< of the active mode, <= 0: none
};

/// roll, pitch, yaw [rad] from a w,x,y,z quaternion (ZYX convention).
std::array<double, 3> quat_to_rpy(const std::array<float, 4> & q);
/// Angle between the body z axis and the world z axis [rad].
double tilt_from_quat(const std::array<float, 4> & q);
/// Position of q inside [lower, upper] as a fraction in [0, 1] (clamped).
double range_fraction(float q, float lower, float upper);

/// Evaluate all alert rules. Alerts are ordered by severity (errors first).
std::vector<Alert> compute_alerts(const Snapshot & s, const Thresholds & t);

/// Worst level per category (categories without alerts are kOk).
Level worst_level(const std::vector<Alert> & alerts, Category c);

/// Standard Go2 joint names in SDK order (FR, FL, RR, RL x hip, thigh, calf).
std::string go2_joint_name(std::size_t index);

}  // namespace go2_policy_runner::monitor
