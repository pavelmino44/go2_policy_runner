// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// Plain data types shared by the ROS-agnostic core (policy, FSM, remote).
// The ROS layer converts unitree_go/LowState <-> these types, so everything
// below this header can be unit-tested without ROS.

#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace go2_policy_runner
{

/// Number of motor slots in unitree_go/LowCmd and LowState.
constexpr std::size_t kMaxMotors = 20;

/// Snapshot of the robot low-level state, motors in SDK order
/// (FR_hip, FR_thigh, FR_calf, FL_..., RR_..., RL_...).
struct RobotState
{
  std::array<float, 4> quaternion{1.0f, 0.0f, 0.0f, 0.0f};  ///< w, x, y, z (body -> world)
  std::array<float, 3> gyroscope{0.0f, 0.0f, 0.0f};         ///< body frame, rad/s
  std::array<float, 3> accelerometer{0.0f, 0.0f, 0.0f};     ///< body frame, m/s^2
  std::vector<float> q;                                     ///< joint positions [rad]
  std::vector<float> dq;                                    ///< joint velocities [rad/s]
  std::vector<float> tau;                                   ///< estimated torques [Nm]
  std::uint32_t tick{0};
  std::array<std::uint8_t, 40> wireless_remote{};           ///< raw Unitree remote frame

  explicit RobotState(std::size_t num_motors = 12)
  : q(num_motors, 0.0f), dq(num_motors, 0.0f), tau(num_motors, 0.0f) {}
};

/// Command for a single motor (joint-space PD + feed-forward torque).
struct MotorCommand
{
  float q{0.0f};
  float dq{0.0f};
  float kp{0.0f};
  float kd{0.0f};
  float tau{0.0f};
};

/// Command for all used motors, SDK order.
using JointCommand = std::vector<MotorCommand>;

/// Base velocity command in the body frame (same meaning as geometry_msgs/Twist on /cmd_vel).
struct VelocityCommand
{
  float vx{0.0f};  ///< forward [m/s]
  float vy{0.0f};  ///< left [m/s]
  float wz{0.0f};  ///< yaw rate, CCW [rad/s]
};

}  // namespace go2_policy_runner
