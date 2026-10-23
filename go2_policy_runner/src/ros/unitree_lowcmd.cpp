// Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
//
// Contains code derived from unitree_ros2 (https://github.com/unitreerobotics/unitree_ros2) (BSD 3-Clause License, see THIRD_PARTY_NOTICES.md):
//   example/src/include/common/motor_crc.h
//   example/src/src/common/motor_crc.cpp
// Changes by unitree-go2-rl contributors (2026):
//   - deterministic padding, static layout checks
//   - LowCmd initialization and LowState conversion helpers

#include "go2_policy_runner/ros/unitree_lowcmd.hpp"

#include <array>
#include <cstring>

namespace go2_policy_runner::ros
{

namespace
{
// Raw in-memory layout the robot computes the CRC over (natural alignment, 812 bytes).
// Copied from unitree_ros2 example/src/include/common/motor_crc.h.
struct RawBmsCmd
{
  std::uint8_t off;
  std::array<std::uint8_t, 3> reserve;
};

struct RawMotorCmd
{
  std::uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  std::array<std::uint32_t, 3> reserve;
};

struct RawLowCmd
{
  std::array<std::uint8_t, 2> head;
  std::uint8_t level_flag;
  std::uint8_t frame_reserve;
  std::array<std::uint32_t, 2> sn;
  std::array<std::uint32_t, 2> version;
  std::uint16_t bandwidth;
  std::array<RawMotorCmd, 20> motor_cmd;
  RawBmsCmd bms;
  std::array<std::uint8_t, 40> wireless_remote;
  std::array<std::uint8_t, 12> led;
  std::array<std::uint8_t, 2> fan;
  std::uint8_t gpio;
  std::uint32_t reserve;
  std::uint32_t crc;
};

static_assert(sizeof(RawMotorCmd) == 36, "unexpected RawMotorCmd layout");
static_assert(sizeof(RawLowCmd) == 812, "unexpected RawLowCmd layout");
}  // namespace

std::uint32_t unitree_crc32(const std::uint32_t * data, std::uint32_t len)
{
  std::uint32_t crc = 0xFFFFFFFF;
  constexpr std::uint32_t kPoly = 0x04c11db7;
  for (std::uint32_t i = 0; i < len; ++i) {
    std::uint32_t xbit = 1u << 31;
    const std::uint32_t word = data[i];
    for (int bit = 0; bit < 32; ++bit) {
      if (crc & 0x80000000u) {
        crc = (crc << 1) ^ kPoly;
      } else {
        crc <<= 1;
      }
      if (word & xbit) {crc ^= kPoly;}
      xbit >>= 1;
    }
  }
  return crc;
}

void update_lowcmd_crc(unitree_go::msg::LowCmd & msg)
{
  RawLowCmd raw;
  std::memset(&raw, 0, sizeof(raw));  // deterministic padding bytes
  std::memcpy(raw.head.data(), msg.head.data(), 2);
  raw.level_flag = msg.level_flag;
  raw.frame_reserve = msg.frame_reserve;
  std::memcpy(raw.sn.data(), msg.sn.data(), sizeof(raw.sn));
  std::memcpy(raw.version.data(), msg.version.data(), sizeof(raw.version));
  raw.bandwidth = msg.bandwidth;
  for (std::size_t i = 0; i < 20; ++i) {
    const auto & m = msg.motor_cmd[i];
    auto & r = raw.motor_cmd[i];
    r.mode = m.mode;
    r.q = m.q;
    r.dq = m.dq;
    r.tau = m.tau;
    r.kp = m.kp;
    r.kd = m.kd;
    std::memcpy(r.reserve.data(), m.reserve.data(), sizeof(r.reserve));
  }
  raw.bms.off = msg.bms_cmd.off;
  std::memcpy(raw.bms.reserve.data(), msg.bms_cmd.reserve.data(), 3);
  std::memcpy(raw.wireless_remote.data(), msg.wireless_remote.data(), 40);
  std::memcpy(raw.led.data(), msg.led.data(), 12);
  std::memcpy(raw.fan.data(), msg.fan.data(), 2);
  raw.gpio = msg.gpio;
  raw.reserve = msg.reserve;

  msg.crc = unitree_crc32(
    reinterpret_cast<const std::uint32_t *>(&raw), (sizeof(RawLowCmd) >> 2) - 1);
}

void init_lowcmd(unitree_go::msg::LowCmd & msg, std::size_t num_motors)
{
  msg = unitree_go::msg::LowCmd();
  msg.head[0] = 0xFE;
  msg.head[1] = 0xEF;
  msg.level_flag = 0xFF;  // low-level control
  msg.gpio = 0;
  for (std::size_t i = 0; i < msg.motor_cmd.size(); ++i) {
    auto & m = msg.motor_cmd[i];
    m.mode = 0x01;  // servo (PMSM) mode
    const bool used = i < num_motors;
    m.q = used ? 0.0f : kPosStop;
    m.dq = used ? 0.0f : kVelStop;
    m.kp = 0.0f;
    m.kd = 0.0f;
    m.tau = 0.0f;
  }
  update_lowcmd_crc(msg);
}

void write_lowcmd(const JointCommand & cmd, unitree_go::msg::LowCmd & msg)
{
  for (std::size_t i = 0; i < cmd.size() && i < msg.motor_cmd.size(); ++i) {
    auto & m = msg.motor_cmd[i];
    m.q = cmd[i].q;
    m.dq = cmd[i].dq;
    m.kp = cmd[i].kp;
    m.kd = cmd[i].kd;
    m.tau = cmd[i].tau;
  }
}

void read_lowstate(const unitree_go::msg::LowState & msg, RobotState & state)
{
  for (std::size_t i = 0; i < 4; ++i) {state.quaternion[i] = msg.imu_state.quaternion[i];}
  for (std::size_t i = 0; i < 3; ++i) {
    state.gyroscope[i] = msg.imu_state.gyroscope[i];
    state.accelerometer[i] = msg.imu_state.accelerometer[i];
  }
  for (std::size_t i = 0; i < state.q.size() && i < msg.motor_state.size(); ++i) {
    state.q[i] = msg.motor_state[i].q;
    state.dq[i] = msg.motor_state[i].dq;
    state.tau[i] = msg.motor_state[i].tau_est;
  }
  state.tick = msg.tick;
  std::memcpy(state.wireless_remote.data(), msg.wireless_remote.data(), 40);
}

}  // namespace go2_policy_runner::ros
