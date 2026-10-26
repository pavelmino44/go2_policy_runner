// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
//
// Contains code derived from unitree_sdk2 (https://github.com/unitreerobotics/unitree_sdk2) (BSD 3-Clause License, see THIRD_PARTY_NOTICES.md):
//   include/unitree/dds_wrapper/common/unitree_joystick.hpp
// Changes by unitree-go2-rl contributors (2026):
//   - time is injected (no hidden steady_clock calls)
//   - frame encode/decode without type punning, raw stick values, key aliases
//   - button bitfield accessors (for unitree_go/WirelessController)
//
// Decoder for the 40-byte Unitree wireless remote frame (LowState.wireless_remote).

#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace go2_policy_runner
{

/// Raw content of the remote frame.
struct RemoteFrame
{
  // buttons
  bool R1{false}, L1{false}, start{false}, select{false}, R2{false}, L2{false}, F1{false}, F2{false};
  bool A{false}, B{false}, X{false}, Y{false}, up{false}, right{false}, down{false}, left{false};
  // sticks, [-1, 1]
  float lx{0.0f}, rx{0.0f}, ry{0.0f}, ly{0.0f};

  /// Decode from LowState.wireless_remote.
  static RemoteFrame decode(const std::array<std::uint8_t, 40> & raw);
  /// Inverse of decode (used in tests and by simulators).
  std::array<std::uint8_t, 40> encode() const;
  /// True if every byte of the frame is zero (no remote connected / no data).
  static bool is_empty(const std::array<std::uint8_t, 40> & raw);

  /// 16-bit button field, same bit layout as the frame and unitree_go/WirelessController.keys.
  std::uint16_t buttons() const;
  void set_buttons(std::uint16_t bits);
};

/// Edge/hold tracking for one key (same semantics as unitree::common::KeyBase).
class KeyState
{
public:
  bool pressed{false};
  bool on_pressed{false};
  bool on_released{false};
  int click_count{0};
  float pressed_time{0.0f};  ///< seconds the key has been held (kept for one cycle after release)

  void update(bool is_pressed, double now);

private:
  bool last_pressed_{false};
  double last_click_time_{-1e9};
  static constexpr double kDoubleClickThreshold = 0.5;
};

/// Axis = smoothed analog value that also behaves like a key when above threshold.
class AxisState : public KeyState
{
public:
  float value{0.0f};
  float smooth{0.03f};
  float deadzone{0.01f};
  float threshold{0.5f};

  void update(float raw, double now);
};

/// Stateful remote: feed it every control cycle with the latest frame.
/// Key names follow unitree_sdk2 / unitree_rl_lab deploy (case-insensitive in the DSL):
///   back start LB RB A B X Y up down left right F1 F2 LT RT lx ly rx ry
class UnitreeRemote
{
public:
  KeyState back, start, LS, RS, LB, RB, A, B, X, Y, up, down, left, right, F1, F2;
  AxisState lx, ly, rx, ry, LT, RT;

  void update(const RemoteFrame & frame, double now);
  void update(const std::array<std::uint8_t, 40> & raw, double now)
  {
    update(RemoteFrame::decode(raw), now);
  }

  /// Accessor for one key of a remote object.
  using KeyGetter = const KeyState * (*)(const UnitreeRemote &);

  /// Resolve a (case-insensitive) key name to an accessor; returns nullptr if unknown.
  static KeyGetter find_key(const std::string & name);

  /// Look up a key by (case-insensitive) name; returns nullptr if unknown.
  const KeyState * key(const std::string & name) const
  {
    const auto getter = find_key(name);
    return getter ? getter(*this) : nullptr;
  }
};

}  // namespace go2_policy_runner
