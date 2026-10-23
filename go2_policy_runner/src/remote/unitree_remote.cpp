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

#include "go2_policy_runner/remote/unitree_remote.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <unordered_map>

namespace go2_policy_runner
{

namespace
{
// Layout of the remote frame (little-endian, packed exactly like unitree_sdk2 REMOTE_DATA_RX):
//   [0..1]   head
//   [2..3]   buttons bitfield (uint16)
//   [4..7]   lx (float)
//   [8..11]  rx (float)
//   [12..15] ry (float)
//   [16..19] L2 analog (float, unused)
//   [20..23] ly (float)
//   [24..39] reserved
constexpr std::size_t kBtnOffset = 2;
constexpr std::size_t kLxOffset = 4;
constexpr std::size_t kRxOffset = 8;
constexpr std::size_t kRyOffset = 12;
constexpr std::size_t kLyOffset = 20;

enum Bit : std::uint16_t
{
  kR1 = 1u << 0, kL1 = 1u << 1, kStart = 1u << 2, kSelect = 1u << 3,
  kR2 = 1u << 4, kL2 = 1u << 5, kF1 = 1u << 6, kF2 = 1u << 7,
  kA = 1u << 8, kB = 1u << 9, kX = 1u << 10, kY = 1u << 11,
  kUp = 1u << 12, kRight = 1u << 13, kDown = 1u << 14, kLeft = 1u << 15,
};

float read_float(const std::array<std::uint8_t, 40> & raw, std::size_t offset)
{
  float v;
  std::memcpy(&v, raw.data() + offset, sizeof(float));
  return std::isfinite(v) ? v : 0.0f;
}

void write_float(std::array<std::uint8_t, 40> & raw, std::size_t offset, float v)
{
  std::memcpy(raw.data() + offset, &v, sizeof(float));
}
}  // namespace

RemoteFrame RemoteFrame::decode(const std::array<std::uint8_t, 40> & raw)
{
  RemoteFrame f;
  const std::uint16_t b = static_cast<std::uint16_t>(raw[kBtnOffset] | (raw[kBtnOffset + 1] << 8));
  f.R1 = b & kR1; f.L1 = b & kL1; f.start = b & kStart; f.select = b & kSelect;
  f.R2 = b & kR2; f.L2 = b & kL2; f.F1 = b & kF1; f.F2 = b & kF2;
  f.A = b & kA; f.B = b & kB; f.X = b & kX; f.Y = b & kY;
  f.up = b & kUp; f.right = b & kRight; f.down = b & kDown; f.left = b & kLeft;
  f.lx = read_float(raw, kLxOffset);
  f.rx = read_float(raw, kRxOffset);
  f.ry = read_float(raw, kRyOffset);
  f.ly = read_float(raw, kLyOffset);
  return f;
}

std::array<std::uint8_t, 40> RemoteFrame::encode() const
{
  std::array<std::uint8_t, 40> raw{};
  std::uint16_t b = 0;
  if (R1) b |= kR1;
  if (L1) b |= kL1;
  if (start) b |= kStart;
  if (select) b |= kSelect;
  if (R2) b |= kR2;
  if (L2) b |= kL2;
  if (F1) b |= kF1;
  if (F2) b |= kF2;
  if (A) b |= kA;
  if (B) b |= kB;
  if (X) b |= kX;
  if (Y) b |= kY;
  if (up) b |= kUp;
  if (right) b |= kRight;
  if (down) b |= kDown;
  if (left) b |= kLeft;
  raw[kBtnOffset] = static_cast<std::uint8_t>(b & 0xFF);
  raw[kBtnOffset + 1] = static_cast<std::uint8_t>(b >> 8);
  write_float(raw, kLxOffset, lx);
  write_float(raw, kRxOffset, rx);
  write_float(raw, kRyOffset, ry);
  write_float(raw, kLyOffset, ly);
  return raw;
}

bool RemoteFrame::is_empty(const std::array<std::uint8_t, 40> & raw)
{
  return std::all_of(raw.begin(), raw.end(), [](std::uint8_t v) {return v == 0;});
}

void KeyState::update(bool is_pressed, double now)
{
  pressed = is_pressed;
  on_pressed = pressed && !last_pressed_;
  on_released = !pressed && last_pressed_;
  last_pressed_ = pressed;

  if (on_pressed) {
    click_count = (now - last_click_time_ < kDoubleClickThreshold) ? click_count + 1 : 1;
    last_click_time_ = now;
  }
  if (pressed) {
    pressed_time = static_cast<float>(now - last_click_time_);
  }
  if (!(pressed || on_released)) {
    pressed_time = 0.0f;
  }
}

void AxisState::update(float raw, double now)
{
  const float dz = std::fabs(raw) < deadzone ? 0.0f : raw;
  value = value * (1.0f - smooth) + dz * smooth;
  KeyState::update(value > threshold, now);
}

void UnitreeRemote::update(const RemoteFrame & f, double now)
{
  back.update(f.select, now);
  start.update(f.start, now);
  LB.update(f.L1, now);
  RB.update(f.R1, now);
  F1.update(f.F1, now);
  F2.update(f.F2, now);
  A.update(f.A, now);
  B.update(f.B, now);
  X.update(f.X, now);
  Y.update(f.Y, now);
  up.update(f.up, now);
  down.update(f.down, now);
  left.update(f.left, now);
  right.update(f.right, now);
  LT.update(f.L2 ? 1.0f : 0.0f, now);
  RT.update(f.R2 ? 1.0f : 0.0f, now);
  lx.update(f.lx, now);
  ly.update(f.ly, now);
  rx.update(f.rx, now);
  ry.update(f.ry, now);
  // LS / RS are not present in the Unitree frame; they stay released.
}

UnitreeRemote::KeyGetter UnitreeRemote::find_key(const std::string & name)
{
  std::string n = name;
  std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) {return std::tolower(c);});
  static const std::unordered_map<std::string, KeyGetter> kMap = {
    {"back", [](const UnitreeRemote & r) -> const KeyState * {return &r.back;}},
    {"select", [](const UnitreeRemote & r) -> const KeyState * {return &r.back;}},
    {"start", [](const UnitreeRemote & r) -> const KeyState * {return &r.start;}},
    {"ls", [](const UnitreeRemote & r) -> const KeyState * {return &r.LS;}},
    {"rs", [](const UnitreeRemote & r) -> const KeyState * {return &r.RS;}},
    {"lb", [](const UnitreeRemote & r) -> const KeyState * {return &r.LB;}},
    {"l1", [](const UnitreeRemote & r) -> const KeyState * {return &r.LB;}},
    {"rb", [](const UnitreeRemote & r) -> const KeyState * {return &r.RB;}},
    {"r1", [](const UnitreeRemote & r) -> const KeyState * {return &r.RB;}},
    {"a", [](const UnitreeRemote & r) -> const KeyState * {return &r.A;}},
    {"b", [](const UnitreeRemote & r) -> const KeyState * {return &r.B;}},
    {"x", [](const UnitreeRemote & r) -> const KeyState * {return &r.X;}},
    {"y", [](const UnitreeRemote & r) -> const KeyState * {return &r.Y;}},
    {"up", [](const UnitreeRemote & r) -> const KeyState * {return &r.up;}},
    {"down", [](const UnitreeRemote & r) -> const KeyState * {return &r.down;}},
    {"left", [](const UnitreeRemote & r) -> const KeyState * {return &r.left;}},
    {"right", [](const UnitreeRemote & r) -> const KeyState * {return &r.right;}},
    {"f1", [](const UnitreeRemote & r) -> const KeyState * {return &r.F1;}},
    {"f2", [](const UnitreeRemote & r) -> const KeyState * {return &r.F2;}},
    {"lx", [](const UnitreeRemote & r) -> const KeyState * {return &r.lx;}},
    {"ly", [](const UnitreeRemote & r) -> const KeyState * {return &r.ly;}},
    {"rx", [](const UnitreeRemote & r) -> const KeyState * {return &r.rx;}},
    {"ry", [](const UnitreeRemote & r) -> const KeyState * {return &r.ry;}},
    {"lt", [](const UnitreeRemote & r) -> const KeyState * {return &r.LT;}},
    {"l2", [](const UnitreeRemote & r) -> const KeyState * {return &r.LT;}},
    {"rt", [](const UnitreeRemote & r) -> const KeyState * {return &r.RT;}},
    {"r2", [](const UnitreeRemote & r) -> const KeyState * {return &r.RT;}},
  };
  auto it = kMap.find(n);
  return it == kMap.end() ? nullptr : it->second;
}

}  // namespace go2_policy_runner
