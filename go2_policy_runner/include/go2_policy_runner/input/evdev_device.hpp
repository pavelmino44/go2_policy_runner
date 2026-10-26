// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// Minimal reader for Linux input devices (/dev/input/event*), no libevdev needed.
// Reading requires access to the device node: gamepads usually get it through the logged-in
// seat (uaccess); keyboards need membership in the `input` group.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "go2_policy_runner/input/input_mapping.hpp"

namespace go2_policy_runner::input
{

enum class DeviceKind { kGamepad, kKeyboard };

struct DeviceInfo
{
  std::string path;
  std::string name;
};

/// Input devices of the given kind, sorted by path. `name_filter` is a case-insensitive
/// substring of the device name ("" = any). Devices that cannot be opened are skipped;
/// `permission_denied` (optional) is set if at least one was skipped for lack of permissions.
std::vector<DeviceInfo> find_devices(
  DeviceKind kind, const std::string & name_filter, bool * permission_denied = nullptr);

class EvdevDevice
{
public:
  /// @throws std::runtime_error if the device cannot be opened
  explicit EvdevDevice(const std::string & path);
  ~EvdevDevice();
  EvdevDevice(const EvdevDevice &) = delete;
  EvdevDevice & operator=(const EvdevDevice &) = delete;

  int fd() const {return fd_;}
  const std::string & path() const {return path_;}
  const std::string & name() const {return name_;}
  bool is(DeviceKind kind) const;
  AxisRange axis_range(std::uint16_t code) const;
  /// Exclusive access: while grabbed, no other program (X/Wayland, terminals) gets the events.
  bool grab(bool on);

  /// Current state as synthetic events (all pressed keys, values of all absolute axes).
  std::vector<Event> snapshot() const;

  /// Read the pending events without blocking and append them to `out`.
  /// After a kernel buffer overflow, `out` receives {EV_SYN, SYN_DROPPED} followed by a
  /// snapshot(): the consumer should reset its state when it sees SYN_DROPPED.
  /// @return false if the device is gone (unplugged, Bluetooth disconnected)
  bool read(std::vector<Event> & out);

private:
  int fd_{-1};
  std::string path_;
  std::string name_;
  bool dropping_{false};
};

}  // namespace go2_policy_runner::input
