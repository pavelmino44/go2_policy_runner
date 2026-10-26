// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// Maps Linux input events (evdev) of a gamepad or a keyboard to a Unitree remote frame, so that
// the FSM remote expressions ("LT + A.on_pressed", ...) and the stick -> cmd_vel mapping work
// the same with every input device. No ROS and no device access here (see evdev_device.hpp).

#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "go2_policy_runner/core/types.hpp"
#include "go2_policy_runner/remote/unitree_remote.hpp"

namespace go2_policy_runner::input
{

/// The 16 buttons of the Unitree remote.
enum class Button : std::uint8_t
{
  kR1, kL1, kStart, kSelect, kR2, kL2, kF1, kF2,
  kA, kB, kX, kY, kUp, kRight, kDown, kLeft,
};

/// Case-insensitive button name as used in the FSM expressions:
/// LT/L2, RT/R2, LB/L1, RB/R1, A, B, X, Y, up, down, left, right, start, back/select, F1, F2.
std::optional<Button> button_from_name(const std::string & name);
void set_button(RemoteFrame & frame, Button b, bool pressed);

/// One evdev event (struct input_event without the timestamp).
struct Event
{
  std::uint16_t type{0};
  std::uint16_t code{0};
  std::int32_t value{0};
};

/// Raw range of an absolute axis (struct input_absinfo).
struct AxisRange
{
  std::int32_t min{-32768};
  std::int32_t max{32767};
};

// ---------------------------------------------------------------------------------------------
/// Gamepad -> remote frame. The default layout is the DualShock 4 with the Linux hid-sony /
/// hid-playstation driver; Xbox-compatible pads (xpad) use the same event codes.
///
///   DualShock 4        Unitree remote       DualShock 4        Unitree remote
///   cross              A                    L1 / R1            LB / RB (L1 / R1)
///   circle             B                    L2 / R2            LT / RT (L2 / R2)
///   square             X                    options            start
///   triangle           Y                    share              back (select)
///   d-pad              up/down/left/right   L3 / R3            F1 / F2
///   left stick         lx, ly               right stick        rx, ry
class GamepadMapper
{
public:
  struct Layout
  {
    std::map<std::uint16_t, Button> buttons;  ///< EV_KEY code -> button
    std::uint16_t lx, ly, rx, ry;              ///< EV_ABS codes of the sticks
    bool invert_ly{true}, invert_ry{true};     ///< evdev: down is positive, Unitree: up
    std::uint16_t l2_axis, r2_axis;            ///< analog triggers, also count as L2 / R2
    std::uint16_t hat_x, hat_y;                ///< d-pad as a hat switch
    static Layout dualshock4();
  };

  explicit GamepadMapper(Layout layout = Layout::dualshock4(), float trigger_threshold = 0.5f);

  /// Raw ranges of the device axes (from EVIOCGABS). Unknown axes use [-32768, 32767].
  void set_axis_range(std::uint16_t code, AxisRange range);
  /// EV_ABS codes the mapper uses (to query their ranges from the device).
  std::vector<std::uint16_t> axes() const;

  void handle(const Event & e);
  /// All buttons released, sticks centered (device lost or events dropped).
  void reset();
  /// Frame for one publish period. A button that was pressed at any time since the previous
  /// call is reported as pressed, so short taps are never lost.
  RemoteFrame take_frame();

private:
  float normalize(std::uint16_t code, std::int32_t value) const;
  std::uint16_t current_bits() const;

  Layout layout_;
  float trigger_threshold_;
  std::map<std::uint16_t, AxisRange> ranges_;
  std::map<std::uint16_t, bool> keys_;  ///< EV_KEY code -> pressed
  bool l2_analog_{false}, r2_analog_{false};
  int hat_x_{0}, hat_y_{0};
  float lx_{0.0f}, ly_{0.0f}, rx_{0.0f}, ry_{0.0f};
  std::uint16_t latched_{0};    ///< buttons pressed since the last take_frame()
  std::uint16_t last_bits_{0};
};

// ---------------------------------------------------------------------------------------------
/// Keyboard -> remote frame through bindings "<KEY>: <target> [+ <target> ...]":
///
///   W: ly+          stick direction (lx, ly, rx, ry with + or -), magnitude `stick`
///   1: LT + A       several remote buttons; the last one is pressed `combo_delay` later, so
///                   expressions like "LT + A.on_pressed" see LT held before A
///   LEFTSHIFT: fast while held, sticks use full deflection instead of `stick`
///
/// Key names are the Linux KEY_* names without the prefix, case-insensitive
/// (W, 1, SPACE, ENTER, UP, LEFTSHIFT, F1, KP0, ...).
class KeyboardMapper
{
public:
  struct Options
  {
    float stick{0.5f};          ///< stick deflection of direction keys
    double combo_delay{0.15};   ///< [s] delay of the last button of a combination
  };

  static std::vector<std::string> default_bindings();

  /// @throws std::invalid_argument on a malformed binding or an unknown key / target
  KeyboardMapper(const std::vector<std::string> & bindings, Options options);
  explicit KeyboardMapper(const std::vector<std::string> & bindings)
  : KeyboardMapper(bindings, Options{}) {}

  /// @param now  time of the event [s] (monotonic, same clock as take_frame)
  void handle(const Event & e, double now);
  void reset();
  bool binds(std::uint16_t key) const;
  RemoteFrame take_frame(double now);

private:
  enum class Kind { kButtons, kStick, kFast };
  struct Binding
  {
    std::uint16_t key{0};
    Kind kind{Kind::kButtons};
    std::vector<Button> buttons;
    int axis{0};           ///< 0 lx, 1 ly, 2 rx, 3 ry
    float sign{1.0f};
    // runtime state
    bool held{false};
    bool pending{false};    ///< pressed, and the buttons have not been reported completely yet
    double pressed_at{0.0};
  };

  std::vector<Binding> bindings_;
  Options options_;
};

/// Linux key code for a KEY_* name without the prefix (case-insensitive, "KEY_" accepted).
std::optional<std::uint16_t> key_code_from_name(const std::string & name);

// ---------------------------------------------------------------------------------------------
/// Remote sticks -> base velocity (the mapping of remote_teleop).
///   left stick up/down -> vx, left stick left/right -> vy, right stick left/right -> wz
struct TeleopLimits
{
  double max_vx{1.0};    ///< [m/s]
  double max_vy{0.5};    ///< [m/s]
  double max_wz{1.0};    ///< [rad/s]
  double deadzone{0.05};
};

VelocityCommand velocity_from_frame(const RemoteFrame & frame, const TeleopLimits & limits);

}  // namespace go2_policy_runner::input
