// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include "go2_policy_runner/input/input_mapping.hpp"

#include <linux/input-event-codes.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace go2_policy_runner::input
{

namespace
{
std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

std::string upper(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::toupper(c);});
  return s;
}

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t");
  if (b == std::string::npos) {return "";}
  const auto e = s.find_last_not_of(" \t");
  return s.substr(b, e - b + 1);
}

std::uint16_t bit(Button b) {return static_cast<std::uint16_t>(1u << static_cast<int>(b));}
}  // namespace

std::optional<Button> button_from_name(const std::string & name)
{
  static const std::unordered_map<std::string, Button> kMap = {
    {"r1", Button::kR1}, {"rb", Button::kR1}, {"l1", Button::kL1}, {"lb", Button::kL1},
    {"start", Button::kStart}, {"select", Button::kSelect}, {"back", Button::kSelect},
    {"r2", Button::kR2}, {"rt", Button::kR2}, {"l2", Button::kL2}, {"lt", Button::kL2},
    {"f1", Button::kF1}, {"f2", Button::kF2},
    {"a", Button::kA}, {"b", Button::kB}, {"x", Button::kX}, {"y", Button::kY},
    {"up", Button::kUp}, {"right", Button::kRight}, {"down", Button::kDown}, {"left", Button::kLeft},
  };
  const auto it = kMap.find(lower(name));
  if (it == kMap.end()) {return std::nullopt;}
  return it->second;
}

void set_button(RemoteFrame & frame, Button b, bool pressed)
{
  std::uint16_t bits = frame.buttons();
  bits = pressed ? (bits | bit(b)) : (bits & ~bit(b));
  frame.set_buttons(bits);
}

// =============================================================================================
// GamepadMapper
// =============================================================================================
GamepadMapper::Layout GamepadMapper::Layout::dualshock4()
{
  Layout l;
  l.buttons = {
    {BTN_SOUTH, Button::kA},       // cross
    {BTN_EAST, Button::kB},        // circle
    {BTN_WEST, Button::kX},        // square
    {BTN_NORTH, Button::kY},       // triangle
    {BTN_TL, Button::kL1},
    {BTN_TR, Button::kR1},
    {BTN_TL2, Button::kL2},
    {BTN_TR2, Button::kR2},
    {BTN_START, Button::kStart},   // options
    {BTN_SELECT, Button::kSelect}, // share
    {BTN_THUMBL, Button::kF1},     // L3
    {BTN_THUMBR, Button::kF2},     // R3
    // d-pad reported as buttons by some drivers
    {BTN_DPAD_UP, Button::kUp},
    {BTN_DPAD_DOWN, Button::kDown},
    {BTN_DPAD_LEFT, Button::kLeft},
    {BTN_DPAD_RIGHT, Button::kRight},
  };
  l.lx = ABS_X;
  l.ly = ABS_Y;
  l.rx = ABS_RX;
  l.ry = ABS_RY;
  l.l2_axis = ABS_Z;
  l.r2_axis = ABS_RZ;
  l.hat_x = ABS_HAT0X;
  l.hat_y = ABS_HAT0Y;
  return l;
}

GamepadMapper::GamepadMapper(Layout layout, float trigger_threshold)
: layout_(std::move(layout)), trigger_threshold_(trigger_threshold)
{
  // DualShock 4 defaults; replaced by the real ranges of the device.
  for (auto c : {layout_.lx, layout_.ly, layout_.rx, layout_.ry}) {ranges_[c] = {0, 255};}
  for (auto c : {layout_.l2_axis, layout_.r2_axis}) {ranges_[c] = {0, 255};}
}

void GamepadMapper::set_axis_range(std::uint16_t code, AxisRange range)
{
  if (range.max > range.min) {ranges_[code] = range;}
}

std::vector<std::uint16_t> GamepadMapper::axes() const
{
  return {layout_.lx, layout_.ly, layout_.rx, layout_.ry, layout_.l2_axis, layout_.r2_axis,
    layout_.hat_x, layout_.hat_y};
}

float GamepadMapper::normalize(std::uint16_t code, std::int32_t value) const
{
  AxisRange r;
  const auto it = ranges_.find(code);
  if (it != ranges_.end()) {r = it->second;}
  const double v = 2.0 * (static_cast<double>(value) - r.min) / (static_cast<double>(r.max) - r.min) - 1.0;
  return static_cast<float>(std::clamp(v, -1.0, 1.0));
}

void GamepadMapper::handle(const Event & e)
{
  if (e.type == EV_KEY) {
    if (layout_.buttons.count(e.code) == 0) {return;}
    const bool pressed = e.value != 0;
    keys_[e.code] = pressed;
  } else if (e.type == EV_ABS) {
    const auto c = e.code;
    if (c == layout_.lx) {
      lx_ = normalize(c, e.value);
    } else if (c == layout_.ly) {
      ly_ = normalize(c, e.value) * (layout_.invert_ly ? -1.0f : 1.0f);
    } else if (c == layout_.rx) {
      rx_ = normalize(c, e.value);
    } else if (c == layout_.ry) {
      ry_ = normalize(c, e.value) * (layout_.invert_ry ? -1.0f : 1.0f);
    } else if (c == layout_.l2_axis) {
      l2_analog_ = (normalize(c, e.value) + 1.0f) / 2.0f > trigger_threshold_;
    } else if (c == layout_.r2_axis) {
      r2_analog_ = (normalize(c, e.value) + 1.0f) / 2.0f > trigger_threshold_;
    } else if (c == layout_.hat_x) {
      hat_x_ = (e.value > 0) - (e.value < 0);
    } else if (c == layout_.hat_y) {
      hat_y_ = (e.value > 0) - (e.value < 0);
    } else {
      return;
    }
  } else {
    return;
  }
  // remember buttons that went down, so a press + release within one period is still reported
  const std::uint16_t now_bits = current_bits();
  latched_ |= static_cast<std::uint16_t>(now_bits & ~last_bits_);
  last_bits_ = now_bits;
}

std::uint16_t GamepadMapper::current_bits() const
{
  std::uint16_t b = 0;
  for (const auto & kv : keys_) {
    if (kv.second) {b |= bit(layout_.buttons.at(kv.first));}
  }
  if (l2_analog_) {b |= bit(Button::kL2);}
  if (r2_analog_) {b |= bit(Button::kR2);}
  if (hat_x_ < 0) {b |= bit(Button::kLeft);}
  if (hat_x_ > 0) {b |= bit(Button::kRight);}
  if (hat_y_ < 0) {b |= bit(Button::kUp);}
  if (hat_y_ > 0) {b |= bit(Button::kDown);}
  return b;
}

void GamepadMapper::reset()
{
  keys_.clear();
  l2_analog_ = r2_analog_ = false;
  hat_x_ = hat_y_ = 0;
  lx_ = ly_ = rx_ = ry_ = 0.0f;
  latched_ = 0;
  last_bits_ = 0;
}

RemoteFrame GamepadMapper::take_frame()
{
  RemoteFrame f;
  f.set_buttons(static_cast<std::uint16_t>(current_bits() | latched_));
  f.lx = lx_;
  f.ly = ly_;
  f.rx = rx_;
  f.ry = ry_;
  latched_ = 0;
  return f;
}

// =============================================================================================
// KeyboardMapper
// =============================================================================================
std::optional<std::uint16_t> key_code_from_name(const std::string & name)
{
  static const std::unordered_map<std::string, std::uint16_t> kMap = [] {
      std::unordered_map<std::string, std::uint16_t> m = {
        {"A", KEY_A}, {"B", KEY_B}, {"C", KEY_C}, {"D", KEY_D}, {"E", KEY_E}, {"F", KEY_F},
        {"G", KEY_G}, {"H", KEY_H}, {"I", KEY_I}, {"J", KEY_J}, {"K", KEY_K}, {"L", KEY_L},
        {"M", KEY_M}, {"N", KEY_N}, {"O", KEY_O}, {"P", KEY_P}, {"Q", KEY_Q}, {"R", KEY_R},
        {"S", KEY_S}, {"T", KEY_T}, {"U", KEY_U}, {"V", KEY_V}, {"W", KEY_W}, {"X", KEY_X},
        {"Y", KEY_Y}, {"Z", KEY_Z},
        {"1", KEY_1}, {"2", KEY_2}, {"3", KEY_3}, {"4", KEY_4}, {"5", KEY_5},
        {"6", KEY_6}, {"7", KEY_7}, {"8", KEY_8}, {"9", KEY_9}, {"0", KEY_0},
        {"F1", KEY_F1}, {"F2", KEY_F2}, {"F3", KEY_F3}, {"F4", KEY_F4}, {"F5", KEY_F5},
        {"F6", KEY_F6}, {"F7", KEY_F7}, {"F8", KEY_F8}, {"F9", KEY_F9}, {"F10", KEY_F10},
        {"F11", KEY_F11}, {"F12", KEY_F12},
        {"SPACE", KEY_SPACE}, {"ENTER", KEY_ENTER}, {"ESC", KEY_ESC}, {"TAB", KEY_TAB},
        {"BACKSPACE", KEY_BACKSPACE}, {"CAPSLOCK", KEY_CAPSLOCK},
        {"UP", KEY_UP}, {"DOWN", KEY_DOWN}, {"LEFT", KEY_LEFT}, {"RIGHT", KEY_RIGHT},
        {"LEFTSHIFT", KEY_LEFTSHIFT}, {"RIGHTSHIFT", KEY_RIGHTSHIFT},
        {"LEFTCTRL", KEY_LEFTCTRL}, {"RIGHTCTRL", KEY_RIGHTCTRL},
        {"LEFTALT", KEY_LEFTALT}, {"RIGHTALT", KEY_RIGHTALT},
        {"MINUS", KEY_MINUS}, {"EQUAL", KEY_EQUAL}, {"COMMA", KEY_COMMA}, {"DOT", KEY_DOT},
        {"SLASH", KEY_SLASH}, {"BACKSLASH", KEY_BACKSLASH}, {"SEMICOLON", KEY_SEMICOLON},
        {"APOSTROPHE", KEY_APOSTROPHE}, {"GRAVE", KEY_GRAVE},
        {"LEFTBRACE", KEY_LEFTBRACE}, {"RIGHTBRACE", KEY_RIGHTBRACE},
        {"HOME", KEY_HOME}, {"END", KEY_END}, {"PAGEUP", KEY_PAGEUP}, {"PAGEDOWN", KEY_PAGEDOWN},
        {"INSERT", KEY_INSERT}, {"DELETE", KEY_DELETE},
        {"KP0", KEY_KP0}, {"KP1", KEY_KP1}, {"KP2", KEY_KP2}, {"KP3", KEY_KP3},
        {"KP4", KEY_KP4}, {"KP5", KEY_KP5}, {"KP6", KEY_KP6}, {"KP7", KEY_KP7},
        {"KP8", KEY_KP8}, {"KP9", KEY_KP9}, {"KPENTER", KEY_KPENTER}, {"KPPLUS", KEY_KPPLUS},
        {"KPMINUS", KEY_KPMINUS}, {"KPDOT", KEY_KPDOT},
      };
      return m;
    }();
  std::string n = upper(trim(name));
  if (n.rfind("KEY_", 0) == 0) {n = n.substr(4);}
  const auto it = kMap.find(n);
  if (it == kMap.end()) {return std::nullopt;}
  return it->second;
}

std::vector<std::string> KeyboardMapper::default_bindings()
{
  return {
    "W: ly+", "S: ly-", "A: lx-", "D: lx+",   // move forward / back / left / right
    "Q: rx-", "E: rx+",                       // turn left / right
    "LEFTSHIFT: fast",                        // full speed while held
    "SPACE: LT + B",                          // passive (damping) from any mode
    "1: LT + A",                              // stand
    "2: start",                               // walk
    "3: LT + down",                           // sit
    "UP: up", "DOWN: down", "LEFT: left", "RIGHT: right",
  };
}

KeyboardMapper::KeyboardMapper(const std::vector<std::string> & bindings, Options options)
: options_(options)
{
  for (const auto & text : bindings) {
    const auto colon = text.find(':');
    if (colon == std::string::npos) {
      throw std::invalid_argument("keyboard binding '" + text + "': expected '<KEY>: <target>'");
    }
    Binding b;
    const auto key = key_code_from_name(text.substr(0, colon));
    if (!key) {
      throw std::invalid_argument("keyboard binding '" + text + "': unknown key '" +
              trim(text.substr(0, colon)) + "'");
    }
    b.key = *key;
    const std::string target = trim(text.substr(colon + 1));
    const std::string t = lower(target);
    if (t == "fast") {
      b.kind = Kind::kFast;
    } else if (t.size() == 3 && (t[2] == '+' || t[2] == '-') &&
      (t.substr(0, 2) == "lx" || t.substr(0, 2) == "ly" || t.substr(0, 2) == "rx" ||
      t.substr(0, 2) == "ry"))
    {
      b.kind = Kind::kStick;
      static const std::string kAxes[] = {"lx", "ly", "rx", "ry"};
      b.axis = static_cast<int>(std::find(std::begin(kAxes), std::end(kAxes), t.substr(0, 2)) -
        std::begin(kAxes));
      b.sign = t[2] == '+' ? 1.0f : -1.0f;
    } else {
      b.kind = Kind::kButtons;
      std::stringstream ss(target);
      std::string item;
      while (std::getline(ss, item, '+')) {
        const auto button = button_from_name(trim(item));
        if (!button) {
          throw std::invalid_argument("keyboard binding '" + text + "': unknown target '" +
                  trim(item) + "' (remote buttons, lx+/lx-/ly+/ly-/rx+/rx-/ry+/ry- or fast)");
        }
        b.buttons.push_back(*button);
      }
      if (b.buttons.empty()) {throw std::invalid_argument("keyboard binding '" + text + "': empty");}
    }
    for (const auto & other : bindings_) {
      if (other.key == b.key) {
        throw std::invalid_argument("keyboard binding '" + text + "': key bound twice");
      }
    }
    bindings_.push_back(std::move(b));
  }
}

void KeyboardMapper::handle(const Event & e, double now)
{
  if (e.type != EV_KEY || e.value == 2) {return;}  // ignore autorepeat
  for (auto & b : bindings_) {
    if (b.key != e.code) {continue;}
    const bool pressed = e.value != 0;
    if (pressed && !b.held) {
      b.pending = true;
      b.pressed_at = now;
    }
    b.held = pressed;
  }
}

bool KeyboardMapper::binds(std::uint16_t key) const
{
  return std::any_of(bindings_.begin(), bindings_.end(), [key](const Binding & b) {return b.key == key;});
}

void KeyboardMapper::reset()
{
  for (auto & b : bindings_) {
    b.held = false;
    b.pending = false;
  }
}

RemoteFrame KeyboardMapper::take_frame(double now)
{
  RemoteFrame f;
  std::uint16_t bits = 0;
  std::array<float, 4> stick{0.0f, 0.0f, 0.0f, 0.0f};
  bool fast = false;
  for (const auto & b : bindings_) {
    if (b.kind == Kind::kFast) {fast = fast || b.held;}
  }
  const float magnitude = fast ? 1.0f : options_.stick;

  for (auto & b : bindings_) {
    switch (b.kind) {
      case Kind::kFast:
        break;
      case Kind::kStick:
        if (b.held) {stick[b.axis] += b.sign * magnitude;}
        break;
      case Kind::kButtons: {
          if (!b.held && !b.pending) {break;}
          for (std::size_t i = 0; i + 1 < b.buttons.size(); ++i) {bits |= bit(b.buttons[i]);}
          const bool last_due = b.buttons.size() == 1 || now - b.pressed_at >= options_.combo_delay;
          if (last_due) {
            bits |= bit(b.buttons.back());
            if (!b.held) {b.pending = false;}  // a short tap is reported exactly once, completely
          }
          break;
        }
    }
  }
  f.set_buttons(bits);
  f.lx = std::clamp(stick[0], -1.0f, 1.0f);
  f.ly = std::clamp(stick[1], -1.0f, 1.0f);
  f.rx = std::clamp(stick[2], -1.0f, 1.0f);
  f.ry = std::clamp(stick[3], -1.0f, 1.0f);
  return f;
}

// =============================================================================================
VelocityCommand velocity_from_frame(const RemoteFrame & f, const TeleopLimits & l)
{
  auto shape = [&l](float v) {
      const double a = std::fabs(v);
      if (a < l.deadzone) {return 0.0;}
      // rescale so the output starts at 0 right after the deadzone
      return std::copysign(std::min(1.0, (a - l.deadzone) / (1.0 - l.deadzone)), static_cast<double>(v));
    };
  VelocityCommand c;
  c.vx = static_cast<float>(shape(f.ly) * l.max_vx);
  c.vy = static_cast<float>(-shape(f.lx) * l.max_vy);
  c.wz = static_cast<float>(-shape(f.rx) * l.max_wz);
  return c;
}

}  // namespace go2_policy_runner::input
