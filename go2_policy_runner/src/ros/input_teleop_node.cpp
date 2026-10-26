// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// input_teleop: a DualShock 4 (or another evdev gamepad) or a keyboard, read directly from
// /dev/input, used instead of the Unitree wireless remote.
//
//   publishes  remote   (unitree_go/WirelessController)  buttons + sticks in the Unitree remote
//                                                         layout; policy_controller uses it for
//                                                         the FSM when remote_source = topic
//              cmd_vel  (geometry_msgs/Twist)            same stick mapping as remote_teleop
//
// Both topics are published at `rate` while a device is connected. When the device is lost,
// a zero cmd_vel is sent once and publishing stops: the controller then sees a stale remote
// (all buttons released) and a stale cmd_vel (zero velocity command).
//
// The keyboard is read from the device, not from a terminal, so it works in any window. To keep
// typing elsewhere from moving the robot, keyboard control starts OFF and is switched on and off
// with `keyboard.toggle_key` (F12). While it is ON, the keyboard is grabbed: other programs get
// no keys (no typing, no Ctrl-C in terminals) until it is switched OFF again.

#include <poll.h>
#include <linux/input-event-codes.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "go2_policy_runner/input/evdev_device.hpp"
#include "go2_policy_runner/input/input_mapping.hpp"
#include "unitree_go/msg/wireless_controller.hpp"

namespace go2_policy_runner::ros
{

using namespace std::chrono_literals;

class InputTeleopNode : public rclcpp::Node
{
public:
  explicit InputTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("input_teleop", options)
  {
    const auto device = declare_parameter<std::string>("device", "dualshock4");
    if (device == "dualshock4" || device == "gamepad") {
      kind_ = input::DeviceKind::kGamepad;
    } else if (device == "keyboard") {
      kind_ = input::DeviceKind::kKeyboard;
    } else {
      throw std::runtime_error("device must be dualshock4 | keyboard, got '" + device + "'");
    }
    device_path_ = declare_parameter<std::string>("device_path", "");
    device_name_ = declare_parameter<std::string>("device_name", "");
    const double rate = declare_parameter<double>("rate", 50.0);
    limits_.max_vx = declare_parameter<double>("max_vx", 1.0);
    limits_.max_vy = declare_parameter<double>("max_vy", 0.5);
    limits_.max_wz = declare_parameter<double>("max_wz", 1.0);
    limits_.deadzone = declare_parameter<double>("deadzone", 0.05);

    const float threshold =
      static_cast<float>(declare_parameter<double>("gamepad.trigger_threshold", 0.5));
    gamepad_ = std::make_unique<input::GamepadMapper>(
      input::GamepadMapper::Layout::dualshock4(), threshold);
    input::KeyboardMapper::Options kopt;
    kopt.stick = static_cast<float>(declare_parameter<double>("keyboard.stick", 0.5));
    kopt.combo_delay = declare_parameter<double>("keyboard.combo_delay", 0.15);
    const auto bindings = declare_parameter<std::vector<std::string>>(
      "keyboard.bindings", input::KeyboardMapper::default_bindings());
    keyboard_ = std::make_unique<input::KeyboardMapper>(bindings, kopt);
    const auto toggle = declare_parameter<std::string>("keyboard.toggle_key", "F12");
    const auto toggle_code = input::key_code_from_name(toggle);
    if (!toggle_code) {throw std::runtime_error("keyboard.toggle_key: unknown key '" + toggle + "'");}
    if (keyboard_->binds(*toggle_code)) {
      throw std::runtime_error("keyboard.toggle_key '" + toggle + "' is also used in keyboard.bindings");
    }
    toggle_key_ = *toggle_code;
    toggle_name_ = toggle;
    grab_ = declare_parameter<bool>("keyboard.grab", true);

    remote_pub_ = create_publisher<unitree_go::msg::WirelessController>("remote", rclcpp::QoS(10));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      [this]() {publish();});

    if (kind_ == input::DeviceKind::kKeyboard) {
      RCLCPP_INFO(
        get_logger(), "keyboard control is OFF - press %s to switch it on%s", toggle_name_.c_str(),
        grab_ ? " (the keyboard is then grabbed until you press it again)" : "");
    }
    reader_ = std::thread([this]() {reader_loop();});
  }

  ~InputTeleopNode() override
  {
    stop_ = true;
    if (reader_.joinable()) {reader_.join();}
  }

private:
  double now_s() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
  }

  // ---- device thread --------------------------------------------------------------------------
  bool open_devices()
  {
    std::vector<std::string> paths;
    if (!device_path_.empty()) {
      paths.push_back(device_path_);
    } else {
      bool denied = false;
      const auto found = input::find_devices(kind_, device_name_, &denied);
      for (const auto & d : found) {
        paths.push_back(d.path);
        if (kind_ == input::DeviceKind::kGamepad) {break;}  // one gamepad, every keyboard
      }
      if (paths.empty()) {
        std::string what = kind_ == input::DeviceKind::kGamepad ? "a gamepad" : "a keyboard";
        if (!device_name_.empty()) {what += " named *" + device_name_ + "*";}
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "waiting for %s in /dev/input ...", what.c_str());
        if (denied) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000, "some input devices are not readable: "
            "sudo usermod -aG input $USER, then log in again");
        }
        return false;
      }
    }
    for (const auto & p : paths) {
      try {
        auto dev = std::make_unique<input::EvdevDevice>(p);
        RCLCPP_INFO(get_logger(), "using %s (%s)", dev->name().c_str(), p.c_str());
        devices_.push_back(std::move(dev));
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "%s", e.what());
      }
    }
    if (devices_.empty()) {return false;}

    // initial state (keys already held, stick positions) and axis ranges
    std::lock_guard<std::mutex> lock(mutex_);
    gamepad_->reset();
    keyboard_->reset();
    const double t = now_s();
    for (const auto & dev : devices_) {
      for (auto code : gamepad_->axes()) {gamepad_->set_axis_range(code, dev->axis_range(code));}
      for (const auto & e : dev->snapshot()) {apply(e, t);}
    }
    gamepad_->take_frame();  // drop the edges produced by the snapshot
    connected_ = true;
    return true;
  }

  /// Called with mutex_ held.
  void apply(const input::Event & e, double t)
  {
    if (e.type == EV_SYN && e.code == SYN_DROPPED) {
      gamepad_->reset();
      keyboard_->reset();
      return;
    }
    if (kind_ == input::DeviceKind::kGamepad) {
      gamepad_->handle(e);
      return;
    }
    if (e.type == EV_KEY && e.code == toggle_key_) {
      // switch on release (grabbing while the key is down would leave it "stuck" for other
      // programs), and only after a press was seen (snapshots report every key as released)
      if (e.value == 1) {toggle_down_ = true;}
      if (e.value == 0 && toggle_down_) {
        toggle_down_ = false;
        toggle_requested_ = true;
      }
      return;
    }
    if (armed_) {keyboard_->handle(e, t);}
  }

  /// Reader thread, mutex_ not held.
  void apply_toggle()
  {
    bool armed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!toggle_requested_) {return;}
      toggle_requested_ = false;
      armed_ = !armed_;
      keyboard_->reset();
      armed = armed_;
    }
    if (grab_) {
      for (const auto & d : devices_) {
        if (!d->grab(armed) && armed) {
          RCLCPP_WARN(get_logger(), "could not grab %s: keys still reach other programs",
            d->name().c_str());
        }
      }
    }
    RCLCPP_INFO(
      get_logger(), "keyboard control %s - press %s to switch it %s", armed ? "ON" : "OFF",
      toggle_name_.c_str(), armed ? "off" : "on");
  }

  void reader_loop()
  {
    std::vector<input::Event> events;
    while (!stop_ && rclcpp::ok()) {
      if (devices_.empty() && !open_devices()) {
        for (int i = 0; i < 10 && !stop_; ++i) {std::this_thread::sleep_for(100ms);}
        continue;
      }
      std::vector<pollfd> fds;
      for (const auto & d : devices_) {fds.push_back({d->fd(), POLLIN, 0});}
      if (::poll(fds.data(), fds.size(), 100) < 0) {continue;}

      bool lost = false;
      for (std::size_t i = 0; i < devices_.size(); ++i) {
        if (fds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) {lost = true; continue;}
        if (!(fds[i].revents & POLLIN)) {continue;}
        events.clear();
        const bool ok = devices_[i]->read(events);
        const double t = now_s();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          for (const auto & e : events) {apply(e, t);}
        }
        lost = lost || !ok;
      }
      if (kind_ == input::DeviceKind::kKeyboard && !lost) {apply_toggle();}
      if (lost) {
        RCLCPP_WARN(get_logger(), "input device lost - waiting for it to come back");
        devices_.clear();  // closing releases a grab
        std::lock_guard<std::mutex> lock(mutex_);
        gamepad_->reset();
        keyboard_->reset();
        connected_ = false;
        if (armed_) {
          armed_ = false;
          RCLCPP_INFO(get_logger(), "keyboard control OFF");
        }
      }
    }
  }

  // ---- publishing (executor thread) -----------------------------------------------------------
  void publish()
  {
    RemoteFrame f;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!connected_) {
        if (was_connected_) {cmd_vel_pub_->publish(geometry_msgs::msg::Twist());}  // stop once
        was_connected_ = false;
        return;
      }
      f = kind_ == input::DeviceKind::kGamepad ? gamepad_->take_frame() : keyboard_->take_frame(now_s());
    }
    was_connected_ = true;

    unitree_go::msg::WirelessController remote;
    remote.lx = f.lx;
    remote.ly = f.ly;
    remote.rx = f.rx;
    remote.ry = f.ry;
    remote.keys = f.buttons();
    remote_pub_->publish(remote);

    const auto v = input::velocity_from_frame(f, limits_);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v.vx;
    cmd.linear.y = v.vy;
    cmd.angular.z = v.wz;
    cmd_vel_pub_->publish(cmd);
  }

  input::DeviceKind kind_{input::DeviceKind::kGamepad};
  std::string device_path_, device_name_;
  input::TeleopLimits limits_;

  rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr remote_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::unique_ptr<input::EvdevDevice>> devices_;  // reader thread only
  std::thread reader_;
  std::atomic<bool> stop_{false};

  std::mutex mutex_;  // guards the mappers and connected_
  std::unique_ptr<input::GamepadMapper> gamepad_;
  std::unique_ptr<input::KeyboardMapper> keyboard_;
  bool connected_{false};
  bool was_connected_{false};
  bool armed_{false};             // keyboard: control switched on
  bool toggle_requested_{false};
  bool toggle_down_{false};

  std::uint16_t toggle_key_{KEY_F12};
  std::string toggle_name_{"F12"};
  bool grab_{true};

  const std::chrono::steady_clock::time_point t0_{std::chrono::steady_clock::now()};
};

}  // namespace go2_policy_runner::ros

RCLCPP_COMPONENTS_REGISTER_NODE(go2_policy_runner::ros::InputTeleopNode)
