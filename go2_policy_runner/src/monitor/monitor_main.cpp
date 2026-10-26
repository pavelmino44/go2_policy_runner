// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// robot_monitor: terminal UI (ncurses) for go2_policy_runner.
//
//   ros2 run go2_policy_runner robot_monitor
//
// Keys: [space] request passive (damping) mode, [q] quit. Needs a terminal of at least 80x40.
// Run it in a terminal with the same DDS settings as the controller (dds_env.sh <iface>).

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "go2_policy_runner/monitor/monitor_model.hpp"
#include "go2_policy_runner/monitor/monitor_node.hpp"
#include "go2_policy_runner/remote/unitree_remote.hpp"

// ncurses last: it defines macros such as OK and ERR that clash with ROS message constants
// (diagnostic_msgs::msg::DiagnosticStatus::OK). NCURSES_NOMACROS keeps the API as functions.
#define NCURSES_NOMACROS
#include <ncurses.h>
#undef OK

namespace
{
using go2_policy_runner::monitor::Alert;
using go2_policy_runner::monitor::Category;
using go2_policy_runner::monitor::Level;
using go2_policy_runner::monitor::Snapshot;
using go2_policy_runner::monitor::Thresholds;

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr int kMinCols = 80;

enum Pair { kGreen = 1, kYellow, kRed, kTitle, kDim };

int level_attr(Level l)
{
  switch (l) {
    case Level::kOk: return COLOR_PAIR(kGreen);
    case Level::kWarn: return COLOR_PAIR(kYellow) | A_BOLD;
    case Level::kError: return COLOR_PAIR(kRed) | A_BOLD;
  }
  return 0;
}

/// printf at (y, x), clipped to the screen width.
void put(int y, int x, int attr, const char * fmt, ...)
{
  if (y >= LINES || x >= COLS) {return;}
  char buf[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  attron(attr);
  mvaddnstr(y, x, buf, COLS - x);
  attroff(attr);
}

/// Section separator: "-- Title --------" colored by the worst level of its category.
void section(int y, const char * title, Level level)
{
  if (y >= LINES) {return;}
  attron(COLOR_PAIR(kDim));
  mvhline(y, 0, ACS_HLINE, COLS);
  attroff(COLOR_PAIR(kDim));
  put(y, 2, (level == Level::kOk ? COLOR_PAIR(kTitle) : level_attr(level)) | A_BOLD, " %s ", title);
}

/// Network interface from CYCLONEDDS_URI (as set by the launch file / dds_env.sh).
std::string dds_interface()
{
  const char * uri = std::getenv("CYCLONEDDS_URI");
  if (!uri) {return "auto";}
  const std::string u(uri);
  const std::string key = "NetworkInterface name=\"";
  const auto b = u.find(key);
  if (b == std::string::npos) {return "custom";}
  const auto e = u.find('"', b + key.size());
  return e == std::string::npos ? "custom" : u.substr(b + key.size(), e - b - key.size());
}

std::string fmt_time(double s)
{
  const int t = static_cast<int>(s);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", t / 3600, (t / 60) % 60, t % 60);
  return buf;
}

std::string range_bar(double fraction, int width)
{
  std::string bar(width, '-');
  const int pos = std::clamp(static_cast<int>(std::lround(fraction * (width - 1))), 0, width - 1);
  bar[pos] = 'o';
  return "[" + bar + "]";
}

std::string pressed_buttons(const go2_policy_runner::RemoteFrame & f)
{
  std::string s;
  auto add = [&s](bool on, const char * n) {if (on) {s += std::string(s.empty() ? "" : " ") + n;}};
  add(f.L2, "LT"); add(f.R2, "RT"); add(f.L1, "LB"); add(f.R1, "RB");
  add(f.A, "A"); add(f.B, "B"); add(f.X, "X"); add(f.Y, "Y");
  add(f.up, "up"); add(f.down, "down"); add(f.left, "left"); add(f.right, "right");
  add(f.start, "start"); add(f.select, "back"); add(f.F1, "F1"); add(f.F2, "F2");
  return s.empty() ? "-" : s;
}

void draw(const Snapshot & s, const std::vector<Alert> & alerts, const Thresholds & t,
  const std::string & controller, const std::string & action_status)
{
  using go2_policy_runner::monitor::worst_level;
  erase();
  if (COLS < kMinCols) {
    put(0, 0, level_attr(Level::kWarn), "terminal too narrow (%d columns, need %d)", COLS, kMinCols);
    refresh();
    return;
  }

  const std::string iface = dds_interface();
  const char * domain = std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "0";
  int y = 0;
  put(y, 0, A_BOLD, " go2_policy_runner monitor");
  put(y, 30, COLOR_PAIR(kDim), "controller /%s   iface %s   domain %s", controller.c_str(), iface.c_str(), domain);
  ++y;

  // ---------------------------------------------------------------- controller
  section(y++, "Controller", std::max(worst_level(alerts, Category::kController),
    worst_level(alerts, Category::kCommunication)));
  if (!s.have_state || s.state_age > 1.0) {
    put(y++, 1, level_attr(Level::kError), "no state from /%s", controller.c_str());
    y += 3;
  } else {
    std::string mode = s.mode;
    std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);
    put(y, 1, A_BOLD, "Mode   %s", mode.c_str());
    put(y, 22, 0, "(%s)  %s in mode", s.mode_type.c_str(), fmt_time(s.mode_time).c_str());
    put(y++, 60, s.ready ? level_attr(Level::kOk) : level_attr(Level::kWarn),
      s.ready ? "ready" : "NOT READY");
    std::string next;
    for (const auto & m : s.allowed) {next += (next.empty() ? "" : " | ") + m;}
    put(y++, 1, 0, "Next   %s", next.empty() ? "-" : next.c_str());
    put(y++, 1, 0, "Last   %s", s.last_reason.c_str());
    const Level comm = worst_level(alerts, Category::kCommunication);
    put(y++, 1, comm == Level::kOk ? 0 : level_attr(comm),
      "Loop   lowstate %4.0f Hz (age %3.0f ms)   control %4.0f Hz   max cycle %.2f ms",
      s.lowstate_rate, std::min(s.lowstate_age, 9.999) * 1000.0, s.control_rate, s.max_cycle_ms);
  }

  // ---------------------------------------------------------------- command
  section(y++, "Command", worst_level(alerts, Category::kCommand));
  if (s.have_cmd_vel && s.cmd_vel_age < 5.0) {
    std::string pubs;
    for (const auto & p : s.cmd_vel_publishers) {pubs += (pubs.empty() ? "" : ", ") + p;}
    put(y++, 1, s.cmd_vel_ok ? 0 : COLOR_PAIR(kDim),
      "cmd_vel  vx %+5.2f  vy %+5.2f  wz %+5.2f   age %5.2f s   from: %s",
      s.cmd_vel[0], s.cmd_vel[1], s.cmd_vel[2], s.cmd_vel_age, pubs.empty() ? "-" : pubs.c_str());
  } else {
    put(y++, 1, COLOR_PAIR(kDim), "cmd_vel  -");
  }
  if (s.have_policy && s.policy_age < 1.0) {
    put(y++, 1, 0, "policy   vx %+5.2f  vy %+5.2f  wz %+5.2f   (command seen by the policy)",
      s.policy_cmd[0], s.policy_cmd[1], s.policy_cmd[2]);
  } else {
    put(y++, 1, COLOR_PAIR(kDim), "policy   -");
  }
  if (s.have_remote_topic && s.remote_topic_age < 0.5) {
    // input_teleop (DualShock 4 / keyboard) on the remote topic
    const auto f = go2_policy_runner::RemoteFrame::decode(s.remote_topic);
    put(y++, 1, 0, "remote*  [%-28s]  lx %+5.2f  ly %+5.2f  rx %+5.2f  ry %+5.2f",
      pressed_buttons(f).c_str(), f.lx, f.ly, f.rx, f.ry);
  } else if (s.have_lowstate && !go2_policy_runner::RemoteFrame::is_empty(s.wireless_remote)) {
    const auto f = go2_policy_runner::RemoteFrame::decode(s.wireless_remote);
    put(y++, 1, 0, "remote   [%-28s]  lx %+5.2f  ly %+5.2f  rx %+5.2f  ry %+5.2f",
      pressed_buttons(f).c_str(), f.lx, f.ly, f.rx, f.ry);
  } else {
    put(y++, 1, COLOR_PAIR(kDim), "remote   not connected");
  }

  // ---------------------------------------------------------------- base
  section(y++, "Base", worst_level(alerts, Category::kBase));
  if (s.have_lowstate) {
    const auto rpy = go2_policy_runner::monitor::quat_to_rpy(s.quat);
    const double tilt = go2_policy_runner::monitor::tilt_from_quat(s.quat);
    put(y, 1, 0, "roll %+6.1f deg   pitch %+6.1f deg   yaw %+6.1f deg",
      rpy[0] * kRadToDeg, rpy[1] * kRadToDeg, rpy[2] * kRadToDeg);
    if (s.max_tilt > 0.0) {
      put(y, 56, level_attr(worst_level(alerts, Category::kBase)), "tilt %4.1f / %4.1f deg",
        tilt * kRadToDeg, s.max_tilt * kRadToDeg);
    } else {
      put(y, 56, 0, "tilt %4.1f deg", tilt * kRadToDeg);
    }
    ++y;
    put(y++, 1, 0, "gyro %+6.2f %+6.2f %+6.2f rad/s     acc %+6.2f %+6.2f %+6.2f m/s2",
      s.gyro[0], s.gyro[1], s.gyro[2], s.acc[0], s.acc[1], s.acc[2]);
  } else {
    put(y++, 1, COLOR_PAIR(kDim), "no lowstate");
    ++y;
  }

  // ---------------------------------------------------------------- joints
  section(y++, "Joints", worst_level(alerts, Category::kJoints));
  put(y++, 1, A_UNDERLINE, "%-9s %7s %7s %7s %7s %7s %5s %5s %5s  %-9s",
    "joint", "q", "q_cmd", "err", "dq", "tau", "T(C)", "kp", "kd", "range");
  for (const auto & j : s.joints) {
    if (y >= LINES - 1) {break;}
    const bool hot = j.temperature > t.motor_temp_warn;
    double margin = 1.0;
    std::string bar = "";
    if (j.has_limits) {
      const double f = go2_policy_runner::monitor::range_fraction(j.q, j.lower, j.upper);
      margin = std::min(f, 1.0 - f);
      bar = range_bar(f, 7);
    }
    put(y, 1, 0, "%-9s %+7.3f", j.name.c_str(), j.q);
    if (s.have_lowcmd) {
      put(y, 19, 0, "%+7.3f %+7.3f", j.q_cmd, j.q - j.q_cmd);
    } else {
      put(y, 19, COLOR_PAIR(kDim), "      -       -");
    }
    put(y, 35, 0, "%+7.2f %+7.2f", j.dq, j.tau);
    put(y, 51, hot ? level_attr(Level::kWarn) : 0, "%5d", j.temperature);
    if (s.have_lowcmd) {put(y, 57, 0, "%5.1f %5.2f", j.kp, j.kd);}
    put(y, 70, margin < t.joint_limit_margin ? level_attr(Level::kWarn) : 0, "%s", bar.c_str());
    ++y;
  }

  // ---------------------------------------------------------------- policy
  section(y++, "Policy", worst_level(alerts, Category::kPolicy));
  if (s.have_policy && s.policy_age < 1.0) {
    put(y++, 1, 0, "step %8llu   inference %.3f ms / period %.1f ms   action min %+6.2f max %+6.2f",
      static_cast<unsigned long long>(s.policy_step), s.inference_ms, s.policy_period_ms,
      s.action_min, s.action_max);
  } else {
    put(y++, 1, COLOR_PAIR(kDim), "no policy running");
  }

  // ---------------------------------------------------------------- robot
  section(y++, "Robot", worst_level(alerts, Category::kBattery));
  if (s.have_lowstate && s.battery_valid) {
    put(y++, 1, 0, "battery %3d %%  %5.1f V  %5.1f A     foot force  FR %4d  FL %4d  RR %4d  RL %4d",
      s.battery_soc, s.power_v, s.power_a,
      s.foot_force[0], s.foot_force[1], s.foot_force[2], s.foot_force[3]);
  } else {
    put(y++, 1, COLOR_PAIR(kDim), "battery n/a (simulation)   foot force n/a");
  }

  // ---------------------------------------------------------------- alerts
  section(y++, "Alerts", alerts.empty() ? Level::kOk : alerts.front().level);
  if (alerts.empty()) {
    put(y++, 1, level_attr(Level::kOk), "all good");
  }
  for (const auto & a : alerts) {
    if (y >= LINES - 1) {break;}
    put(y++, 1, level_attr(a.level), "%s %s", a.level == Level::kError ? "ERROR" : "WARN ", a.text.c_str());
  }

  // ---------------------------------------------------------------- footer
  attron(A_REVERSE);
  mvhline(LINES - 1, 0, ' ', COLS);
  attroff(A_REVERSE);
  put(LINES - 1, 0, A_REVERSE, " [space] passive (damping)   [q] quit   %s", action_status.c_str());
  refresh();
}
}  // namespace

int main(int argc, char ** argv)
{
  // rclcpp console output would corrupt the screen
  std::vector<const char *> args(argv, argv + argc);
  args.push_back("--ros-args");
  args.push_back("--disable-stdout-logs");
  rclcpp::init(static_cast<int>(args.size()), args.data());

  auto node = std::make_shared<go2_policy_runner::ros::MonitorNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  timeout(static_cast<int>(1000.0 / node->refresh_rate()));
  if (has_colors()) {
    start_color();
    use_default_colors();
    init_pair(kGreen, COLOR_GREEN, -1);
    init_pair(kYellow, COLOR_YELLOW, -1);
    init_pair(kRed, COLOR_RED, -1);
    init_pair(kTitle, COLOR_CYAN, -1);
    init_pair(kDim, COLOR_BLUE, -1);
  }

  while (rclcpp::ok()) {
    const auto snap = node->snapshot();
    const auto alerts = go2_policy_runner::monitor::compute_alerts(snap, node->thresholds());
    draw(snap, alerts, node->thresholds(), node->controller_name(), node->action_status());
    const int ch = getch();  // waits up to 1 / refresh_rate
    if (ch == 'q' || ch == 'Q') {break;}
    if (ch == ' ') {node->request_passive();}
  }

  endwin();
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
