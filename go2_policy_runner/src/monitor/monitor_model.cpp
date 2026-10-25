// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include "go2_policy_runner/monitor/monitor_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace go2_policy_runner::monitor
{

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kStaleState = 1.0;      // [s] ~/state is published at >= 1 Hz
constexpr double kStaleLowstate = 0.25;  // [s]

std::string fmt(const char * f, double a)
{
  char buf[128];
  std::snprintf(buf, sizeof(buf), f, a);
  return buf;
}
std::string fmt(const char * f, const char * s, double a)
{
  char buf[160];
  std::snprintf(buf, sizeof(buf), f, s, a);
  return buf;
}
std::string fmt(const char * f, double a, double b)
{
  char buf[160];
  std::snprintf(buf, sizeof(buf), f, a, b);
  return buf;
}
}  // namespace

const char * category_name(Category c)
{
  switch (c) {
    case Category::kController: return "controller";
    case Category::kCommunication: return "communication";
    case Category::kCommand: return "command";
    case Category::kBase: return "base";
    case Category::kJoints: return "joints";
    case Category::kPolicy: return "policy";
    case Category::kBattery: return "battery";
  }
  return "unknown";
}

std::array<double, 3> quat_to_rpy(const std::array<float, 4> & q)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return {roll, pitch, yaw};
}

double tilt_from_quat(const std::array<float, 4> & q)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-6) {return 0.0;}
  const double x = q[1] / n, y = q[2] / n;
  // z component of the body z axis in the world frame: 1 - 2(x^2 + y^2)
  return std::acos(std::clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0));
}

double range_fraction(float q, float lower, float upper)
{
  if (upper <= lower) {return 0.5;}
  return std::clamp((static_cast<double>(q) - lower) / (upper - lower), 0.0, 1.0);
}

std::string go2_joint_name(std::size_t i)
{
  static const char * legs[4] = {"FR", "FL", "RR", "RL"};
  static const char * parts[3] = {"hip", "thigh", "calf"};
  if (i >= 12) {return "m" + std::to_string(i);}
  return std::string(legs[i / 3]) + "_" + parts[i % 3];
}

std::vector<Alert> compute_alerts(const Snapshot & s, const Thresholds & t)
{
  std::vector<Alert> a;
  auto add = [&a](Level l, Category c, std::string text) {a.push_back({l, c, std::move(text)});};

  // --- controller ---
  if (!s.have_state || s.state_age > kStaleState) {
    add(Level::kError, Category::kController, "no /policy_controller/state (controller not running?)");
  } else if (!s.ready) {
    add(Level::kWarn, Category::kController, "controller not ready: " + s.last_reason);
  }

  // --- communication ---
  if (!s.have_lowstate || s.lowstate_age > kStaleLowstate) {
    add(Level::kError, Category::kCommunication, "no lowstate (check network_interface / domain)");
  } else if (s.nominal_lowstate_rate > 0.0 && s.have_state &&
    s.lowstate_rate < t.lowstate_rate_ratio * s.nominal_lowstate_rate)
  {
    add(Level::kWarn, Category::kCommunication,
      fmt("lowstate rate %.0f Hz below %.0f Hz", s.lowstate_rate,
      t.lowstate_rate_ratio * s.nominal_lowstate_rate));
  }

  // --- command ---
  if (s.cmd_vel_publishers.size() > 1) {
    std::string names;
    for (const auto & n : s.cmd_vel_publishers) {names += (names.empty() ? "" : ", ") + n;}
    add(Level::kWarn, Category::kCommand, "several cmd_vel publishers: " + names);
  }
  if (s.mode_type == "rl_policy" && s.have_state && !s.cmd_vel_ok) {
    add(Level::kWarn, Category::kCommand, "policy running without fresh cmd_vel (zero command)");
  }

  // --- base ---
  if (s.have_lowstate && s.max_tilt > 0.0) {
    const double tilt = tilt_from_quat(s.quat);
    if (tilt > s.max_tilt) {
      add(Level::kError, Category::kBase,
        fmt("tilt %.0f deg exceeds max_tilt %.0f deg", tilt * kRadToDeg, s.max_tilt * kRadToDeg));
    } else if (tilt > t.tilt_ratio * s.max_tilt) {
      add(Level::kWarn, Category::kBase,
        fmt("tilt %.0f deg close to max_tilt %.0f deg", tilt * kRadToDeg, s.max_tilt * kRadToDeg));
    }
  }

  // --- joints ---
  if (s.have_lowstate) {
    for (const auto & j : s.joints) {
      if (j.temperature > t.motor_temp_warn) {
        add(Level::kWarn, Category::kJoints,
          fmt("%s temperature %.0f C", j.name.c_str(), static_cast<double>(j.temperature)));
      }
      if (j.has_limits) {
        const double f = range_fraction(j.q, j.lower, j.upper);
        const double margin = std::min(f, 1.0 - f);
        if (margin < t.joint_limit_margin) {
          add(Level::kWarn, Category::kJoints,
            fmt("%s near limit (%.0f %% of range)", j.name.c_str(), margin * 100.0));
        }
      }
    }
  }

  // --- policy ---
  if (s.mode_type == "rl_policy" && s.have_policy && s.policy_period_ms > 0.0 &&
    s.inference_ms > t.inference_ratio * s.policy_period_ms)
  {
    add(Level::kWarn, Category::kPolicy,
      fmt("inference %.2f ms > %.0f %% of policy period", s.inference_ms,
      t.inference_ratio * 100.0));
  }

  // --- battery (real robot only) ---
  if (s.battery_valid && s.battery_soc < t.battery_warn) {
    add(Level::kWarn, Category::kBattery, fmt("battery %.0f %%", static_cast<double>(s.battery_soc)));
  }

  std::stable_sort(a.begin(), a.end(), [](const Alert & x, const Alert & y) {
      return static_cast<int>(x.level) > static_cast<int>(y.level);
    });
  return a;
}

Level worst_level(const std::vector<Alert> & alerts, Category c)
{
  Level l = Level::kOk;
  for (const auto & a : alerts) {
    if (a.category == c && static_cast<int>(a.level) > static_cast<int>(l)) {l = a.level;}
  }
  return l;
}

}  // namespace go2_policy_runner::monitor
