// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>

#include "go2_policy_runner/monitor/monitor_model.hpp"

using namespace go2_policy_runner::monitor;

namespace
{
Snapshot healthy()
{
  Snapshot s;
  s.have_state = true;
  s.state_age = 0.05;
  s.ready = true;
  s.mode = "walk";
  s.mode_type = "rl_policy";
  s.cmd_vel_ok = true;
  s.lowstate_rate = 1000.0f;
  s.nominal_lowstate_rate = 500.0;
  s.have_lowstate = true;
  s.lowstate_age = 0.002;
  s.max_tilt = 1.0;
  for (std::size_t i = 0; i < 12; ++i) {
    JointSnapshot j;
    j.name = go2_joint_name(i);
    j.q = 0.0f;
    j.temperature = 35;
    j.has_limits = true;
    j.lower = -1.0f;
    j.upper = 1.0f;
    s.joints.push_back(j);
  }
  s.have_policy = true;
  s.inference_ms = 0.2f;
  s.policy_period_ms = 20.0;
  return s;
}

bool has(const std::vector<Alert> & a, Category c, Level l)
{
  for (const auto & x : a) {
    if (x.category == c && x.level == l) {return true;}
  }
  return false;
}
}  // namespace

TEST(MonitorModel, HealthySnapshotHasNoAlerts)
{
  EXPECT_TRUE(compute_alerts(healthy(), Thresholds{}).empty());
}

TEST(MonitorModel, MissingSourcesAreErrors)
{
  Snapshot s;  // nothing received
  const auto a = compute_alerts(s, Thresholds{});
  EXPECT_TRUE(has(a, Category::kController, Level::kError));
  EXPECT_TRUE(has(a, Category::kCommunication, Level::kError));
  EXPECT_EQ(a.front().level, Level::kError);
}

TEST(MonitorModel, Thresholds)
{
  Thresholds t;
  auto s = healthy();
  s.joints[5].temperature = 70;            // hot
  s.joints[2].q = 0.95f;                   // 2.5 % from the upper limit
  s.lowstate_rate = 300.0f;                // < 0.8 * 500
  s.inference_ms = 15.0f;                  // > 0.5 * 20 ms
  s.battery_valid = true;
  s.battery_soc = 10;
  s.cmd_vel_publishers = {"remote_teleop", "teleop_twist_keyboard"};
  const auto a = compute_alerts(s, t);
  EXPECT_TRUE(has(a, Category::kJoints, Level::kWarn));
  EXPECT_TRUE(has(a, Category::kCommunication, Level::kWarn));
  EXPECT_TRUE(has(a, Category::kPolicy, Level::kWarn));
  EXPECT_TRUE(has(a, Category::kBattery, Level::kWarn));
  EXPECT_TRUE(has(a, Category::kCommand, Level::kWarn));
  int joint_alerts = 0;
  for (const auto & x : a) {joint_alerts += x.category == Category::kJoints;}
  EXPECT_EQ(joint_alerts, 2);
}

TEST(MonitorModel, TiltWarningAndError)
{
  auto s = healthy();
  const float warn = 0.8f, err = 1.2f;  // max_tilt 1.0, warn above 0.7
  s.quat = {std::cos(warn / 2), std::sin(warn / 2), 0.0f, 0.0f};
  EXPECT_EQ(worst_level(compute_alerts(s, Thresholds{}), Category::kBase), Level::kWarn);
  s.quat = {std::cos(err / 2), 0.0f, std::sin(err / 2), 0.0f};
  EXPECT_EQ(worst_level(compute_alerts(s, Thresholds{}), Category::kBase), Level::kError);
}

TEST(MonitorModel, PolicyAlertsOnlyInRlMode)
{
  auto s = healthy();
  s.mode_type = "fix_stand";
  s.cmd_vel_ok = false;
  s.inference_ms = 100.0f;
  EXPECT_TRUE(compute_alerts(s, Thresholds{}).empty());
}

TEST(MonitorModel, Geometry)
{
  const float a = 0.3f;
  const auto rpy = quat_to_rpy({std::cos(a / 2), 0.0f, 0.0f, std::sin(a / 2)});
  EXPECT_NEAR(rpy[2], a, 1e-5);
  EXPECT_NEAR(rpy[0], 0.0, 1e-6);
  EXPECT_NEAR(tilt_from_quat({std::cos(a / 2), 0.0f, std::sin(a / 2), 0.0f}), a, 1e-5);
  EXPECT_NEAR(tilt_from_quat({std::cos(a / 2), 0.0f, 0.0f, std::sin(a / 2)}), 0.0, 1e-5);  // yaw only
  EXPECT_DOUBLE_EQ(range_fraction(0.0f, -1.0f, 1.0f), 0.5);
  EXPECT_DOUBLE_EQ(range_fraction(5.0f, -1.0f, 1.0f), 1.0);
  EXPECT_EQ(go2_joint_name(0), "FR_hip");
  EXPECT_EQ(go2_joint_name(11), "RL_calf");
}
