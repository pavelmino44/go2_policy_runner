// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <linux/input-event-codes.h>

#include "go2_policy_runner/input/input_mapping.hpp"
#include "go2_policy_runner/remote/remote_dsl.hpp"

using namespace go2_policy_runner;
using namespace go2_policy_runner::input;

TEST(Input, RemoteFrameButtonBitsRoundTrip)
{
  RemoteFrame f;
  f.L2 = f.A = f.left = true;
  const auto bits = f.buttons();
  RemoteFrame g;
  g.set_buttons(bits);
  EXPECT_EQ(g.encode(), f.encode());
  EXPECT_EQ(RemoteFrame::decode(f.encode()).buttons(), bits);
  set_button(g, Button::kA, false);
  EXPECT_FALSE(g.A);
  EXPECT_TRUE(g.L2);
}

TEST(Input, GamepadDualShock4)
{
  GamepadMapper m;
  m.set_axis_range(ABS_X, {0, 255});
  m.set_axis_range(ABS_Y, {0, 255});
  m.handle({EV_KEY, BTN_SOUTH, 1});    // cross
  m.handle({EV_KEY, BTN_NORTH, 1});    // triangle
  m.handle({EV_ABS, ABS_X, 255});      // left stick right
  m.handle({EV_ABS, ABS_Y, 0});        // left stick up
  m.handle({EV_ABS, ABS_Z, 200});      // L2 analog
  m.handle({EV_ABS, ABS_HAT0Y, -1});   // d-pad up
  auto f = m.take_frame();
  EXPECT_TRUE(f.A);
  EXPECT_TRUE(f.Y);
  EXPECT_FALSE(f.X);
  EXPECT_TRUE(f.L2);
  EXPECT_TRUE(f.up);
  EXPECT_FLOAT_EQ(f.lx, 1.0f);
  EXPECT_FLOAT_EQ(f.ly, 1.0f);  // up is positive on the Unitree remote

  m.handle({EV_KEY, BTN_SOUTH, 0});
  m.handle({EV_ABS, ABS_HAT0Y, 0});
  m.handle({EV_ABS, ABS_Z, 0});
  f = m.take_frame();
  EXPECT_FALSE(f.A);
  EXPECT_FALSE(f.up);
  EXPECT_FALSE(f.L2);

  m.handle({EV_KEY, BTN_TL2, 1});  // digital L2 alone also counts
  EXPECT_TRUE(m.take_frame().L2);
  m.reset();
  f = m.take_frame();
  EXPECT_EQ(f.buttons(), 0);
  EXPECT_FLOAT_EQ(f.lx, 0.0f);
}

TEST(Input, GamepadShortTapIsNotLost)
{
  GamepadMapper m;
  m.handle({EV_KEY, BTN_START, 1});
  m.handle({EV_KEY, BTN_START, 0});
  EXPECT_TRUE(m.take_frame().start);   // pressed during the period
  EXPECT_FALSE(m.take_frame().start);  // and released afterwards
}

TEST(Input, KeyboardSticksAndFast)
{
  KeyboardMapper k(KeyboardMapper::default_bindings(), {0.5f, 0.15});
  k.handle({EV_KEY, KEY_W, 1}, 0.0);
  k.handle({EV_KEY, KEY_A, 1}, 0.0);
  auto f = k.take_frame(0.0);
  EXPECT_FLOAT_EQ(f.ly, 0.5f);
  EXPECT_FLOAT_EQ(f.lx, -0.5f);
  k.handle({EV_KEY, KEY_LEFTSHIFT, 1}, 0.1);
  k.handle({EV_KEY, KEY_S, 1}, 0.1);   // W + S cancel
  f = k.take_frame(0.1);
  EXPECT_FLOAT_EQ(f.ly, 0.0f);
  EXPECT_FLOAT_EQ(f.lx, -1.0f);
  k.handle({EV_KEY, KEY_W, 2}, 0.2);   // autorepeat changes nothing
  k.reset();
  f = k.take_frame(0.3);
  EXPECT_FLOAT_EQ(f.lx, 0.0f);
  EXPECT_EQ(f.buttons(), 0);

  const auto v = velocity_from_frame(k.take_frame(0.3), TeleopLimits{});
  EXPECT_FLOAT_EQ(v.vx, 0.0f);
}

TEST(Input, KeyboardComboSatisfiesRemoteExpression)
{
  // "1: LT + A" must trigger "LT + A.on_pressed" although both buttons come from one key press:
  // LT is pressed first, A combo_delay later. Simulates the controller at 500 Hz receiving
  // frames published at 50 Hz.
  for (double hold : {1.0, 0.03}) {  // long press and a short tap
    KeyboardMapper k(KeyboardMapper::default_bindings(), {0.5f, 0.15});
    UnitreeRemote remote;
    const auto cond = compile_remote_condition("LT + A.on_pressed");
    bool fired = false;
    RemoteFrame frame;
    k.handle({EV_KEY, KEY_1, 1}, 0.0);
    for (int i = 0; i < 1000; ++i) {
      const double t = i * 0.002;
      if (t >= hold) {k.handle({EV_KEY, KEY_1, 0}, t);}
      if (i % 10 == 0) {frame = k.take_frame(t);}
      remote.update(frame, t);
      fired = fired || cond(remote);
    }
    EXPECT_TRUE(fired) << "hold " << hold;
    EXPECT_EQ(k.take_frame(10.0).buttons(), 0);
  }
}

TEST(Input, KeyboardBindingErrors)
{
  EXPECT_THROW(KeyboardMapper({"W ly+"}), std::invalid_argument);
  EXPECT_THROW(KeyboardMapper({"NOPE: A"}), std::invalid_argument);
  EXPECT_THROW(KeyboardMapper({"W: jump"}), std::invalid_argument);
  EXPECT_THROW(KeyboardMapper({"W: ly+", "w: A"}), std::invalid_argument);
  EXPECT_NO_THROW(KeyboardMapper({"key_space: lt + b", "KP0: start"}));
  EXPECT_EQ(key_code_from_name("leftshift"), KEY_LEFTSHIFT);
}

TEST(Input, VelocityMapping)
{
  RemoteFrame f;
  f.ly = 1.0f;
  f.lx = 1.0f;   // right
  f.rx = -1.0f;  // left
  const auto v = velocity_from_frame(f, TeleopLimits{1.0, 0.5, 1.0, 0.05});
  EXPECT_FLOAT_EQ(v.vx, 1.0f);
  EXPECT_FLOAT_EQ(v.vy, -0.5f);
  EXPECT_FLOAT_EQ(v.wz, 1.0f);
  f.ly = 0.04f;  // inside the deadzone
  EXPECT_FLOAT_EQ(velocity_from_frame(f, TeleopLimits{}).vx, 0.0f);
}
