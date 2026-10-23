// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "go2_policy_runner/remote/remote_dsl.hpp"
#include "go2_policy_runner/remote/unitree_remote.hpp"

using namespace go2_policy_runner;

TEST(RemoteFrame, EncodeDecodeRoundTrip)
{
  RemoteFrame f;
  f.L2 = true; f.A = true; f.left = true; f.start = true;
  f.lx = -0.25f; f.ly = 0.75f; f.rx = 0.5f; f.ry = -1.0f;
  const auto raw = f.encode();
  EXPECT_FALSE(RemoteFrame::is_empty(raw));
  const auto g = RemoteFrame::decode(raw);
  EXPECT_TRUE(g.L2 && g.A && g.left && g.start);
  EXPECT_FALSE(g.B || g.R2 || g.up || g.select);
  EXPECT_FLOAT_EQ(g.lx, -0.25f);
  EXPECT_FLOAT_EQ(g.ly, 0.75f);
  EXPECT_FLOAT_EQ(g.rx, 0.5f);
  EXPECT_FLOAT_EQ(g.ry, -1.0f);
}

TEST(RemoteFrame, BitLayoutMatchesUnitreeSdk)
{
  // unitree_sdk2 BtnUnion: bit0 R1, bit1 L1, bit2 start, bit3 select, bit4 R2, bit5 L2,
  // bit6 F1, bit7 F2, bit8 A, bit9 B, bit10 X, bit11 Y, bit12 up, bit13 right, bit14 down, bit15 left
  std::array<std::uint8_t, 40> raw{};
  raw[2] = 0b00100100;  // start + L2
  raw[3] = 0b10000010;  // B + left
  const auto f = RemoteFrame::decode(raw);
  EXPECT_TRUE(f.start && f.L2 && f.B && f.left);
  EXPECT_FALSE(f.R1 || f.L1 || f.select || f.R2 || f.A || f.X || f.Y || f.up || f.right || f.down);
}

TEST(RemoteFrame, EmptyFrame)
{
  std::array<std::uint8_t, 40> raw{};
  EXPECT_TRUE(RemoteFrame::is_empty(raw));
}

namespace
{
void feed(UnitreeRemote & r, const RemoteFrame & f, double t0, double t1, double dt = 0.002)
{
  for (double t = t0; t < t1 - 1e-9; t += dt) {r.update(f, t);}
}
}  // namespace

TEST(RemoteDsl, OnPressedIsOneCycle)
{
  auto cond = compile_remote_condition("LT + A.on_pressed");
  UnitreeRemote r;
  RemoteFrame f;
  f.L2 = true;
  feed(r, f, 0.0, 0.2);           // LT axis smoothed above threshold
  EXPECT_TRUE(r.LT.pressed);
  EXPECT_FALSE(cond(r));
  f.A = true;
  r.update(f, 0.2);
  EXPECT_TRUE(cond(r));           // rising edge
  r.update(f, 0.202);
  EXPECT_FALSE(cond(r));          // only once
}

TEST(RemoteDsl, HoldTimeAndLogic)
{
  auto hold = compile_remote_condition("LT(0.5s) + down");
  auto any = compile_remote_condition("X | (Y + !B)");
  UnitreeRemote r;
  RemoteFrame f;
  f.L2 = true; f.down = true;
  feed(r, f, 0.0, 0.3);
  EXPECT_FALSE(hold(r));
  feed(r, f, 0.3, 0.7);
  EXPECT_TRUE(hold(r));

  RemoteFrame g;
  g.Y = true;
  r.update(g, 1.0);
  EXPECT_TRUE(any(r));
  g.B = true;
  r.update(g, 1.002);
  EXPECT_FALSE(any(r));
}

TEST(RemoteDsl, CaseInsensitiveAndAliases)
{
  EXPECT_NO_THROW(compile_remote_condition("lt + a.on_pressed"));
  EXPECT_NO_THROW(compile_remote_condition("Start.on_released"));
  EXPECT_NO_THROW(compile_remote_condition("L2 + R1 + select"));
  EXPECT_NO_THROW(compile_remote_condition("LT(2) + up.pressed"));
}

TEST(RemoteDsl, Errors)
{
  EXPECT_THROW(compile_remote_condition("LT + Q"), std::invalid_argument);
  EXPECT_THROW(compile_remote_condition("LT +"), std::invalid_argument);
  EXPECT_THROW(compile_remote_condition("(A + B"), std::invalid_argument);
  EXPECT_THROW(compile_remote_condition("A.clicked"), std::invalid_argument);
  EXPECT_THROW(compile_remote_condition("LT(2min)"), std::invalid_argument);
}
