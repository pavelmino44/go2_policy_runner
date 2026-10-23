// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>

#include "go2_policy_runner/fsm/mode_manager.hpp"
#include "go2_policy_runner/fsm/modes.hpp"

using namespace go2_policy_runner;
using namespace go2_policy_runner::fsm;

namespace
{
constexpr std::size_t kN = 12;
constexpr double kRate = 500.0;
constexpr double kDt = 1.0 / kRate;

class CountingModel : public policy::PolicyModel
{
public:
  explicit CountingModel(int * calls)
  : calls_(calls) {}
  const std::vector<std::pair<std::string, std::size_t>> & inputs() const override {return in_;}
  std::size_t output_size() const override {return 12;}
  std::vector<float> act(const std::map<std::string, std::vector<float>> &) override
  {
    ++*calls_;
    return std::vector<float>(12, 0.4f);
  }

private:
  int * calls_;
  std::vector<std::pair<std::string, std::size_t>> in_{{"obs", 45}};
};

std::unique_ptr<policy::PolicyEnv> make_env(int * calls)
{
  YAML::Node cfg = YAML::LoadFile("fixtures/go2_linear/params/deploy.yaml");
  return std::make_unique<policy::PolicyEnv>(cfg, std::make_unique<CountingModel>(calls), kN);
}

const std::vector<float> kStand{-0.1f, 0.8f, -1.5f, 0.1f, 0.8f, -1.5f, -0.1f, 1.0f, -1.5f, 0.1f, 1.0f, -1.5f};

struct Harness
{
  int policy_calls{0};
  std::unique_ptr<ModeManager> fsm;
  RobotState state{kN};
  VelocityCommand cmd;
  JointCommand out = JointCommand(kN);
  RemoteFrame remote;
  double t{0.0};

  Harness()
  {
    std::vector<std::unique_ptr<Mode>> modes;
    auto passive = std::make_unique<PassiveMode>("passive", kN, std::vector<float>(kN, 3.0f));
    passive->add_transition("stand", "LT + A.on_pressed");
    auto stand = std::make_unique<FixStandMode>(
      "stand", kN, std::vector<float>(kN, 60.0f), std::vector<float>(kN, 5.0f),
      std::vector<std::vector<float>>{kStand}, std::vector<double>{1.0},
      FixStandMode::Interpolation::kLinear);
    stand->add_transition("passive", "LT + B.on_pressed");
    stand->add_transition("walk", "start.on_pressed");
    auto walk = std::make_unique<RlPolicyMode>("walk", kN, make_env(&policy_calls), kRate, 1.0);
    walk->add_transition("passive", "LT + B.on_pressed");
    walk->add_transition("stand", "");
    modes.push_back(std::move(passive));
    modes.push_back(std::move(stand));
    modes.push_back(std::move(walk));
    fsm = std::make_unique<ModeManager>(std::move(modes), "passive", "passive");
  }

  void cycle(const std::optional<std::string> & fault = std::nullopt)
  {
    state.wireless_remote = remote.encode();
    fsm->update(ControlInput{state, cmd, t}, out, fault);
    t += kDt;
  }
  void cycles(int n) {for (int i = 0; i < n; ++i) {cycle();}}
};
}  // namespace

TEST(Fsm, StartsPassiveWithDampingOnly)
{
  Harness h;
  h.cycle();
  EXPECT_EQ(h.fsm->current_mode(), "passive");
  for (const auto & m : h.out) {
    EXPECT_EQ(m.kp, 0.0f);
    EXPECT_EQ(m.kd, 3.0f);
    EXPECT_EQ(m.dq, 0.0f);
  }
}

TEST(Fsm, RemoteComboStandsUpAndInterpolates)
{
  Harness h;
  h.remote.L2 = true;
  h.cycles(100);  // hold LT
  EXPECT_EQ(h.fsm->current_mode(), "passive");
  h.remote.A = true;
  h.cycle();
  EXPECT_EQ(h.fsm->current_mode(), "stand");
  h.remote = RemoteFrame();
  // linear interpolation from q = 0 to kStand over 1 s
  h.cycles(249);  // t_rel = 0.498 at the last one
  EXPECT_NEAR(h.out[1].q, 0.8f * 0.5f, 0.01f);
  h.cycles(300);
  EXPECT_NEAR(h.out[1].q, 0.8f, 1e-6f);
  EXPECT_EQ(h.out[1].kp, 60.0f);
}

TEST(Fsm, ServiceRequestsFollowTheGraph)
{
  Harness h;
  h.cycle();
  EXPECT_NE(h.fsm->request("walk", "test"), "");      // passive -> walk not allowed
  EXPECT_NE(h.fsm->request("fly", "test"), "");       // unknown
  EXPECT_EQ(h.fsm->request("stand", "test"), "");
  EXPECT_EQ(h.fsm->current_mode(), "passive");        // applied on the next cycle
  h.cycle();
  EXPECT_EQ(h.fsm->current_mode(), "stand");
  EXPECT_EQ(h.fsm->request("walk", "test"), "");
  h.cycle();
  EXPECT_EQ(h.fsm->current_mode(), "walk");
  EXPECT_EQ(h.fsm->request("stand", "test"), "");     // service-only transition
  h.cycle();
  EXPECT_EQ(h.fsm->current_mode(), "stand");
  EXPECT_EQ(h.fsm->request("passive", "test"), "");   // fallback always allowed
  h.cycle();
  EXPECT_EQ(h.fsm->current_mode(), "passive");
}

TEST(Fsm, PolicyRunsAtStepDtAndWritesPolicyGains)
{
  Harness h;
  h.cycle();
  h.fsm->request("stand", "");
  h.cycle();
  h.fsm->request("walk", "");
  h.cycle();                       // enter + first policy step
  EXPECT_EQ(h.policy_calls, 1);
  h.cycles(9);
  EXPECT_EQ(h.policy_calls, 1);    // decimation 10 (0.02 s * 500 Hz)
  h.cycle();
  EXPECT_EQ(h.policy_calls, 2);
  h.cycles(100);
  EXPECT_EQ(h.policy_calls, 12);
  // targets: default + 0.25 * 0.4; FR_thigh (sdk 1) default 0.8
  EXPECT_NEAR(h.out[1].q, 0.8f + 0.1f, 1e-6f);
  EXPECT_EQ(h.out[1].kp, 25.0f);
  EXPECT_EQ(h.out[1].kd, 0.5f);
}

TEST(Fsm, FaultForcesFallbackAndBlocksTransitions)
{
  Harness h;
  h.cycle();
  h.fsm->request("stand", "");
  h.cycle();
  h.cycle(std::string("lowstate timeout"));
  EXPECT_EQ(h.fsm->current_mode(), "passive");
  EXPECT_EQ(h.fsm->last_transition_reason(), "lowstate timeout");
  // remote combo is ignored while the fault persists
  h.remote.L2 = true;
  for (int i = 0; i < 100; ++i) {h.cycle(std::string("lowstate timeout"));}
  h.remote.A = true;
  h.cycle(std::string("lowstate timeout"));
  EXPECT_EQ(h.fsm->current_mode(), "passive");
}

TEST(Fsm, TiltAbortsPolicy)
{
  Harness h;
  h.cycle();
  h.fsm->request("stand", "");
  h.cycle();
  h.fsm->request("walk", "");
  h.cycle();
  ASSERT_EQ(h.fsm->current_mode(), "walk");
  const float a = 1.2f;  // roll 1.2 rad > max_tilt 1.0
  h.state.quaternion = {std::cos(a / 2), std::sin(a / 2), 0.0f, 0.0f};
  h.cycle();
  EXPECT_EQ(h.fsm->current_mode(), "passive");
}

TEST(Fsm, InvalidConfigurationIsRejected)
{
  std::vector<std::unique_ptr<Mode>> modes;
  auto p = std::make_unique<PassiveMode>("passive", kN, std::vector<float>(kN, 1.0f));
  p->add_transition("nowhere", "A");
  modes.push_back(std::move(p));
  EXPECT_THROW(ModeManager(std::move(modes), "passive", "passive"), std::invalid_argument);
  EXPECT_THROW(PassiveMode("p", kN, std::vector<float>(3, 1.0f)), std::invalid_argument);
  int calls = 0;
  EXPECT_THROW(RlPolicyMode("w", kN, make_env(&calls), 333.0, 1.0), std::invalid_argument);
}
