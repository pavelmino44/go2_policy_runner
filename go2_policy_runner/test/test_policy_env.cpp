// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// Runs from <package>/test (WORKING_DIRECTORY in CMakeLists.txt).

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "go2_policy_runner/policy/policy_env.hpp"

using namespace go2_policy_runner;
using namespace go2_policy_runner::policy;
namespace fs = std::filesystem;

namespace
{
const char * kFixture = "fixtures/go2_linear";

// isaac index -> sdk index for the fixture
const std::vector<int> kMap{3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8};
const std::vector<float> kDefault{0.1f, -0.1f, 0.1f, -0.1f, 0.8f, 0.8f, 1.0f, 1.0f, -1.5f, -1.5f, -1.5f, -1.5f};

RobotState make_state()
{
  RobotState s(12);
  s.quaternion = {0.9950042f, 0.0f, 0.0998334f, 0.0f};  // 0.2 rad pitch
  s.gyroscope = {0.1f, -0.2f, 0.3f};
  for (int i = 0; i < 12; ++i) {
    s.q[i] = 0.05f * static_cast<float>(i) - 0.3f;
    s.dq[i] = 0.5f - 0.1f * static_cast<float>(i);
  }
  return s;
}

/// Linear model from make_linear_policy.py
std::vector<float> linear(const std::vector<float> & obs, std::size_t m)
{
  std::vector<float> a(m);
  for (std::size_t j = 0; j < m; ++j) {
    double acc = 0.001 * static_cast<double>(j);
    for (std::size_t i = 0; i < obs.size(); ++i) {
      acc += obs[i] * 0.01 * static_cast<double>(static_cast<int>((i + j) % 5) - 2);
    }
    a[j] = static_cast<float>(acc);
  }
  return a;
}

class FakeModel : public PolicyModel
{
public:
  FakeModel(std::size_t in, std::size_t out)
  : inputs_{{"obs", in}}, out_(out) {}
  const std::vector<std::pair<std::string, std::size_t>> & inputs() const override {return inputs_;}
  std::size_t output_size() const override {return out_;}
  std::vector<float> act(const std::map<std::string, std::vector<float>> & obs) override
  {
    last = obs.at("obs");
    ++calls;
    return std::vector<float>(out_, value);
  }
  std::vector<float> last;
  int calls{0};
  float value{0.0f};

private:
  std::vector<std::pair<std::string, std::size_t>> inputs_;
  std::size_t out_;
};
}  // namespace

TEST(PolicyEnv, LoadsGo2FixtureAndMatchesReferenceMath)
{
  auto env = load_policy_env(kFixture, 12);
  ASSERT_EQ(env->action_manager().total_dim(), 12u);
  EXPECT_FLOAT_EQ(env->step_dt(), 0.02f);

  const RobotState s = make_state();
  const VelocityCommand cmd{0.5f, -0.2f, 0.3f};
  env->reset(s, cmd);
  env->step(s, cmd);

  // Build the expected observation by hand (IsaacLab order, clip then scale)
  std::vector<float> obs;
  for (float g : s.gyroscope) {obs.push_back(0.2f * g);}
  // projected gravity for a pitch of 0.2 rad: (sin 0.2, 0, -cos 0.2)
  obs.push_back(std::sin(0.2f)); obs.push_back(0.0f); obs.push_back(-std::cos(0.2f));
  obs.push_back(0.5f); obs.push_back(-0.2f); obs.push_back(0.3f);
  for (int i = 0; i < 12; ++i) {obs.push_back(s.q[kMap[i]] - kDefault[i]);}
  for (int i = 0; i < 12; ++i) {obs.push_back(0.05f * s.dq[kMap[i]]);}
  for (int i = 0; i < 12; ++i) {obs.push_back(0.0f);}  // last action after reset

  const auto & got = env->observation_manager().last().at("obs");
  ASSERT_EQ(got.size(), 45u);
  for (std::size_t i = 0; i < obs.size(); ++i) {EXPECT_NEAR(got[i], obs[i], 1e-5) << "obs " << i;}

  const auto expected = linear(obs, 12);
  for (int i = 0; i < 12; ++i) {
    EXPECT_NEAR(env->action_manager().action()[i], expected[i], 1e-5);
    // target in SDK order = default + 0.25 * action
    EXPECT_NEAR(env->position_targets_sdk()[kMap[i]], kDefault[i] + 0.25f * expected[i], 1e-5);
  }

  // second step: last_action must be the previous raw action
  env->step(s, cmd);
  const auto & got2 = env->observation_manager().last().at("obs");
  for (int i = 0; i < 12; ++i) {EXPECT_NEAR(got2[33 + i], expected[i], 1e-6);}
}

TEST(PolicyEnv, VelocityCommandIsClampedToTrainingRanges)
{
  auto env = load_policy_env(kFixture, 12);
  const RobotState s = make_state();
  env->reset(s, {5.0f, -5.0f, 0.1f});
  env->step(s, {5.0f, -5.0f, 0.1f});
  const auto & o = env->observation_manager().last().at("obs");
  EXPECT_FLOAT_EQ(o[6], 1.0f);
  EXPECT_FLOAT_EQ(o[7], -0.4f);
  EXPECT_FLOAT_EQ(o[8], 0.1f);
}

TEST(PolicyEnv, HistoryIsOldestFirstPerTerm)
{
  YAML::Node cfg = YAML::LoadFile(std::string(kFixture) + "/params/deploy.yaml");
  for (auto it = cfg["observations"].begin(); it != cfg["observations"].end(); ++it) {
    it->second["history_length"] = 3;
  }
  auto model = std::make_unique<FakeModel>(45 * 3, 12);
  FakeModel * m = model.get();
  PolicyEnv env(cfg, std::move(model), 12);

  RobotState s = make_state();
  env.reset(s, {});
  s.gyroscope = {1.0f, 2.0f, 3.0f};
  env.step(s, {});
  // base_ang_vel block: [reset, reset, new]
  ASSERT_EQ(m->last.size(), 135u);
  EXPECT_NEAR(m->last[0], 0.2f * 0.1f, 1e-6);
  EXPECT_NEAR(m->last[3], 0.2f * 0.1f, 1e-6);
  EXPECT_NEAR(m->last[6], 0.2f * 1.0f, 1e-6);
  // next term (projected_gravity) starts after 3 * 3 values
  EXPECT_NEAR(m->last[9 + 2], -std::cos(0.2f), 1e-5);
}

TEST(PolicyEnv, RejectsModelSizeMismatch)
{
  YAML::Node cfg = YAML::LoadFile(std::string(kFixture) + "/params/deploy.yaml");
  EXPECT_THROW(PolicyEnv(cfg, std::make_unique<FakeModel>(44, 12), 12), std::runtime_error);
  EXPECT_THROW(PolicyEnv(cfg, std::make_unique<FakeModel>(45, 11), 12), std::runtime_error);
  EXPECT_NO_THROW(PolicyEnv(cfg, std::make_unique<FakeModel>(45, 12), 12));
}

TEST(PolicyEnv, RejectsUnknownObservation)
{
  YAML::Node cfg = YAML::LoadFile(std::string(kFixture) + "/params/deploy.yaml");
  cfg["observations"]["base_lin_vel"]["params"] = YAML::Node(YAML::NodeType::Map);
  EXPECT_THROW(PolicyEnv(cfg, std::make_unique<FakeModel>(48, 12), 12), std::runtime_error);
}

TEST(PolicyEnv, RejectsBadJointMap)
{
  YAML::Node cfg = YAML::LoadFile(std::string(kFixture) + "/params/deploy.yaml");
  cfg["joint_ids_map"][0] = 12;
  EXPECT_THROW(PolicyEnv(cfg, std::make_unique<FakeModel>(45, 12), 12), std::runtime_error);
}

TEST(PolicyDir, PicksLatestRun)
{
  const fs::path root = fs::temp_directory_path() / "go2_policy_runner_test_runs";
  fs::remove_all(root);
  for (const char * run : {"2026-01-01_10-00-00", "2026-02-01_10-00-00", "2026-03-01_no_export"}) {
    fs::create_directories(root / run);
  }
  for (const char * run : {"2026-01-01_10-00-00", "2026-02-01_10-00-00"}) {
    fs::create_directories(root / run / "exported");
    fs::create_directories(root / run / "params");
    std::ofstream(root / run / "exported" / "policy.onnx") << "x";
    std::ofstream(root / run / "params" / "deploy.yaml") << "x: 1";
  }
  const auto files = resolve_policy_dir(root);
  EXPECT_EQ(files.dir.filename(), "2026-02-01_10-00-00");
  EXPECT_THROW(resolve_policy_dir(root / "2026-03-01_no_export"), std::runtime_error);
  fs::remove_all(root);
}
