// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include "go2_policy_runner/ros/policy_controller_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <filesystem>
#include <stdexcept>

#include "go2_policy_runner/policy/policy_env.hpp"
#include "go2_policy_runner/ros/param_utils.hpp"
#include "go2_policy_runner/ros/unitree_lowcmd.hpp"

namespace go2_policy_runner::ros
{

using namespace std::chrono_literals;

namespace
{
rclcpp::NodeOptions with_auto_params(const rclcpp::NodeOptions & options)
{
  rclcpp::NodeOptions o(options);
  o.automatically_declare_parameters_from_overrides(true);
  return o;
}
}  // namespace

// =============================================================================================
// construction
// =============================================================================================
PolicyControllerNode::PolicyControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("policy_controller", with_auto_params(options))
{
  load_parameters();
  robot_state_ = RobotState(num_motors_);
  joint_cmd_.assign(num_motors_, MotorCommand{});
  init_lowcmd(lowcmd_msg_, num_motors_);

  // Build the FSM first: any configuration error (bad yaml, missing policy, size mismatch)
  // aborts here, before anything is sent to the robot.
  fsm_ = build_fsm();
  fsm_->on_transition = [this](const std::string & from, const std::string & to,
      const std::string & reason) {
      if (from.empty()) {
        RCLCPP_INFO(get_logger(), "FSM: start in '%s' (%s)", to.c_str(), reason.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "FSM: '%s' -> '%s' (%s)", from.c_str(), to.c_str(), reason.c_str());
      }
      publish_state();
    };

  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions io_options;
  io_options.callback_group = io_group_;

  // Unitree topics. LowState: best-effort reader matches any writer QoS.
  lowstate_sub_ = create_subscription<LowState>(
    "lowstate", rclcpp::SensorDataQoS(),
    [this](const LowState::SharedPtr msg) {on_lowstate(msg);}, io_options);
  lowcmd_pub_ = create_publisher<LowCmd>("lowcmd", rclcpp::QoS(10));

  // Standard ROS interfaces.
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {on_cmd_vel(msg);}, io_options);
  state_pub_ = create_publisher<ControllerState>(
    "~/state", rclcpp::QoS(1).reliable().transient_local());
  if (publish_policy_step_) {
    policy_step_pub_ = create_publisher<PolicyStep>("~/policy_step", rclcpp::QoS(50));
  }
  set_mode_srv_ = create_service<SetMode>(
    "~/set_mode",
    [this](const std::shared_ptr<SetMode::Request> req, std::shared_ptr<SetMode::Response> res) {
      on_set_mode(req, res);
    },
    rmw_qos_profile_services_default, io_group_);

  motion_switcher_ = std::make_unique<MotionSwitcherClient>(*this, io_group_);

  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_)),
    [this]() {control_cycle();}, control_group_);
  state_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(0.1, state_rate_))),
    [this]() {publish_state();}, io_group_);

  std::string mode_list;
  for (const auto & n : fsm_->mode_names()) {mode_list += (mode_list.empty() ? "" : ", ") + n;}
  RCLCPP_INFO(
    get_logger(), "policy_controller: %zu motors, control %.0f Hz, modes: %s",
    num_motors_, control_rate_, mode_list.c_str());

  startup_thread_ = std::thread([this]() {startup_sequence();});
}

PolicyControllerNode::~PolicyControllerNode()
{
  stopping_ = true;
  if (startup_thread_.joinable()) {startup_thread_.join();}
}

double PolicyControllerNode::now_s() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start_).count();
}

void PolicyControllerNode::load_parameters()
{
  num_motors_ = static_cast<std::size_t>(param<int64_t>(*this, "num_motors", 12));
  if (num_motors_ == 0 || num_motors_ > kMaxMotors) {
    throw std::runtime_error("num_motors must be in [1, 20]");
  }
  control_rate_ = param<double>(*this, "control_rate", 500.0);
  if (control_rate_ <= 0.0) {throw std::runtime_error("control_rate must be > 0");}
  lowstate_timeout_ = param<double>(*this, "lowstate_timeout", 0.1);
  cmd_vel_timeout_ = param<double>(*this, "cmd_vel_timeout", 0.5);
  state_rate_ = param<double>(*this, "state_rate", 10.0);
  publish_policy_step_ = param<bool>(*this, "publish_policy_step", true);
  onnx_threads_ = static_cast<int>(param<int64_t>(*this, "onnx_threads", 1));
  motion_switcher_mode_ = param<std::string>(*this, "motion_switcher.mode", "auto");
  if (motion_switcher_mode_ != "auto" && motion_switcher_mode_ != "release" &&
    motion_switcher_mode_ != "skip")
  {
    throw std::runtime_error("motion_switcher.mode must be auto | release | skip");
  }
  motion_switcher_discovery_timeout_ = param<double>(*this, "motion_switcher.discovery_timeout", 3.0);
  motion_switcher_call_timeout_ = param<double>(*this, "motion_switcher.call_timeout", 5.0);
  lowcmd_check_duration_ = param<double>(*this, "lowcmd_check_duration", 0.5);
  policy_root_ = param<std::string>(*this, "policy_root", "");
  param<std::string>(*this, "default_policy_dir", "");

  // Optional hard limits for position targets (SDK order), applied to every mode.
  if (has_parameter("joint_limits.lower") || has_parameter("joint_limits.upper")) {
    q_lower_ = motor_array(required_param(*this, "joint_limits.lower"), num_motors_);
    q_upper_ = motor_array(required_param(*this, "joint_limits.upper"), num_motors_);
    for (std::size_t i = 0; i < num_motors_; ++i) {
      if (q_lower_[i] >= q_upper_[i]) {
        throw std::runtime_error("joint_limits: lower >= upper for motor " + std::to_string(i));
      }
    }
  }
}

// =============================================================================================
// FSM construction from parameters
//
//   fsm.modes:    [passive, stand, walk, ...]
//   fsm.initial:  passive
//   fsm.fallback: passive
//   fsm.<mode>.type: passive | fix_stand | rl_policy
//   fsm.<mode>.transitions.<target>: "<remote expression>"   ("" = only via ~/set_mode)
//   poses.<name>: [...]                    shared key poses (SDK order)
// =============================================================================================
std::unique_ptr<fsm::ModeManager> PolicyControllerNode::build_fsm()
{
  const auto modes_param = required_param(*this, "fsm.modes");
  const auto mode_names = modes_param.as_string_array();
  if (mode_names.empty()) {throw std::runtime_error("fsm.modes is empty");}

  std::vector<std::unique_ptr<fsm::Mode>> modes;
  for (const auto & name : mode_names) {
    try {
      auto mode = build_mode(name);
      const std::string tprefix = "fsm." + name + ".transitions";
      for (const auto & target : child_names(*this, tprefix)) {
        const auto expr = get_parameter(tprefix + "." + target);
        if (expr.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          throw std::runtime_error("transition to '" + target + "' must be a string expression");
        }
        mode->add_transition(target, expr.as_string());
      }
      modes.push_back(std::move(mode));
    } catch (const std::exception & e) {
      throw std::runtime_error("mode '" + name + "': " + e.what());
    }
  }
  const auto initial = param<std::string>(*this, "fsm.initial", mode_names.front());
  fallback_mode_ = param<std::string>(*this, "fsm.fallback", "passive");
  return std::make_unique<fsm::ModeManager>(std::move(modes), initial, fallback_mode_);
}

std::vector<float> PolicyControllerNode::pose(
  const std::string & mode_prefix, const std::string & pose_name)
{
  for (const auto & key : {mode_prefix + ".poses." + pose_name, "poses." + pose_name}) {
    if (has_parameter(key)) {return motor_array(get_parameter(key), num_motors_);}
  }
  throw std::runtime_error("pose '" + pose_name + "' not found (" + mode_prefix + ".poses." +
          pose_name + " or poses." + pose_name + ")");
}

std::unique_ptr<fsm::Mode> PolicyControllerNode::build_mode(const std::string & name)
{
  const std::string p = "fsm." + name;
  const auto type = required_param(*this, p + ".type").as_string();

  if (type == "passive") {
    return std::make_unique<fsm::PassiveMode>(
      name, num_motors_, motor_array(required_param(*this, p + ".kd"), num_motors_));
  }

  if (type == "fix_stand") {
    const auto sequence = required_param(*this, p + ".sequence").as_string_array();
    std::vector<std::vector<float>> poses;
    for (const auto & s : sequence) {poses.push_back(pose(p, s));}
    std::vector<double> durations;
    for (float d : float_array(required_param(*this, p + ".durations"))) {durations.push_back(d);}
    const auto interp = param<std::string>(*this, p + ".interpolation", "smooth");
    if (interp != "smooth" && interp != "linear") {
      throw std::runtime_error("interpolation must be smooth | linear");
    }
    return std::make_unique<fsm::FixStandMode>(
      name, num_motors_,
      motor_array(required_param(*this, p + ".kp"), num_motors_),
      motor_array(required_param(*this, p + ".kd"), num_motors_),
      poses, durations,
      interp == "smooth" ? fsm::FixStandMode::Interpolation::kSmooth :
      fsm::FixStandMode::Interpolation::kLinear);
  }

  if (type == "rl_policy") {
    std::string dir = param<std::string>(*this, p + ".policy_dir", "");
    if (dir.empty()) {dir = get_parameter("default_policy_dir").as_string();}
    if (dir.empty()) {
      throw std::runtime_error(p + ".policy_dir is empty and no default_policy_dir is given "
              "(launch argument policy_dir:=...)");
    }
    const auto resolved = resolve_policy_path(dir);
    policy::OnnxOptions onnx;
    onnx.intra_op_threads = onnx_threads_;
    auto env = policy::load_policy_env(resolved, num_motors_, onnx);
    const auto files = policy::resolve_policy_dir(resolved);
    RCLCPP_INFO(
      get_logger(), "mode '%s': policy %s (step_dt %.3f s, %zu actions)", name.c_str(),
      files.dir.c_str(), env->step_dt(), env->action_manager().total_dim());
    auto mode = std::make_unique<fsm::RlPolicyMode>(
      name, num_motors_, std::move(env), control_rate_, param<double>(*this, p + ".max_tilt", 1.0));
    if (publish_policy_step_) {
      mode->on_policy_step = [this](const fsm::PolicyStepInfo & info) {publish_policy_step(info);};
    }
    return mode;
  }

  throw std::runtime_error("unknown mode type '" + type + "' (passive | fix_stand | rl_policy)");
}

std::string PolicyControllerNode::resolve_policy_path(const std::string & path) const
{
  namespace fs = std::filesystem;
  const std::string pkg_prefix = "package://";
  if (path.rfind(pkg_prefix, 0) == 0) {
    const auto rest = path.substr(pkg_prefix.size());
    const auto slash = rest.find('/');
    const auto pkg = rest.substr(0, slash);
    const auto sub = slash == std::string::npos ? "" : rest.substr(slash + 1);
    return (fs::path(ament_index_cpp::get_package_share_directory(pkg)) / sub).string();
  }
  fs::path p(path);
  if (p.is_absolute()) {return p.string();}
  fs::path root = policy_root_.empty() ?
    fs::path(ament_index_cpp::get_package_share_directory("go2_policy_runner")) / "policies" :
    fs::path(policy_root_);
  return (root / p).string();
}

// =============================================================================================
// startup
// =============================================================================================
void PolicyControllerNode::startup_sequence()
{
  auto set_status = [this](const std::string & s) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      startup_status_ = s;
    };

  // 1. robot / simulator must be publishing its state
  set_status("waiting for lowstate");
  RCLCPP_INFO(get_logger(), "waiting for %s ...", lowstate_sub_->get_topic_name());
  for (int i = 1; !stopping_ && rclcpp::ok(); ++i) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (have_lowstate_) {break;}
    }
    std::this_thread::sleep_for(100ms);
    if (i % 50 == 0) {
      RCLCPP_WARN(
        get_logger(), "no %s after %d s - check network_interface / ROS_DOMAIN_ID / "
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", lowstate_sub_->get_topic_name(), i / 10);
    }
  }
  if (stopping_ || !rclcpp::ok()) {return;}
  RCLCPP_INFO(get_logger(), "lowstate received");

  // 2. built-in sport mode must not fight with us
  set_status("releasing motion switcher");
  if (!release_motion_switcher()) {
    set_status("FAILED: could not release the motion switcher");
    return;
  }
  // 3. nobody else may be sending low-level commands
  set_status("checking for other lowcmd publishers");
  if (!check_other_controllers()) {
    set_status("FAILED: another controller publishes lowcmd");
    return;
  }
  if (stopping_) {return;}

  set_status("ready");
  ready_ = true;
  RCLCPP_INFO(get_logger(), "ready - controlling the robot");
}

bool PolicyControllerNode::release_motion_switcher()
{
  if (motion_switcher_mode_ == "skip") {
    RCLCPP_INFO(get_logger(), "motion_switcher.mode = skip: not touching the motion switcher");
    return true;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const auto discovery = std::chrono::duration<double>(motion_switcher_discovery_timeout_);
  while (!motion_switcher_->service_available() && !stopping_ &&
    std::chrono::steady_clock::now() - t0 < discovery)
  {
    std::this_thread::sleep_for(100ms);
  }
  if (!motion_switcher_->service_available()) {
    if (motion_switcher_mode_ == "release") {
      RCLCPP_FATAL(
        get_logger(), "motion_switcher service not found on the network "
        "(motion_switcher.mode = release)");
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "no motion_switcher service on the network (simulation) - nothing to release");
    return true;
  }
  std::this_thread::sleep_for(300ms);  // let the request writer match the service reader

  const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(motion_switcher_call_timeout_));
  for (int attempt = 0; attempt < 5 && !stopping_; ++attempt) {
    int32_t code = 0;
    const auto mode = motion_switcher_->check_mode(timeout, &code);
    if (!mode) {
      RCLCPP_ERROR(get_logger(), "motion_switcher CheckMode failed (code %d)", code);
      std::this_thread::sleep_for(500ms);
      continue;
    }
    if (mode->empty()) {
      RCLCPP_INFO(get_logger(), "motion switcher: no active motion mode - low-level control allowed");
      return true;
    }
    RCLCPP_WARN(
      get_logger(), "motion switcher: active mode '%s' - releasing it (the robot will go limp!)",
      mode->c_str());
    const auto r = motion_switcher_->release_mode(timeout);
    if (!r.ok()) {RCLCPP_ERROR(get_logger(), "ReleaseMode failed (code %d)", r.code);}
    std::this_thread::sleep_for(1s);
  }
  RCLCPP_FATAL(get_logger(), "could not release the motion switcher");
  return false;
}

bool PolicyControllerNode::check_other_controllers()
{
  if (lowcmd_check_duration_ <= 0.0) {return true;}
  // Same check as unitree_rl_lab deploy: listen to lowcmd for a while before publishing.
  auto count = std::make_shared<std::atomic<int>>(0);
  rclcpp::SubscriptionOptions options;
  options.callback_group = io_group_;
  auto sub = create_subscription<LowCmd>(
    "lowcmd", rclcpp::SensorDataQoS(), [count](const LowCmd::SharedPtr) {++(*count);}, options);
  std::this_thread::sleep_for(std::chrono::duration<double>(lowcmd_check_duration_));
  sub.reset();
  if (*count > 0) {
    RCLCPP_FATAL(
      get_logger(), "another process is publishing %s (%d messages in %.1f s). "
      "Stop it before starting the policy controller.", lowcmd_pub_->get_topic_name(),
      count->load(), lowcmd_check_duration_);
    return false;
  }
  return true;
}

// =============================================================================================
// runtime
// =============================================================================================
void PolicyControllerNode::on_lowstate(const LowState::SharedPtr msg)
{
  const double t = now_s();
  std::lock_guard<std::mutex> lock(state_mutex_);
  read_lowstate(*msg, robot_state_);
  have_lowstate_ = true;
  last_lowstate_time_ = t;
  ++lowstate_count_;
  if (t - lowstate_rate_window_start_ >= 1.0) {
    lowstate_rate_ = lowstate_count_ / (t - lowstate_rate_window_start_);
    lowstate_count_ = 0;
    lowstate_rate_window_start_ = t;
  }
}

void PolicyControllerNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cmd_vel_.vx = static_cast<float>(msg->linear.x);
  cmd_vel_.vy = static_cast<float>(msg->linear.y);
  cmd_vel_.wz = static_cast<float>(msg->angular.z);
  last_cmd_vel_time_ = now_s();
}

void PolicyControllerNode::on_set_mode(
  const std::shared_ptr<SetMode::Request> req, std::shared_ptr<SetMode::Response> res)
{
  if (!ready_) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    res->success = false;
    res->message = "controller is not ready: " + startup_status_;
    return;
  }
  const auto err = fsm_->request(req->mode, "service request");
  res->success = err.empty();
  res->message = err.empty() ? "switching to '" + req->mode + "'" : err;
  if (!err.empty()) {RCLCPP_WARN(get_logger(), "set_mode rejected: %s", err.c_str());}
}

void PolicyControllerNode::control_cycle()
{
  if (!ready_ || stopping_) {return;}
  const auto wall0 = std::chrono::steady_clock::now();
  const double t = now_s();

  RobotState state(num_motors_);
  VelocityCommand cmd;
  double age;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = robot_state_;
    age = t - last_lowstate_time_;
    if (last_cmd_vel_time_ >= 0.0 && t - last_cmd_vel_time_ <= cmd_vel_timeout_) {cmd = cmd_vel_;}
  }
  std::optional<std::string> fault;
  if (age > lowstate_timeout_) {
    fault = "lowstate timeout (" + std::to_string(static_cast<int>(age * 1000)) + " ms)";
  }

  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    try {
      fsm_->update(fsm::ControlInput{state, cmd, t}, joint_cmd_, fault);
    } catch (const std::exception & e) {
      // Only reachable if the fallback mode itself failed: command pure damping.
      RCLCPP_FATAL_THROTTLE(get_logger(), *get_clock(), 1000, "FSM failure: %s", e.what());
      for (auto & m : joint_cmd_) {m.kp = 0.0f; m.dq = 0.0f; m.tau = 0.0f;}
    }
    publish_lowcmd();
  }

  const double cycle_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall0).count();
  std::lock_guard<std::mutex> lock(stats_mutex_);
  max_cycle_ms_ = std::max(max_cycle_ms_, cycle_ms);
  ++cycle_count_;
  if (t - stats_window_start_ >= 1.0) {
    control_rate_measured_ = cycle_count_ / (t - stats_window_start_);
    cycle_count_ = 0;
    stats_window_start_ = t;
  }
}

void PolicyControllerNode::publish_lowcmd()
{
  if (!q_lower_.empty()) {
    for (std::size_t i = 0; i < num_motors_; ++i) {
      joint_cmd_[i].q = std::clamp(joint_cmd_[i].q, q_lower_[i], q_upper_[i]);
    }
  }
  write_lowcmd(joint_cmd_, lowcmd_msg_);
  update_lowcmd_crc(lowcmd_msg_);
  lowcmd_pub_->publish(lowcmd_msg_);
}

void PolicyControllerNode::safe_stop(std::chrono::milliseconds duration)
{
  stopping_ = true;
  if (startup_thread_.joinable()) {startup_thread_.join();}
  if (!ready_) {return;}
  fsm::Mode * fallback = fsm_->find(fallback_mode_);
  if (!fallback) {return;}
  RCLCPP_INFO(get_logger(), "stopping: switching motors to '%s'", fallback->name().c_str());
  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  const auto t_end = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < t_end && rclcpp::ok()) {
    RobotState state(num_motors_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = robot_state_;
    }
    VelocityCommand zero;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      fallback->run(fsm::ControlInput{state, zero, now_s()}, joint_cmd_);
      publish_lowcmd();
    }
    std::this_thread::sleep_for(period);
  }
}

void PolicyControllerNode::publish_state()
{
  ControllerState msg;
  msg.header.stamp = now();
  msg.mode = fsm_->current_mode();
  if (auto * m = fsm_->find(msg.mode)) {msg.mode_type = m->type();}
  msg.modes = fsm_->mode_names();
  msg.allowed_transitions = fsm_->allowed_targets();
  msg.last_transition_reason = fsm_->last_transition_reason();
  msg.ready = ready_;
  const double t = now_s();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    msg.lowstate_age = have_lowstate_ ? static_cast<float>(t - last_lowstate_time_) : -1.0f;
    msg.lowstate_ok = have_lowstate_ && (t - last_lowstate_time_) <= lowstate_timeout_;
    msg.lowstate_rate = (t - lowstate_rate_window_start_ < 2.0) ? static_cast<float>(lowstate_rate_) : 0.0f;
    msg.cmd_vel_ok = last_cmd_vel_time_ >= 0.0 && (t - last_cmd_vel_time_) <= cmd_vel_timeout_;
    if (!ready_) {msg.last_transition_reason = startup_status_;}
  }
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    msg.control_rate = static_cast<float>(control_rate_measured_);
    msg.max_cycle_time = static_cast<float>(max_cycle_ms_);
    max_cycle_ms_ = 0.0;
  }
  state_pub_->publish(msg);
}

void PolicyControllerNode::publish_policy_step(const fsm::PolicyStepInfo & info)
{
  if (!policy_step_pub_) {return;}
  PolicyStep msg;
  msg.header.stamp = now();
  msg.mode = info.mode;
  msg.step = static_cast<uint64_t>(info.step);
  if (info.observations) {
    for (const auto & group : *info.observations) {
      msg.observation_groups.push_back(group.first);
      msg.observation_sizes.push_back(static_cast<uint32_t>(group.second.size()));
      msg.observation.insert(msg.observation.end(), group.second.begin(), group.second.end());
    }
  }
  if (info.action) {msg.action = *info.action;}
  if (info.position_targets) {msg.joint_position_target = *info.position_targets;}
  msg.command.x = info.command.vx;
  msg.command.y = info.command.vy;
  msg.command.z = info.command.wz;
  msg.inference_time = static_cast<float>(info.inference_ms);
  policy_step_pub_->publish(msg);
}

}  // namespace go2_policy_runner::ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(go2_policy_runner::ros::PolicyControllerNode)
