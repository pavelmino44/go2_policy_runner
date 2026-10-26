// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0

#include "go2_policy_runner/monitor/monitor_node.hpp"

#include "go2_policy_runner/remote/unitree_remote.hpp"

#include <algorithm>
#include <cstring>
#include <functional>

namespace go2_policy_runner::ros
{

namespace
{
std::vector<float> to_floats(const rclcpp::Parameter & p)
{
  std::vector<float> out;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    for (double v : p.as_double_array()) {out.push_back(static_cast<float>(v));}
  } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    for (int64_t v : p.as_integer_array()) {out.push_back(static_cast<float>(v));}
  }
  return out;
}

std::optional<double> to_number(const rclcpp::Parameter & p)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {return p.as_double();}
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(p.as_int());
  }
  return std::nullopt;
}
}  // namespace

MonitorNode::MonitorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_monitor", options)
{
  controller_ = declare_parameter<std::string>("controller", "policy_controller");
  refresh_rate_ = std::max(1.0, declare_parameter<double>("refresh_rate", 10.0));
  nominal_lowstate_rate_ = declare_parameter<double>("nominal_lowstate_rate", 0.0);
  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);
  thresholds_.motor_temp_warn = declare_parameter<double>("thresholds.motor_temp", 65.0);
  thresholds_.joint_limit_margin = declare_parameter<double>("thresholds.joint_limit_margin", 0.05);
  thresholds_.battery_warn = declare_parameter<double>("thresholds.battery", 20.0);
  thresholds_.lowstate_rate_ratio = declare_parameter<double>("thresholds.lowstate_rate_ratio", 0.8);
  thresholds_.tilt_ratio = declare_parameter<double>("thresholds.tilt_ratio", 0.7);
  thresholds_.inference_ratio = declare_parameter<double>("thresholds.inference_ratio", 0.5);

  const std::string ctrl = "/" + controller_;

  state_sub_ = create_subscription<ControllerState>(
    ctrl + "/state", rclcpp::QoS(1).reliable().transient_local(),
    [this](const ControllerState::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      t_state_ = now_s();
      auto & d = data_;
      d.have_state = true;
      d.ready = m->ready;
      d.mode = m->mode;
      d.mode_type = m->mode_type;
      d.mode_time = m->mode_time;
      d.allowed = m->allowed_transitions;
      d.last_reason = m->last_transition_reason;
      d.lowstate_ok = m->lowstate_ok;
      d.lowstate_rate = m->lowstate_rate;
      d.control_rate = m->control_rate;
      d.max_cycle_ms = m->max_cycle_time;
      d.cmd_vel_ok = m->cmd_vel_ok;
      auto it = max_tilt_by_mode_.find(m->mode);
      d.max_tilt = it == max_tilt_by_mode_.end() ? 0.0 : it->second;
    });

  policy_sub_ = create_subscription<PolicyStep>(
    ctrl + "/policy_step", rclcpp::QoS(10),
    [this](const PolicyStep::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      const double t = now_s();
      auto & d = data_;
      if (t_policy_ > 0.0 && m->step == d.policy_step + 1) {
        const double period = (t - t_policy_) * 1000.0;
        d.policy_period_ms = d.policy_period_ms > 0.0 ? 0.9 * d.policy_period_ms + 0.1 * period : period;
      }
      t_policy_ = t;
      d.have_policy = true;
      d.policy_step = m->step;
      d.inference_ms = m->inference_time;
      d.policy_cmd = {m->command.x, m->command.y, m->command.z};
      if (!m->action.empty()) {
        const auto mm = std::minmax_element(m->action.begin(), m->action.end());
        d.action_min = *mm.first;
        d.action_max = *mm.second;
      }
    });

  lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
    "lowstate", rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::LowState::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      t_lowstate_ = now_s();
      auto & d = data_;
      d.have_lowstate = true;
      for (int i = 0; i < 4; ++i) {d.quat[i] = m->imu_state.quaternion[i];}
      for (int i = 0; i < 3; ++i) {
        d.gyro[i] = m->imu_state.gyroscope[i];
        d.acc[i] = m->imu_state.accelerometer[i];
      }
      const std::size_t n = d.joints.empty() ? 12 : d.joints.size();
      d.joints.resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        auto & j = d.joints[i];
        j.name = monitor::go2_joint_name(i);
        j.q = m->motor_state[i].q;
        j.dq = m->motor_state[i].dq;
        j.tau = m->motor_state[i].tau_est;
        j.temperature = m->motor_state[i].temperature;
        j.has_limits = i < lower_.size() && i < upper_.size();
        if (j.has_limits) {
          j.lower = lower_[i];
          j.upper = upper_[i];
        }
      }
      d.battery_soc = m->bms_state.soc;
      d.power_v = m->power_v;
      d.power_a = m->power_a;
      // the simulator leaves the battery fields at zero
      d.battery_valid = m->bms_state.soc > 0 || m->power_v > 0.0f;
      for (int i = 0; i < 4; ++i) {d.foot_force[i] = m->foot_force[i];}
      std::memcpy(d.wireless_remote.data(), m->wireless_remote.data(), 40);
    });

  lowcmd_sub_ = create_subscription<unitree_go::msg::LowCmd>(
    "lowcmd", rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::LowCmd::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto & d = data_;
      d.have_lowcmd = true;
      const std::size_t n = d.joints.empty() ? 12 : d.joints.size();
      d.joints.resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        d.joints[i].q_cmd = m->motor_cmd[i].q;
        d.joints[i].kp = m->motor_cmd[i].kp;
        d.joints[i].kd = m->motor_cmd[i].kd;
      }
    });

  // best-effort reader matches both reliable and best-effort cmd_vel publishers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10).best_effort(),
    [this](const geometry_msgs::msg::Twist::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      t_cmd_vel_ = now_s();
      data_.have_cmd_vel = true;
      data_.cmd_vel = {m->linear.x, m->linear.y, m->angular.z};
    });

  remote_sub_ = create_subscription<unitree_go::msg::WirelessController>(
    "remote", rclcpp::QoS(10).best_effort(),
    [this](const unitree_go::msg::WirelessController::SharedPtr m) {
      RemoteFrame f;
      f.set_buttons(m->keys);
      f.lx = m->lx;
      f.ly = m->ly;
      f.rx = m->rx;
      f.ry = m->ry;
      std::lock_guard<std::mutex> lock(mutex_);
      t_remote_ = now_s();
      data_.have_remote_topic = true;
      data_.remote_topic = f.encode();
    });

  if (publish_diagnostics_) {
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  }
  set_mode_client_ = create_client<SetMode>(ctrl + "/set_mode");
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, controller_);

  slow_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {on_slow_timer();});
}

double MonitorNode::now_s() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
}

monitor::Snapshot MonitorNode::snapshot()
{
  std::lock_guard<std::mutex> lock(mutex_);
  monitor::Snapshot s = data_;
  s.now = now_s();
  auto age = [&s](double t) {return t < 0.0 ? 1e9 : s.now - t;};
  s.state_age = age(t_state_);
  s.lowstate_age = age(t_lowstate_);
  s.cmd_vel_age = age(t_cmd_vel_);
  s.policy_age = age(t_policy_);
  s.remote_topic_age = age(t_remote_);
  if (s.nominal_lowstate_rate <= 0.0) {s.nominal_lowstate_rate = nominal_lowstate_rate_;}
  return s;
}

void MonitorNode::on_slow_timer()
{
  // who publishes cmd_vel (helps to spot teleop / keyboard / planner conflicts)
  std::vector<std::string> pubs;
  for (const auto & info : get_publishers_info_by_topic(cmd_vel_sub_->get_topic_name())) {
    pubs.push_back(info.node_name());
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.cmd_vel_publishers = pubs;
  }
  // controller parameters: refresh every 10 s (the controller may have been restarted)
  if (now_s() - t_params_ > 10.0 && param_client_->service_is_ready()) {
    fetch_controller_parameters();
  }
  if (diag_pub_) {publish_diagnostics();}
}

void MonitorNode::fetch_controller_parameters()
{
  // One request per parameter: if a batch contains an undeclared name, the whole request fails.
  auto fetch = [this](const std::string & name, std::function<void(const rclcpp::Parameter &)> use) {
      param_client_->get_parameters(
        {name}, [use](std::shared_future<std::vector<rclcpp::Parameter>> f) {
          try {
            const auto params = f.get();
            if (!params.empty()) {use(params.front());}
          } catch (const std::exception &) {
          }
        });
    };

  fetch("control_rate", [this](const rclcpp::Parameter & p) {
      if (auto v = to_number(p)) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (nominal_lowstate_rate_ <= 0.0) {data_.nominal_lowstate_rate = *v;}
      }
    });
  fetch("joint_limits.lower", [this](const rclcpp::Parameter & p) {
      std::lock_guard<std::mutex> lock(mutex_);
      lower_ = to_floats(p);
    });
  fetch("joint_limits.upper", [this](const rclcpp::Parameter & p) {
      std::lock_guard<std::mutex> lock(mutex_);
      upper_ = to_floats(p);
    });
  // Only rl_policy modes declare max_tilt. List what exists instead of asking for every mode:
  // a get_parameters call for an undeclared name makes the controller log a warning.
  param_client_->list_parameters(
    {"fsm"}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE,
    [fetch, this](std::shared_future<rcl_interfaces::msg::ListParametersResult> f) {
      static const std::string kSuffix = ".max_tilt";
      try {
        for (const auto & name : f.get().names) {
          if (name.size() <= 4 + kSuffix.size() ||
            name.compare(name.size() - kSuffix.size(), kSuffix.size(), kSuffix) != 0)
          {
            continue;
          }
          const std::string mode = name.substr(4, name.size() - 4 - kSuffix.size());  // fsm.<mode>.max_tilt
          fetch(name, [this, mode](const rclcpp::Parameter & t) {
              if (auto v = to_number(t)) {
                std::lock_guard<std::mutex> lock(mutex_);
                max_tilt_by_mode_[mode] = *v;
              }
            });
        }
      } catch (const std::exception &) {
      }
    });
  t_params_ = now_s();
}

void MonitorNode::request_passive()
{
  if (!set_mode_client_->service_is_ready()) {
    std::lock_guard<std::mutex> lock(mutex_);
    action_status_ = "set_mode service not available";
    return;
  }
  auto req = std::make_shared<SetMode::Request>();
  req->mode = "passive";
  {
    std::lock_guard<std::mutex> lock(mutex_);
    action_status_ = "passive requested...";
  }
  set_mode_client_->async_send_request(
    req, [this](rclcpp::Client<SetMode>::SharedFuture f) {
      std::lock_guard<std::mutex> lock(mutex_);
      try {
        const auto res = f.get();
        action_status_ = (res->success ? "passive: " : "passive rejected: ") + res->message;
      } catch (const std::exception & e) {
        action_status_ = std::string("passive request failed: ") + e.what();
      }
    });
}

std::string MonitorNode::action_status()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return action_status_;
}

void MonitorNode::publish_diagnostics()
{
  const auto s = snapshot();
  const auto alerts = monitor::compute_alerts(s, thresholds_);
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();
  for (auto c : {monitor::Category::kController, monitor::Category::kCommunication,
      monitor::Category::kCommand, monitor::Category::kBase, monitor::Category::kJoints,
      monitor::Category::kPolicy, monitor::Category::kBattery})
  {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = std::string("go2_policy_runner/") + monitor::category_name(c);
    st.hardware_id = "go2";
    st.level = static_cast<unsigned char>(monitor::worst_level(alerts, c));
    for (const auto & a : alerts) {
      if (a.category == c) {st.message += (st.message.empty() ? "" : "; ") + a.text;}
    }
    if (st.message.empty()) {st.message = "OK";}
    if (c == monitor::Category::kController) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "mode";
      kv.value = s.mode;
      st.values.push_back(kv);
    }
    arr.status.push_back(st);
  }
  diag_pub_->publish(arr);
}

}  // namespace go2_policy_runner::ros
