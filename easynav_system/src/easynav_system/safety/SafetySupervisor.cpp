// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Implementation of the SafetySupervisor class.

#include <cmath>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "easynav_system/RealTime.hpp"
#include "easynav_system/safety/SafetySupervisor.hpp"

namespace easynav::safety
{

void
SafetySupervisor::declare_parameters(rclcpp_lifecycle::LifecycleNode & node)
{
  node.declare_parameter("safety.mode", false);
  node.declare_parameter("safety.lock_memory", false);
  // Limits configured in the safety channel, checked against robot_limits (0: not given).
  node.declare_parameter("safety.plc_limits.max_linear_vel", 0.0);
  node.declare_parameter("safety.plc_limits.max_angular_vel", 0.0);
  node.declare_parameter("safety.heartbeat.period", 0.0);
  node.declare_parameter("safety.rt_monitor.max_period_factor", max_period_factor_);
  node.declare_parameter("safety.rt_monitor.max_late_cycles", 10);
}

bool
SafetySupervisor::check_system(rclcpp_lifecycle::LifecycleNode & node)
{
  logger_ = node.get_logger();
  namespace_ = node.get_namespace();
  if (!configuration_pub_) {
    configuration_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
      node.get_node_topics_interface(), "easynav_configuration",
      rclcpp::QoS(1).reliable().transient_local());
  }
  safety_mode_ = node.get_parameter("safety.mode").as_bool();
  lock_memory_ = node.get_parameter("safety.lock_memory").as_bool();
  max_linear_vel_ = node.get_parameter("safety.plc_limits.max_linear_vel").as_double();
  max_angular_vel_ = node.get_parameter("safety.plc_limits.max_angular_vel").as_double();
  heartbeat_period_ = node.get_parameter("safety.heartbeat.period").as_double();
  max_period_factor_ = node.get_parameter("safety.rt_monitor.max_period_factor").as_double();
  const auto max_late_cycles = node.get_parameter("safety.rt_monitor.max_late_cycles").as_int();
  hardware_id_ = node.get_name();
  clock_ = node.get_clock();

  std::vector<std::string> errors;
  for (const auto & [name, value] : {
      std::pair<std::string, double>{"safety.plc_limits.max_linear_vel", max_linear_vel_},
      std::pair<std::string, double>{"safety.plc_limits.max_angular_vel", max_angular_vel_}})
  {
    if (!std::isfinite(value) || value < 0.0 || (safety_mode_ && value == 0.0)) {
      errors.push_back(
        name + " = " + std::to_string(value) +
        (safety_mode_ ? " (> 0, required in safety.mode)" : " (>= 0)"));
    }
  }
  if (!std::isfinite(heartbeat_period_) || heartbeat_period_ < 0.0 ||
    (safety_mode_ && heartbeat_period_ == 0.0))
  {
    errors.push_back(
      "safety.heartbeat.period = " + std::to_string(heartbeat_period_) +
      (safety_mode_ ? " (> 0, required in safety.mode)" : " (>= 0)"));
  }
  if (!std::isfinite(max_period_factor_) || max_period_factor_ <= 1.0) {
    errors.push_back(
      "safety.rt_monitor.max_period_factor = " + std::to_string(max_period_factor_) + " (> 1)");
  }
  if (max_late_cycles < 1) {
    errors.push_back(
      "safety.rt_monitor.max_late_cycles = " + std::to_string(max_late_cycles) + " (>= 1)");
  }
  const double rt_freq = node.get_parameter("rt_freq").as_double();
  if (std::isfinite(rt_freq) && rt_freq > 0.0) {  // Otherwise, SystemNode's own checks fail.
    rt_monitor_.configure(1.0 / rt_freq, max_period_factor_, static_cast<int>(max_late_cycles));
  }

  // Not a lifecycle publisher: it publishes from the RT cycle, while active. Its QoS promises
  // the period, so it is created again on every configure.
  heartbeat_pub_.reset();
  if (std::isfinite(heartbeat_period_) && heartbeat_period_ > 0.0) {
    const auto period = rclcpp::Duration::from_seconds(2.0 * heartbeat_period_);
    heartbeat_pub_ = rclcpp::create_publisher<easynav_interfaces::msg::Heartbeat>(
      node.get_node_topics_interface(), "easynav_heartbeat",
      rclcpp::QoS(1).reliable().deadline(period).liveliness(rclcpp::LivelinessPolicy::Automatic)
      .liveliness_lease_duration(period));
  }
  heartbeat_.safety_mode = safety_mode_;

  // Checked now, before anything is activated, whoever drives SystemNode.
  if (safety_mode_) {
    if (!node.get_parameter("use_real_time").as_bool()) {
      errors.push_back("use_real_time = false (required in safety.mode)");
    } else if (const auto error = check_real_time_priority(kRealTimePriority); !error.empty()) {
      errors.push_back("safety.mode requires real-time scheduling: " + error);
    }
  }
  if (lock_memory_) {
    if (const auto error = check_memory_lock(); !error.empty()) {
      errors.push_back("safety.lock_memory: " + error);
    }
  }

  // EasyNav is (re)configuring: its plugins may declare parameters until on_configured().
  freezer_.accept_new_parameters(true);

  for (const auto & error : errors) {
    RCLCPP_ERROR(logger_, "Invalid parameter: %s", error.c_str());
  }
  return errors.empty();
}

bool
SafetySupervisor::check_controller(ControllerNode & controller) const
{
  std::vector<std::string> errors;

  if (safety_mode_) {
    for (const std::string name : {"cmd_timeout", "cmd_vel_keepalive_period"}) {
      if (controller.get_parameter(name).as_double() <= 0.0) {
        errors.push_back("controller_node." + name + " must be > 0 in safety.mode");
      }
    }
  }

  // The limits enforced, deprecated per-controller ones included.
  const auto limits = controller.get_robot_limits();
  auto check = [&](const std::string & name, double value, double limit, const char * safety) {
      if (limit > 0.0 && std::abs(value) > limit) {
        errors.push_back(
          "controller_node.robot_limits." + name + " = " + std::to_string(value) +
          " exceeds system_node." + safety + " = " + std::to_string(limit));
      }
    };
  check("max_linear_vel", limits.max_linear_vel, max_linear_vel_,
      "safety.plc_limits.max_linear_vel");
  check("min_linear_vel", limits.min_linear_vel, max_linear_vel_,
      "safety.plc_limits.max_linear_vel");
  check(
    "max_angular_vel", limits.max_angular_vel, max_angular_vel_,
      "safety.plc_limits.max_angular_vel");

  for (const auto & error : errors) {
    RCLCPP_ERROR(logger_, "Invalid parameter: %s", error.c_str());
  }
  return errors.empty();
}

void
SafetySupervisor::on_configured(const Nodes & nodes, NavState & nav_state)
{
  const auto dump = configuration_dump(nodes);
  const auto hash = sha256_hex(dump);
  {
    std::lock_guard<std::mutex> lock(configuration_hash_mutex_);
    configuration_hash_ = hash;
  }
  nav_state.set("configuration_hash", hash);
  heartbeat_.configuration_hash = hash;

  // Saved and published, to see what differs when two fingerprints do.
  const auto path =
    (std::filesystem::path(log_directory()) / dump_file_name(namespace_, hash)).string();
  const auto error = save_dump(dump, path);
  if (configuration_pub_) {
    std_msgs::msg::String msg;
    msg.data = "# SHA-256: " + hash + "\n" + dump;
    configuration_pub_->publish(msg);
  }

  RCLCPP_INFO(
    logger_, "%sConfiguration SHA-256: %s (%s). Plugins:%s",
    safety_mode_ ? "[safety.mode] " : "", hash.c_str(), path.c_str(),
    loaded_plugins(nodes).c_str());
  if (!error.empty()) {
    RCLCPP_WARN(logger_, "Unable to save the configuration in %s: %s", path.c_str(), error.c_str());
  }

  if (safety_mode_) {
    freezer_.freeze(nodes);
    freezer_.accept_new_parameters(false);
    RCLCPP_INFO(logger_, "[safety.mode] Configuration frozen");
  }
}

bool
SafetySupervisor::allows_reconfiguration(const std::string & reason) const
{
  if (safety_mode_) {
    RCLCPP_ERROR(
      logger_, "[safety.mode] Reconfiguration rejected (%s): the configuration is frozen",
      reason.c_str());
    return false;
  }
  return true;
}

void
SafetySupervisor::on_activate()
{
  rt_monitor_.reset();
  last_heartbeat_.reset();
  failure_.clear();
}

bool
SafetySupervisor::cycle_rt(NavState & nav_state, RtMonitor::Clock::time_point now)
{
  const auto status = rt_monitor_.cycle_started(now);
  report_rt_status(nav_state, status);

  if (heartbeat_pub_ &&
    (!last_heartbeat_ ||
    std::chrono::duration<double>(now - *last_heartbeat_).count() >= heartbeat_period_))
  {
    last_heartbeat_ = now;
    heartbeat_.header.stamp = clock_->now();
    ++heartbeat_.sequence;
    heartbeat_.rt_status = static_cast<uint8_t>(status);
    heartbeat_.late_cycles = rt_monitor_.late_cycles();
    heartbeat_pub_->publish(heartbeat_);
  }

  if (safety_mode_ && status == RtMonitor::Status::ERROR && failure_.empty()) {
    failure_ = "[safety.mode] " + std::to_string(rt_monitor_.consecutive_late_cycles()) +
      " real-time cycles in a row started late";
  }
  return failure_.empty();  // Once it asks to stop, it keeps asking until activated again.
}

void
SafetySupervisor::report_rt_status(NavState & nav_state, RtMonitor::Status status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  // Nothing to report until something goes wrong; then, only changes.
  if (last_rt_report_ ? *last_rt_report_ == status : status == RtMonitor::Status::OK) {
    return;
  }
  last_rt_report_ = status;

  std::ostringstream message;
  diagnostic_msgs::msg::DiagnosticStatus diagnostic;
  switch (status) {
    case RtMonitor::Status::OK:
      diagnostic.level = DiagnosticStatus::OK;
      message << "Real-time cycles on time";
      RCLCPP_INFO(logger_, "%s", message.str().c_str());
      break;
    case RtMonitor::Status::LATE:
      diagnostic.level = DiagnosticStatus::WARN;
      message << "A real-time cycle started late: " << rt_monitor_.last_period() * 1e3 <<
        " ms after the previous one";
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s", message.str().c_str());
      break;
    case RtMonitor::Status::ERROR:
      diagnostic.level = DiagnosticStatus::ERROR;
      message << rt_monitor_.consecutive_late_cycles() <<
        " real-time cycles in a row started late (more than " << max_period_factor_ <<
        " periods after the previous one)";
      RCLCPP_ERROR(logger_, "%s", message.str().c_str());
      break;
  }
  diagnostic.name = "rt_cycle";
  diagnostic.hardware_id = hardware_id_;
  diagnostic.message = message.str();
  nav_state.set("diagnostics.rt_cycle", diagnostic);
  nav_state.add_to_group("diagnostics", "diagnostics.rt_cycle");
}

std::string
SafetySupervisor::get_configuration_hash() const
{
  std::lock_guard<std::mutex> lock(configuration_hash_mutex_);
  return configuration_hash_;
}

}  // namespace easynav::safety
