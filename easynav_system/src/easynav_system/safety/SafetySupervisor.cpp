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
#include <string>
#include <vector>

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
  if (safety_mode_ && !node.get_parameter("use_real_time").as_bool()) {
    errors.push_back("use_real_time = false (required in safety.mode)");
  }

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

std::string
SafetySupervisor::get_configuration_hash() const
{
  std::lock_guard<std::mutex> lock(configuration_hash_mutex_);
  return configuration_hash_;
}

}  // namespace easynav::safety
