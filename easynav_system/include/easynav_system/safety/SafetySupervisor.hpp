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
/// \brief Declaration of the SafetySupervisor class.

#ifndef EASYNAV_SYSTEM__SAFETY__SAFETYSUPERVISOR_HPP_
#define EASYNAV_SYSTEM__SAFETY__SAFETYSUPERVISOR_HPP_

#include <atomic>
#include <mutex>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_controller/ControllerNode.hpp"
#include "easynav_system/safety/ConfigurationFingerprint.hpp"
#include "easynav_system/safety/ParameterFreezer.hpp"

namespace easynav::safety
{

/**
 * @class SafetySupervisor
 * @brief What SystemNode does for safety, kept out of the navigation logic.
 *
 * Its parameters, in SystemNode:
 * - "safety.plc_limits.*": the limits configured in the safety channel (e.g. the PLC's safely
 *   limited speed). Not applied to the commands (controller_node's "robot_limits" are, in any
 *   mode): "robot_limits" may not exceed them.
 * - "safety.mode" (default false) makes EasyNav stricter: "safety.plc_limits.*", the command
 *   keepalive and timeout, and real-time scheduling (checked on configure) are required; the
 *   configuration is frozen once configured, and reconfiguration requests are rejected.
 * - "safety.lock_memory" (default false, any mode): system_main locks the process memory; if
 *   RLIMIT_MEMLOCK does not allow it, configuring fails.
 *
 * Every configure, it fingerprints the configuration: a SHA-256 of every parameter, logged and
 * shared in NavState ("configuration_hash"), with the parameters saved in the ROS log directory
 * ("easynav_configuration_[<ns>_]<hash>.txt") and published, latched, on "easynav_configuration".
 */
class SafetySupervisor
{
public:
  /// @brief Declares the "safety.*" parameters in \p node (SystemNode).
  void declare_parameters(rclcpp_lifecycle::LifecycleNode & node);

  /// @brief Reads \p node's safety parameters and checks them, before the subnodes configure.
  bool check_system(rclcpp_lifecycle::LifecycleNode & node);

  /// @brief Checks \p controller against "safety.plc_limits" and what "safety.mode" requires.
  bool check_controller(ControllerNode & controller) const;

  /// @brief After a successful configure: fingerprints and, in safety mode, freezes \p nodes.
  void on_configured(const Nodes & nodes, NavState & nav_state);

  /// @brief Whether a reconfiguration may be requested; logs why not.
  bool allows_reconfiguration(const std::string & reason) const;

  /// @brief "safety.mode", as of the last configure.
  [[nodiscard]] bool is_safety_mode() const {return safety_mode_;}

  /// @brief "safety.lock_memory", as of the last configure.
  [[nodiscard]] bool is_memory_lock_requested() const {return lock_memory_;}

  /// @brief SHA-256 (hex) of every EasyNav parameter, as of the last configure.
  [[nodiscard]] std::string get_configuration_hash() const;

private:
  std::atomic<bool> safety_mode_ {false};
  std::atomic<bool> lock_memory_ {false};
  double max_linear_vel_ {0.0};
  double max_angular_vel_ {0.0};
  rclcpp::Logger logger_ {rclcpp::get_logger("system_node")};
  std::string namespace_;
  /// @brief Not a lifecycle publisher: it publishes on configure, while still inactive.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configuration_pub_;

  ParameterFreezer freezer_;
  std::string configuration_hash_;
  mutable std::mutex configuration_hash_mutex_;
};

}  // namespace easynav::safety

#endif  // EASYNAV_SYSTEM__SAFETY__SAFETYSUPERVISOR_HPP_
