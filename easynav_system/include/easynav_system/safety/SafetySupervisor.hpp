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
#include <optional>
#include <string>

#include "easynav_interfaces/msg/heartbeat.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_controller/ControllerNode.hpp"
#include "easynav_system/safety/ConfigurationFingerprint.hpp"
#include "easynav_system/safety/ParameterFreezer.hpp"
#include "easynav_system/safety/RtMonitor.hpp"
#include "easynav_system/safety/SafetyChannelMonitor.hpp"

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
 * - "safety.heartbeat.period" (s, default 0: off; required in safety mode): a Heartbeat is
 *   published on "easynav_heartbeat" from the RT cycle, so it stops if that cycle does.
 * - "safety.rt_monitor.*": an RT cycle starting more than "max_period_factor" periods after the
 *   previous one is late; "max_late_cycles" in a row are reported ("diagnostics.rt_cycle"): a WARN,
 *   or in safety mode an ERROR that stops EasyNav (the RT cycle receives the sensors and publishes
 *   the commands). Whether each component keeps its own frequency is reported separately, as a
 *   WARN ("diagnostics.<plugin>.rt_rate" / ".rate", see MethodBase).
 * - "safety.status.timeout" (s, default 0: off; required in safety mode): the safety channel's
 *   state (SafetyStatus) is received on "easynav_safety_status" and applied every RT cycle through
 *   NavState ("safety_status"): during a protective stop EasyNav commands zero and keeps the
 *   mission; a safely limited speed cuts the robot limits down. With no valid status within the
 *   timeout, it is a protective stop.
 * - "safety.max_pose_age" (s, default 1.0; 0: off): a "robot_pose" older than this (its stamp,
 *   ROS time), or not finite, is an ERROR ("diagnostics.robot_pose"); in safety mode, motion is
 *   also inhibited ("inhibit_motion": the robot brakes to zero) until it is usable again.
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
  /// The safety status is received in \p rt_group (SystemNode's RT callback group), if given.
  bool check_system(
    rclcpp_lifecycle::LifecycleNode & node, rclcpp::CallbackGroup::SharedPtr rt_group = nullptr);

  /// @brief Checks \p controller against "safety.plc_limits" and what "safety.mode" requires.
  bool check_controller(ControllerNode & controller) const;

  /// @brief After a successful configure: fingerprints and, in safety mode, freezes \p nodes.
  void on_configured(const Nodes & nodes, NavState & nav_state);

  /// @brief Whether a reconfiguration may be requested; logs why not.
  bool allows_reconfiguration(const std::string & reason) const;

  /// @brief On activation: the RT monitor starts over.
  void on_activate();

  /// @brief At the start of each RT cycle, at \p now: monitors it, publishes the heartbeat when
  /// due, and writes the safety channel's state to NavState. @return false if EasyNav must stop (see failure()), until on_activate().
  bool cycle_rt(NavState & nav_state, RtMonitor::Clock::time_point now);

  /// @brief Why cycle_rt() asked to stop.
  [[nodiscard]] const std::string & failure() const {return failure_;}

  [[nodiscard]] const RtMonitor & get_rt_monitor() const {return rt_monitor_;}

  /// @brief Whether the safety status is received ("safety.status.timeout" > 0).
  [[nodiscard]] bool is_safety_status_enabled() const {return safety_status_sub_ != nullptr;}

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

  /// @brief Updates "diagnostics.rt_cycle" when the RT monitor's status changes.
  void report_rt_status(NavState & nav_state, RtMonitor::Status status);

  /// @brief Checks the age of "robot_pose" at \p now; reports and, in safety mode, inhibits motion.
  void check_pose_age(NavState & nav_state, const rclcpp::Time & now);

  /// @brief Updates "diagnostics.safety_status" when the safety channel's state changes.
  void report_safety_status(
    NavState & nav_state, const SafetyChannelMonitor::Evaluation & evaluation);

  ParameterFreezer freezer_;

  // Real-time cycle: monitor and heartbeat, used only from the RT cycle once active.
  RtMonitor rt_monitor_;
  double max_period_factor_ {2.0};
  std::optional<RtMonitor::Status> last_rt_report_;
  std::string hardware_id_ {"system_node"};
  std::string failure_;
  rclcpp::Clock::SharedPtr clock_;
  double heartbeat_period_ {0.0};
  std::optional<RtMonitor::Clock::time_point> last_heartbeat_;
  easynav_interfaces::msg::Heartbeat heartbeat_;
  rclcpp::Publisher<easynav_interfaces::msg::Heartbeat>::SharedPtr heartbeat_pub_;
  std::string configuration_hash_;

  // Age of the robot pose, checked from the RT cycle.
  double max_pose_age_ {1.0};
  std::optional<bool> last_pose_stale_;

  // Safety channel: received in the RT callback group, applied from the RT cycle.
  double safety_status_timeout_ {0.0};
  SafetyChannelMonitor safety_channel_;
  rclcpp::Subscription<easynav_interfaces::msg::SafetyStatus>::SharedPtr safety_status_sub_;
  std::optional<SafetyChannelMonitor::Evaluation> last_safety_report_;
  mutable std::mutex configuration_hash_mutex_;
};

}  // namespace easynav::safety

#endif  // EASYNAV_SYSTEM__SAFETY__SAFETYSUPERVISOR_HPP_
