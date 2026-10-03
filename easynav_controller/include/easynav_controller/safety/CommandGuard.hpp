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
/// \brief Declaration of the CommandGuard class.

#ifndef EASYNAV_CONTROLLER__SAFETY__COMMANDGUARD_HPP_
#define EASYNAV_CONTROLLER__SAFETY__COMMANDGUARD_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_controller/VelocityMux.hpp"

namespace easynav::safety
{

/**
 * @class CommandGuard
 * @brief Keeps the velocity command sent to the robot from being stale or invalid.
 *
 * Around ControllerNode's VelocityMux, every RT cycle:
 * - only new controller commands (stamp or value changed) are proposed (is_new());
 * - non-finite proposals are discarded before the mux selects (discard_non_finite());
 * - a non-zero target with no new proposal for "cmd_timeout" s becomes zero (supervise());
 * - the command is republished every "cmd_vel_keepalive_period" s (keepalive_due()), and the
 *   publisher offers deadline and liveliness QoS of twice that period (qos());
 * - problems are reported in NavState as "diagnostics.cmd_vel" (report()).
 */
class CommandGuard
{
public:
  /// @brief Declares "cmd_timeout" and "cmd_vel_keepalive_period" in \p node.
  void declare_parameters(rclcpp_lifecycle::LifecycleNode & node);

  /// @brief Reads and checks the parameters, and starts over. Logs and returns false if invalid.
  bool configure(rclcpp_lifecycle::LifecycleNode & node);

  /// @brief Checks "cmd_timeout" against the period (s) of \p controller. Logs if not longer.
  bool check_controller_period(double period, const std::string & controller) const;

  /// @brief QoS of the velocity publisher.
  [[nodiscard]] rclcpp::QoS qos() const;

  /// @brief Whether a controller command is new (stamp or value changed). Remembers it.
  bool is_new(const geometry_msgs::msg::TwistStamped & cmd);

  /// @brief Consumes this cycle's non-finite proposals. @return Whether any was discarded.
  bool discard_non_finite(NavState & nav_state);

  /// @brief \p selection, or zero if its non-zero target is held for longer than the timeout.
  VelocityMux::Selection supervise(
    const VelocityMux::Selection & selection, const rclcpp::Time & now);

  /// @brief Whether the target was zeroed because no proposal arrived in time.
  [[nodiscard]] bool timed_out() const {return timed_out_;}

  /// @brief Updates "diagnostics.cmd_vel" after this cycle's selection.
  void report(NavState & nav_state, bool discarded, bool fresh);

  /// @brief Whether the command must be republished at \p now (keepalive).
  [[nodiscard]] bool keepalive_due(const rclcpp::Time & now) const;

  /// @brief Records that a command stamped \p stamp was published.
  void published(const rclcpp::Time & stamp) {last_publish_ = stamp;}

  /// @brief Forgets the timeout, keepalive and last command (e.g. once the robot is stopped).
  /// The last report is kept: NavState outlives a reconfiguration, its ERROR must be cleared.
  void reset();

  [[nodiscard]] double cmd_timeout() const {return cmd_timeout_;}
  [[nodiscard]] double keepalive_period() const {return keepalive_period_;}

private:
  void write_report(NavState & nav_state, uint8_t level, const std::string & message);

  double cmd_timeout_ {0.5};
  double keepalive_period_ {0.0};
  rclcpp::Logger logger_ {rclcpp::get_logger("controller_node")};
  std::string hardware_id_ {"controller_node"};
  std::string timeout_message_;

  std::optional<geometry_msgs::msg::TwistStamped> last_controller_cmd_;
  std::optional<rclcpp::Time> last_proposal_;
  std::optional<rclcpp::Time> last_publish_;
  bool timed_out_ {false};
  std::optional<std::pair<uint8_t, std::string>> last_report_;
};

}  // namespace easynav::safety

#endif  // EASYNAV_CONTROLLER__SAFETY__COMMANDGUARD_HPP_
