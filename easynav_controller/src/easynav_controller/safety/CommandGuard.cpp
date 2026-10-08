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
/// \brief Implementation of the CommandGuard class.

#include <cmath>
#include <sstream>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_controller/safety/CommandGuard.hpp"
#include "easynav_core/VelocityCommand.hpp"

namespace easynav::safety
{

namespace
{
// Static: no allocation in the RT cycle.
const std::string kDiagnostic {"diagnostics.cmd_vel"};
const std::string kDiscardedMessage {"Non-finite velocity command discarded"};
const std::string kReceivingMessage {"Receiving velocity commands"};

bool is_finite(const geometry_msgs::msg::Twist & t)
{
  return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
         std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z);
}
}  // namespace

void
CommandGuard::declare_parameters(rclcpp_lifecycle::LifecycleNode & node)
{
  declare_parameter_if_absent(node, "cmd_timeout", cmd_timeout_);
  declare_parameter_if_absent(node, "cmd_vel_keepalive_period", keepalive_period_);
}

bool
CommandGuard::configure(rclcpp_lifecycle::LifecycleNode & node)
{
  logger_ = node.get_logger();
  hardware_id_ = node.get_name();
  node.get_parameter("cmd_timeout", cmd_timeout_);
  node.get_parameter("cmd_vel_keepalive_period", keepalive_period_);
  reset();

  // Not just < 0: a NaN would silently disable them.
  if (!std::isfinite(cmd_timeout_) || cmd_timeout_ < 0.0 ||
    !std::isfinite(keepalive_period_) || keepalive_period_ < 0.0)
  {
    RCLCPP_ERROR(
      logger_, "cmd_timeout (%f) and cmd_vel_keepalive_period (%f) must be finite and >= 0",
      cmd_timeout_, keepalive_period_);
    return false;
  }

  std::ostringstream message;
  message << "No new velocity command in " << cmd_timeout_ << " s: stopping";
  timeout_message_ = message.str();
  return true;
}

bool
CommandGuard::check_controller_period(double period, const std::string & controller) const
{
  if (cmd_timeout_ > 0.0 && cmd_timeout_ <= period) {
    RCLCPP_ERROR(
      logger_, "cmd_timeout (%f s) must be longer than the period of controller [%s] (%f s), or "
      "every command would time out", cmd_timeout_, controller.c_str(), period);
    return false;
  }
  return true;
}

rclcpp::QoS
CommandGuard::qos() const
{
  // Only the latest command matters.
  rclcpp::QoS qos(1);
  if (keepalive_period_ > 0.0) {
    const auto period = rclcpp::Duration::from_seconds(2.0 * keepalive_period_);
    qos.deadline(period);
    qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
    qos.liveliness_lease_duration(period);
  }
  return qos;
}

bool
CommandGuard::is_new(const geometry_msgs::msg::TwistStamped & cmd)
{
  if (last_controller_cmd_ && cmd == *last_controller_cmd_) {
    return false;
  }
  last_controller_cmd_ = cmd;
  return true;
}

bool
CommandGuard::discard_non_finite(NavState & nav_state)
{
  bool discarded = false;
  for (const auto source : {VelocitySource::OVERRIDE, VelocitySource::TAKEOVER,
      VelocitySource::CONTROLLER})
  {
    const auto cmd = velocity_command::peek(nav_state, source);
    if (cmd && !is_finite(cmd->twist)) {
      velocity_command::take(nav_state, source);
      discarded = true;
    }
  }
  return discarded;
}

VelocityMux::Selection
CommandGuard::supervise(const VelocityMux::Selection & selection, const rclcpp::Time & now)
{
  if (selection.fresh) {
    last_proposal_ = now;
    timed_out_ = false;
    return selection;
  }
  if (last_proposal_ && now < *last_proposal_) {
    last_proposal_ = now;  // The clock jumped back: count the timeout from now.
  }

  auto stop = selection;
  stop.cmd.twist = geometry_msgs::msg::Twist();
  if (timed_out_) {
    return stop;
  }
  if (cmd_timeout_ > 0.0 && last_proposal_ && selection.cmd.twist != stop.cmd.twist &&
    (now - *last_proposal_).seconds() > cmd_timeout_)
  {
    // Nobody keeps commanding the robot: stop it instead of holding a stale command.
    timed_out_ = true;
    stop.fresh = true;  // A new (zero) target.
    return stop;
  }
  return selection;
}

void
CommandGuard::report(NavState & nav_state, bool discarded, bool fresh)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (timed_out_) {
    write_report(nav_state, DiagnosticStatus::ERROR, timeout_message_);
  } else if (discarded) {
    write_report(nav_state, DiagnosticStatus::ERROR, kDiscardedMessage);
  } else if (fresh) {
    write_report(nav_state, DiagnosticStatus::OK, kReceivingMessage);
  }
}

bool
CommandGuard::keepalive_due(const rclcpp::Time & now) const
{
  if (keepalive_period_ <= 0.0) {
    return false;
  }
  if (!last_publish_) {
    return true;
  }
  // Also due if the clock jumped back (e.g. a simulation restarted).
  const double since_publish = (now - *last_publish_).seconds();
  return since_publish < 0.0 || since_publish >= keepalive_period_;
}

void
CommandGuard::reset()
{
  last_controller_cmd_.reset();
  last_proposal_.reset();
  last_publish_.reset();
  timed_out_ = false;
}

void
CommandGuard::write_report(NavState & nav_state, uint8_t level, const std::string & message)
{
  // Nothing to report until something goes wrong; then, only changes.
  if (!last_report_ && level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    return;
  }
  if (last_report_ && last_report_->first == level && last_report_->second == message) {
    return;
  }
  last_report_ = {level, message};

  if (level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    RCLCPP_INFO(logger_, "%s", message.c_str());
  } else {
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "cmd_vel";
  status.hardware_id = hardware_id_;
  status.level = level;
  status.message = message;
  nav_state.set(kDiagnostic, status);
  nav_state.add_to_group("diagnostics", kDiagnostic);
}

}  // namespace easynav::safety
