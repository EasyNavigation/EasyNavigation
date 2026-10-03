// Copyright 2025 Intelligent Robotics Lab
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
/// \brief Implementation of the ControllerNode class.

#include <algorithm>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_controller/ControllerNode.hpp"
#include "easynav_core/VelocityCommand.hpp"

namespace easynav
{

using namespace std::chrono_literals;

namespace
{
/// @brief Longest time a smoother step may cover (s): after a pause of the RT loop, the ramp
/// resumes from where it was instead of jumping.
constexpr double kMaxSmootherStep = 0.1;
/// @brief Period of the braking ramp on deactivation/shutdown (s).
constexpr double kStopPeriod = 0.02;
/// @brief Extra time allowed for the braking ramp over the theoretical one (s).
constexpr double kStopMargin = 0.2;

/// @brief "robot_limits.*" fields and their RobotLimits member.
std::vector<std::pair<std::string, double RobotLimits::*>> limit_fields()
{
  return {
    {"max_linear_vel", &RobotLimits::max_linear_vel},
    {"min_linear_vel", &RobotLimits::min_linear_vel},
    {"max_angular_vel", &RobotLimits::max_angular_vel},
    {"max_linear_acc", &RobotLimits::max_linear_acc},
    {"max_linear_decel", &RobotLimits::max_linear_decel},
    {"max_angular_acc", &RobotLimits::max_angular_acc},
    {"max_angular_decel", &RobotLimits::max_angular_decel},
  };
}
}  // namespace

ControllerNode::ControllerNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("controller_node", options),
  controller_(*this, "easynav_core", "easynav::ControllerMethodBase", "controller_types")
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  NavState::register_printer<geometry_msgs::msg::TwistStamped>(
    [](const geometry_msgs::msg::TwistStamped & twist) {
      std::ostringstream ret;

      ret << "{ " << rclcpp::Time(twist.header.stamp).seconds() << "} Twist with (" <<
        twist.twist.linear.x << ", " <<
        twist.twist.linear.y << ", " <<
        twist.twist.linear.z << ") (" << twist.twist.angular.x << ", " <<
        twist.twist.angular.y << ", " << twist.twist.angular.z << ")";

      return ret.str();
    });

  // "cmd_vel.proposal.<source>" slots.
  NavState::register_printer<VelocityProposal>(
    [](const VelocityProposal & proposal) {
      const auto & twist = proposal.cmd;
      std::ostringstream ret;

      ret << "{ " << rclcpp::Time(twist.header.stamp).seconds() << "} " <<
        (proposal.pending ? "pending" : "taken") << " Twist with (" <<
        twist.twist.linear.x << ", " <<
        twist.twist.linear.y << ", " <<
        twist.twist.linear.z << ") (" << twist.twist.angular.x << ", " <<
        twist.twist.angular.y << ", " << twist.twist.angular.z << ")";

      return ret.str();
    });

  // Declared before any plugin is loaded, so plugins can query them while initializing.
  const RobotLimits defaults;
  for (const auto & [field, member] : limit_fields()) {
    declare_parameter_if_absent(*this, "robot_limits." + field, defaults.*member);
  }
  declare_parameter_if_absent(*this, "use_cmd_vel_stamped", use_cmd_vel_stamped_);
  command_guard_.declare_parameters(*this);
  read_parameters();
}

ControllerNode::~ControllerNode()
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }

  controller_.release();
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ControllerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  // Limits first: plugins query them while initializing. Configured ones, until the RT cycle
  // applies the safety channel's state.
  apply_safety_channel(SafetyChannelState());
  read_parameters();

  if (!command_guard_.configure(*this)) {
    return CallbackReturnT::FAILURE;
  }

  if (use_cmd_vel_stamped_) {
    vel_pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_stamped", command_guard_.qos());
  } else {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", command_guard_.qos());
  }

  // The robot was left stopped (see stop_robot()).
  smoother_.reset();
  mux_.reset();
  last_smoother_step_.reset();

  if (!controller_.configure()) {
    return CallbackReturnT::FAILURE;
  }
  if (!check_cmd_timeout()) {
    controller_.release();
    return CallbackReturnT::FAILURE;
  }
  // After the plugin: it may apply deprecated limits of its own.
  const auto invalid = invalid_robot_limits(get_robot_limits());
  if (!invalid.empty()) {
    RCLCPP_ERROR(get_logger(), "Invalid robot limits: %s", invalid.c_str());
    controller_.release();
    return CallbackReturnT::FAILURE;
  }
  return CallbackReturnT::SUCCESS;
}

bool
ControllerNode::check_cmd_timeout()
{
  const auto alias = get_loaded_controller();
  if (alias.empty() || !has_parameter(alias + ".rt_freq")) {
    return true;
  }
  return command_guard_.check_controller_period(
    1.0 / get_parameter(alias + ".rt_freq").as_double(), alias);
}

CallbackReturnT
ControllerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  // No RT cycle runs outside Active, and drivers usually keep executing the last command.
  stop_robot();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  controller_.release();
  vel_pub_ = nullptr;
  vel_pub_stamped_ = nullptr;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  // Also when shut down straight from Active (e.g. Ctrl+C), without deactivating.
  stop_robot();
  controller_.release();
  vel_pub_ = nullptr;
  vel_pub_stamped_ = nullptr;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  controller_.release();
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
ControllerNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

bool
ControllerNode::cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger)
{
  // get() returns a copy, so the plugin stays alive for this call even if
  // on_cleanup() releases it concurrently.
  auto controller_method = controller_.get();
  if (controller_method == nullptr) {return false;}

  const bool ran = controller_method->internal_update_rt(*nav_state, trigger);

  // Controller plugins write their command to "cmd_vel": propose it if it is a new one.
  if (ran && nav_state->has("cmd_vel")) {
    const auto cmd = nav_state->get_safe<geometry_msgs::msg::TwistStamped>("cmd_vel");
    if (command_guard_.is_new(cmd)) {
      velocity_command::propose(*nav_state, VelocitySource::CONTROLLER, cmd);
    }
  }
  return ran;
}

std::string
ControllerNode::get_loaded_controller() const
{
  const auto types = controller_.loaded_types();
  return types.empty() ? "" : types.front();
}

void
ControllerNode::publish_cmd_vel_rt(std::shared_ptr<NavState> nav_state)
{
  const auto now = this->now();
  if (nav_state->has(kSafetyStatusKey)) {
    const auto safety_channel = nav_state->get_safe<SafetyChannelState>(kSafetyStatusKey);
    if (safety_channel != safety_channel_) {
      apply_safety_channel(safety_channel);
    }
  }
  const bool discarded = command_guard_.discard_non_finite(*nav_state);
  const auto selection = command_guard_.supervise(mux_.select(*nav_state), now);
  command_guard_.report(*nav_state, discarded, selection.fresh);

  const double dt = last_smoother_step_ ?
    std::clamp((now - *last_smoother_step_).seconds(), 0.0, kMaxSmootherStep) : 0.0;
  last_smoother_step_ = now;

  auto cmd = selection.cmd;
  if (selection.choice == VelocityMux::Choice::SAFETY_STOP) {
    smoother_.reset();  // The safety channel stops the robot: no ramp from what was commanded.
  }
  if (selection.smooth) {
    const bool ramping = !smoother_.reached(selection.cmd.twist);
    if (!selection.fresh && !ramping && !command_guard_.keepalive_due(now)) {
      return;  // Nothing new, and the robot is already at the last target.
    }
    cmd.twist = smoother_.step(selection.cmd.twist, dt);
  } else {
    // An emergency override: published as is; the smoother continues from it.
    smoother_.reset(cmd.twist);
  }

  cmd.header.stamp = now;
  publish(cmd);
}

RobotLimits
ControllerNode::get_robot_limits() const
{
  std::lock_guard<std::mutex> lock(robot_limits_mutex_);
  return limited_by(robot_limits_, safety_channel_);
}

bool
ControllerNode::is_robot_limit_configured(const std::string & field) const
{
  std::lock_guard<std::mutex> lock(robot_limits_mutex_);
  return configured_limits_.count(field) > 0;
}

void
ControllerNode::set_robot_limits(const RobotLimits & limits)
{
  {
    std::lock_guard<std::mutex> lock(robot_limits_mutex_);
    robot_limits_ = limits;
  }
  smoother_.set_limits(get_robot_limits());
}

void
ControllerNode::apply_safety_channel(const SafetyChannelState & state)
{
  {
    std::lock_guard<std::mutex> lock(robot_limits_mutex_);
    safety_channel_ = state;
  }
  // A lower limit is reached with the deceleration limits.
  smoother_.set_limits(get_robot_limits());
}

void
ControllerNode::read_parameters()
{
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  const RobotLimits defaults;
  RobotLimits limits;
  std::set<std::string> configured;
  for (const auto & [field, member] : limit_fields()) {
    const auto name = "robot_limits." + field;
    get_parameter(name, limits.*member);
    // Explicitly configured: in the parameter files/overrides, or changed at runtime.
    if (overrides.count(name) > 0 || limits.*member != defaults.*member) {
      configured.insert(field);
    }
  }
  get_parameter("use_cmd_vel_stamped", use_cmd_vel_stamped_);

  {
    std::lock_guard<std::mutex> lock(robot_limits_mutex_);
    configured_limits_ = std::move(configured);
  }
  set_robot_limits(limits);
}

void
ControllerNode::publish(const geometry_msgs::msg::TwistStamped & cmd)
{
  if (use_cmd_vel_stamped_ && vel_pub_stamped_) {
    vel_pub_stamped_->publish(cmd);
  }
  if (!use_cmd_vel_stamped_ && vel_pub_) {
    vel_pub_->publish(cmd.twist);
  }
  command_guard_.published(rclcpp::Time(cmd.header.stamp, get_clock()->get_clock_type()));
}

void
ControllerNode::stop_robot()
{
  if (!vel_pub_ && !vel_pub_stamped_) {
    return;  // Never configured: nothing was commanded.
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;

  // Brake within the deceleration limits, so the robot does not stop dead...
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(smoother_.time_to_stop() + kStopMargin);
  while (!smoother_.reached(geometry_msgs::msg::Twist()) &&
    std::chrono::steady_clock::now() < deadline)
  {
    cmd.header.stamp = this->now();
    cmd.twist = smoother_.step(geometry_msgs::msg::Twist(), kStopPeriod);
    publish(cmd);
    std::this_thread::sleep_for(std::chrono::duration<double>(kStopPeriod));
  }

  // ...and whatever happened, the last command sent is an exact zero.
  cmd.header.stamp = this->now();
  cmd.twist = geometry_msgs::msg::Twist();
  publish(cmd);
  smoother_.reset();
  mux_.reset();
  command_guard_.reset();
}

}  // namespace easynav
