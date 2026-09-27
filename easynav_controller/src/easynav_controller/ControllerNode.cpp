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
#include <thread>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

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
}  // namespace

ControllerNode::ControllerNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("controller_node", options),
  controller_(*this, "easynav_core", "easynav::ControllerMethodBase", "controller_types")
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  // Declared here, before any plugin is loaded, so they can query them while initializing.
  const RobotLimits defaults;
  declare_parameter("robot_limits.max_linear_vel", defaults.max_linear_vel);
  declare_parameter("robot_limits.min_linear_vel", defaults.min_linear_vel);
  declare_parameter("robot_limits.max_angular_vel", defaults.max_angular_vel);
  declare_parameter("robot_limits.max_linear_acc", defaults.max_linear_acc);
  declare_parameter("robot_limits.max_linear_decel", defaults.max_linear_decel);
  declare_parameter("robot_limits.max_angular_acc", defaults.max_angular_acc);
  declare_parameter("robot_limits.max_angular_decel", defaults.max_angular_decel);
  declare_parameter("use_cmd_vel_stamped", use_cmd_vel_stamped_);
  read_parameters();

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
  // Limits first: the controller plugins query them while initializing.
  read_parameters();

  if (use_cmd_vel_stamped_) {
    vel_pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 100);
  } else {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
  }

  return controller_.configure() ? CallbackReturnT::SUCCESS : CallbackReturnT::FAILURE;
}

CallbackReturnT
ControllerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  // The robot was left stopped (see stop_robot()).
  mux_.reset();
  smoother_.reset();
  last_smoother_step_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  // No RT cycle runs outside Active, so nothing will command the robot until it is activated
  // again, and many drivers keep executing the last velocity they received: stop it.
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
  // Also when EasyNav is shut down straight from Active (e.g. Ctrl+C), without deactivating.
  stop_robot();
  controller_.release();
  vel_pub_ = nullptr;
  vel_pub_stamped_ = nullptr;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  stop_robot();
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

  // Controller plugins write their command to "cmd_vel": propose it for this cycle.
  if (ran && nav_state->has("cmd_vel")) {
    velocity_command::propose(
      *nav_state, VelocitySource::CONTROLLER,
      nav_state->get<geometry_msgs::msg::TwistStamped>("cmd_vel"));
  }
  return ran;
}

void
ControllerNode::publish_cmd_vel_rt(std::shared_ptr<NavState> nav_state)
{
  const auto selection = mux_.select(*nav_state);

  const auto now = this->now();
  const double dt = last_smoother_step_ ?
    std::clamp((now - *last_smoother_step_).seconds(), 0.0, kMaxSmootherStep) : 0.0;
  last_smoother_step_ = now;

  auto cmd = selection.cmd;
  if (selection.smooth) {
    const bool ramping = !smoother_.reached(selection.cmd.twist);
    cmd.twist = smoother_.step(selection.cmd.twist, dt);
    if (!selection.fresh && !ramping) {
      return;  // Nothing new, and the robot is already at the last target.
    }
  } else {
    // A reflex override: published as is; the smoother continues from it.
    smoother_.reset(cmd.twist);
  }

  cmd.header.stamp = now;
  publish(cmd);
}

RobotLimits
ControllerNode::get_robot_limits() const
{
  std::lock_guard<std::mutex> lock(robot_limits_mutex_);
  return robot_limits_;
}

void
ControllerNode::read_parameters()
{
  RobotLimits limits;
  get_parameter("robot_limits.max_linear_vel", limits.max_linear_vel);
  get_parameter("robot_limits.min_linear_vel", limits.min_linear_vel);
  get_parameter("robot_limits.max_angular_vel", limits.max_angular_vel);
  get_parameter("robot_limits.max_linear_acc", limits.max_linear_acc);
  get_parameter("robot_limits.max_linear_decel", limits.max_linear_decel);
  get_parameter("robot_limits.max_angular_acc", limits.max_angular_acc);
  get_parameter("robot_limits.max_angular_decel", limits.max_angular_decel);
  get_parameter("use_cmd_vel_stamped", use_cmd_vel_stamped_);

  {
    std::lock_guard<std::mutex> lock(robot_limits_mutex_);
    robot_limits_ = limits;
  }
  smoother_.set_limits(limits);
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
}

std::string
ControllerNode::get_loaded_controller() const
{
  const auto types = controller_.loaded_types();
  return types.empty() ? "" : types.front();
}

}  // namespace easynav
