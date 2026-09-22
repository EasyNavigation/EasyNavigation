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

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "easynav_controller/ControllerNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

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
  return controller_.configure() ? CallbackReturnT::SUCCESS : CallbackReturnT::FAILURE;
}

CallbackReturnT
ControllerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  controller_.release();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  controller_.release();
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

  return controller_method->internal_update_rt(*nav_state, trigger);
}

std::string
ControllerNode::get_loaded_controller() const
{
  const auto types = controller_.loaded_types();
  return types.empty() ? "" : types.front();
}

}  // namespace easynav
