// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_controller/ControllerNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

ControllerNode::ControllerNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("controller_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  controller_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::ControllerMethodBase>>(
    "easynav_core", "easynav::ControllerMethodBase");

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

  controller_method_ = nullptr;
  std::vector<std::string> controller_types;
  get_parameter("controller_types", controller_types);
  for (const auto & controller_type : controller_types) {
    std::string plugin;
    if (has_parameter(controller_type + ".plugin")) {
      get_parameter(controller_type + ".plugin", plugin);
      try {
        controller_loader_->unloadLibraryForClass(plugin);
      } catch (const std::exception &) {
      }
    }
  }
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ControllerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> controller_types;
  declare_parameter("controller_types", controller_types);
  get_parameter("controller_types", controller_types);

  if (controller_types.size() > 1) {
    RCLCPP_ERROR(get_logger(),
      "You must instance one controller.  [%lu] found", controller_types.size());
    return CallbackReturnT::FAILURE;
  }

  for (const auto & controller_type : controller_types) {
    std::string plugin;
    declare_parameter(controller_type + std::string(".plugin"), plugin);
    get_parameter(controller_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading ControllerMethodBase %s [%s]", controller_type.c_str(), plugin.c_str());

      controller_method_ = controller_loader_->createSharedInstance(plugin);

      try {
        controller_method_->initialize(shared_from_this(), controller_type);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), e.what());
        return CallbackReturnT::FAILURE;
      }

      RCLCPP_INFO(get_logger(),
        "Loaded ControllerMethodBase %s [%s]", controller_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin easynav::ControllerMethodBase. Error: %s", ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
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
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
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
  if (controller_method_ == nullptr) {return false;}

  return controller_method_->internal_update_rt(*nav_state, trigger);
}

}  // namespace easynav
