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
/// \brief Implementation of the PlannerNode class.

#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_planner/PlannerNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

PlannerNode::PlannerNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("planner_node", options)
{
  planner_loader_ = std::make_unique<pluginlib::ClassLoader<PlannerMethodBase>>(
    "easynav_core", "easynav::PlannerMethodBase");

}

PlannerNode::~PlannerNode()
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

  planner_method_ = nullptr;
  std::vector<std::string> planner_types;
  get_parameter("planner_types", planner_types);
  for (const auto & planner_type : planner_types) {
    std::string plugin;
    if (has_parameter(planner_type + ".plugin")) {
      get_parameter(planner_type + ".plugin", plugin);
      try {
        planner_loader_->unloadLibraryForClass(plugin);
      } catch (const std::exception &) {
      }
    }
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PlannerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> planner_types;
  declare_parameter("planner_types", planner_types);
  get_parameter("planner_types", planner_types);

  if (planner_types.size() > 1) {
    RCLCPP_ERROR(get_logger(),
      "You must instance one planner.  [%lu] found", planner_types.size());
    return CallbackReturnT::FAILURE;
  }

  for (const auto & planner_type : planner_types) {
    std::string plugin;
    declare_parameter(planner_type + std::string(".plugin"), plugin);
    get_parameter(planner_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading PlannerMethodBase %s [%s]", planner_type.c_str(), plugin.c_str());

      planner_method_ = planner_loader_->createSharedInstance(plugin);

      try {
        planner_method_->initialize(shared_from_this(), planner_type);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), e.what());
        return CallbackReturnT::FAILURE;
      }

      RCLCPP_INFO(get_logger(),
        "Loaded PlannerMethodBase %s [%s]", planner_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin %s. Error: %s", plugin.c_str(), ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

void
PlannerNode::cycle(std::shared_ptr<NavState> nav_state, bool trigger)
{
  if (planner_method_ == nullptr) {return;}

  if (trigger) {
    planner_method_->force_update(*nav_state);
  } else {
    planner_method_->internal_update(*nav_state);
  }
}

const rclcpp::Time
PlannerNode::get_last_rt_execution_ts() const
{
  if (planner_method_ == nullptr) {return rclcpp::Time();}

  return planner_method_->get_last_rt_execution_ts();
}

const rclcpp::Time
PlannerNode::get_last_execution_ts() const
{
  if (planner_method_ == nullptr) {return rclcpp::Time();}

  return planner_method_->get_last_execution_ts();
}


}  // namespace easynav
