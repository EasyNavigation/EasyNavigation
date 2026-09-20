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
: LifecycleNode("planner_node", options),
  planner_(*this, "easynav_core", "easynav::PlannerMethodBase", "planner_types")
{
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

  planner_.release();
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PlannerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return planner_.configure() ? CallbackReturnT::SUCCESS : CallbackReturnT::FAILURE;
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
  planner_.release();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  planner_.release();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  planner_.release();
  return CallbackReturnT::SUCCESS;
}

void
PlannerNode::cycle(std::shared_ptr<NavState> nav_state, bool trigger)
{
  auto planner_method = planner_.get();
  if (planner_method == nullptr) {return;}

  if (trigger) {
    planner_method->force_update(*nav_state);
  } else {
    planner_method->internal_update(*nav_state);
  }
}

const rclcpp::Time
PlannerNode::get_last_rt_execution_ts() const
{
  auto planner_method = planner_.get();
  if (planner_method == nullptr) {return rclcpp::Time();}

  return planner_method->get_last_rt_execution_ts();
}

const rclcpp::Time
PlannerNode::get_last_execution_ts() const
{
  auto planner_method = planner_.get();
  if (planner_method == nullptr) {return rclcpp::Time();}

  return planner_method->get_last_execution_ts();
}

}  // namespace easynav
