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
/// \brief Implementation of the MapsManagerNode class.

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_maps_manager/MapsManagerNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

MapsManagerNode::MapsManagerNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("maps_manager_node", options),
  maps_managers_(*this, "easynav_core", "easynav::MapsManagerBase", "map_types", 0)
{
}

MapsManagerNode::~MapsManagerNode()
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

  maps_managers_.release();
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
MapsManagerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return maps_managers_.configure() ? CallbackReturnT::SUCCESS : CallbackReturnT::FAILURE;
}

CallbackReturnT
MapsManagerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  maps_managers_.release();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  maps_managers_.release();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  maps_managers_.release();
  return CallbackReturnT::SUCCESS;
}

void
MapsManagerNode::cycle(std::shared_ptr<NavState> nav_state)
{
  // get_all() returns a copy, so the plugins stay alive for the whole cycle.
  for (auto & map_manager : maps_managers_.get_all()) {
    map_manager->internal_update(*nav_state);
  }
}

}  // namespace easynav
