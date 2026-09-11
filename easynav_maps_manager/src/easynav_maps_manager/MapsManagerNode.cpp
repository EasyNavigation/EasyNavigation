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
: LifecycleNode("maps_manager_node", options)
{
  maps_manager_loader_ = std::make_unique<pluginlib::ClassLoader<MapsManagerBase>>(
    "easynav_core", "easynav::MapsManagerBase");
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

  maps_managers_.clear();
  maps_manager_loader_.reset();
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
MapsManagerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> map_types;
  declare_parameter("map_types", map_types);
  get_parameter("map_types", map_types);

  for (const auto & map_type : map_types) {
    std::string plugin;
    declare_parameter(map_type + std::string(".plugin"), plugin);
    get_parameter(map_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading MapsManagerBase %s [%s]", map_type.c_str(), plugin.c_str());

      std::shared_ptr<MapsManagerBase> instance;
      instance = maps_manager_loader_->createSharedInstance(plugin);

      try {
        instance->initialize(shared_from_this(), map_type);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), e.what());
        return CallbackReturnT::FAILURE;
      }

      maps_managers_.push_back(instance);

      RCLCPP_INFO(get_logger(),
        "Loaded MapsManagerBase %s [%s]", map_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin easynav::MapsManagerBase. Error: %s", ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
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
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

void
MapsManagerNode::cycle(std::shared_ptr<NavState> nav_state)
{
  for (auto & map_manager : maps_managers_) {
    map_manager->internal_update(*nav_state);
  }
}

}  // namespace easynav
