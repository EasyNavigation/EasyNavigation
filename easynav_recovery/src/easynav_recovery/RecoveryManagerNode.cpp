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
/// \brief Implementation of the RecoveryManagerNode class.

#include <string>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "easynav_recovery/RecoveryManagerNode.hpp"

namespace easynav
{

RecoveryManagerNode::RecoveryManagerNode(const rclcpp::NodeOptions & options)
: LifecycleNode("recovery_node", options)
{
  loader_ = std::make_unique<pluginlib::ClassLoader<RecoveryManagerBase>>(
    "easynav_core", "easynav::RecoveryManagerBase");
  declare_parameter(std::string(kManagerName) + ".plugin", std::string(kDefaultManager));
}

RecoveryManagerNode::~RecoveryManagerNode()
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
  set_manager(nullptr);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
RecoveryManagerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::string plugin;
  get_parameter(std::string(kManagerName) + ".plugin", plugin);

  RCLCPP_INFO(get_logger(), "Loading recovery manager [%s]", plugin.c_str());
  std::shared_ptr<RecoveryManagerBase> manager;
  try {
    manager = loader_->createSharedInstance(plugin);
    manager->set_system_actions(system_actions_);
    manager->initialize(shared_from_this(), kManagerName);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Unable to load recovery manager [%s]: %s", plugin.c_str(), e.what());
    return CallbackReturnT::FAILURE;
  }

  set_manager(manager);
  RCLCPP_INFO(get_logger(), "Loaded recovery manager [%s]", plugin.c_str());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  if (auto manager = get_recovery_manager()) {
    manager->on_activate();
  }
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  if (auto manager = get_recovery_manager()) {
    manager->on_deactivate();
  }
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  set_manager(nullptr);
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  set_manager(nullptr);
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  set_manager(nullptr);
  return CallbackReturnT::SUCCESS;
}

void
RecoveryManagerNode::cycle(std::shared_ptr<NavState> nav_state)
{
  if (auto manager = get_recovery_manager()) {
    manager->internal_update(*nav_state);
  }
}

bool
RecoveryManagerNode::cycle_rt(std::shared_ptr<NavState> nav_state)
{
  auto manager = get_recovery_manager();
  return manager ? manager->internal_update_rt(*nav_state) : false;
}

void
RecoveryManagerNode::set_system_actions(std::weak_ptr<SystemActions> actions)
{
  system_actions_ = actions;
  if (auto manager = get_recovery_manager()) {
    manager->set_system_actions(actions);
  }
}

std::shared_ptr<RecoveryManagerBase>
RecoveryManagerNode::get_recovery_manager() const
{
  std::lock_guard<std::mutex> lock(manager_mutex_);
  return manager_;
}

void
RecoveryManagerNode::set_manager(std::shared_ptr<RecoveryManagerBase> manager)
{
  std::lock_guard<std::mutex> lock(manager_mutex_);
  manager_ = std::move(manager);
}

}  // namespace easynav
