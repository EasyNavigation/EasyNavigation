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
/// \brief Declaration of the ControllerNode class, a ROS 2 lifecycle node for speed computation in Easy Navigation.

#ifndef EASYNAV_CONTROLLER__CONTROLLERNODE_HPP_
#define EASYNAV_CONTROLLER__CONTROLLERNODE_HPP_

#include <memory>
#include <string>

#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_core/PluginSwitcher.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/// \file
/// \brief Declaration of the ControllerNode class, a ROS 2 lifecycle node for calculating speeds tasks in Easy Navigation.

/**
 * @class ControllerNode
 * @brief ROS 2 lifecycle node that manages calculating speeds for the Easy Navigation system.
 *
 * This node provides the interface between the controller module in EasyNav and the ROS 2 ecosystem.
 * It handles lifecycle transitions, real-time scheduling of periodic tasks, and parameter setup.
 */

class ControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ControllerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a ControllerNode lifecycle node with the specified options.
   * @param options Node options to configure the ControllerNode node.
   */
  explicit ControllerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroys the ControllerNode object.
   */
  ~ControllerNode();

  /**
   * @brief Configures the ControllerNode node.
   * This is typically where parameters and interfaces are declared.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the ControllerNode node.
   * This starts periodic navigation control cycles.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the ControllerNode node.
   * Control loops are stopped and interfaces are disabled.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the ControllerNode node.
   * Releases resources and resets the internal state.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the ControllerNode node.
   * Called on final shutdown of the node's lifecycle.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the ControllerNode node.
   * This is called when a failure occurs during a lifecycle transition.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating error handling is complete.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Returns the real-time callback group.
   *
   * This callback group can be used to assign callbacks that require
   * low latency or have real-time constraints.
   *
   * @return Shared pointer to the real-time callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Executes one cycle of real-time controller logic.
   *
   * This method is invoked periodically by a high-priority timer and is expected
   * to compute control commands based on the current navigation state and input data.
   * @param nav_state Shared pointer to the navigation state structure.
   * @return Bool value to indicate if trigger subsequent processes
   */
  bool cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger = false);

  /**
   * @brief Alias of the loaded controller (its "controller_types" entry).
   * @return The alias, or an empty string if no controller is loaded.
   */
  std::string get_loaded_controller() const;

private:
  /**
   * @brief Callback group intended for real-time tasks.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Owns the controller plugin and reloads it on every configure.
   *
   * To change the controller: deactivate, cleanup, set "controller_types" and
   * configure again. Declared after realtime_cbg_ so the plugin (and its
   * library) is destroyed first.
   */
  PluginSwitcher<easynav::ControllerMethodBase> controller_;

  /**
   * @brief Current navigation state.
   *
   * This is the current state of the navigation system.
   */
  const std::shared_ptr<const NavState> nav_state_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__CONTROLLERNODE_HPP_
