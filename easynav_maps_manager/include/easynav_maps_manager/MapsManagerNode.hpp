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
/// \brief Declaration of the MapsManagerNode class, a ROS 2 lifecycle node for map handling in Easy Navigation.

#ifndef EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_
#define EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "pluginlib/class_loader.hpp"

namespace easynav
{

/**
 * @class MapsManagerNode
 * @brief ROS 2 lifecycle node that manages map-related plugins in Easy Navigation.
 *
 * Handles lifecycle transitions, plugin loading, and periodic map updates.
 */
class MapsManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MapsManagerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param nav_state Shared pointer to navigation state.
   * @param options Node options.
   */
  explicit MapsManagerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~MapsManagerNode();

  /**
   * @brief Configure the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if configuration succeeds.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if activation succeeds.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if deactivation succeeds.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleanup the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if cleanup succeeds.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if shutdown succeeds.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle errors during lifecycle transitions.
   * @param state Current lifecycle state.
   * @return SUCCESS if error was handled.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Execute one update cycle (non real-time).
   * @param nav_state Shared pointer to the navigation state structure.
   */
  void cycle(std::shared_ptr<NavState> nav_state);

private:
  /// @brief Plugin loader for map manager implementations.
  std::unique_ptr<pluginlib::ClassLoader<MapsManagerBase>> maps_manager_loader_;

  /// @brief Active map manager plugins.
  std::vector<std::shared_ptr<MapsManagerBase>> maps_managers_;
};

}  // namespace easynav

#endif  // EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_
