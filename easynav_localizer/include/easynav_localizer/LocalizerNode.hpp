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
/// \brief Declaration of the LocalizerNode class, a ROS 2 lifecycle node for localization tasks in Easy Navigation.

#ifndef EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_
#define EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_

#include <mutex>

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class LocalizerNode
 * @brief ROS 2 lifecycle node that manages localization in Easy Navigation.
 *
 * Handles lifecycle transitions, dynamic loading of the localization plugin,
 * and periodic update execution in real-time or non-real-time contexts.
 */
class LocalizerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LocalizerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Optional node configuration.
   */
  explicit LocalizerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~LocalizerNode();

  /**
   * @brief Configure the node.
   * @param state Lifecycle state.
   * @return SUCCESS if configuration succeeded.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if activation succeeded.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if deactivation succeeded.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Clean up the node.
   * @param state Lifecycle state.
   * @return SUCCESS if cleanup succeeded.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Lifecycle state.
   * @return SUCCESS if shutdown succeeded.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle lifecycle error transition.
   * @param state Lifecycle state.
   * @return SUCCESS after handling error.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the callback group for real-time execution.
   * @return Shared pointer to the callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Run a real-time localization cycle.
   * @param nav_state Shared pointer to the navigation state structure.
   * @param trigger Optional override to force execution.
   * @return True if plugin update was executed.
   */
  bool cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger = false);

  /**
   * @brief Run a non-real-time localization cycle.
   * @param nav_state Shared pointer to the navigation state structure.
   */
  void cycle(std::shared_ptr<NavState> nav_state);

private:
  /// @brief Callback group reserved for real-time operations.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /// @brief Plugin loader for LocalizerMethodBase implementations.
  std::unique_ptr<pluginlib::ClassLoader<easynav::LocalizerMethodBase>> localizer_loader_;

  /// @brief Instance of the loaded localization plugin.
  std::shared_ptr<LocalizerMethodBase> localizer_method_ {nullptr};

  /**
   * @brief Guards \ref localizer_method_ between the RT thread (cycle_rt)
   * and the non-RT thread (cycle / on_cleanup), which run concurrently.
   */
  std::mutex localizer_method_mutex_;
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_
