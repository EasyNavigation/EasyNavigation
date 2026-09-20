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
/// \brief Declaration of the PlannerNode class, a ROS 2 lifecycle node for computing navigation paths in Easy Navigation.

#ifndef EASYNAV_PLANNER__PLANNERNODE_HPP_
#define EASYNAV_PLANNER__PLANNERNODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_core/PluginSwitcher.hpp"
#include "nav_msgs/msg/path.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class PlannerNode
 * @brief ROS 2 lifecycle node that manages path planning in Easy Navigation.
 *
 * Handles lifecycle transitions, plugin loading, and invokes the planner to compute paths.
 */
class PlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PlannerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Optional node configuration.
   */
  explicit PlannerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~PlannerNode();

  /**
   * @brief Configure the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if configuration succeeded.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if activation succeeded.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if deactivation succeeded.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Clean up the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if cleanup succeeded.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if shutdown succeeded.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle lifecycle transition errors.
   * @param state Current lifecycle state.
   * @return SUCCESS if error handling completed.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the current planned path.
   * @return Path message with the last computed path.
   */
  [[nodiscard]] nav_msgs::msg::Path get_path() const;

  /**
   * @brief Execute a non-real-time cycle.
   * @param nav_state Shared pointer to the navigation state structure.
   * @param trigger If tru, a cycle is executed independently of the frequency set
   */
  void cycle(std::shared_ptr<NavState> nav_state, bool trigger = false);

  /**
   * @brief Get the timestamp of the last real-time execution.
   * @return Timestamp of the last RT execution.
   */
  const rclcpp::Time get_last_rt_execution_ts() const;

  /**
   * @brief Get the timestamp of the last non-RT execution.
   * @return Timestamp of the last execution.
   */
  const rclcpp::Time get_last_execution_ts() const;

private:
  /// @brief Owns the planner plugin. To change it: deactivate, cleanup, set
  /// "planner_types" and configure again.
  PluginSwitcher<PlannerMethodBase> planner_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__PLANNERNODE_HPP_
