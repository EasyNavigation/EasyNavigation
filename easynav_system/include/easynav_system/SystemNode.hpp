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
/// \brief Declaration of the SystemNode class, the central coordinator node for Easy Navigation components.

#ifndef EASYNAV_SYSTEM__SYSTEMNODE_HPP_
#define EASYNAV_SYSTEM__SYSTEMNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_controller/ControllerNode.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_planner/PlannerNode.hpp"
#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_system/GoalManager.hpp"

namespace easynav
{

/**
 * @struct SystemNodeInfo
 * @brief Structure holding runtime information for a subnode.
 */
struct SystemNodeInfo
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr; ///< Shared pointer to the managed lifecycle node.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg;       ///< Associated real-time callback group.
};

/**
 * @class SystemNode
 * @brief ROS 2 lifecycle node coordinating all Easy Navigation components.
 *
 * Manages lifecycle transitions, real-time execution, and communication
 * between planner, controller, localizer, map manager, and sensor nodes.
 */
class SystemNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SystemNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Node options.
   */
  explicit SystemNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~SystemNode();

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
   * @brief Cleanup the node.
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
   * @brief Handle lifecycle transition error.
   * @param state Lifecycle state.
   * @return SUCCESS if error handled.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the real-time callback group.
   * @return Shared pointer to the callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Get all system nodes managed by this coordinator.
   * @return Map of node names to node information.
   */
  std::map<std::string, SystemNodeInfo> get_system_nodes();

  /**
   * @brief Real-time system cycle.
   */
  void system_cycle_rt();

  /**
   * @brief Non-real-time system cycle.
   */
  void system_cycle();

  /**
   * @brief Access to the shared navigation state (for testing and tools).
   * @return Shared pointer to the NavState.
   */
  [[nodiscard]] std::shared_ptr<NavState> get_nav_state() const {return nav_state_;}

private:
  /// @brief Real-time callback group.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /// @brief Controller node.
  ControllerNode::SharedPtr controller_node_;

  /// @brief Localizer node.
  LocalizerNode::SharedPtr localizer_node_;

  /// @brief Maps manager node.
  MapsManagerNode::SharedPtr maps_manager_node_;

  /// @brief Planner node.
  PlannerNode::SharedPtr planner_node_;

  /// @brief Sensors node.
  SensorsNode::SharedPtr sensors_node_;

  /// @brief Shared navigation state.
  std::shared_ptr<NavState> nav_state_;

  /// @brief Goal manager.
  GoalManager::SharedPtr goal_manager_;

  /// @brief Wheter publish stamped or unstamped speed
  bool use_cmd_vel_stamped_ {false};

  /// @brief Publisher for nav_state as string.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navstate_pub_;

  /// @brief Publisher for velocity command (stamped).
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_stamped_;

  /// @brief Publisher for velocity command (legacy Twist).
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__SYSTEMNODE_HPP_
