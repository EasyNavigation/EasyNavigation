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

#include <atomic>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_controller/ControllerNode.hpp"
#include "easynav_core/SystemActions.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_planner/PlannerNode.hpp"
#include "easynav_recovery/RecoveryManagerNode.hpp"
#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_system/GoalManager.hpp"
#include "easynav_system/safety/SafetySupervisor.hpp"

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
 *
 * Safety ("safety.*" parameters, configuration hash): safety::SafetySupervisor.
 */
class SystemNode : public rclcpp_lifecycle::LifecycleNode, public SystemActions
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

  /**
   * @brief Whether the recovery system asked EasyNav to terminate (request_shutdown()).
   * Whoever drives this node's lifecycle should then deactivate it, which ends in Finalized
   * (see on_deactivate()/on_error()).
   */
  [[nodiscard]] bool is_shutdown_requested() const {return shutdown_requested_;}

  /// @brief Why the shutdown was requested.
  [[nodiscard]] std::string get_shutdown_reason() const;

  /// @brief SystemActions: aborts the active mission, if any, telling its client why.
  void abort_mission(const std::string & reason) override;

  /// @brief SystemActions: while held, GoalManager takes no goal as reached.
  void hold_mission_progress(bool hold) override;

  /// @brief SystemActions: records that EasyNav must terminate (see is_shutdown_requested()).
  void request_shutdown(const std::string & reason) override;

  /// @brief SystemActions: pending until apply_pending_reconfigure(); rejected in safety mode.
  bool request_reconfigure(
    const std::vector<ParameterChange> & changes, const std::string & reason) override;

  /// @brief SystemActions: pending until apply_pending_reconfigure(); rejected in safety mode.
  bool request_restore_parameters(const std::string & reason) override;

  /// @brief Safety mode, memory lock and configuration hash, as of the last configure.
  [[nodiscard]] const safety::SafetySupervisor & get_safety() const {return safety_;}

  /// @brief Every EasyNav parameter, one "node/parameter=value" per line, sorted.
  [[nodiscard]] std::string get_configuration_dump();

  /**
   * @brief Applies the pending reconfiguration request, if any, while Active: cycles through
   * unconfigured, setting the parameters there. Called by whoever drives this node's lifecycle,
   * between cycles (never from a cycle: the recovery system is reloaded).
   * @return True if EasyNav was reconfigured (with the new values or, if they failed, the
   * previous ones).
   */
  bool apply_pending_reconfigure();

  /// @brief Whether a reconfiguration is pending.
  [[nodiscard]] bool is_reconfigure_pending() const;

private:
  /// @brief Leaves a zero "cmd_vel" in NavState (ControllerNode stops the robot).
  void clear_cmd_vel();

  /// @brief Applies a deprecated "system_node.use_cmd_vel_stamped" to controller_node.
  void forward_deprecated_use_cmd_vel_stamped();

  /// @brief Shares "robot_geometry.*" (RobotGeometryRegistry) before the subnodes configure.
  void configure_robot_geometry();

  /// @brief Checks this node's parameters (frequencies, geometry).
  bool check_system_parameters();

  /// @brief Checks that no component's "*.rt_freq" / "*.freq" exceeds rt_freq / freq.
  bool check_component_frequencies();

  /// @brief Leaves every configured subnode unconfigured again (after a failed configure).
  void cleanup_subnodes();

  /// @brief This node and its subnodes, by name (get_system_nodes(): only the subnodes).
  std::map<std::string, rclcpp_lifecycle::LifecycleNode::SharedPtr> get_all_nodes();

  /// @brief Safety checks, configuration hash and frozen configuration.
  safety::SafetySupervisor safety_;

  /// @brief Serializes the RT cycle with activation/deactivation.
  std::mutex rt_mutex_;

  /// @brief Whether the RT cycle may run and publish (guarded by rt_mutex_).
  bool active_ {false};

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

  /// @brief Hosts the recovery system (a RecoveryManagerBase plugin).
  RecoveryManagerNode::SharedPtr recovery_node_;

  /// @brief Set by request_shutdown(), see is_shutdown_requested().
  std::atomic<bool> shutdown_requested_ {false};
  std::string shutdown_reason_;
  mutable std::mutex shutdown_reason_mutex_;

  /// @brief A reconfiguration requested by the recovery system (see apply_pending_reconfigure()).
  struct ReconfigureRequest
  {
    std::vector<ParameterChange> changes;
    bool restore {false};
    std::string reason;
  };
  std::optional<ReconfigureRequest> pending_reconfigure_;
  mutable std::mutex reconfigure_mutex_;

  /// @brief Value of each changed parameter before its first change, by "node/parameter".
  std::map<std::string, ParameterChange> original_parameters_;

  /// @brief EasyNav node by name (a subnode or this one), or nullptr.
  rclcpp_lifecycle::LifecycleNode::SharedPtr find_node(const std::string & name);

  /// @brief Goes to unconfigured, sets \p changes and goes back to active.
  /// @return True if every change was set and EasyNav is active again.
  bool restart_with(const std::vector<ParameterChange> & changes);

  /// @brief Shared navigation state.
  std::shared_ptr<NavState> nav_state_;

  /// @brief Goal manager.
  GoalManager::SharedPtr goal_manager_;


  /// @brief Publisher for nav_state as string.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navstate_pub_;


};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__SYSTEMNODE_HPP_
