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
/// \brief Declaration of the RecoveryManagerNode class.

#ifndef EASYNAV_RECOVERY__RECOVERYMANAGERNODE_HPP_
#define EASYNAV_RECOVERY__RECOVERYMANAGERNODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/RecoveryManagerBase.hpp"
#include "easynav_core/SystemActions.hpp"

namespace easynav
{

/**
 * @class RecoveryManagerNode
 * @brief ROS 2 lifecycle node that hosts EasyNav's recovery system, a RecoveryManagerBase plugin.
 *
 * Like ControllerNode or PlannerNode host their plugin, this node hosts the recovery system
 * chosen with "recovery_manager.plugin" (default "easynav_recovery/DefaultRecoveryManager"),
 * loaded on every configure and released on cleanup, so the whole recovery system can be
 * replaced by configuration. It forwards EasyNav's cycles and activation to it, and hands it
 * the SystemActions it may take (set by SystemNode).
 *
 * Owned by SystemNode and cycled on its loops, in the same process as the rest of the
 * navigation stack, so the recovery system reads NavState with no IPC latency.
 */
class RecoveryManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RecoveryManagerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// @brief Name of the plugin instance, and so the namespace of its own parameters.
  static constexpr char kManagerName[] = "recovery_manager";

  /// @brief Recovery system loaded when "recovery_manager.plugin" is not set.
  static constexpr char kDefaultManager[] = "easynav_recovery/DefaultRecoveryManager";

  /**
   * @brief Constructor.
   * @param options Optional node configuration.
   */
  explicit RecoveryManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~RecoveryManagerNode();

  /// @brief Loads and initializes the recovery system ("recovery_manager.plugin").
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /// @brief Forwards the activation to the recovery system.
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /// @brief Forwards the deactivation to the recovery system.
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /// @brief Releases the recovery system.
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /// @brief Releases the recovery system.
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /// @brief Releases the recovery system.
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Runs one non-RT cycle of the recovery system, after the rest of EasyNav.
   * @param nav_state Shared navigation state.
   */
  void cycle(std::shared_ptr<NavState> nav_state);

  /**
   * @brief Runs one RT cycle of the recovery system, after the controller proposed its command
   * and before ControllerNode publishes the velocity command.
   * @param nav_state Shared navigation state.
   * @return True if the recovery system commanded the robot this cycle.
   */
  bool cycle_rt(std::shared_ptr<NavState> nav_state);

  /// @brief What the recovery system may ask of the navigation system (see SystemActions).
  void set_system_actions(std::weak_ptr<SystemActions> actions);

  /// @brief The loaded recovery system, or nullptr if not configured.
  [[nodiscard]] std::shared_ptr<RecoveryManagerBase> get_recovery_manager() const;

private:
  std::unique_ptr<pluginlib::ClassLoader<RecoveryManagerBase>> loader_;

  /// @brief Replaces the loaded recovery system. Releasing it (nullptr) also releases any hold it
  /// left on the mission's progress (SystemActions::hold_mission_progress()).
  void set_manager(std::shared_ptr<RecoveryManagerBase> manager);

  /// @brief Handed out as copies (get_recovery_manager()), so a cycle running in another thread
  /// keeps it alive even if a transition releases it concurrently.
  std::shared_ptr<RecoveryManagerBase> manager_;
  mutable std::mutex manager_mutex_;

  std::weak_ptr<SystemActions> system_actions_;
};

}  // namespace easynav

#endif  // EASYNAV_RECOVERY__RECOVERYMANAGERNODE_HPP_
