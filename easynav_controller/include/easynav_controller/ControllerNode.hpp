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
#include <mutex>
#include <string>
#include <vector>

#include "easynav_core/ControllerMethodBase.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
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

private:
  /**
   * @brief Applies a pending runtime controller change.
   *
   * The controller plugin is created and initialized outside the parameter
   * callback. The new controller is then swapped into controller_method_
   * under the corresponding mutex.
   */
  void apply_pending_controller();

  /**
   * @brief Guards access to the pending controller change.
   */
  std::mutex pending_controller_mutex_;

  /**
   * @brief Controller type pending to be loaded.
   */
  std::string pending_controller_type_;

  /**
   * @brief Indicates whether a controller change is pending.
   */
  bool controller_change_pending_{false};

  /**
   * @brief Indicates that the node is currently being configured.
   *
   * While configuring, parameter callbacks must not apply runtime validation
   * to the initial parameter declarations.
   */
  bool configuring_{false};

  /**
   * @brief Validates runtime changes of the "controller_types" parameter.
   *
   * Controller changes are only allowed while the node is inactive. During
   * initial configuration, validation is skipped while configuring_ is true.
   */
  rcl_interfaces::msg::SetParametersResult
  on_set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Creates and initializes a controller plugin instance.
   *
   * The created instance is not assigned to controller_method_ by this
   * function.
   *
   * @param controller_type Alias used to configure the controller.
   * @param plugin Pluginlib class name.
   * @return The initialized controller instance, or nullptr on failure.
   */
  std::shared_ptr<ControllerMethodBase>
  create_and_initialize_controller(
    const std::string & controller_type,
    const std::string & plugin);

  /**
   * @brief Callback group intended for real-time tasks.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Pluginlib loader used to dynamically load controller implementations.
   *
   * This allows runtime selection and loading of different controller strategies
   * that inherit from ControllerMethodBase, using ROS pluginlib.
   *
   */
  std::unique_ptr<pluginlib::ClassLoader<easynav::ControllerMethodBase>> controller_loader_;

  /**
   * @brief Pointer to the controller method.
   *
   * This is the actual control algorithm that will be used.
   */
  std::shared_ptr<ControllerMethodBase> controller_method_ {nullptr};

  /**
   * @brief Guards \ref controller_method_ between the RT thread (cycle_rt)
   * and the non-RT thread (on_cleanup), which run concurrently.
   */
  std::mutex controller_method_mutex_;

/**
 * @brief Handle for the on-set-parameters callback.
 *
 * The callback validates runtime changes to "controller_types" and stores
 * pending controller changes for later application.
 */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;

  /**
   * @brief Current navigation state.
   *
   * This is the current state of the navigation system.
   */
  const std::shared_ptr<const NavState> nav_state_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__CONTROLLERNODE_HPP_
