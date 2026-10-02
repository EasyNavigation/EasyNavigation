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
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_controller/VelocityMux.hpp"
#include "easynav_controller/VelocitySmoother.hpp"
#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_core/RobotLimits.hpp"
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
 *
 * It is also the single point where the velocity command leaves EasyNav:
 * - it owns the robot limits ("robot_limits.*"), which controller plugins query through
 *   ControllerMethodBase::get_robot_limits();
 * - every RT cycle, publish_cmd_vel_rt() selects the command (VelocityMux: override > takeover >
 *   pause > controller), smooths it within those limits (except an override) and publishes it
 *   on "cmd_vel" (or "cmd_vel_stamped", see "use_cmd_vel_stamped");
 * - on deactivation or shutdown, it brakes within the deceleration limits and ends by
 *   publishing an exact zero.
 *
 * Robustness:
 * - "cmd_timeout" (s, 0 disables): with no new command for that long, brakes to zero. A
 *   controller command is new only if its stamp or value changed.
 * - "cmd_vel_keepalive_period" (s, 0 disables): republishes at least this often, offering
 *   deadline and liveliness QoS of twice this period.
 * - Non-finite commands are discarded. Problems go to "diagnostics.cmd_vel" in NavState.
 */
class ControllerNode : public rclcpp_lifecycle::LifecycleNode, public RobotLimitsProvider
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

  /**
   * @brief Selects, smooths and publishes this RT cycle's velocity command.
   *
   * VelocityMux picks, by priority, among the commands proposed this cycle (see
   * velocity_command): an override (published as is), a takeover, zero while paused, or the
   * controller's. Publishes when a new command was proposed, or while the smoother is still
   * ramping towards the last one.
   * @param nav_state Shared navigation state.
   */
  void publish_cmd_vel_rt(std::shared_ptr<NavState> nav_state);

  /// @brief Robot limits being enforced.
  RobotLimits get_robot_limits() const override;

  /// @brief Whether "robot_limits.<field>" was configured explicitly.
  bool is_robot_limit_configured(const std::string & field) const override;

  /// @brief Replaces the limits enforced until the next configure.
  void set_robot_limits(const RobotLimits & limits) override;

private:
  /// @brief Reads "robot_limits.*" and "use_cmd_vel_stamped".
  void read_parameters();

  /// @brief Publishes \p cmd on the configured velocity topic.
  void publish(const geometry_msgs::msg::TwistStamped & cmd);

  /// @brief Brakes within the deceleration limits and ends by publishing an exact zero.
  void stop_robot();

  /// @brief QoS of the velocity publisher.
  rclcpp::QoS cmd_vel_qos() const;

  /// @brief Checks "cmd_timeout" against the loaded controller's period.
  bool check_cmd_timeout();

  /// @brief Writes "diagnostics.cmd_vel" when its level or message changes.
  void report_cmd_vel(NavState & nav_state, uint8_t level, const std::string & message);

  RobotLimits robot_limits_;
  std::set<std::string> configured_limits_;
  mutable std::mutex robot_limits_mutex_;

  bool use_cmd_vel_stamped_ {false};
  double cmd_timeout_ {0.5};
  double cmd_vel_keepalive_period_ {0.0};
  std::string timeout_message_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_stamped_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  VelocityMux mux_;
  VelocitySmoother smoother_;

  /// @brief When the smoother last stepped (node clock), to know how much time it covers.
  std::optional<rclcpp::Time> last_smoother_step_;

  /// @brief When a command was last published (node clock), for the keepalive.
  std::optional<rclcpp::Time> last_publish_;

  /// @brief Last controller command proposed, to propose only new ones.
  std::optional<geometry_msgs::msg::TwistStamped> last_controller_cmd_;

  /// @brief Last "diagnostics.cmd_vel" written (level, message).
  std::optional<std::pair<uint8_t, std::string>> last_cmd_vel_report_;

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
