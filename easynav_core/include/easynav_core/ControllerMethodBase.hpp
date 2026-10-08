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
/// \brief Declaration of the abstract base class ControllerMethodBase.

#ifndef EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
#define EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_

#include <string>

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"
#include "easynav_core/RobotLimits.hpp"

namespace easynav
{

/**
 * @class ControllerMethodBase
 * @brief Abstract base class for control methods in Easy Navigation.
 *
 * This class defines the interface for control algorithm implementations.
 * Derived classes must implement the control logic and provide access to the computed command.
 */
class ControllerMethodBase : public MethodBase
{
public:
  /// @brief Default constructor.
  ControllerMethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~ControllerMethodBase() = default;

  /**
   * @brief Initialize the controller method.
   *
   * Forwards initialization to MethodBase.
   *
   * @param parent_node Reference to the parent lifecycle node.
   * @param plugin_name Plugin identifier used for namespacing parameters.
   * @throws std::runtime_error on initialization failure.
   */
  virtual void initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string & plugin_name);

  /**
   * @brief Helper to run the real-time control method if appropriate.
   *
   * Invokes update_rt() only if the method is due or forced by trigger.
   *
   * @param nav_state The current state of the navigation system.
   * @param trigger Force execution regardless of timing.
   * @return True if update_rt() was called, false otherwise.
   */
  bool internal_update_rt(NavState & nav_state, bool trigger = false);

  /**
   * @brief The robot limits, configured in the node that loaded this plugin (ControllerNode,
   * "robot_limits.*"); the RobotLimits defaults without such a node (e.g. a plain node in a test).
   *
   * Controllers must use them instead of declaring their own: the velocity smoother enforces the
   * same limits on every command. \p legacy lists this controller's deprecated parameters: a
   * value still configured under one of them is applied, with a deprecation warning, unless
   * "robot_limits.*" configures that limit. The resulting limits are the ones the node enforces.
   */
  RobotLimits get_robot_limits(const LegacyRobotLimitNames & legacy = {});

protected:
  /**
   * @brief Reads the deprecated parameter "<plugin>.<name>" if it is configured, warning that
   * \p replacement should be used instead.
   * @return Whether it was configured (then \p value holds it).
   */
  bool get_deprecated_parameter(
    const std::string & name, const std::string & replacement, double & value);

private:
  /// @brief As get_deprecated_parameter(), without warning.
  bool read_deprecated_parameter(const std::string & name, double & value);

protected:
  /**
   * @brief Run the control method and update the control command.
   *
   * Called by the system to compute a new control command using the current navigation state.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update_rt([[maybe_unused]] NavState & nav_state) {}
};

}  // namespace easynav

#endif  // EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
