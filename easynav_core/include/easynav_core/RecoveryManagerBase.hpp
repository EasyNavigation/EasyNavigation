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
/// \brief Declaration of RecoveryManagerBase, the interface of a recovery system.

#ifndef EASYNAV_CORE__RECOVERYMANAGERBASE_HPP_
#define EASYNAV_CORE__RECOVERYMANAGERBASE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"
#include "easynav_core/SystemActions.hpp"

namespace easynav
{

/**
 * @class RecoveryManagerBase
 * @brief A whole recovery system, as a plugin: the only recovery code EasyNav depends on.
 *
 * RecoveryManagerNode loads one implementation ("recovery_manager.plugin"), so the recovery
 * system can be replaced without touching the rest of EasyNav. An implementation decides how to
 * detect and handle problems (the default one, easynav_recovery/DefaultRecoveryManager, uses
 * evaluator, mitigation and safety reflex plugins); EasyNav only offers it:
 *
 * Inputs:
 * - update(): every non-RT cycle, after the rest of EasyNav (sensors, localization, maps, goals,
 *   planning) has run, with the shared NavState.
 * - update_rt(): every RT cycle, after the controller proposed its command and before the
 *   velocity command is published.
 * - on_activate()/on_deactivate(): EasyNav's activation and deactivation.
 *
 * Outputs:
 * - command_velocity(): take over the robot's motion (preferred over the controller's command,
 *   smoothed within the robot limits).
 * - override_velocity(): emergency override (highest priority, published as is).
 * - abort_mission(), hold_mission_progress() and request_shutdown(): see SystemActions.
 * - Anything written to NavState for others to read (e.g. diagnostics).
 */
class RecoveryManagerBase : public MethodBase
{
public:
  RecoveryManagerBase() = default;
  virtual ~RecoveryManagerBase() = default;

  /// @brief Runs update() without letting it escape or crash the process.
  void internal_update(NavState & nav_state);

  /**
   * @brief Runs update_rt() without letting it escape the RT thread.
   * @return True if the recovery system commanded the robot this cycle. If update_rt() throws,
   * the robot is stopped (emergency override) as a fail-safe and it returns true.
   */
  bool internal_update_rt(NavState & nav_state);

  /// @brief Called when EasyNav is activated.
  virtual void on_activate() {}

  /// @brief Called when EasyNav is deactivated.
  virtual void on_deactivate() {}

  /// @brief Set by the node that hosts this plugin (see SystemActions).
  void set_system_actions(std::weak_ptr<SystemActions> actions) {system_actions_ = actions;}

protected:
  /// @brief One non-RT cycle: diagnose, decide, act. Rate-limit yourself if needed.
  virtual void update(NavState & nav_state) = 0;

  /**
   * @brief One RT cycle, right before the velocity command is published.
   * @return True if it commanded the robot (command_velocity()/override_velocity()).
   */
  virtual bool update_rt([[maybe_unused]] NavState & nav_state) {return false;}

  /// @brief Takes over the robot's motion this RT cycle (preferred over the controller).
  void command_velocity(NavState & nav_state, const geometry_msgs::msg::TwistStamped & cmd);

  /// @brief Emergency override this RT cycle: highest priority, published without smoothing.
  void override_velocity(NavState & nav_state, const geometry_msgs::msg::TwistStamped & cmd);

  /// @brief Aborts the active mission (see SystemActions::abort_mission()).
  void abort_mission(const std::string & reason);

  /// @brief Holds or releases the mission's progress (see SystemActions::hold_mission_progress()).
  void hold_mission_progress(bool hold);

  /// @brief Asks EasyNav to terminate (see SystemActions::request_shutdown()).
  void request_shutdown(const std::string & reason);

private:
  std::weak_ptr<SystemActions> system_actions_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__RECOVERYMANAGERBASE_HPP_
