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
/// \brief Actions a recovery system may take on the navigation system as a whole.

#ifndef EASYNAV_CORE__SYSTEMACTIONS_HPP_
#define EASYNAV_CORE__SYSTEMACTIONS_HPP_

#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"

namespace easynav
{

/// @brief A parameter to change on one of EasyNav's nodes (e.g. "controller_node").
struct ParameterChange
{
  std::string node;
  rclcpp::Parameter parameter;
};

/**
 * @class SystemActions
 * @brief What a recovery system can ask of EasyNav beyond commanding velocity.
 *
 * Implemented by SystemNode and handed to the recovery manager (RecoveryManagerBase), so a
 * recovery system acts on the mission and on EasyNav's lifecycle through this interface instead
 * of through signals that other components would have to know about.
 */
class SystemActions
{
public:
  virtual ~SystemActions() = default;

  /**
   * @brief Aborts the active mission, if any, telling its client why.
   * @param reason Human-readable cause, sent to the client.
   */
  virtual void abort_mission(const std::string & reason) = 0;

  /**
   * @brief Holds (or releases) the mission's progress.
   *
   * While held, the mission stays active and its feedback keeps flowing, but no goal is taken
   * as reached: the recovery system is handling a problem (e.g. localization diverged), so the
   * robot pose cannot be trusted to decide that the robot arrived. The hold lasts until
   * released, across missions.
   *
   * @param hold True to hold the mission's progress, false to release it.
   */
  virtual void hold_mission_progress(bool hold) = 0;

  /**
   * @brief Asks EasyNav to terminate because of an unrecoverable problem.
   *
   * EasyNav stops the robot, leaves Active through the lifecycle's error path
   * (ErrorProcessing -> Finalized) and exits, reporting \p reason.
   *
   * @param reason Human-readable cause, reported on termination.
   */
  virtual void request_shutdown(const std::string & reason) = 0;

  /**
   * @brief Asks EasyNav to change parameters and reconfigure to apply them.
   *
   * Applied between cycles, not during the call: EasyNav goes active -> inactive -> unconfigured,
   * sets the parameters, and goes back to active. The mission goes on, and the robot only stops
   * during the transitions. The recovery system is reloaded too (a new instance): keep in
   * NavState anything to remember. "reconfigured_parameters" in NavState lists the parameters
   * changed so far ("node/parameter"). If the new values do not apply, the previous ones are
   * restored. A newer request replaces a pending one.
   *
   * @param changes Parameters to change.
   * @param reason Human-readable cause, logged.
   */
  virtual void request_reconfigure(
    const std::vector<ParameterChange> & changes, const std::string & reason) = 0;

  /**
   * @brief Asks EasyNav to restore every parameter changed by request_reconfigure() to its value
   * before the first change, and reconfigure to apply them. Nothing to do if none changed.
   * @param reason Human-readable cause, logged.
   */
  virtual void request_restore_parameters(const std::string & reason) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__SYSTEMACTIONS_HPP_
