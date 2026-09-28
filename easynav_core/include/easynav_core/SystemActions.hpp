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
/// \brief Actions a recovery system may take on the navigation system as a whole.

#ifndef EASYNAV_CORE__SYSTEMACTIONS_HPP_
#define EASYNAV_CORE__SYSTEMACTIONS_HPP_

#include <string>

namespace easynav
{

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
   * @brief Asks EasyNav to terminate because of an unrecoverable problem.
   *
   * EasyNav stops the robot, leaves Active through the lifecycle's error path
   * (ErrorProcessing -> Finalized) and exits, reporting \p reason.
   *
   * @param reason Human-readable cause, reported on termination.
   */
  virtual void request_shutdown(const std::string & reason) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__SYSTEMACTIONS_HPP_
