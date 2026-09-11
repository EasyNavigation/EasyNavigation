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
/// \brief Declaration of the abstract base class SafetyReflexBase.

#ifndef EASYNAV_CORE__SAFETYREFLEXBASE_HPP_
#define EASYNAV_CORE__SAFETYREFLEXBASE_HPP_

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class SafetyReflexBase
 * @brief Base class for level-0 (real-time) safety reflexes, see docs/recoveries_easynav.md.
 *
 * A reflex is checked by SystemNode on every RT cycle, right before "cmd_vel" is published,
 * regardless of whether it was produced by the nominal controller or by a movement recovery
 * mitigator: this is what lets a single, small, independently verifiable component gate every
 * current and future producer of "cmd_vel". Unlike other MethodBase-derived plugins, a reflex
 * is not rate-limited by MethodBase::isTime2RunRT() — SystemNode already controls the overall
 * RT rate — and its own failure is treated as unsafe: if check() or mitigate() throws, the
 * robot is stopped as a fail-safe default instead of assuming the reflex is inactive.
 */
class SafetyReflexBase : public MethodBase
{
public:
  SafetyReflexBase() = default;
  virtual ~SafetyReflexBase() = default;

  /**
   * @brief Runs one RT cycle of this reflex without letting check()/mitigate() escape.
   *
   * @param nav_state Current navigation state; "cmd_vel" is read and possibly overwritten.
   * @return True if the reflex modified "cmd_vel" this cycle (a mitigation was applied).
   */
  bool internal_check_and_mitigate(NavState & nav_state);

protected:
  /**
   * @brief Decides whether mitigate() must run this cycle.
   * @param nav_state Current navigation state.
   * @return True if the situation requires intervening on "cmd_vel".
   */
  virtual bool check(NavState & nav_state) = 0;

  /**
   * @brief Applies the safety intervention, typically overwriting "cmd_vel".
   * @param nav_state Navigation state to modify.
   */
  virtual void mitigate(NavState & nav_state) = 0;

  /// @brief Fail-safe default: writes a zero-velocity TwistStamped to "cmd_vel".
  void stop_robot(NavState & nav_state);
};

}  // namespace easynav

#endif  // EASYNAV_CORE__SAFETYREFLEXBASE_HPP_
