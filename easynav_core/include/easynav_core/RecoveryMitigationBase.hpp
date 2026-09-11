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
/// \brief Declaration of the abstract base class RecoveryMitigationBase.

#ifndef EASYNAV_CORE__RECOVERYMITIGATIONBASE_HPP_
#define EASYNAV_CORE__RECOVERYMITIGATIONBASE_HPP_

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @enum RecoveryStatus
 * @brief Outcome of one cycle of a RecoveryMitigationBase plugin.
 */
enum class RecoveryStatus
{
  RUNNING,    ///< Still working; call on_cycle() again next cycle.
  SUCCEEDED,  ///< Done; the diagnostic that triggered this mitigation is considered resolved.
  FAILED      ///< Gave up; RecoveryManagerNode may try the next applicable mitigator, if any.
};

/**
 * @class RecoveryMitigationBase
 * @brief Base class for level-1 (deliberative) mitigation plugins.
 *
 * See docs/recoveries_easynav.md, level 1. A mitigation is selected by RecoveryManagerNode when
 * one of its can_handle() returns true for the current highest-priority diagnostic. Mitigations
 * that requires_control() (move the robot) are cycled by RecoveryManagerNode::cycle_rt(), at RT
 * rate, and take over "cmd_vel" via the "control_owner" NavState key; mitigations that don't
 * (e.g. a future ClearMapRecovery/ForceReplanRecovery) are cycled from the non-RT cycle()
 * instead and never touch "control_owner".
 */
class RecoveryMitigationBase : public MethodBase
{
public:
  RecoveryMitigationBase() = default;
  virtual ~RecoveryMitigationBase() = default;

  /**
   * @brief Whether this plugin knows how to address the given diagnostic.
   * @param status A non-OK diagnostic currently present in NavState's "diagnostics" group.
   * @return True if this plugin can attempt a mitigation for it.
   */
  virtual bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const = 0;

  /**
   * @brief Whether this mitigation needs to own "cmd_vel" (via "control_owner") while active.
   * @return True for movement mitigations; false (the default) for anything else.
   */
  virtual bool requires_control() const {return false;}

  /// @brief Called once when RecoveryManagerNode selects this mitigation. Exception-safe.
  void internal_start(NavState & nav_state);

  /// @brief Called once per cycle while this mitigation is active. Exception-safe: an
  /// exception in on_cycle() is treated as RecoveryStatus::FAILED, stopping the robot first.
  RecoveryStatus internal_cycle(NavState & nav_state);

  /// @brief Called once when this mitigation stops (succeeded, failed, or superseded).
  /// Exception-safe.
  void internal_stop(NavState & nav_state);

protected:
  /// @brief Hook for one-time setup when this mitigation is selected.
  virtual void on_start([[maybe_unused]] NavState & nav_state) {}

  /// @brief Runs one cycle of the mitigation.
  virtual RecoveryStatus on_cycle(NavState & nav_state) = 0;

  /// @brief Hook for cleanup when this mitigation stops.
  virtual void on_stop([[maybe_unused]] NavState & nav_state) {}

  /// @brief Fail-safe default: writes a zero-velocity TwistStamped to "cmd_vel". Available to
  /// movement mitigations for their SUCCEEDED/FAILED exit paths.
  void stop_robot(NavState & nav_state);
};

}  // namespace easynav

#endif  // EASYNAV_CORE__RECOVERYMITIGATIONBASE_HPP_
