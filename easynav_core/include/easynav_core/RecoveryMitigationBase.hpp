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

#include <cstdint>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rcl_interfaces/msg/log.hpp"

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
 * @struct MitigationReport
 * @brief One human-readable status line from a RecoveryMitigationBase plugin, queued via
 * RecoveryMitigationBase::report() for RecoveryManagerNode to publish on the "mitigation" topic.
 *
 * NavState only ever holds the single *latest* report under one key, not a growing queue: \c seq
 * is a global counter, shared by every mitigation instance in the process, that lets
 * RecoveryManagerNode detect "a new report arrived" with a single atomic read/write instead of a
 * thread-safe queue. A mitigation that would report every cycle must throttle itself well below
 * RecoveryManagerNode's own cycle rate — see report()'s doc comment.
 */
struct MitigationReport
{
  uint64_t seq {0};
  rcl_interfaces::msg::Log log;
};

/**
 * @class RecoveryMitigationBase
 * @brief Base class for level-1 (deliberative) mitigation plugins.
 *
 * A mitigation is selected by RecoveryManagerNode when one of its can_handle() returns true for
 * the current highest-priority diagnostic. Mitigations that requires_control() (move the robot)
 * are cycled by RecoveryManagerNode::cycle_rt(), at RT rate, and take over "cmd_vel" via the
 * "control_owner" NavState key; mitigations that don't are cycled from the non-RT cycle()
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

  /**
   * @brief Reports a human-readable status line about what this mitigation is doing, on both
   * rosout (at \p level) and RecoveryManagerNode's "mitigation" topic.
   *
   * Call this instead of RCLCPP_* directly from on_start()/on_cycle(), so both channels always
   * carry the same content from one call site. Only the single latest report is kept between
   * RecoveryManagerNode cycles (see MitigationReport), so a mitigation that would otherwise
   * report every cycle must throttle itself, as it would with RCLCPP_*_THROTTLE.
   *
   * @param nav_state Navigation state to queue the report into.
   * @param level One of rcl_interfaces::msg::Log's level constants (DEBUG/INFO/WARN/ERROR/FATAL).
   * @param msg Human-readable message, already fully formatted (no printf-style varargs).
   */
  void report(NavState & nav_state, uint8_t level, const std::string & msg);
};

}  // namespace easynav

#endif  // EASYNAV_CORE__RECOVERYMITIGATIONBASE_HPP_
