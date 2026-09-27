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

#include <cstdint>
#include <optional>
#include <string>

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"
#include "easynav_core/VelocityCommand.hpp"

namespace easynav
{

/**
 * @class SafetyReflexBase
 * @brief Base class for level-0 (real-time) safety reflexes.
 *
 * A reflex is checked by RecoveryManagerNode on every RT cycle, right before the velocity
 * command is published, against the command about to be sent (commanded_velocity()), whether it
 * comes from the nominal controller or from a movement recovery mitigation: this is what lets a
 * single, small, independently verifiable component gate every current and future source of
 * velocity. Its intervention (override_velocity()/stop_robot()) has the highest priority and is
 * published as is, without smoothing. Unlike other MethodBase-derived plugins, a reflex is not
 * rate-limited by MethodBase::isTime2RunRT() — the RT loop already controls the overall rate —
 * and its own failure is treated as unsafe: if check() or mitigate() throws, the
 * robot is stopped as a fail-safe default instead of assuming the reflex is inactive.
 *
 * A reflex also reports its own severity level (OK/WARN/ERROR) under "diagnostics.<plugin_name>"
 * in the shared "diagnostics" group of NavState, so a future evaluator can notice a reflex
 * triggering repeatedly and escalate to a deliberative mitigation. To keep this cheap on the RT
 * path, it is only written when the severity level actually changes, not on every cycle.
 */
class SafetyReflexBase : public MethodBase
{
public:
  SafetyReflexBase() = default;
  virtual ~SafetyReflexBase() = default;

  /**
   * @brief Runs one RT cycle of this reflex without letting check()/mitigate() escape.
   *
   * @param nav_state Current navigation state.
   * @return True if the reflex overrode the velocity command this cycle.
   */
  bool internal_check_and_mitigate(NavState & nav_state);

protected:
  /**
   * @brief Decides whether mitigate() must run this cycle.
   * @param nav_state Current navigation state.
   * @return True if the situation requires overriding the velocity command.
   */
  virtual bool check(NavState & nav_state) = 0;

  /**
   * @brief Applies the safety intervention, typically overriding the command with
   * override_velocity() or stop_robot().
   * @param nav_state Navigation state to modify.
   */
  virtual void mitigate(NavState & nav_state) = 0;

  /**
   * @brief The velocity about to be commanded this cycle, before any reflex: the control-owning
   * mitigation's proposal if there is one, otherwise the controller's. What check() should
   * evaluate.
   */
  std::optional<geometry_msgs::msg::TwistStamped> commanded_velocity(
    const NavState & nav_state) const;

  /// @brief Overrides this cycle's command with \p cmd: highest priority, published as is
  /// (not smoothed: an emergency may need more deceleration than the nominal limits).
  void override_velocity(NavState & nav_state, const geometry_msgs::msg::TwistStamped & cmd);

  /// @brief Fail-safe default: overrides the command with a zero velocity.
  void stop_robot(NavState & nav_state);

private:
  /**
   * @brief Publishes (or overwrites) this reflex's entry in the shared "diagnostics" group,
   * but only if \p level differs from the last level reported by this reflex.
   *
   * Mirrors RecoveryEvaluatorBase::publish_diagnostic()'s key/group convention
   * ("diagnostics.<plugin_name>", membership in the "diagnostics" group), edge-triggered
   * instead of every-cycle so it stays cheap on the RT path.
   *
   * @param nav_state Navigation state to write to.
   * @param level A diagnostic_msgs::msg::DiagnosticStatus level (OK/WARN/ERROR/STALE).
   * @param message Human-readable explanation for this level.
   */
  void report_diagnostic(NavState & nav_state, uint8_t level, const std::string & message);

  std::optional<uint8_t> last_reported_level_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__SAFETYREFLEXBASE_HPP_
