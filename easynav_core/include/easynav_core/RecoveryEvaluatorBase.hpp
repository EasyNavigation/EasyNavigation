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
/// \brief Declaration of the abstract base class RecoveryEvaluatorBase.

#ifndef EASYNAV_CORE__RECOVERYEVALUATORBASE_HPP_
#define EASYNAV_CORE__RECOVERYEVALUATORBASE_HPP_

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class RecoveryEvaluatorBase
 * @brief Base class for level-1 (deliberative, non-RT) diagnosis plugins.
 *
 * An evaluator only reads NavState — it never writes "cmd_vel" or otherwise acts on the robot;
 * deciding and acting on a diagnosis is the responsibility of RecoveryMitigationBase plugins and
 * RecoveryManagerNode, not the evaluator itself. Loaded and run by RecoveryManagerNode on its
 * non-RT cycle.
 *
 * Every call to update() is expected to call publish_diagnostic() with the evaluator's
 * *current* assessment, including a benign one (e.g. diagnostic_msgs::msg::DiagnosticStatus::OK)
 * once whatever it was reporting has been resolved. Diagnostics are not accumulated: publishing
 * again under the same plugin name overwrites the previous entry, so a resolved condition does
 * not linger forever in the shared "diagnostics" group.
 */
class RecoveryEvaluatorBase : public MethodBase
{
public:
  RecoveryEvaluatorBase() = default;
  virtual ~RecoveryEvaluatorBase() = default;

  /**
   * @brief Runs one non-RT cycle of this evaluator if it is due, without letting update()
   * escape or crash the process.
   *
   * @param nav_state Current navigation state.
   */
  void internal_update(NavState & nav_state);

protected:
  /**
   * @brief Reads nav_state and, if appropriate, calls publish_diagnostic() with the current
   * assessment. Must not write "cmd_vel" or any other actuation-related key.
   *
   * @param nav_state Current navigation state.
   */
  virtual void update(NavState & nav_state) = 0;

  /**
   * @brief Publishes (or overwrites) this evaluator's entry in the shared "diagnostics" group.
   *
   * Stores \p status under the key "diagnostics.<plugin_name>" and ensures that key is a
   * member of the "diagnostics" group, growing the group's membership list if this is the
   * first time this plugin publishes.
   *
   * @param nav_state Navigation state to write to.
   * @param status The plugin's current diagnostic assessment.
   */
  void publish_diagnostic(
    NavState & nav_state, const diagnostic_msgs::msg::DiagnosticStatus & status);
};

}  // namespace easynav

#endif  // EASYNAV_CORE__RECOVERYEVALUATORBASE_HPP_
