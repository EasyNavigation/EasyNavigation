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
/// \brief Declaration of the RecoveryManagerNode class, EasyNav's level-1 (deliberative,
/// non-RT) diagnosis coordinator.

#ifndef EASYNAV_RECOVERY__RECOVERYMANAGERNODE_HPP_
#define EASYNAV_RECOVERY__RECOVERYMANAGERNODE_HPP_

#include <unordered_map>

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/RecoveryEvaluatorBase.hpp"
#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class RecoveryManagerNode
 * @brief ROS 2 lifecycle node hosting level-1 (deliberative, non-RT) recovery evaluators.
 *
 * See docs/recoveries_easynav.md, level 1. Owned by SystemNode and cycled on its non-RT loop,
 * exactly like PlannerNode/MapsManagerNode — this is a deliberate placement, not an
 * afterthought: evaluators need to read NavState with minimal cost and no IPC latency, which
 * requires living in the same process as the rest of the navigation stack.
 *
 * Unlike ControllerNode/PlannerNode (which host exactly one active plugin instance),
 * RecoveryManagerNode loads and runs every configured RecoveryEvaluatorBase plugin — recovery
 * evaluation is meant to be composed from several independent, narrowly-scoped diagnoses
 * rather than a single monolithic one.
 *
 * It also loads RecoveryMitigationBase plugins ("mitigation_types") and arbitrates between
 * them: at most one is active at a time. Selection here is deliberately simple — the first
 * non-OK diagnostic (in whatever order NavState's "diagnostics" group returns) whose
 * can_handle() a loaded mitigation accepts is selected, skipping candidates that have already
 * had "max_attempts_per_mitigation" turns for that specific diagnostic since it was last OK
 * (see docs/recoveries_easynav_implementation.md, Fase 4 — this is a partial step towards the
 * full priority/cooldown table of recoveries_easynav.md §5.6 point 4: it lets a second
 * candidate escalate after the first one has had its chances for one occurrence of a
 * diagnostic, but there is still no configurable priority order, no real time-based cooldown,
 * and no reasoning across different diagnostic codes). A mitigation that
 * requires_control() is cycled from cycle_rt() (RT rate, drives "cmd_vel" via "control_owner");
 * one that does not is cycled from cycle() (non-RT rate) instead.
 */
class RecoveryManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RecoveryManagerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Optional node configuration.
   */
  explicit RecoveryManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~RecoveryManagerNode();

  /**
   * @brief Configure the node: loads the plugins listed in the "evaluator_types" parameter.
   * @param state Current lifecycle state.
   * @return SUCCESS if every configured evaluator plugin loaded and initialized correctly.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if activation succeeded.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if deactivation succeeded.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Clean up the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if cleanup succeeded.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if shutdown succeeded.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle lifecycle transition errors.
   * @param state Current lifecycle state.
   * @return SUCCESS if error handling completed.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Runs one non-RT cycle: evaluators, then mitigation selection/arbitration.
   *
   * Order: (1) every loaded evaluator's internal_update(); (2) if a mitigation is already
   * active, cycle it here (only if it does not require control — control-owning ones are
   * cycled by cycle_rt() instead) and return, without attempting a new selection this same
   * cycle; (3) only if nothing was active at the start of this cycle, scan "diagnostics" for a
   * non-OK entry a loaded mitigation can_handle(), and select it.
   *
   * @param nav_state Shared navigation state.
   */
  void cycle(std::shared_ptr<NavState> nav_state);

  /**
   * @brief Runs one RT cycle: if a control-owning mitigation is active, its internal_cycle().
   *
   * On SUCCEEDED/FAILED, stops the mitigation and resets "control_owner" to "controller".
   *
   * @param nav_state Shared navigation state.
   * @return True if a mitigation produced "cmd_vel" this cycle (so SystemNode should publish
   * it), false if there is no active control-owning mitigation.
   */
  bool cycle_rt(std::shared_ptr<NavState> nav_state);

  /**
   * @brief Number of currently loaded evaluator plugins. Exposed mainly for testing.
   */
  [[nodiscard]] size_t get_num_evaluators() const {return evaluators_.size();}

  /**
   * @brief Number of currently loaded mitigation plugins. Exposed mainly for testing.
   */
  [[nodiscard]] size_t get_num_mitigations() const {return mitigations_.size();}

  /**
   * @brief Plugin name of the currently active mitigation, or empty if none. For testing.
   */
  [[nodiscard]] std::string get_active_mitigation_name() const;

private:
  /// @brief Plugin loader for recovery evaluators.
  std::unique_ptr<pluginlib::ClassLoader<RecoveryEvaluatorBase>> evaluator_loader_;

  /// @brief Loaded evaluator plugins, all run every cycle.
  std::vector<std::shared_ptr<RecoveryEvaluatorBase>> evaluators_;

  /// @brief Plugin loader for recovery mitigations.
  std::unique_ptr<pluginlib::ClassLoader<RecoveryMitigationBase>> mitigation_loader_;

  /// @brief Loaded mitigation plugins, candidates for selection.
  std::vector<std::shared_ptr<RecoveryMitigationBase>> mitigations_;

  /// @brief Currently active mitigation, or nullptr if none.
  std::shared_ptr<RecoveryMitigationBase> active_mitigation_;

  /// @brief Maximum number of times the same mitigation may be selected for the same
  /// diagnostic (identified by its NavState key) before try_select_mitigation() skips it in
  /// favor of the next applicable candidate, if any. See the class doc comment.
  int max_attempts_per_mitigation_ {1};

  /// @brief Per-diagnostic-key attempt counts, keyed by mitigation plugin name. An entry is
  /// dropped as soon as its diagnostic is observed OK again, so a future recurrence of the
  /// same diagnostic starts escalation from the first applicable mitigation again.
  std::unordered_map<std::string, std::unordered_map<std::string, int>> attempt_counts_;

  /// @brief Attempts to select and start a mitigation for the current diagnostics, if none is
  /// already active. Extracted from cycle() for readability/testability.
  void try_select_mitigation(NavState & nav_state);
};

}  // namespace easynav

#endif  // EASYNAV_RECOVERY__RECOVERYMANAGERNODE_HPP_
