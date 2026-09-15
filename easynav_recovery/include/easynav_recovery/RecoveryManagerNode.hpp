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

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rcl_interfaces/msg/log.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/RecoveryEvaluatorBase.hpp"
#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class RecoveryManagerNode
 * @brief ROS 2 lifecycle node hosting level-1 (deliberative, non-RT) recovery evaluators.
 *
 * Owned by SystemNode and cycled on its non-RT loop, exactly like PlannerNode/MapsManagerNode:
 * evaluators need to read NavState with minimal cost and no IPC latency, which requires living
 * in the same process as the rest of the navigation stack.
 *
 * Unlike ControllerNode/PlannerNode (which host exactly one active plugin instance),
 * RecoveryManagerNode loads and runs every configured RecoveryEvaluatorBase plugin — recovery
 * evaluation is composed from several independent, narrowly-scoped diagnoses.
 *
 * It also loads RecoveryMitigationBase plugins ("mitigation_types") and arbitrates between
 * them: at most one is active at a time. Selection scans the first non-OK diagnostic (in
 * whatever order NavState's "diagnostics" group returns) and picks the first loaded mitigation,
 * in priority order, whose can_handle() accepts it, skipping any mitigation already excluded
 * for that specific diagnostic key (see excluded_mitigations_ below). Priority is configured
 * per mitigation instance via "<mitigation_type>.priority" (lower number tried first, default
 * 100, ties broken by "mitigation_types" list order). A mitigation that requires_control() is
 * cycled from cycle_rt() (RT rate, drives "cmd_vel" via "control_owner"); one that does not is
 * cycled from cycle() (non-RT rate) instead.
 *
 * It also republishes what mitigations report doing (RecoveryMitigationBase::report()) on the
 * "mitigation" topic, plus a "resolved" sentinel once a diagnostic that had an active mitigation
 * clears — see publish_mitigation_log()/publish_mitigation_resolved().
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
  /// @brief Publishes the current "diagnostics" group as a standard DiagnosticArray, so
  /// tooling that expects the real /diagnostics topic (rqt_robot_monitor,
  /// diagnostic_aggregator, the EasyNav TUI) can consume it directly.
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  /// @brief Publishes what mitigations report doing via RecoveryMitigationBase::report(), plus
  /// this node's own "resolved" sentinel (see keys_with_mitigation_history_ below) — an
  /// append-only narrative feed, distinct from /diagnostics' per-key current-state snapshot.
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr mitigation_pub_;

  /// @brief Highest MitigationReport::seq already published to "mitigation", so
  /// publish_mitigation_log() only republishes a report once, however many cycles it lingers in
  /// NavState before the next one overwrites it.
  uint64_t last_published_report_seq_ {0};

  /// @brief Diagnostic keys for which a mitigation has actually been selected/started at least
  /// once since that key last cleared to OK — so try_select_mitigation() only announces
  /// "resolved" on "mitigation" for keys that were actually mitigated (excluded_mitigations_
  /// alone is not enough: it only tracks FAILED attempts, not a first-try SUCCEEDED).
  std::unordered_set<std::string> keys_with_mitigation_history_;

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

  /// @brief "diagnostics" key that selected active_mitigation_, so a FAILED outcome can be
  /// recorded against the right key in excluded_mitigations_ below.
  std::string active_diagnostic_key_;

  /// @brief Mitigations already tried and given up on (RecoveryStatus::FAILED), per diagnostic
  /// key, so try_select_mitigation() moves on to the next applicable candidate instead of
  /// reselecting the same one in a tight loop. An entry is dropped as soon as its diagnostic is
  /// observed OK again, so a future recurrence starts escalation from the first candidate.
  std::unordered_map<std::string, std::unordered_set<std::string>> excluded_mitigations_;

  /// @brief Attempts to select and start a mitigation for the current diagnostics, if none is
  /// already active. Extracted from cycle() for readability/testability.
  void try_select_mitigation(NavState & nav_state);

  /// @brief Publishes the current "diagnostics" group to /diagnostics as a DiagnosticArray, if
  /// there is at least one subscriber. Called once at the end of every cycle().
  void publish_diagnostics(NavState & nav_state);

  /// @brief Publishes the latest pending RecoveryMitigationBase::report() to "mitigation", if
  /// it has not been published yet. Called only from cycle(), never cycle_rt(), so a
  /// control-owning mitigation's report never triggers a publish from the RT thread.
  void publish_mitigation_log(NavState & nav_state);

  /// @brief Publishes a "resolved" sentinel (level DEBUG — never used by report() itself) to
  /// "mitigation" for \p key, so subscribers know to clear their running log.
  void publish_mitigation_resolved(const std::string & key);
};

}  // namespace easynav

#endif  // EASYNAV_RECOVERY__RECOVERYMANAGERNODE_HPP_
