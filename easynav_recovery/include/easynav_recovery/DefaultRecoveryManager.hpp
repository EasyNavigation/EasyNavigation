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
/// \brief Declaration of DefaultRecoveryManager, EasyNav's default recovery system.

#ifndef EASYNAV_RECOVERY__DEFAULTRECOVERYMANAGER_HPP_
#define EASYNAV_RECOVERY__DEFAULTRECOVERYMANAGER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rcl_interfaces/msg/log.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/RecoveryEvaluatorBase.hpp"
#include "easynav_core/RecoveryManagerBase.hpp"
#include "easynav_core/RecoveryMitigationBase.hpp"
#include "easynav_core/SafetyReflexBase.hpp"

namespace easynav
{

/**
 * @class DefaultRecoveryManager
 * @brief EasyNav's default recovery system (plugin easynav_recovery/DefaultRecoveryManager).
 *
 * Two levels, each made of plugins. Its parameters, like any plugin's, live under its own name
 * ("recovery_manager.*" in recovery_node), and so do those of the plugins it loads, whose names
 * are "recovery_manager.<type>" (e.g. diagnostics are "diagnostics.recovery_manager.<type>"):
 * - Level 0, RT: SafetyReflexBase plugins ("safety_reflex_types"), checked every RT cycle against
 *   the command about to be sent, overriding it on imminent danger.
 * - Level 1, non-RT: RecoveryEvaluatorBase plugins ("evaluator_types") write diagnostics to the
 *   NavState "diagnostics" group; RecoveryMitigationBase plugins ("mitigation_types") handle
 *   them, arbitrated here: at most one is active at a time. Selection scans the first non-OK
 *   diagnostic and picks the first loaded mitigation, in priority order, whose can_handle()
 *   accepts it, skipping any mitigation already excluded for that specific diagnostic key.
 *   Priority is "<mitigation_type>.priority" (lower tried first, default 100, ties broken by
 *   "mitigation_types" order). A mitigation that requires_control() runs from update_rt() and
 *   holds "control_owner"; one that does not runs from update().
 *
 * Its mitigations talk to it through NavState signals it translates into SystemActions:
 * "mission_cancel_requested" (see CancelMissionRecovery) into abort_mission(), and
 * "system_shutdown_requested"/"system_shutdown_reason" (see ShutdownRecovery) into
 * request_shutdown(). No other component knows about those keys, nor about "control_owner".
 * While a mitigation is active or a diagnostic is in ERROR, it holds the mission's progress
 * (hold_mission_progress()), so a goal is not taken as reached from an untrusted robot pose.
 *
 * It also publishes the diagnostics on "diagnostics" (diagnostic_msgs/DiagnosticArray) and what
 * mitigations report doing on "mitigation" (plus a "resolved" sentinel once a diagnostic that
 * had an active mitigation clears).
 */
class DefaultRecoveryManager : public RecoveryManagerBase
{
public:
  DefaultRecoveryManager() = default;
  ~DefaultRecoveryManager() override;

  /// @brief Loads the reflex, evaluator and mitigation plugins. Throws if any fails.
  void on_initialize() override;

  void on_activate() override;
  void on_deactivate() override;

  /// @brief Number of loaded safety reflex plugins. For testing.
  [[nodiscard]] size_t get_num_safety_reflexes() const {return safety_reflexes_.size();}

  /// @brief Number of loaded evaluator plugins. For testing.
  [[nodiscard]] size_t get_num_evaluators() const {return evaluators_.size();}

  /// @brief Number of loaded mitigation plugins. For testing.
  [[nodiscard]] size_t get_num_mitigations() const {return mitigations_.size();}

  /// @brief Plugin name of the active mitigation, or empty if none. For testing.
  [[nodiscard]] std::string get_active_mitigation_name() const;

protected:
  void update(NavState & nav_state) override;
  bool update_rt(NavState & nav_state) override;

private:
  void load_reflexes();
  void load_evaluators();
  void load_mitigations();

  /// @brief On the first cycle of this instance: if another instance ran with this NavState
  /// before (EasyNav was reconfigured), the state it left is stale: control back to the
  /// controller, diagnostics dropped (the loaded evaluators re-register).
  void reset_shared_state_on_first_cycle(NavState & nav_state);

  /// @brief Translates the mitigations' NavState signals into SystemActions.
  void handle_system_requests(NavState & nav_state);

  /// @brief Holds the mission's progress while recovering: a mitigation is active or a
  /// diagnostic is in ERROR (SystemActions::hold_mission_progress()).
  void update_mission_hold(NavState & nav_state);

  void try_select_mitigation(NavState & nav_state);
  void publish_diagnostics(NavState & nav_state);
  void publish_mitigation_log(NavState & nav_state);
  void publish_mitigation_resolved(const std::string & key);

  std::unique_ptr<pluginlib::ClassLoader<SafetyReflexBase>> safety_reflex_loader_;
  std::unique_ptr<pluginlib::ClassLoader<RecoveryEvaluatorBase>> evaluator_loader_;
  std::unique_ptr<pluginlib::ClassLoader<RecoveryMitigationBase>> mitigation_loader_;

  std::vector<std::shared_ptr<SafetyReflexBase>> safety_reflexes_;
  std::vector<std::shared_ptr<RecoveryEvaluatorBase>> evaluators_;
  std::vector<std::shared_ptr<RecoveryMitigationBase>> mitigations_;

  std::shared_ptr<RecoveryMitigationBase> active_mitigation_;

  /// @brief "diagnostics" key that selected active_mitigation_, so a FAILED outcome can be
  /// recorded against the right key in excluded_mitigations_.
  std::string active_diagnostic_key_;

  /// @brief Per diagnostic key, mitigations that already FAILED for it: skipped until the key
  /// clears to OK, so escalation moves on to the next applicable candidate.
  std::unordered_map<std::string, std::unordered_set<std::string>> excluded_mitigations_;

  /// @brief Keys a mitigation ran for since they last cleared to OK, to announce "resolved".
  std::unordered_set<std::string> keys_with_mitigation_history_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr mitigation_pub_;
  uint64_t last_published_report_seq_ {0};

  std::atomic<bool> first_cycle_ {true};

  /// @brief Identifies this instance in NavState ("recovery.default_manager.instance").
  const uint64_t instance_id_ {next_instance_id_++};
  static inline std::atomic<uint64_t> next_instance_id_ {1};
  bool shutdown_requested_ {false};
  bool mission_progress_held_ {false};
  /// @brief The first update always sets the hold: a previous instance may have left it held.
  bool first_hold_update_ {true};
};

}  // namespace easynav

#endif  // EASYNAV_RECOVERY__DEFAULTRECOVERYMANAGER_HPP_
