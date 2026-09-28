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
/// \brief Implementation of DefaultRecoveryManager.

#include <algorithm>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "easynav_recovery/DefaultRecoveryManager.hpp"

namespace easynav
{

namespace
{

/// @brief Keeps a plugin loader (and so its libraries) alive until the process exits: plugins
/// leave code of their library in global state, see PluginSwitcher::keep_libraries_loaded().
void keep_libraries_loaded(std::shared_ptr<void> loader)
{
  static auto * const holder = new std::vector<std::shared_ptr<void>>();
  static std::mutex holder_mutex;
  std::lock_guard<std::mutex> lock(holder_mutex);
  holder->push_back(std::move(loader));
}

}  // namespace

DefaultRecoveryManager::~DefaultRecoveryManager()
{
  // Instances first: their code lives in the libraries the loaders keep open.
  active_mitigation_.reset();
  mitigations_.clear();
  evaluators_.clear();
  safety_reflexes_.clear();
  keep_libraries_loaded(std::shared_ptr<void>(std::move(safety_reflex_loader_)));
  keep_libraries_loaded(std::shared_ptr<void>(std::move(evaluator_loader_)));
  keep_libraries_loaded(std::shared_ptr<void>(std::move(mitigation_loader_)));
}

void
DefaultRecoveryManager::on_initialize()
{
  auto node = get_node();

  safety_reflex_loader_ = std::make_unique<pluginlib::ClassLoader<SafetyReflexBase>>(
    "easynav_core", "easynav::SafetyReflexBase");
  evaluator_loader_ = std::make_unique<pluginlib::ClassLoader<RecoveryEvaluatorBase>>(
    "easynav_core", "easynav::RecoveryEvaluatorBase");
  mitigation_loader_ = std::make_unique<pluginlib::ClassLoader<RecoveryMitigationBase>>(
    "easynav_core", "easynav::RecoveryMitigationBase");

  // Standard ROS 2 diagnostics topic, so existing tooling (rqt_robot_monitor,
  // diagnostic_aggregator, ...) can consume what lives inside NavState's "diagnostics" group.
  diagnostics_pub_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  // rcl_interfaces/msg/Log is the same message /rosout already carries — reused here so a
  // mitigation's narrative has a dedicated, low-noise topic instead of the busier /rosout.
  mitigation_pub_ = node->create_publisher<rcl_interfaces::msg::Log>("mitigation", 10);

  NavState::register_printer<diagnostic_msgs::msg::DiagnosticStatus>(
    [](const diagnostic_msgs::msg::DiagnosticStatus & status) {
      std::ostringstream ret;
      switch (status.level) {
        case diagnostic_msgs::msg::DiagnosticStatus::OK: ret << "OK"; break;
        case diagnostic_msgs::msg::DiagnosticStatus::WARN: ret << "WARN"; break;
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR: ret << "ERROR"; break;
        case diagnostic_msgs::msg::DiagnosticStatus::STALE: ret << "STALE"; break;
        default: ret << "UNKNOWN(" << static_cast<int>(status.level) << ")"; break;
      }
      ret << " [" << status.name << "]";
      if (!status.hardware_id.empty()) {
        ret << " (" << status.hardware_id << ")";
      }
      ret << ": " << status.message;
      if (!status.values.empty()) {
        ret << " {";
        for (size_t i = 0; i < status.values.size(); ++i) {
          if (i > 0) {ret << ", ";}
          ret << status.values[i].key << "=" << status.values[i].value;
        }
        ret << "}";
      }
      return ret.str();
    });

  load_reflexes();
  load_evaluators();
  load_mitigations();
}

namespace
{

/// @brief Parameters stay declared after a cleanup (rclcpp cannot undeclare them), so a second
/// configure must not declare them again.
template<typename T>
T declare_and_get(rclcpp_lifecycle::LifecycleNode & node, const std::string & name, T value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, value);
  }
  node.get_parameter(name, value);
  return value;
}

/// @brief Loads and initializes one plugin, throwing std::runtime_error if it cannot.
template<typename PluginT>
std::shared_ptr<PluginT> load_plugin(
  pluginlib::ClassLoader<PluginT> & loader, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & kind, const std::string & name, const std::string & plugin)
{
  RCLCPP_INFO(
    node->get_logger(), "Loading %s %s [%s]", kind.c_str(), name.c_str(), plugin.c_str());
  std::shared_ptr<PluginT> instance;
  try {
    instance = loader.createSharedInstance(plugin);
  } catch (pluginlib::PluginlibException & ex) {
    throw std::runtime_error("Unable to load plugin easynav::" + kind + ": " + ex.what());
  }
  try {
    instance->initialize(node, name);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Unable to initialize [" + plugin + "]: " + e.what());
  }
  RCLCPP_INFO(
    node->get_logger(), "Loaded %s %s [%s]", kind.c_str(), name.c_str(), plugin.c_str());
  return instance;
}

}  // namespace

void
DefaultRecoveryManager::load_reflexes()
{
  auto node = get_node();
  const auto prefix = get_plugin_name() + ".";
  const auto types =
    declare_and_get(*node, prefix + "safety_reflex_types", std::vector<std::string>{});
  for (const auto & type : types) {
    const auto plugin = declare_and_get(*node, prefix + type + ".plugin", std::string());
    safety_reflexes_.push_back(
      load_plugin(*safety_reflex_loader_, node, "SafetyReflexBase", prefix + type, plugin));
  }
}

void
DefaultRecoveryManager::load_evaluators()
{
  auto node = get_node();
  const auto prefix = get_plugin_name() + ".";
  const auto types =
    declare_and_get(*node, prefix + "evaluator_types", std::vector<std::string>{});
  for (const auto & type : types) {
    const auto plugin = declare_and_get(*node, prefix + type + ".plugin", std::string());
    evaluators_.push_back(
      load_plugin(*evaluator_loader_, node, "RecoveryEvaluatorBase", prefix + type, plugin));
  }
}

void
DefaultRecoveryManager::load_mitigations()
{
  auto node = get_node();
  const auto prefix = get_plugin_name() + ".";
  const auto types =
    declare_and_get(*node, prefix + "mitigation_types", std::vector<std::string>{});

  // (priority, plugin) pairs, sorted below. Priority is an arbitration detail this manager
  // owns, not something a mitigation plugin needs to know about itself.
  std::vector<std::pair<int, std::shared_ptr<RecoveryMitigationBase>>> loaded;
  for (const auto & type : types) {
    const auto plugin = declare_and_get(*node, prefix + type + ".plugin", std::string());
    const int priority = declare_and_get(*node, prefix + type + ".priority", 100);
    loaded.emplace_back(
      priority,
      load_plugin(*mitigation_loader_, node, "RecoveryMitigationBase", prefix + type, plugin));
    RCLCPP_INFO(node->get_logger(), "Mitigation %s has priority %d", type.c_str(), priority);
  }

  // Lower priority number = tried first; stable_sort keeps "mitigation_types" order on ties.
  std::stable_sort(
    loaded.begin(), loaded.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
  for (auto & [priority, mitigation] : loaded) {
    mitigations_.push_back(mitigation);
  }
}

void
DefaultRecoveryManager::on_activate()
{
  for (auto & evaluator : evaluators_) {
    evaluator->on_activate();
  }
}

void
DefaultRecoveryManager::on_deactivate()
{
  for (auto & evaluator : evaluators_) {
    evaluator->on_deactivate();
  }
}

void
DefaultRecoveryManager::reset_shared_state_on_first_cycle(NavState & nav_state)
{
  if (!first_cycle_.exchange(false)) {
    return;
  }

  const std::string key = "recovery.default_manager.instance";
  const bool replaces_another = nav_state.has(key) && nav_state.get<uint64_t>(key) != instance_id_;
  nav_state.set(key, instance_id_);
  if (replaces_another) {
    nav_state.set("control_owner", std::string("controller"));
    nav_state.set_group("diagnostics", std::vector<std::string>{});
  }
}

void
DefaultRecoveryManager::update(NavState & nav_state)
{
  reset_shared_state_on_first_cycle(nav_state);

  for (auto & evaluator : evaluators_) {
    evaluator->internal_update(nav_state);
  }

  if (active_mitigation_) {
    // Only cycle it here if it does not require control (control-owning mitigations are
    // driven by update_rt() instead). Either way, do not attempt a new selection in the same
    // cycle a mitigation just ran or just stopped: the diagnostic that triggered it may still
    // read non-OK until the evaluator that owns it runs again and re-assesses, and selecting
    // again immediately would just restart the same mitigation in a tight loop.
    if (!active_mitigation_->requires_control()) {
      RecoveryStatus status = active_mitigation_->internal_cycle(nav_state);
      if (status != RecoveryStatus::RUNNING) {
        if (status == RecoveryStatus::FAILED) {
          excluded_mitigations_[active_diagnostic_key_].insert(
            active_mitigation_->get_plugin_name());
        }
        active_mitigation_->internal_stop(nav_state);
        active_mitigation_.reset();
      }
    }
  } else {
    try_select_mitigation(nav_state);
  }

  handle_system_requests(nav_state);
  update_mission_hold(nav_state);
  publish_diagnostics(nav_state);
  publish_mitigation_log(nav_state);
}

bool
DefaultRecoveryManager::update_rt(NavState & nav_state)
{
  reset_shared_state_on_first_cycle(nav_state);

  bool commanded = false;

  // 1. A control-owning mitigation commands the robot instead of the controller.
  if (active_mitigation_ && active_mitigation_->requires_control()) {
    RecoveryStatus status = active_mitigation_->internal_cycle(nav_state);
    if (status != RecoveryStatus::RUNNING) {
      if (status == RecoveryStatus::FAILED) {
        excluded_mitigations_[active_diagnostic_key_].insert(
          active_mitigation_->get_plugin_name());
      }
      active_mitigation_->internal_stop(nav_state);
      active_mitigation_.reset();
      nav_state.set("control_owner", std::string("controller"));
    }
    commanded = true;
  }

  // 2. Level-0 safety reflexes: every RT cycle, against the command about to be sent (the
  // mitigation's or the controller's), right before it is published.
  for (auto & reflex : safety_reflexes_) {
    if (reflex->internal_check_and_mitigate(nav_state)) {
      commanded = true;
    }
  }

  return commanded;
}

void
DefaultRecoveryManager::handle_system_requests(NavState & nav_state)
{
  // CancelMissionRecovery (and ShutdownRecovery) ask for the mission to be cancelled.
  if (nav_state.has("mission_cancel_requested") &&
    nav_state.get<bool>("mission_cancel_requested"))
  {
    std::string reason;
    for (const auto & key : nav_state.get_group_keys("diagnostics")) {
      if (!nav_state.has(key)) {continue;}
      // Some entries (e.g. a SafetyReflexBase's) are written from the RT cycle: get_safe().
      const auto status = nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>(key);
      if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
        if (!reason.empty()) {reason += "; ";}
        reason += key + " (" + status.message + ")";
      }
    }
    if (reason.empty()) {reason = "mission cancelled by recovery escalation";}

    abort_mission(reason);
    nav_state.set("mission_cancel_requested", false);
  }

  // ShutdownRecovery asks EasyNav to terminate (after the mission was cancelled above).
  if (!shutdown_requested_ && nav_state.has("system_shutdown_requested") &&
    nav_state.get<bool>("system_shutdown_requested"))
  {
    shutdown_requested_ = true;
    request_shutdown(
      nav_state.has("system_shutdown_reason") ?
      nav_state.get<std::string>("system_shutdown_reason") :
      std::string("unrecoverable diagnostic"));
  }
}

void
DefaultRecoveryManager::update_mission_hold(NavState & nav_state)
{
  // While recovering (a mitigation running, or a diagnostic in ERROR that none resolved), the
  // robot pose may be wrong (e.g. AMCL diverged), so arriving at the goal must not count yet.
  bool hold = static_cast<bool>(active_mitigation_);
  if (!hold) {
    for (const auto & key : nav_state.get_group_keys("diagnostics")) {
      if (!nav_state.has(key)) {continue;}
      // Some entries (e.g. a SafetyReflexBase's) are written from the RT cycle: get_safe().
      const auto status = nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>(key);
      if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
        hold = true;
        break;
      }
    }
  }

  if (hold != mission_progress_held_ || first_hold_update_) {
    hold_mission_progress(hold);
    mission_progress_held_ = hold;
    first_hold_update_ = false;
  }
}

void
DefaultRecoveryManager::publish_diagnostics(NavState & nav_state)
{
  if (diagnostics_pub_->get_subscription_count() == 0) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->now();

  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {continue;}
    // Some entries (e.g. a SafetyReflexBase's) are written from the RT cycle: get_safe().
    array.status.push_back(nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>(key));
  }

  diagnostics_pub_->publish(array);
}

void
DefaultRecoveryManager::publish_mitigation_log(NavState & nav_state)
{
  if (!nav_state.has("mitigation.pending_report")) {
    return;
  }

  // Single overwritten slot, not a queue: seq tells "unchanged since last cycle" apart from
  // "a new report arrived".
  const auto report_entry = nav_state.get_safe<MitigationReport>("mitigation.pending_report");
  if (report_entry.seq == last_published_report_seq_) {
    return;
  }
  last_published_report_seq_ = report_entry.seq;

  if (mitigation_pub_->get_subscription_count() == 0) {
    return;
  }
  mitigation_pub_->publish(report_entry.log);
}

void
DefaultRecoveryManager::publish_mitigation_resolved(const std::string & key)
{
  if (mitigation_pub_->get_subscription_count() == 0) {
    return;
  }

  rcl_interfaces::msg::Log msg;
  msg.stamp = get_node()->now();
  // DEBUG is never used by report() itself, so subscribers can treat it as an unambiguous
  // "clear your log" sentinel instead of parsing message text.
  msg.level = rcl_interfaces::msg::Log::DEBUG;
  msg.name = get_node()->get_name();
  msg.msg = key + " resuelto — mitigación finalizada";
  mitigation_pub_->publish(msg);
}

void
DefaultRecoveryManager::try_select_mitigation(NavState & nav_state)
{
  if (active_mitigation_) {
    return;
  }

  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {
      continue;
    }
    const auto & status = nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key);
    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      // Resolved: forget which mitigations were already excluded for it, so a future
      // recurrence of this diagnostic starts escalation from the first candidate again.
      excluded_mitigations_.erase(key);
      // Only announce "resolved" if a mitigation actually ran for this key at some point.
      if (keys_with_mitigation_history_.erase(key) > 0) {
        publish_mitigation_resolved(key);
      }
      continue;
    }

    const auto & excluded_for_key = excluded_mitigations_[key];
    for (auto & mitigation : mitigations_) {
      if (!mitigation->can_handle(status)) {
        continue;
      }
      if (excluded_for_key.count(mitigation->get_plugin_name()) > 0) {
        // Already gave up on this diagnostic: let the next applicable candidate try instead.
        continue;
      }

      RCLCPP_INFO(
        get_node()->get_logger(), "Selecting mitigation [%s] for diagnostic [%s]",
        mitigation->get_plugin_name().c_str(), key.c_str());

      active_mitigation_ = mitigation;
      active_diagnostic_key_ = key;
      keys_with_mitigation_history_.insert(key);
      active_mitigation_->internal_start(nav_state);
      if (active_mitigation_->requires_control()) {
        nav_state.set(
          "control_owner", std::string("recovery:") + active_mitigation_->get_plugin_name());
      }
      return;
    }
  }
}

std::string
DefaultRecoveryManager::get_active_mitigation_name() const
{
  return active_mitigation_ ? active_mitigation_->get_plugin_name() : std::string();
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DefaultRecoveryManager, easynav::RecoveryManagerBase)
