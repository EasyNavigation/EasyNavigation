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
/// \brief Implementation of the RecoveryManagerNode class.

#include <algorithm>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_recovery/RecoveryManagerNode.hpp"

namespace easynav
{

RecoveryManagerNode::RecoveryManagerNode(const rclcpp::NodeOptions & options)
: LifecycleNode("recovery_node", options)
{
  evaluator_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::RecoveryEvaluatorBase>>(
    "easynav_core", "easynav::RecoveryEvaluatorBase");
  mitigation_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::RecoveryMitigationBase>>(
    "easynav_core", "easynav::RecoveryMitigationBase");

  // Standard ROS 2 diagnostics topic, so existing tooling (rqt_robot_monitor,
  // diagnostic_aggregator, ...) can consume what used to live only inside NavState's internal
  // "diagnostics" group.
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  // rcl_interfaces/msg/Log is the same message /rosout already carries — reused here so a
  // mitigation's narrative has a dedicated, low-noise topic instead of the busier /rosout.
  mitigation_pub_ = create_publisher<rcl_interfaces::msg::Log>("mitigation", 10);

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
}

RecoveryManagerNode::~RecoveryManagerNode()
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }

  evaluators_.clear();
  std::vector<std::string> evaluator_types;
  get_parameter("evaluator_types", evaluator_types);
  for (const auto & evaluator_type : evaluator_types) {
    std::string plugin;
    if (has_parameter(evaluator_type + ".plugin")) {
      get_parameter(evaluator_type + ".plugin", plugin);
      try {
        evaluator_loader_->unloadLibraryForClass(plugin);
      } catch (const std::exception &) {
      }
    }
  }

  active_mitigation_.reset();
  mitigations_.clear();
  excluded_mitigations_.clear();
  std::vector<std::string> mitigation_types;
  get_parameter("mitigation_types", mitigation_types);
  for (const auto & mitigation_type : mitigation_types) {
    std::string plugin;
    if (has_parameter(mitigation_type + ".plugin")) {
      get_parameter(mitigation_type + ".plugin", plugin);
      try {
        mitigation_loader_->unloadLibraryForClass(plugin);
      } catch (const std::exception &) {
      }
    }
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
RecoveryManagerNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> evaluator_types;
  declare_parameter("evaluator_types", evaluator_types);
  get_parameter("evaluator_types", evaluator_types);

  evaluators_.clear();
  for (const auto & evaluator_type : evaluator_types) {
    std::string plugin;
    declare_parameter(evaluator_type + std::string(".plugin"), plugin);
    get_parameter(evaluator_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(
        get_logger(),
        "Loading RecoveryEvaluatorBase %s [%s]", evaluator_type.c_str(), plugin.c_str());

      auto evaluator = evaluator_loader_->createSharedInstance(plugin);

      try {
        evaluator->initialize(shared_from_this(), evaluator_type);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(), "Unable to initialize [%s]. Error: %s", plugin.c_str(), e.what());
        return CallbackReturnT::FAILURE;
      }

      evaluators_.push_back(evaluator);
      RCLCPP_INFO(
        get_logger(), "Loaded RecoveryEvaluatorBase %s [%s]", evaluator_type.c_str(),
        plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Unable to load plugin easynav::RecoveryEvaluatorBase. Error: %s",
        ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  std::vector<std::string> mitigation_types;
  declare_parameter("mitigation_types", mitigation_types);
  get_parameter("mitigation_types", mitigation_types);

  active_mitigation_.reset();
  mitigations_.clear();
  excluded_mitigations_.clear();
  keys_with_mitigation_history_.clear();
  last_published_report_seq_ = 0;

  // (priority, plugin) pairs, sorted below before becoming mitigations_ — see the class doc
  // comment. Declared/read here, per mitigation instance, rather than as a property of
  // RecoveryMitigationBase itself: priority is an arbitration detail this manager owns, not
  // something a mitigation plugin needs to know about itself.
  std::vector<std::pair<int, std::shared_ptr<RecoveryMitigationBase>>> loaded_mitigations;

  for (const auto & mitigation_type : mitigation_types) {
    std::string plugin;
    declare_parameter(mitigation_type + std::string(".plugin"), plugin);
    get_parameter(mitigation_type + std::string(".plugin"), plugin);

    int priority = 100;
    declare_parameter(mitigation_type + std::string(".priority"), priority);
    get_parameter(mitigation_type + std::string(".priority"), priority);

    try {
      RCLCPP_INFO(
        get_logger(),
        "Loading RecoveryMitigationBase %s [%s]", mitigation_type.c_str(), plugin.c_str());

      auto mitigation = mitigation_loader_->createSharedInstance(plugin);

      try {
        mitigation->initialize(shared_from_this(), mitigation_type);
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(), "Unable to initialize [%s]. Error: %s", plugin.c_str(), e.what());
        return CallbackReturnT::FAILURE;
      }

      loaded_mitigations.emplace_back(priority, mitigation);
      RCLCPP_INFO(
        get_logger(), "Loaded RecoveryMitigationBase %s [%s] (priority %d)",
        mitigation_type.c_str(), plugin.c_str(), priority);
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Unable to load plugin easynav::RecoveryMitigationBase. Error: %s",
        ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  // Lower priority number = tried first. stable_sort keeps mitigation_types order as the
  // tie-break, so leaving "priority" unset everywhere reproduces today's list-order behavior
  // exactly. try_select_mitigation() itself does not need to change: it already picks the
  // first non-excluded candidate in mitigations_ order.
  std::stable_sort(
    loaded_mitigations.begin(), loaded_mitigations.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});
  for (auto & [priority, mitigation] : loaded_mitigations) {
    mitigations_.push_back(mitigation);
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
RecoveryManagerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

void
RecoveryManagerNode::cycle(std::shared_ptr<NavState> nav_state)
{
  for (auto & evaluator : evaluators_) {
    evaluator->internal_update(*nav_state);
  }

  if (active_mitigation_) {
    // Only cycle it here if it does not require control (control-owning mitigations are
    // driven by cycle_rt() instead). Either way, do not attempt a new selection in the same
    // cycle a mitigation just ran or just stopped: the diagnostic that triggered it may still
    // read non-OK until the evaluator that owns it runs again and re-assesses, and selecting
    // again immediately would just restart the same mitigation in a tight loop.
    if (!active_mitigation_->requires_control()) {
      RecoveryStatus status = active_mitigation_->internal_cycle(*nav_state);
      if (status != RecoveryStatus::RUNNING) {
        if (status == RecoveryStatus::FAILED) {
          excluded_mitigations_[active_diagnostic_key_].insert(
            active_mitigation_->get_plugin_name());
        }
        active_mitigation_->internal_stop(*nav_state);
        active_mitigation_.reset();
      }
    }
  } else {
    try_select_mitigation(*nav_state);
  }

  publish_diagnostics(*nav_state);
  publish_mitigation_log(*nav_state);
}

void
RecoveryManagerNode::publish_diagnostics(NavState & nav_state)
{
  if (diagnostics_pub_->get_subscription_count() == 0) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  for (const auto & key : nav_state.get_group_keys("diagnostics")) {
    if (!nav_state.has(key)) {continue;}
    // Some entries (e.g. a SafetyReflexBase's) are written from the RT cycle; this runs on
    // the non-RT cycle, so get_safe() (a snapshot copy) is required here, not get(). See
    // NavState's own get()/get_safe() guidance.
    array.status.push_back(nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>(key));
  }

  diagnostics_pub_->publish(array);
}

void
RecoveryManagerNode::publish_mitigation_log(NavState & nav_state)
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
RecoveryManagerNode::publish_mitigation_resolved(const std::string & key)
{
  if (mitigation_pub_->get_subscription_count() == 0) {
    return;
  }

  rcl_interfaces::msg::Log msg;
  msg.stamp = now();
  // DEBUG is never used by report() itself, so subscribers can treat it as an unambiguous
  // "clear your log" sentinel instead of parsing message text.
  msg.level = rcl_interfaces::msg::Log::DEBUG;
  msg.name = get_name();
  msg.msg = key + " resuelto — mitigación finalizada";
  mitigation_pub_->publish(msg);
}

bool
RecoveryManagerNode::cycle_rt(std::shared_ptr<NavState> nav_state)
{
  if (!active_mitigation_ || !active_mitigation_->requires_control()) {
    return false;
  }

  RecoveryStatus status = active_mitigation_->internal_cycle(*nav_state);
  if (status != RecoveryStatus::RUNNING) {
    if (status == RecoveryStatus::FAILED) {
      excluded_mitigations_[active_diagnostic_key_].insert(active_mitigation_->get_plugin_name());
    }
    active_mitigation_->internal_stop(*nav_state);
    active_mitigation_.reset();
    nav_state->set("control_owner", std::string("controller"));
  }

  return true;
}

void
RecoveryManagerNode::try_select_mitigation(NavState & nav_state)
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
        get_logger(), "Selecting mitigation [%s] for diagnostic [%s]",
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
RecoveryManagerNode::get_active_mitigation_name() const
{
  return active_mitigation_ ? active_mitigation_->get_plugin_name() : std::string();
}

}  // namespace easynav
