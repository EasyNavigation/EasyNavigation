// Copyright 2025 Intelligent Robotics Lab
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
/// \brief Implementation of the base class MethodBase used in plugin-based EasyNav method components.

#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RobotGeometry.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

void
MethodBase::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name)
{
  parent_node_ = parent_node;
  plugin_name_ = plugin_name;

  rt_cycle_.parameter = "rt_freq";
  cycle_.parameter = "freq";
  for (auto * cycle : {&rt_cycle_, &cycle_}) {
    const auto name = plugin_name + "." + cycle->parameter;
    double frequency = 10.0;
    easynav::declare_parameter_if_absent(*parent_node, name, frequency);
    parent_node->get_parameter(name, frequency);
    // Not just <= 0: with NaN the plugin would never run, with inf it would run every cycle.
    if (!std::isfinite(frequency) || frequency <= 0.0) {
      throw std::runtime_error(
              "[" + plugin_name + "] Invalid frequency configuration: " + name + "=" +
              std::to_string(frequency) + " (must be finite and > 0.0)");
    }
    cycle->frequency = frequency;
    cycle->last_ts = parent_node->now();
    cycle->next_ts = cycle->last_ts + rclcpp::Duration::from_seconds(1.0 / frequency);
    cycle->scheduled = false;
    cycle->monitor.configure(frequency);
    cycle->name = plugin_name + (cycle == &rt_cycle_ ? ".rt_rate" : ".rate");
    cycle->key = "diagnostics." + cycle->name;
    cycle->reported.reset();
  }

  on_initialize();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
MethodBase::get_node() const
{
  return parent_node_.lock();
}

const std::string &
MethodBase::get_plugin_name() const
{
  return plugin_name_;
}

RobotGeometry
MethodBase::get_robot_geometry(const LegacyRobotGeometryNames & legacy)
{
  auto full = [this](const std::string & name) {
      return name.empty() ? name : plugin_name_ + "." + name;
    };
  return easynav::get_robot_geometry(
    *get_node(), {full(legacy.radius), full(legacy.inscribed_radius), full(legacy.height)});
}

bool
MethodBase::is_time_to_run(Cycle & cycle)
{
  auto node = parent_node_.lock();
  if (!node) {return false;}
  const auto now = node->now();
  if (now < cycle.last_ts) {
    cycle.next_ts = now;  // The clock went back (e.g. a simulation reset)
  }
  if (now < cycle.next_ts) {
    return false;
  }
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / cycle.frequency);
  // From the scheduled time, not from now: late checks do not lower the rate
  cycle.next_ts = cycle.next_ts + period;
  if (cycle.next_ts <= now) {
    cycle.next_ts = now + period;  // More than a period behind: the missed runs are lost
  }
  cycle.scheduled = true;
  return true;
}

void
MethodBase::set_run(Cycle & cycle)
{
  auto node = parent_node_.lock();
  if (!node) {return;}
  cycle.last_ts = node->now();
  if (!cycle.scheduled) {
    // An extra run (e.g. triggered): the next one, a period from now
    cycle.next_ts = cycle.last_ts + rclcpp::Duration::from_seconds(1.0 / cycle.frequency);
  }
  cycle.scheduled = false;
  cycle.monitor.run(cycle.last_ts.seconds());
}

void
MethodBase::report(Cycle & cycle, NavState & nav_state)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  auto node = parent_node_.lock();
  if (!node) {return;}
  const auto status = cycle.monitor.update(node->now().seconds());
  if (cycle.reported && *cycle.reported == status) {
    return;
  }
  // The first status too: it replaces one left by a previous instance of this plugin
  cycle.reported = status;

  char text[160];
  DiagnosticStatus diagnostic;
  switch (status) {
    case RateMonitor::Status::OK:
      diagnostic.level = DiagnosticStatus::OK;
      std::snprintf(
        text, sizeof(text), "%s %.1f Hz kept", cycle.parameter.c_str(), cycle.frequency);
      break;
    case RateMonitor::Status::WARN:
      diagnostic.level = DiagnosticStatus::WARN;
      std::snprintf(
        text, sizeof(text), "%s %.1f Hz not kept: %.1f Hz", cycle.parameter.c_str(),
        cycle.frequency, cycle.monitor.rate());
      RCLCPP_WARN(node->get_logger(), "[%s] %s", plugin_name_.c_str(), text);
      break;
    case RateMonitor::Status::ERROR:
      // Only reported: nothing to mitigate, so never an ERROR for the recovery system
      diagnostic.level = DiagnosticStatus::WARN;
      std::snprintf(
        text, sizeof(text), "%s %.1f Hz not kept for %.0f s: %.1f Hz", cycle.parameter.c_str(),
        cycle.frequency, cycle.monitor.slow_windows() * cycle.monitor.window(),
        cycle.monitor.rate());
      RCLCPP_WARN(node->get_logger(), "[%s] %s", plugin_name_.c_str(), text);
      break;
  }
  if (status == RateMonitor::Status::OK && cycle.monitor.rate() == 0.0) {
    std::snprintf(
      text, sizeof(text), "%s %.1f Hz: measuring", cycle.parameter.c_str(), cycle.frequency);
  } else if (status == RateMonitor::Status::OK) {
    RCLCPP_INFO(node->get_logger(), "[%s] %s", plugin_name_.c_str(), text);
  }
  diagnostic.name = cycle.name;
  diagnostic.hardware_id = node->get_name();
  diagnostic.message = text;
  nav_state.set(cycle.key, diagnostic);
  nav_state.add_to_group("diagnostics", cycle.key);
}

bool
MethodBase::isTime2RunRT()
{
  return is_time_to_run(rt_cycle_);
}

bool
MethodBase::isTime2Run()
{
  return is_time_to_run(cycle_);
}

void
MethodBase::setRunRT()
{
  set_run(rt_cycle_);
}

void
MethodBase::setRun()
{
  set_run(cycle_);
}

void
MethodBase::reset_rate_monitors()
{
  rt_cycle_.monitor.reset();
  cycle_.monitor.reset();
}

void
MethodBase::report_rt_rate(NavState & nav_state)
{
  report(rt_cycle_, nav_state);
}

void
MethodBase::report_rate(NavState & nav_state)
{
  report(cycle_, nav_state);
}

}  // namespace easynav
