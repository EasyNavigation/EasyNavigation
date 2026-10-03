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
#include <memory>
#include <stdexcept>

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

  rt_frequency_ = 10.0;
  frequency_ = 10.0;

  easynav::declare_parameter_if_absent(*parent_node, plugin_name + ".rt_freq", rt_frequency_);
  easynav::declare_parameter_if_absent(*parent_node, plugin_name + ".freq", frequency_);
  parent_node->get_parameter(plugin_name + ".rt_freq", rt_frequency_);
  parent_node->get_parameter(plugin_name + ".freq", frequency_);

  // Not just <= 0: with NaN the plugin would never run, with inf it would run every cycle.
  if (!std::isfinite(rt_frequency_) || !std::isfinite(frequency_) ||
    rt_frequency_ <= 0.0f || frequency_ <= 0.0f)
  {
    throw std::runtime_error(
            "[" + plugin_name + "] Invalid frequency configuration: rt_freq=" +
            std::to_string(rt_frequency_) + ", freq=" + std::to_string(frequency_) +
            " (both must be finite and > 0.0)");
  }

  last_ts_ = parent_node->now();
  rt_last_ts_ = parent_node->now();

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
MethodBase::isTime2RunRT()
{
  auto node = parent_node_.lock();
  if (!node) {return false;}
  const auto now = node->now();
  const double target_cycle_time = 1.0 / rt_frequency_;
  const double cycle_time = (now - rt_last_ts_).seconds();
  if (cycle_time >= target_cycle_time) {
    if (cycle_time > 1.5 * target_cycle_time) {
      RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "[%s] RT cycle time exceeded target by more than 1.5x (%.3f s > %.3f s)",
          plugin_name_.c_str(), cycle_time, target_cycle_time);
    }
    rt_last_ts_ = now;
    return true;
  } else {
    return false;
  }
}

bool
MethodBase::isTime2Run()
{
  auto node = parent_node_.lock();
  if (!node) {return false;}
  const auto now = node->now();
  const double target_cycle_time = 1.0 / frequency_;
  const double cycle_time = (now - last_ts_).seconds();
  if (cycle_time >= target_cycle_time) {
    if (cycle_time > 1.5 * target_cycle_time) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "[%s] target cycle time exceeded by more than 1.5x (%.3f s > %.3f s)",
        plugin_name_.c_str(), cycle_time, target_cycle_time);
    }
    last_ts_ = now;
    return true;
  } else {
    return false;
  }
}

void
MethodBase::setRunRT()
{
  if (auto node = parent_node_.lock()) {
    rt_last_ts_ = node->now();
  }
}

void
MethodBase::setRun()
{
  if (auto node = parent_node_.lock()) {
    last_ts_ = node->now();
  }
}

}  // namespace easynav
