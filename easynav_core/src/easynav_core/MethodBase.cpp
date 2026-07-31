// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

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

  parent_node->declare_parameter(plugin_name + ".rt_freq", rt_frequency_);
  parent_node->declare_parameter(plugin_name + ".freq", frequency_);
  parent_node->get_parameter(plugin_name + ".rt_freq", rt_frequency_);
  parent_node->get_parameter(plugin_name + ".freq", frequency_);

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
