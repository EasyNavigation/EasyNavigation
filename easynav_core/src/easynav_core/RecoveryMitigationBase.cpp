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
/// \brief Implementation of the abstract base class RecoveryMitigationBase.

#include <atomic>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

namespace
{
// Shared by every RecoveryMitigationBase instance in the process, so a report from a brand-new
// instance is never mistaken for a stale one by RecoveryManagerNode.
std::atomic<uint64_t> next_mitigation_report_seq{1};
}  // namespace

void
RecoveryMitigationBase::internal_start(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;
  try {
    on_start(nav_state);
  } catch (const std::exception & e) {
    if (auto node = get_node()) {
      RCLCPP_ERROR_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Exception in on_start() of mitigation [%s]: %s", get_plugin_name().c_str(), e.what());
    }
  }
}

RecoveryStatus
RecoveryMitigationBase::internal_cycle(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;
  try {
    return on_cycle(nav_state);
  } catch (const std::exception & e) {
    if (auto node = get_node()) {
      RCLCPP_ERROR_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Exception in on_cycle() of mitigation [%s]: %s -- failing safe (stopping)",
        get_plugin_name().c_str(), e.what());
    }
    stop_robot(nav_state);
    return RecoveryStatus::FAILED;
  }
}

void
RecoveryMitigationBase::internal_stop(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;
  try {
    on_stop(nav_state);
  } catch (const std::exception & e) {
    if (auto node = get_node()) {
      RCLCPP_ERROR_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Exception in on_stop() of mitigation [%s]: %s", get_plugin_name().c_str(), e.what());
    }
  }
}

void
RecoveryMitigationBase::stop_robot(NavState & nav_state)
{
  geometry_msgs::msg::TwistStamped zero_speed;
  if (auto node = get_node()) {
    zero_speed.header.stamp = node->now();
  }
  zero_speed.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;

  nav_state.set("cmd_vel", zero_speed);
}

void
RecoveryMitigationBase::report(NavState & nav_state, uint8_t level, const std::string & msg)
{
  auto node = get_node();
  if (!node) {
    return;
  }

  switch (level) {
    case rcl_interfaces::msg::Log::DEBUG:
      RCLCPP_DEBUG(node->get_logger(), "%s", msg.c_str());
      break;
    case rcl_interfaces::msg::Log::WARN:
      RCLCPP_WARN(node->get_logger(), "%s", msg.c_str());
      break;
    case rcl_interfaces::msg::Log::ERROR:
      RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
      break;
    case rcl_interfaces::msg::Log::FATAL:
      RCLCPP_FATAL(node->get_logger(), "%s", msg.c_str());
      break;
    case rcl_interfaces::msg::Log::INFO:
    default:
      RCLCPP_INFO(node->get_logger(), "%s", msg.c_str());
      break;
  }

  MitigationReport report_entry;
  report_entry.seq = next_mitigation_report_seq.fetch_add(1);
  report_entry.log.stamp = node->now();
  report_entry.log.level = level;
  report_entry.log.name = get_plugin_name();
  report_entry.log.msg = msg;

  nav_state.set("mitigation.pending_report", report_entry);
}

}  // namespace easynav
