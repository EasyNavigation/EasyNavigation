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
/// \brief Implementation of RecoveryManagerBase.

#include <string>

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/YTSession.hpp"
#include "easynav_core/RecoveryManagerBase.hpp"
#include "easynav_core/VelocityCommand.hpp"

namespace easynav
{

void
RecoveryManagerBase::internal_update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;
  try {
    update(nav_state);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Exception in update() of recovery manager [%s]: %s", get_plugin_name().c_str(), e.what());
  }
}

bool
RecoveryManagerBase::internal_update_rt(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;
  try {
    return update_rt(nav_state);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Exception in update_rt() of recovery manager [%s]: %s -- failing safe (stopping)",
      get_plugin_name().c_str(), e.what());

    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = get_node()->now();
    stop.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
    override_velocity(nav_state, stop);
    return true;
  }
}

void
RecoveryManagerBase::command_velocity(
  NavState & nav_state, const geometry_msgs::msg::TwistStamped & cmd)
{
  velocity_command::propose(nav_state, VelocitySource::RECOVERY, cmd);
}

void
RecoveryManagerBase::override_velocity(
  NavState & nav_state, const geometry_msgs::msg::TwistStamped & cmd)
{
  velocity_command::propose(nav_state, VelocitySource::REFLEX, cmd);
}

void
RecoveryManagerBase::abort_mission(const std::string & reason)
{
  if (auto actions = system_actions_.lock()) {
    actions->abort_mission(reason);
  } else {
    RCLCPP_WARN(
      get_node()->get_logger(), "Recovery manager [%s] cannot abort the mission (%s): no system",
      get_plugin_name().c_str(), reason.c_str());
  }
}

void
RecoveryManagerBase::request_shutdown(const std::string & reason)
{
  if (auto actions = system_actions_.lock()) {
    actions->request_shutdown(reason);
  } else {
    RCLCPP_WARN(
      get_node()->get_logger(), "Recovery manager [%s] cannot request a shutdown (%s): no system",
      get_plugin_name().c_str(), reason.c_str());
  }
}

}  // namespace easynav
