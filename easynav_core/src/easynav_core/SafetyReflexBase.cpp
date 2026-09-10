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
/// \brief Implementation of the abstract base class SafetyReflexBase.

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/SafetyReflexBase.hpp"

namespace easynav
{

bool
SafetyReflexBase::internal_check_and_mitigate(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  bool triggered = false;
  try {
    triggered = check(nav_state);
  } catch (const std::exception & e) {
    if (auto node = get_node()) {
      RCLCPP_ERROR_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Exception in check() of safety reflex [%s]: %s -- failing safe (stopping)",
        get_plugin_name().c_str(), e.what());
    }
    stop_robot(nav_state);
    return true;
  }

  if (!triggered) {return false;}

  try {
    mitigate(nav_state);
  } catch (const std::exception & e) {
    if (auto node = get_node()) {
      RCLCPP_ERROR_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Exception in mitigate() of safety reflex [%s]: %s -- failing safe (stopping)",
        get_plugin_name().c_str(), e.what());
    }
    stop_robot(nav_state);
  }

  return true;
}

void
SafetyReflexBase::stop_robot(NavState & nav_state)
{
  geometry_msgs::msg::TwistStamped zero_speed;
  if (auto node = get_node()) {
    zero_speed.header.stamp = node->now();
  }
  zero_speed.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;

  nav_state.set("cmd_vel", zero_speed);
}

}  // namespace easynav
