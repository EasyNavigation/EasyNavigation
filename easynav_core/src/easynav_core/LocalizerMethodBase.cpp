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
/// \brief Implementation of the abstract base class LocalizerMethodBase.

#include <cmath>

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"

namespace easynav
{

void
LocalizerMethodBase::check_last_known_pose(const NavState & nav_state)
{
  std::call_once(
    last_known_pose_once_, [&]() {
      if (!nav_state.has("robot_pose")) {
        return;
      }
      try {
        const auto odom = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose");
        const auto & p = odom.pose.pose.position;
        const auto & q = odom.pose.pose.orientation;
        const bool valid = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
        (q.x != 0.0 || q.y != 0.0 || q.z != 0.0 || q.w != 0.0) &&
        odom.header.frame_id == RTTFBuffer::getInstance()->get_tf_info().map_frame;
        if (!valid) {
          return;
        }

        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose;
        on_last_known_pose(pose);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_node()->get_logger(), "Localizer [%s] could not reuse the last known pose: %s",
          get_plugin_name().c_str(), e.what());
      } catch (...) {
        RCLCPP_WARN(
          get_node()->get_logger(), "Localizer [%s] could not reuse the last known pose",
          get_plugin_name().c_str());
      }
    });
}

bool
LocalizerMethodBase::internal_update_rt(NavState & nav_state, bool trigger)
{
  report_rt_rate(nav_state);
  if (isTime2RunRT() || trigger) {
    EASYNAV_TRACE_EVENT;

    // Save last execution time, even if triggered
    setRunRT();
    check_last_known_pose(nav_state);

    try {
      update_rt(nav_state);
    } catch (const std::exception & e) {
      // A faulty plugin must not bring down EasyNav.
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception in update_rt() of localizer [%s]: %s", get_plugin_name().c_str(), e.what());
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Unknown exception in update_rt() of localizer [%s]", get_plugin_name().c_str());
    }

    return true;
  } else {
    return false;
  }
}

void
LocalizerMethodBase::internal_update(NavState & nav_state)
{
  report_rate(nav_state);
  if (isTime2Run()) {

    EASYNAV_TRACE_EVENT;
    // Save last execution time, even if triggered
    setRun();
    check_last_known_pose(nav_state);

    try {
      update(nav_state);
    } catch (const std::exception & e) {
      // A faulty plugin must not bring down EasyNav.
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception in update() of localizer [%s]: %s", get_plugin_name().c_str(), e.what());
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Unknown exception in update() of localizer [%s]", get_plugin_name().c_str());
    }
  }
}

}  // namespace easynav
