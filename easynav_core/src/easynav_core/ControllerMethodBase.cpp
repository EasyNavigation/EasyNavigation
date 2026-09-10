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
/// \brief Implementation of the abstract base class ControllerMethodBase.

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/MethodBase.hpp"
#include "easynav_core/ControllerMethodBase.hpp"

#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

void
ControllerMethodBase::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name)
{
  auto node = parent_node;

  node->declare_parameter("colision_checker.active", collision_checker_active_);
  node->get_parameter("colision_checker.active", collision_checker_active_);

  collision_checker_.initialize(node, "colision_checker");

  MethodBase::initialize(parent_node, plugin_name);
}

bool
ControllerMethodBase::internal_update_rt(NavState & nav_state, bool trigger)
{
  if (isTime2RunRT() || trigger) {
    EASYNAV_TRACE_EVENT;

    // Save last execution time, even if triggered
    setRunRT();

    try {
      update_rt(nav_state);
    } catch (const std::exception & e) {
      // A misbehaving plugin must not crash the RT thread; the collision check below
      // still runs on whatever nav_state currently holds. See docs/recoveries_easynav.md.
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception in update_rt() of controller [%s]: %s", get_plugin_name().c_str(), e.what());
    }

    if (collision_checker_active_ && collision_checker_.check(nav_state)) {
      on_inminent_collision(nav_state);
    }

    return true;
  } else {
    return false;
  }
}

void
ControllerMethodBase::on_inminent_collision(NavState & nav_state)
{

  RCLCPP_WARN_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "ControllerMethodBase::on_inminent_collision: Inminent collision!! Stopping");

  geometry_msgs::msg::TwistStamped zero_speed;
  zero_speed.header.stamp = get_node()->now();
  zero_speed.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;

  nav_state.set("cmd_vel", zero_speed);
}

}  // namespace easynav
