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
/// \brief Implementation of the CollisionSafetyReflex plugin.

#include "easynav_controller/CollisionSafetyReflex.hpp"

namespace easynav
{

void
CollisionSafetyReflex::on_initialize()
{
  collision_checker_.initialize(get_node(), get_plugin_name());
}

bool
CollisionSafetyReflex::check(NavState & nav_state)
{
  return collision_checker_.check(nav_state);
}

void
CollisionSafetyReflex::mitigate(NavState & nav_state)
{
  RCLCPP_WARN_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "CollisionSafetyReflex [%s]: imminent collision, stopping", get_plugin_name().c_str());

  stop_robot(nav_state);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CollisionSafetyReflex, easynav::SafetyReflexBase)
