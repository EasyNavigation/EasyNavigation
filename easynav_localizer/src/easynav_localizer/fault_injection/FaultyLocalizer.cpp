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
/// \brief Implementation of the FaultyLocalizer class.

#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_localizer/fault_injection/FaultyLocalizer.hpp"

namespace easynav
{

void
FaultyLocalizer::on_initialize()
{
  auto node = get_node();
  const auto & name = get_plugin_name();

  declare_parameter_if_absent(*node, name + ".fault", fault_);
  declare_parameter_if_absent(*node, name + ".fault_after", fault_after_);
  declare_parameter_if_absent(*node, name + ".x", x_);
  declare_parameter_if_absent(*node, name + ".y", y_);
  declare_parameter_if_absent(*node, name + ".yaw", yaw_);
  declare_parameter_if_absent(*node, name + ".hang_time", hang_time_);
  declare_parameter_if_absent(*node, name + ".jump_distance", jump_distance_);
  node->get_parameter(name + ".fault", fault_);
  node->get_parameter(name + ".fault_after", fault_after_);
  node->get_parameter(name + ".x", x_);
  node->get_parameter(name + ".y", y_);
  node->get_parameter(name + ".yaw", yaw_);
  node->get_parameter(name + ".hang_time", hang_time_);
  node->get_parameter(name + ".jump_distance", jump_distance_);

  static const std::set<std::string> faults {
    "none", "throw", "hang", "freeze", "stop_publishing", "nan", "jump"};
  if (faults.count(fault_) == 0) {
    throw std::invalid_argument("[" + name + "] unknown fault: " + fault_);
  }
  updates_ = 0;
}

void
FaultyLocalizer::update_rt(NavState & nav_state)
{
  if (updates_++ < fault_after_ || fault_ == "none") {
    publish(nav_state, x_, y_, yaw_);
  } else if (fault_ == "throw") {
    throw std::runtime_error("injected fault");
  } else if (fault_ == "hang") {
    std::this_thread::sleep_for(std::chrono::duration<double>(hang_time_));
    publish(nav_state, x_, y_, yaw_);
  } else if (fault_ == "freeze") {
    nav_state.set("robot_pose", pose_);  // Same pose, same stamp.
  } else if (fault_ == "nan") {
    publish(nav_state, std::nan(""), std::nan(""), std::nan(""));
  } else if (fault_ == "jump") {
    publish(nav_state, x_ + jump_distance_, y_, yaw_);
  }
  // "stop_publishing": nothing written.
}

void
FaultyLocalizer::publish(NavState & nav_state, double x, double y, double yaw)
{
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  pose_.header.stamp = get_node()->now();
  pose_.header.frame_id = tf_info.map_frame;
  pose_.child_frame_id = tf_info.robot_footprint_frame;
  pose_.pose.pose.position.x = x;
  pose_.pose.pose.position.y = y;
  pose_.pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose_.pose.pose.orientation.w = std::cos(yaw / 2.0);
  nav_state.set("robot_pose", pose_);
}

}  // namespace easynav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(easynav::FaultyLocalizer, easynav::LocalizerMethodBase)
