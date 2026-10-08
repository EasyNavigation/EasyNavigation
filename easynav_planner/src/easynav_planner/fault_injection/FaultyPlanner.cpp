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
/// \brief Implementation of the FaultyPlanner class.

#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_planner/fault_injection/FaultyPlanner.hpp"

namespace easynav
{

namespace
{
constexpr int kPathPoses = 10;
}  // namespace

void
FaultyPlanner::on_initialize()
{
  auto node = get_node();
  const auto & name = get_plugin_name();

  declare_parameter_if_absent(*node, name + ".fault", fault_);
  declare_parameter_if_absent(*node, name + ".fault_after", fault_after_);
  declare_parameter_if_absent(*node, name + ".hang_time", hang_time_);
  node->get_parameter(name + ".fault", fault_);
  node->get_parameter(name + ".fault_after", fault_after_);
  node->get_parameter(name + ".hang_time", hang_time_);

  static const std::set<std::string> faults {
    "none", "throw", "hang", "empty_path", "freeze", "nan"};
  if (faults.count(fault_) == 0) {
    throw std::invalid_argument("[" + name + "] unknown fault: " + fault_);
  }
  updates_ = 0;
}

void
FaultyPlanner::update(NavState & nav_state)
{
  if (updates_++ < fault_after_ || fault_ == "none") {
    path_ = plan(nav_state);
  } else if (fault_ == "throw") {
    throw std::runtime_error("injected fault");
  } else if (fault_ == "hang") {
    std::this_thread::sleep_for(std::chrono::duration<double>(hang_time_));
    path_ = plan(nav_state);
  } else if (fault_ == "empty_path") {
    path_ = plan(nav_state);
    path_.poses.clear();
  } else if (fault_ == "nan") {
    path_ = plan(nav_state);
    for (auto & pose : path_.poses) {
      pose.pose.position.x = std::nan("");
      pose.pose.position.y = std::nan("");
    }
  }
  // "freeze": the last path, as it was.
  nav_state.set("path", path_);
}

nav_msgs::msg::Path
FaultyPlanner::plan(const NavState & nav_state) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = get_node()->now();
  path.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().map_frame;
  if (!nav_state.has("robot_pose") || !nav_state.has("goals")) {
    return path;
  }
  const auto goals = nav_state.get_safe<nav_msgs::msg::Goals>("goals");
  if (goals.goals.empty()) {
    return path;
  }
  const auto start = nav_state.get_safe<nav_msgs::msg::Odometry>("robot_pose").pose.pose;
  const auto & goal = goals.goals.front().pose;
  for (int i = 0; i <= kPathPoses; ++i) {
    const double t = static_cast<double>(i) / kPathPoses;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = start.position.x + t * (goal.position.x - start.position.x);
    pose.pose.position.y = start.position.y + t * (goal.position.y - start.position.y);
    pose.pose.orientation = goal.orientation;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace easynav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(easynav::FaultyPlanner, easynav::PlannerMethodBase)
