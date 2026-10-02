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
/// \brief Implementation of the FaultyController class.

#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_controller/fault_injection/FaultyController.hpp"

namespace easynav
{

void
FaultyController::on_initialize()
{
  auto node = get_node();
  const auto & name = get_plugin_name();

  declare_parameter_if_absent(*node, name + ".fault", fault_);
  declare_parameter_if_absent(*node, name + ".fault_after", fault_after_);
  declare_parameter_if_absent(*node, name + ".linear_vel", linear_vel_);
  declare_parameter_if_absent(*node, name + ".angular_vel", angular_vel_);
  declare_parameter_if_absent(*node, name + ".hang_time", hang_time_);
  node->get_parameter(name + ".fault", fault_);
  node->get_parameter(name + ".fault_after", fault_after_);
  node->get_parameter(name + ".linear_vel", linear_vel_);
  node->get_parameter(name + ".angular_vel", angular_vel_);
  node->get_parameter(name + ".hang_time", hang_time_);

  static const std::set<std::string> faults {
    "none", "throw", "hang", "stop_proposing", "freeze", "max_velocity", "nan"};
  if (faults.count(fault_) == 0) {
    throw std::invalid_argument("[" + name + "] unknown fault: " + fault_);
  }
  updates_ = 0;
}

void
FaultyController::update_rt(NavState & nav_state)
{
  if (updates_++ < fault_after_ || fault_ == "none") {
    command(nav_state, linear_vel_, angular_vel_);
  } else if (fault_ == "throw") {
    throw std::runtime_error("injected fault");
  } else if (fault_ == "hang") {
    std::this_thread::sleep_for(std::chrono::duration<double>(hang_time_));
    command(nav_state, linear_vel_, angular_vel_);
  } else if (fault_ == "freeze") {
    nav_state.set("cmd_vel", cmd_vel_);  // Same command, same stamp.
  } else if (fault_ == "max_velocity") {
    command(nav_state, 1e3, 1e3);
  } else if (fault_ == "nan") {
    command(nav_state, std::nan(""), std::nan(""));
  }
  // "stop_proposing": nothing written.
}

void
FaultyController::command(NavState & nav_state, double linear, double angular)
{
  cmd_vel_.header.stamp = get_node()->now();
  cmd_vel_.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  cmd_vel_.twist.linear.x = linear;
  cmd_vel_.twist.angular.z = angular;
  nav_state.set("cmd_vel", cmd_vel_);
}

}  // namespace easynav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(easynav::FaultyController, easynav::ControllerMethodBase)
