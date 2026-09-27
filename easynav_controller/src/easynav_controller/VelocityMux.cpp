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
/// \brief Implementation of the VelocityMux class.

#include "easynav_controller/VelocityMux.hpp"

namespace easynav
{

VelocityMux::Selection
VelocityMux::select(NavState & nav_state)
{
  // Take every proposal, so none of them lingers into the next cycle.
  const auto reflex = velocity_command::take(nav_state, VelocitySource::REFLEX);
  const auto recovery = velocity_command::take(nav_state, VelocitySource::RECOVERY);
  const auto controller = velocity_command::take(nav_state, VelocitySource::CONTROLLER);
  const bool paused = nav_state.has("navigation_paused") &&
    nav_state.get_safe<bool>("navigation_paused");

  Selection selection;
  if (reflex) {
    selection = {*reflex, Choice::REFLEX, true, false};
  } else if (recovery) {
    selection = {*recovery, Choice::RECOVERY, true, true};
  } else if (paused) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header = controller ? controller->header : last_target_.header;
    selection = {stop, Choice::PAUSED, controller.has_value(), true};
  } else if (controller) {
    selection = {*controller, Choice::CONTROLLER, true, true};
  } else {
    selection = {last_target_, Choice::NONE, false, true};
  }

  last_target_ = selection.cmd;
  return selection;
}

}  // namespace easynav
