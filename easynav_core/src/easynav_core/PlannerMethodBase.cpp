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
/// \brief Implementation of the abstract base class PlannerMethodBase.

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/PlannerMethodBase.hpp"

namespace easynav
{

void
PlannerMethodBase::internal_update(NavState & nav_state)
{
  if (isTime2Run()) {
    EASYNAV_TRACE_EVENT;

    // Save last execution time, even if triggered
    setRun();

    try {
      update(nav_state);
    } catch (const std::exception & e) {
      // A misbehaving plugin must not crash the process. See docs/recoveries_easynav.md.
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception in update() of planner [%s]: %s", get_plugin_name().c_str(), e.what());
    }
  }
}

void
PlannerMethodBase::force_update(NavState & nav_state)
{
  setRun();
  try {
    update(nav_state);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Exception in force_update() of planner [%s]: %s", get_plugin_name().c_str(), e.what());
  }
}

}  // namespace easynav
