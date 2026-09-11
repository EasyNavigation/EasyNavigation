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
/// \brief Implementation of the abstract base class MapsManagerBase.

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/MapsManagerBase.hpp"

namespace easynav
{

void
MapsManagerBase::internal_update(NavState & nav_state)
{
  if (isTime2Run()) {
    EASYNAV_TRACE_NAMED_EVENT("MapsManagerBase::internal_update [" + get_plugin_name() + "]");

    // Save last execution time, even if triggered
    setRun();

    try {
      update(nav_state);
    } catch (const std::exception & e) {
      // A misbehaving plugin must not crash the process. See docs/recoveries_easynav.md.
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception in update() of maps manager [%s]: %s", get_plugin_name().c_str(), e.what());
    }
  }
}

bool
MapsManagerBase::internal_reset(NavState & nav_state)
{
  EASYNAV_TRACE_NAMED_EVENT("MapsManagerBase::internal_reset [" + get_plugin_name() + "]");

  try {
    return reset(nav_state);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Exception in reset() of maps manager [%s]: %s", get_plugin_name().c_str(), e.what());
    return false;
  }
}

}  // namespace easynav
