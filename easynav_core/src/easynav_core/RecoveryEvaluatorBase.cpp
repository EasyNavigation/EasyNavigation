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
/// \brief Implementation of the abstract base class RecoveryEvaluatorBase.

#include <algorithm>
#include <string>

#include "easynav_common/YTSession.hpp"

#include "easynav_core/RecoveryEvaluatorBase.hpp"

namespace easynav
{

void
RecoveryEvaluatorBase::internal_update(NavState & nav_state)
{
  if (isTime2Run()) {
    EASYNAV_TRACE_EVENT;

    // Save last execution time, even if triggered
    setRun();

    try {
      update(nav_state);
    } catch (const std::exception & e) {
      // A misbehaving evaluator must not crash the process.
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception in update() of evaluator [%s]: %s", get_plugin_name().c_str(), e.what());
    }
  }
}

void
RecoveryEvaluatorBase::publish_diagnostic(
  NavState & nav_state, const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  const std::string key = "diagnostics." + get_plugin_name();

  nav_state.set(key, status);

  auto members = nav_state.get_group_keys("diagnostics");
  if (std::find(members.begin(), members.end(), key) == members.end()) {
    members.push_back(key);
    nav_state.set_group("diagnostics", members);
  }
}

}  // namespace easynav
