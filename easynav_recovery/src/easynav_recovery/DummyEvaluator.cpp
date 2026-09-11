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
/// \brief Implementation of the DummyEvaluator class.

#include "easynav_recovery/DummyEvaluator.hpp"

namespace easynav
{

void DummyEvaluator::on_initialize()
{
}

void DummyEvaluator::update(NavState & nav_state)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = get_plugin_name();
  status.message = "dummy evaluator: nothing checked";

  publish_diagnostic(nav_state, status);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyEvaluator, easynav::RecoveryEvaluatorBase)
