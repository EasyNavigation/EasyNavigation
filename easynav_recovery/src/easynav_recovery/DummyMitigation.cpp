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
/// \brief Implementation of the DummyMitigation class.

#include "easynav_recovery/DummyMitigation.hpp"

namespace easynav
{

void DummyMitigation::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<bool>(plugin_name + ".requires_control", requires_control_);
  node->get_parameter<bool>(plugin_name + ".requires_control", requires_control_);
}

bool DummyMitigation::can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  return status.level != diagnostic_msgs::msg::DiagnosticStatus::OK;
}

RecoveryStatus DummyMitigation::on_cycle(NavState &)
{
  return RecoveryStatus::SUCCEEDED;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyMitigation, easynav::RecoveryMitigationBase)
