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
/// \brief Implementation of the FaultyMapsManager class.

#include <chrono>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include "easynav_common/Parameters.hpp"
#include "easynav_maps_manager/fault_injection/FaultyMapsManager.hpp"

namespace easynav
{

void
FaultyMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & name = get_plugin_name();

  declare_parameter_if_absent(*node, name + ".fault", fault_);
  declare_parameter_if_absent(*node, name + ".fault_after", fault_after_);
  declare_parameter_if_absent(*node, name + ".hang_time", hang_time_);
  node->get_parameter(name + ".fault", fault_);
  node->get_parameter(name + ".fault_after", fault_after_);
  node->get_parameter(name + ".hang_time", hang_time_);

  static const std::set<std::string> faults {"none", "throw", "hang"};
  if (faults.count(fault_) == 0) {
    throw std::invalid_argument("[" + name + "] unknown fault: " + fault_);
  }
  updates_ = 0;
}

void
FaultyMapsManager::update([[maybe_unused]] NavState & nav_state)
{
  if (updates_++ < fault_after_ || fault_ == "none") {
    return;
  }
  if (fault_ == "throw") {
    throw std::runtime_error("injected fault");
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(hang_time_));  // "hang"
}

}  // namespace easynav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(easynav::FaultyMapsManager, easynav::MapsManagerBase)
