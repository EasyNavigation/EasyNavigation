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
/// \brief Implementation of the ParameterFreezer class.

#include <string>
#include <vector>

#include "easynav_system/safety/ParameterFreezer.hpp"

namespace easynav::safety
{

void
ParameterFreezer::freeze(const Nodes & nodes)
{
  const bool installed = is_frozen();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [name, node] : nodes) {
    auto & frozen = frozen_[name];
    for (const auto & parameter : node->get_parameters(node->list_parameters({}, 0).names)) {
      frozen[parameter.get_name()] = parameter.get_parameter_value();
    }
    if (installed) {
      continue;
    }
    handles_.push_back(
      node->add_on_set_parameters_callback(
        [this, node_name = name](const std::vector<rclcpp::Parameter> & parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          std::lock_guard<std::mutex> lock(mutex_);
          const auto & frozen = frozen_[node_name];
          for (const auto & parameter : parameters) {
            const auto it = frozen.find(parameter.get_name());
            const bool is_new = it == frozen.end();
            if (is_new ? !accept_new_ : it->second != parameter.get_parameter_value()) {
              result.successful = false;
              result.reason = "safety.mode: the configuration is frozen (" + node_name + "/" +
              parameter.get_name() + (is_new ? ", a new parameter)" : ")");
              break;
            }
          }
          return result;
        }));
  }
}

}  // namespace easynav::safety
