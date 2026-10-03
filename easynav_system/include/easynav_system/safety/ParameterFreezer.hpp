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
/// \brief Declaration of the ParameterFreezer class.

#ifndef EASYNAV_SYSTEM__SAFETY__PARAMETERFREEZER_HPP_
#define EASYNAV_SYSTEM__SAFETY__PARAMETERFREEZER_HPP_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easynav_system/safety/ConfigurationFingerprint.hpp"

namespace easynav::safety
{

/**
 * @class ParameterFreezer
 * @brief Rejects any change to the parameters of a set of nodes.
 *
 * A declaration (a parameter not frozen yet) or setting the same value is accepted, so the nodes
 * can still go through their lifecycle.
 */
class ParameterFreezer
{
public:
  /// @brief Freezes the current values of \p nodes. Called again, refreezes them.
  void freeze(const Nodes & nodes);

  /// @brief Whether anything was frozen.
  [[nodiscard]] bool is_frozen() const {return !handles_.empty();}

private:
  /// @brief Frozen values, by node name.
  std::map<std::string, std::map<std::string, rclcpp::ParameterValue>> frozen_;
  std::mutex mutex_;
  std::vector<rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr> handles_;
};

}  // namespace easynav::safety

#endif  // EASYNAV_SYSTEM__SAFETY__PARAMETERFREEZER_HPP_
