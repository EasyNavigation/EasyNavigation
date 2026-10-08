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

#include <atomic>
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
 * Setting the value a parameter already has is accepted. A parameter not frozen yet can only be
 * declared while accept_new_parameters() is on (e.g. while EasyNav configures).
 */
class ParameterFreezer
{
public:
  /// @brief Freezes the current values of \p nodes. Called again, refreezes them.
  void freeze(const Nodes & nodes);

  /// @brief Whether anything was frozen.
  [[nodiscard]] bool is_frozen() const {return !handles_.empty();}

  /// @brief Whether parameters not frozen yet may be declared (on by default).
  void accept_new_parameters(bool accept) {accept_new_ = accept;}

private:
  /// @brief Frozen values, by node name.
  std::map<std::string, std::map<std::string, rclcpp::ParameterValue>> frozen_;
  std::mutex mutex_;
  std::atomic<bool> accept_new_ {true};
  std::vector<rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr> handles_;
};

}  // namespace easynav::safety

#endif  // EASYNAV_SYSTEM__SAFETY__PARAMETERFREEZER_HPP_
