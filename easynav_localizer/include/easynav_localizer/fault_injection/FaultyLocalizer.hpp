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
/// \brief Declaration of the FaultyLocalizer class.

#ifndef EASYNAV_LOCALIZER__FAULT_INJECTION__FAULTYLOCALIZER_HPP_
#define EASYNAV_LOCALIZER__FAULT_INJECTION__FAULTYLOCALIZER_HPP_

#include <string>

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"

namespace easynav
{

/**
 * @class FaultyLocalizer
 * @brief Localizer that misbehaves on purpose, to test how EasyNav copes with it.
 *
 * Every RT update writes "robot_pose" at a fixed pose ("<name>.x", "<name>.y", "<name>.yaw"),
 * stamped now, and, after "<name>.fault_after" updates, injects the fault in "<name>.fault":
 * - "none": keeps localizing.
 * - "throw": throws from update_rt().
 * - "hang": blocks update_rt() for "<name>.hang_time" seconds on each update.
 * - "freeze": keeps writing the last pose, with its old stamp.
 * - "stop_publishing": stops writing "robot_pose".
 * - "nan": writes a NaN pose.
 * - "jump": the pose jumps "<name>.jump_distance" meters along x, once, and stays there.
 */
class FaultyLocalizer : public LocalizerMethodBase
{
public:
  /// @brief Reads the parameters; throws on an unknown fault.
  void on_initialize() override;

  /// @brief Localizes or injects the fault.
  void update_rt(NavState & nav_state) override;

  /// @brief Nothing: everything happens in update_rt().
  void update([[maybe_unused]] NavState & nav_state) override {}

private:
  void publish(NavState & nav_state, double x, double y, double yaw);

  std::string fault_ {"none"};
  int fault_after_ {0};
  double x_ {0.0};
  double y_ {0.0};
  double yaw_ {0.0};
  double hang_time_ {1.0};
  double jump_distance_ {1.0};

  int updates_ {0};
  nav_msgs::msg::Odometry pose_;
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__FAULT_INJECTION__FAULTYLOCALIZER_HPP_
