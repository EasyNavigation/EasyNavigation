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
/// \brief Declaration of the FaultyController class.

#ifndef EASYNAV_CONTROLLER__FAULT_INJECTION__FAULTYCONTROLLER_HPP_
#define EASYNAV_CONTROLLER__FAULT_INJECTION__FAULTYCONTROLLER_HPP_

#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_core/ControllerMethodBase.hpp"

namespace easynav
{

/**
 * @class FaultyController
 * @brief Controller that misbehaves on purpose, to test how EasyNav copes with it.
 *
 * Commands a constant velocity ("<name>.linear_vel", "<name>.angular_vel") and, after
 * "<name>.fault_after" updates, injects the fault in "<name>.fault":
 * - "none": keeps commanding the constant velocity.
 * - "throw": throws from update_rt().
 * - "hang": blocks update_rt() for "<name>.hang_time" seconds on each update.
 * - "stop_proposing": stops writing "cmd_vel".
 * - "freeze": keeps writing the last command, with its old stamp.
 * - "max_velocity": commands a huge velocity.
 * - "nan": commands NaN.
 */
class FaultyController : public ControllerMethodBase
{
public:
  /// @brief Reads the parameters; throws on an unknown fault.
  void on_initialize() override;

  /// @brief Commands the constant velocity or injects the fault.
  void update_rt(NavState & nav_state) override;

private:
  void command(NavState & nav_state, double linear, double angular);

  std::string fault_ {"none"};
  int fault_after_ {0};
  double linear_vel_ {0.5};
  double angular_vel_ {0.0};
  double hang_time_ {1.0};

  int updates_ {0};
  geometry_msgs::msg::TwistStamped cmd_vel_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__FAULT_INJECTION__FAULTYCONTROLLER_HPP_
