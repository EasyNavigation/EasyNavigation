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
/// \brief Declaration of the VelocitySmoother class.

#ifndef EASYNAV_CONTROLLER__VELOCITYSMOOTHER_HPP_
#define EASYNAV_CONTROLLER__VELOCITYSMOOTHER_HPP_

#include "geometry_msgs/msg/twist.hpp"

#include "easynav_core/RobotLimits.hpp"

namespace easynav
{

/**
 * @class VelocitySmoother
 * @brief Brings the commanded velocity towards a target within the robot limits.
 *
 * Each step clamps the target to the velocity limits and moves the current command towards it
 * by at most acceleration * dt (speeding up) or deceleration * dt (slowing down), per axis
 * (linear x and y, angular z). A change of direction stops at zero first. So a command never
 * jumps, e.g. from full speed to zero, which would make the robot brake harder than it can.
 */
class VelocitySmoother
{
public:
  /// @brief Sets the limits to enforce.
  void set_limits(const RobotLimits & limits) {limits_ = limits;}

  /// @brief Limits being enforced.
  [[nodiscard]] const RobotLimits & get_limits() const {return limits_;}

  /**
   * @brief Next command towards \p target, \p dt seconds after the previous one.
   * @return The new current command.
   */
  const geometry_msgs::msg::Twist & step(const geometry_msgs::msg::Twist & target, double dt);

  /// @brief Makes \p current the current command, e.g. after a command that bypassed smoothing.
  void reset(const geometry_msgs::msg::Twist & current = geometry_msgs::msg::Twist());

  /// @brief The current (last) command.
  [[nodiscard]] const geometry_msgs::msg::Twist & current() const {return current_;}

  /// @brief Whether the current command already equals \p target, clamped to the limits.
  [[nodiscard]] bool reached(const geometry_msgs::msg::Twist & target) const;

  /// @brief Seconds needed to stop from the current command at the maximum deceleration.
  [[nodiscard]] double time_to_stop() const;

private:
  [[nodiscard]] geometry_msgs::msg::Twist clamp(const geometry_msgs::msg::Twist & target) const;

  RobotLimits limits_;
  geometry_msgs::msg::Twist current_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__VELOCITYSMOOTHER_HPP_
