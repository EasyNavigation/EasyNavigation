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
/// \brief Robot velocity and acceleration limits, and the interface to query them.

#ifndef EASYNAV_CORE__ROBOTLIMITS_HPP_
#define EASYNAV_CORE__ROBOTLIMITS_HPP_

#include <string>

namespace easynav
{

/**
 * @struct RobotLimits
 * @brief Velocity and acceleration limits of the robot, shared by everything that commands it.
 *
 * Configured once, in ControllerNode ("robot_limits.*" parameters): controller plugins query
 * them (ControllerMethodBase::get_robot_limits()) instead of declaring their own, and the
 * velocity smoother enforces them on every command published.
 */
struct RobotLimits
{
  double max_linear_vel {0.5};     ///< Forward linear velocity (m/s, >= 0).
  double min_linear_vel {-0.2};    ///< Backward linear velocity (m/s, <= 0). 0 = no reversing
                                   ///< (then retreat recoveries cannot back up either).
  double max_angular_vel {1.0};    ///< Angular velocity, either direction (rad/s, >= 0).
  double max_linear_acc {0.5};     ///< Linear acceleration, speeding up (m/s^2, > 0).
  double max_linear_decel {1.0};   ///< Linear deceleration, slowing down (m/s^2, > 0).
  double max_angular_acc {1.0};    ///< Angular acceleration, speeding up (rad/s^2, > 0).
  double max_angular_decel {2.0};  ///< Angular deceleration, slowing down (rad/s^2, > 0).
};

/**
 * @struct LegacyRobotLimitNames
 * @brief Deprecated per-controller parameter names for each robot limit ("" = none).
 *
 * Controllers used to declare their own limits. A value still configured under an old name is
 * applied, with a deprecation warning, unless "robot_limits.*" configures that limit.
 */
struct LegacyRobotLimitNames
{
  std::string max_linear_vel;
  std::string min_linear_vel;
  std::string max_angular_vel;
  std::string max_linear_acc;
  std::string max_linear_decel;
  std::string max_angular_acc;
  std::string max_angular_decel;
};

/**
 * @class RobotLimitsProvider
 * @brief Implemented by the node that owns the robot limits (ControllerNode), so the plugins it
 * loads can query them through their parent node.
 */
class RobotLimitsProvider
{
public:
  virtual ~RobotLimitsProvider() = default;

  /// @brief Current robot limits.
  virtual RobotLimits get_robot_limits() const = 0;

  /// @brief Whether "robot_limits.<field>" was configured explicitly (not left by default).
  virtual bool is_robot_limit_configured(const std::string & field) const = 0;

  /// @brief Replaces the limits enforced (e.g. with deprecated per-controller values applied).
  virtual void set_robot_limits(const RobotLimits & limits) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__ROBOTLIMITS_HPP_
