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
/// \brief Implementation of the VelocitySmoother class.

#include <algorithm>
#include <cmath>

#include "easynav_controller/VelocitySmoother.hpp"

namespace easynav
{

namespace
{

constexpr double kEpsilon = 1e-6;

/// @brief One axis: towards \p target, limited by \p acc when speeding up and \p decel when
/// slowing down; a change of sign stops at zero first.
double step_axis(double current, double target, double acc, double decel, double dt)
{
  const bool slowing = (current > 0.0 && target < current) || (current<0.0 && target> current);
  const double max_delta = (slowing ? decel : acc) * dt;
  double next = current + std::clamp(target - current, -max_delta, max_delta);
  if ((current > 0.0 && next < 0.0) || (current < 0.0 && next > 0.0)) {
    next = 0.0;
  }
  return next;
}

/// @brief Within the turning radius: |angular.z| <= |linear.x| / radius (radius 0: no limit).
double within_radius(double angular, double linear, double radius)
{
  if (radius <= 0.0) {return angular;}
  const double max_angular = std::abs(linear) / radius;
  return std::clamp(angular, -max_angular, max_angular);
}

}  // namespace

geometry_msgs::msg::Twist
VelocitySmoother::clamp(const geometry_msgs::msg::Twist & target) const
{
  geometry_msgs::msg::Twist clamped;
  clamped.linear.x = std::clamp(target.linear.x, limits_.min_linear_vel, limits_.max_linear_vel);
  clamped.linear.y =
    std::clamp(target.linear.y, -limits_.max_lateral_vel, limits_.max_lateral_vel);
  clamped.linear.z =
    std::clamp(target.linear.z, -limits_.max_vertical_vel, limits_.max_vertical_vel);
  clamped.angular.z = within_radius(
    std::clamp(target.angular.z, -limits_.max_angular_vel, limits_.max_angular_vel),
    clamped.linear.x, limits_.min_turning_radius);
  return clamped;
}

const geometry_msgs::msg::Twist &
VelocitySmoother::step(const geometry_msgs::msg::Twist & target, double dt)
{
  const auto goal = clamp(target);
  dt = std::max(dt, 0.0);

  current_.linear.x = step_axis(
    current_.linear.x, goal.linear.x, limits_.max_linear_acc, limits_.max_linear_decel, dt);
  current_.linear.y = step_axis(
    current_.linear.y, goal.linear.y, limits_.max_lateral_acc, limits_.max_lateral_decel, dt);
  current_.linear.z = step_axis(
    current_.linear.z, goal.linear.z, limits_.max_vertical_acc, limits_.max_vertical_decel, dt);
  current_.angular.z = step_axis(
    current_.angular.z, goal.angular.z, limits_.max_angular_acc, limits_.max_angular_decel, dt);
  // While linear.x ramps, the curvature published must stay within the turning radius too
  current_.angular.z =
    within_radius(current_.angular.z, current_.linear.x, limits_.min_turning_radius);
  return current_;
}

void
VelocitySmoother::reset(const geometry_msgs::msg::Twist & current)
{
  current_ = geometry_msgs::msg::Twist();
  current_.linear.x = current.linear.x;
  current_.linear.y = current.linear.y;
  current_.linear.z = current.linear.z;
  current_.angular.z = current.angular.z;
}

bool
VelocitySmoother::reached(const geometry_msgs::msg::Twist & target) const
{
  const auto goal = clamp(target);
  return std::abs(current_.linear.x - goal.linear.x) < kEpsilon &&
         std::abs(current_.linear.y - goal.linear.y) < kEpsilon &&
         std::abs(current_.linear.z - goal.linear.z) < kEpsilon &&
         std::abs(current_.angular.z - goal.angular.z) < kEpsilon;
}

double
VelocitySmoother::time_to_stop() const
{
  return std::max(
    {std::abs(current_.linear.x) / std::max(limits_.max_linear_decel, kEpsilon),
      std::abs(current_.linear.y) / std::max(limits_.max_lateral_decel, kEpsilon),
      std::abs(current_.linear.z) / std::max(limits_.max_vertical_decel, kEpsilon),
      std::abs(current_.angular.z) / std::max(limits_.max_angular_decel, kEpsilon)});
}

}  // namespace easynav
