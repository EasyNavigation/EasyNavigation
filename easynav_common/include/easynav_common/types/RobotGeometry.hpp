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
/// \brief Declaration of RobotGeometry and LegacyRobotGeometryNames.

#ifndef EASYNAV_COMMON__TYPES__ROBOTGEOMETRY_HPP_
#define EASYNAV_COMMON__TYPES__ROBOTGEOMETRY_HPP_

#include <string>

namespace easynav
{

/**
 * @struct RobotGeometry
 * @brief The robot's shape, shared by everything that needs it.
 *
 * Configured once, in SystemNode ("robot_geometry.*" parameters), and read with
 * get_robot_geometry() (easynav_common/RobotGeometry.hpp).
 */
struct RobotGeometry
{
  double radius {0.3};            ///< Circumscribed: smallest circle containing the robot (m).
  double inscribed_radius {0.3};  ///< Largest circle inside the robot (m). Defaults to radius.
  double height {0.5};            ///< Top of the robot, above the robot frame (m).
};

/**
 * @struct LegacyRobotGeometryNames
 * @brief Deprecated parameter names (full names on the node, "" = none) for each field.
 *
 * Components used to declare their own geometry. A value still configured under an old name is
 * applied, with a deprecation warning, unless "robot_geometry.*" configures that field.
 */
struct LegacyRobotGeometryNames
{
  std::string radius;
  std::string inscribed_radius;
  std::string height;
};

}  // namespace easynav

#endif  // EASYNAV_COMMON__TYPES__ROBOTGEOMETRY_HPP_
