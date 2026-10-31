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

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace easynav
{

/// @brief A vertex of the robot's footprint, in the robot frame (m).
struct FootprintPoint
{
  double x {0.0};
  double y {0.0};
};

/**
 * @struct RobotGeometry
 * @brief The robot's shape, shared by everything that needs it.
 *
 * Configured once, in SystemNode ("robot_geometry.*" parameters), and read with
 * get_robot_geometry() (easynav_common/RobotGeometry.hpp).
 */
struct RobotGeometry
{
  RobotGeometry() = default;
  RobotGeometry(
    double radius, double inscribed_radius, double height,
    std::vector<FootprintPoint> footprint = {})
  : radius(radius), inscribed_radius(inscribed_radius), height(height),
    footprint(std::move(footprint)) {}

  double radius {0.3};            ///< Circumscribed: smallest circle containing the robot (m).
  double inscribed_radius {0.3};  ///< Largest circle inside the robot (m). Defaults to radius.
  double height {0.5};            ///< Top of the robot, above the robot frame (m).
  /// Polygon of the robot (robot frame, in order around it). Empty: a circle of \p radius.
  /// When given, \p radius contains it and \p inscribed_radius fits inside it.
  std::vector<FootprintPoint> footprint;
};

/// @brief Radius of the smallest circle around the robot frame containing \p footprint.
inline double footprint_radius(const std::vector<FootprintPoint> & footprint)
{
  double radius = 0.0;
  for (const auto & p : footprint) {
    radius = std::max(radius, std::hypot(p.x, p.y));
  }
  return radius;
}

/// @brief Whether (\p x, \p y) is inside \p footprint (crossing number).
inline bool footprint_contains(const std::vector<FootprintPoint> & footprint, double x, double y)
{
  bool inside = false;
  for (size_t i = 0, j = footprint.size() - 1; i < footprint.size(); j = i++) {
    const auto & a = footprint[i];
    const auto & b = footprint[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
  }
  return inside;
}

/// @brief Distance from (\p x, \p y) to the border of \p footprint.
inline double footprint_border_distance(
  const std::vector<FootprintPoint> & footprint, double x, double y)
{
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0, j = footprint.size() - 1; i < footprint.size(); j = i++) {
    const auto & a = footprint[j];
    const auto & b = footprint[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    const double t =
      len2 > 0.0 ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(x - (a.x + t * dx), y - (a.y + t * dy)));
  }
  return best;
}

/// @brief Radius of the largest circle around the robot frame inside \p footprint (0 if the
/// robot frame is outside it).
inline double footprint_inscribed_radius(const std::vector<FootprintPoint> & footprint)
{
  if (footprint.size() < 3 || !footprint_contains(footprint, 0.0, 0.0)) {
    return 0.0;
  }
  return footprint_border_distance(footprint, 0.0, 0.0);
}

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
