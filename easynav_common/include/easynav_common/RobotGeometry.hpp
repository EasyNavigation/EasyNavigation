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
/// \brief RobotGeometryRegistry, which shares the robot's geometry, and get_robot_geometry().

#ifndef EASYNAV_COMMON__ROBOTGEOMETRY_HPP_
#define EASYNAV_COMMON__ROBOTGEOMETRY_HPP_

#include <mutex>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/Singleton.hpp"
#include "easynav_common/types/RobotGeometry.hpp"

namespace easynav
{

/**
 * @class RobotGeometryRegistry
 * @brief Shares the robot's geometry across EasyNav's nodes.
 *
 * The geometry is configured in SystemNode ("robot_geometry.*"), which sets it here before its
 * subnodes configure, so their plugins read it while initializing (see get_robot_geometry()).
 */
class RobotGeometryRegistry : public Singleton<RobotGeometryRegistry>
{
public:
  RobotGeometryRegistry() = default;

  /// @brief The robot's geometry.
  RobotGeometry get_geometry() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return geometry_;
  }

  /// @brief Whether "robot_geometry.<field>" was configured explicitly.
  bool is_configured(const std::string & field) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return configured_.count(field) > 0;
  }

  /// @brief Sets the robot's geometry, and which fields were configured explicitly.
  void set_geometry(const RobotGeometry & geometry, const std::set<std::string> & configured = {})
  {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_ = geometry;
    configured_ = configured;
  }

private:
  RobotGeometry geometry_;
  std::set<std::string> configured_;
  mutable std::mutex mutex_;

  SINGLETON_DEFINITIONS(RobotGeometryRegistry)
};

/**
 * @brief The robot's geometry ("system_node.robot_geometry.*").
 *
 * A deprecated parameter of \p node listed in \p legacy still applies, with a warning, when it
 * is configured (in the parameter files, or declared by a previous instance) and
 * "robot_geometry" does not configure that field; otherwise it is ignored, with a warning too.
 *
 * @param node Node holding the deprecated parameters.
 * @param legacy Full names of the deprecated parameters.
 */
template<typename NodeT>
RobotGeometry get_robot_geometry(NodeT & node, const LegacyRobotGeometryNames & legacy = {})
{
  auto registry = RobotGeometryRegistry::getInstance();
  RobotGeometry geometry = registry->get_geometry();

  auto apply = [&](const std::string & name, const std::string & field, double & value) {
      if (name.empty()) {
        return;
      }
      const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
      if (overrides.count(name) == 0 && !node.has_parameter(name)) {
        return;
      }
      double legacy_value = value;
      declare_parameter_if_absent(node, name, legacy_value);
      node.get_parameter(name, legacy_value);

      const auto replacement = "system_node.robot_geometry." + field;
      if (registry->is_configured(field)) {
        RCLCPP_WARN(
          node.get_logger(), "'%s' is deprecated and ignored: '%s' takes precedence",
          name.c_str(), replacement.c_str());
        return;
      }
      RCLCPP_WARN(
        node.get_logger(), "'%s' is deprecated: configure '%s' instead. It will stop working soon.",
        name.c_str(), replacement.c_str());
      value = legacy_value;
    };

  apply(legacy.radius, "radius", geometry.radius);
  apply(legacy.inscribed_radius, "inscribed_radius", geometry.inscribed_radius);
  apply(legacy.height, "height", geometry.height);
  return geometry;
}

}  // namespace easynav

#endif  // EASYNAV_COMMON__ROBOTGEOMETRY_HPP_
