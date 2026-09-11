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
/// \brief Declaration of the CollisionSafetyReflex plugin.

#ifndef EASYNAV_CONTROLLER__COLLISIONSAFETYREFLEX_HPP_
#define EASYNAV_CONTROLLER__COLLISIONSAFETYREFLEX_HPP_

#include <string>
#include <vector>

#include "visualization_msgs/msg/marker_array.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "easynav_core/SafetyReflexBase.hpp"

namespace easynav
{

/**
 * @class CollisionSafetyReflex
 * @brief Reference level-0 safety reflex: stops the robot on imminent collision.
 *
 * Loaded and run directly by SystemNode on every RT cycle, right before "cmd_vel" is
 * published, regardless of whether it was produced by the active controller or by a movement
 * recovery mitigator. See docs/recoveries_easynav.md, level 0.
 *
 * Forward-projects the commanded "cmd_vel" against nearby point-cloud perceptions to decide
 * whether continuing would cause a collision within the current braking distance.
 */
class CollisionSafetyReflex : public SafetyReflexBase
{
public:
  CollisionSafetyReflex() = default;
  ~CollisionSafetyReflex() = default;

  void on_initialize() override;

protected:
  bool check(NavState & nav_state) override;
  void mitigate(NavState & nav_state) override;

private:
  void publish_collision_zone_marker(
    const std::vector<double> & min,
    const std::vector<double> & max,
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    bool imminent_collision,
    const rclcpp::Time & stamp);

  bool debug_markers_{false};
  double robot_radius_{0.35};
  double robot_height_{0.5};
  double z_min_filter_{0.0};
  double brake_acc_{0.5};
  double safety_margin_{0.1};
  double downsample_leaf_size_{0.1};

  rclcpp::Time collision_stamp_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_marker_pub_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__COLLISIONSAFETYREFLEX_HPP_
