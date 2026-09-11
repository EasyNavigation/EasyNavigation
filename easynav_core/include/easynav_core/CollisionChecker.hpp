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
/// \brief Declaration of CollisionChecker, the forward-projection collision guard shared by
/// ControllerMethodBase and the CollisionSafetyReflex plugin.

#ifndef EASYNAV_CORE__COLLISIONCHECKER_HPP_
#define EASYNAV_CORE__COLLISIONCHECKER_HPP_

#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class CollisionChecker
 * @brief Forward-projects the commanded "cmd_vel" against nearby point-cloud perceptions to
 * decide whether continuing would cause a collision within the current braking distance.
 *
 * Backs the CollisionSafetyReflex plugin, which SystemNode checks on every RT cycle for every
 * producer of "cmd_vel" — the nominal controller or any future movement recovery mitigator —
 * regardless of which one wrote it (see docs/recoveries_easynav.md, level 0).
 */
class CollisionChecker
{
public:
  CollisionChecker() = default;

  /**
   * @brief Declares and reads the checker's parameters under "<param_prefix>.<name>", and
   * creates the debug marker publisher on \p node.
   *
   * @param node Owning lifecycle node, used for parameters, the clock, and the publisher.
   * @param param_prefix Parameter namespace, typically the reflex's plugin instance id.
   */
  void initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & param_prefix);

  /**
   * @brief Checks whether an imminent collision is present.
   *
   * Uses the "cmd_vel" and point-cloud perceptions currently in \p nav_state. Returns false
   * (without error) if there is no "cmd_vel" yet or no point-cloud perceptions available.
   *
   * @param nav_state Current navigation state.
   * @return True if a collision is predicted within the braking distance.
   */
  bool check(NavState & nav_state);

private:
  void publish_collision_zone_marker(
    const std::vector<double> & min,
    const std::vector<double> & max,
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    bool imminent_collision,
    const rclcpp::Time & stamp);

  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node_;

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

#endif  // EASYNAV_CORE__COLLISIONCHECKER_HPP_
