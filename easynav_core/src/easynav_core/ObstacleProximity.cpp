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
/// \brief Implementation of compute_nearest_obstacle().

#include <cmath>

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

#include "easynav_core/ObstacleProximity.hpp"

namespace easynav
{

ObstacleProximity compute_nearest_obstacle(NavState & nav_state)
{
  ObstacleProximity result;

  const auto & perceptions = nav_state.get_by_type<PointPerception>();
  if (perceptions.empty()) {
    return result;
  }

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  auto view = PointPerceptionsOpsView(perceptions);
  view.fuse(tf_info.robot_frame);
  const auto & cloud = view.as_points();

  double min_dist_sq = std::numeric_limits<double>::infinity();
  double nearest_x = 0.0;
  double nearest_y = 0.0;

  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}

    const double d_sq = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y;
    if (d_sq < min_dist_sq) {
      min_dist_sq = d_sq;
      nearest_x = p.x;
      nearest_y = p.y;
    }
  }

  if (std::isfinite(min_dist_sq)) {
    result.distance = std::sqrt(min_dist_sq);
    result.bearing = std::atan2(nearest_y, nearest_x);
  }

  return result;
}

}  // namespace easynav
