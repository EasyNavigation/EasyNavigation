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

#ifndef EASYNAV_COMMON__TYPES__TFINFO_HPP_
#define EASYNAV_COMMON__TYPES__TFINFO_HPP_

#include <string>

namespace easynav
{

/// @brief Aggregated TF configuration used across EasyNav.
struct TFInfo
{
  // These parameters are designed to enforce compliance with REP-105:
  // http://www.ros.org/reps/rep-0105.html

  /// Optional TF prefix applied to frame names.
  std::string tf_prefix {""};

  /// Global map frame.
  std::string map_frame {"map"};

  /// Odometry frame.
  std::string odom_frame {"odom"};

  /// Robot base frame (base_link equivalent).
  std::string robot_frame {"base_link"};

  /// Robot base frame (base_footprint equivalent).
  std::string robot_footprint_frame {"base_footprint"};

  /// World frame used by global estimators (e.g. earth-fixed frame).
  std::string world_frame{"earth"};
};

}  // namespace easynav

#endif  // EASYNAV_COMMON__TYPES__TFINFO_HPP_
