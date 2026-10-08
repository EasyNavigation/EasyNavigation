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
/// \brief The safety channel's state, as EasyNav applies it.

#ifndef EASYNAV_CORE__SAFETYCHANNEL_HPP_
#define EASYNAV_CORE__SAFETYCHANNEL_HPP_

#include <algorithm>
#include <limits>

#include "easynav_core/RobotLimits.hpp"

namespace easynav
{

/**
 * @struct SafetyChannelState
 * @brief What the safety channel (safety PLC, scanner...) imposes on EasyNav right now.
 *
 * Written by SystemNode every RT cycle to NavState ("safety_status"), only if the safety status
 * is enabled (system_node's "safety.status.timeout" > 0). Plain data: read every RT cycle.
 */
struct SafetyChannelState
{
  bool protective_stop {false};  ///< Stopped by the safety channel, or its status is lost.
  bool status_lost {false};      ///< No valid status recently: treated as a protective stop.
  double max_linear_vel {std::numeric_limits<double>::infinity()};   ///< Current SLS (m/s).
  double max_angular_vel {std::numeric_limits<double>::infinity()};  ///< Current SLS (rad/s).

  bool operator==(const SafetyChannelState & other) const
  {
    return protective_stop == other.protective_stop && status_lost == other.status_lost &&
           max_linear_vel == other.max_linear_vel && max_angular_vel == other.max_angular_vel;
  }
  bool operator!=(const SafetyChannelState & other) const {return !(*this == other);}
};

/// @brief NavState key of the SafetyChannelState.
inline constexpr char kSafetyStatusKey[] = "safety_status";

/// @brief \p limits cut down to the safely limited speed in \p state.
inline RobotLimits limited_by(RobotLimits limits, const SafetyChannelState & state)
{
  limits.max_linear_vel = std::min(limits.max_linear_vel, state.max_linear_vel);
  limits.min_linear_vel = std::max(limits.min_linear_vel, -state.max_linear_vel);
  limits.max_angular_vel = std::min(limits.max_angular_vel, state.max_angular_vel);
  return limits;
}

}  // namespace easynav

#endif  // EASYNAV_CORE__SAFETYCHANNEL_HPP_
