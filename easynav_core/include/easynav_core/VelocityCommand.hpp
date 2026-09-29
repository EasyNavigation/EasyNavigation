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
/// \brief Velocity commands proposed by each source, for ControllerNode to arbitrate.

#ifndef EASYNAV_CORE__VELOCITYCOMMAND_HPP_
#define EASYNAV_CORE__VELOCITYCOMMAND_HPP_

#include <optional>

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/// @brief The role a proposed velocity command plays, from lowest to highest priority.
enum class VelocitySource
{
  CONTROLLER,  ///< Nominal motion: the controller plugin's command.
  TAKEOVER,    ///< Takes over the robot's motion from the controller (smoothed).
  OVERRIDE,    ///< Emergency override of everything else (published as is, not smoothed).
};

/// @brief One source's slot in NavState: its last command, and whether it is still pending
/// (not taken yet).
struct VelocityProposal
{
  bool pending {false};
  geometry_msgs::msg::TwistStamped cmd;
};

/**
 * Every source proposes its command in its own NavState slot, once per RT cycle it produces
 * one; ControllerNode's VelocityMux takes (consumes) them, picks by priority, smooths and
 * publishes. No source overwrites another's command.
 */
namespace velocity_command
{

/// @brief Proposes \p cmd as \p source's command for this RT cycle.
void propose(
  NavState & nav_state, VelocitySource source, const geometry_msgs::msg::TwistStamped & cmd);

/// @brief \p source's command proposed this cycle, if any, without consuming it.
std::optional<geometry_msgs::msg::TwistStamped> peek(
  const NavState & nav_state, VelocitySource source);

/// @brief \p source's command proposed this cycle, if any, consuming it.
std::optional<geometry_msgs::msg::TwistStamped> take(NavState & nav_state, VelocitySource source);

}  // namespace velocity_command

}  // namespace easynav

#endif  // EASYNAV_CORE__VELOCITYCOMMAND_HPP_
