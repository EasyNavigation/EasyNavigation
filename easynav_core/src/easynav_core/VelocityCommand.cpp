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
/// \brief Implementation of the velocity command proposals.

#include <string>

#include "easynav_core/VelocityCommand.hpp"

namespace easynav
{

namespace
{

/// @brief One source's slot: its last command, and whether it is still pending (not taken).
struct VelocityProposal
{
  bool pending {false};
  geometry_msgs::msg::TwistStamped cmd;
};

// Built once: these calls run every RT cycle, where no memory may be allocated (a key longer
// than the small-string buffer would otherwise be allocated on every call).
const std::string & key(VelocitySource source)
{
  static const std::string controller {"cmd_vel.proposal.controller"};
  static const std::string recovery {"cmd_vel.proposal.recovery"};
  static const std::string reflex {"cmd_vel.proposal.reflex"};
  switch (source) {
    case VelocitySource::RECOVERY: return recovery;
    case VelocitySource::REFLEX: return reflex;
    case VelocitySource::CONTROLLER:
    default: return controller;
  }
}

}  // namespace

namespace velocity_command
{

void propose(
  NavState & nav_state, VelocitySource source, const geometry_msgs::msg::TwistStamped & cmd)
{
  nav_state.set(key(source), VelocityProposal{true, cmd});
}

std::optional<geometry_msgs::msg::TwistStamped> peek(
  const NavState & nav_state, VelocitySource source)
{
  const auto & k = key(source);
  if (!nav_state.has(k)) {
    return std::nullopt;
  }
  const auto proposal = nav_state.get_safe<VelocityProposal>(k);
  return proposal.pending ? std::optional(proposal.cmd) : std::nullopt;
}

std::optional<geometry_msgs::msg::TwistStamped> take(NavState & nav_state, VelocitySource source)
{
  auto cmd = peek(nav_state, source);
  if (cmd.has_value()) {
    nav_state.set(key(source), VelocityProposal{false, *cmd});
  }
  return cmd;
}

}  // namespace velocity_command

}  // namespace easynav
