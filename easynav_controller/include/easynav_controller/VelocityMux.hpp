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
/// \brief Declaration of the VelocityMux class.

#ifndef EASYNAV_CONTROLLER__VELOCITYMUX_HPP_
#define EASYNAV_CONTROLLER__VELOCITYMUX_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/VelocityCommand.hpp"

namespace easynav
{

/**
 * @class VelocityMux
 * @brief Decides, every RT cycle, which velocity command is sent to the robot.
 *
 * Takes the commands proposed this cycle (see velocity_command) and picks, by priority:
 * 1. a safety reflex's override (published as is, not smoothed);
 * 2. a control-owning recovery mitigation's command;
 * 3. zero, while navigation is paused ("navigation_paused");
 * 4. the nominal controller's command.
 * With no new proposal, the last target is kept, so the smoother can keep ramping towards it.
 */
class VelocityMux
{
public:
  /// @brief Who the selected command comes from.
  enum class Choice {NONE, CONTROLLER, PAUSED, RECOVERY, REFLEX};

  /// @brief This cycle's selection.
  struct Selection
  {
    geometry_msgs::msg::TwistStamped cmd;  ///< Target command.
    Choice choice {Choice::NONE};          ///< Who it comes from.
    bool fresh {false};                    ///< A new command was proposed this cycle.
    bool smooth {true};                    ///< Whether it must go through the smoother.
  };

  /// @brief Takes this cycle's proposals from \p nav_state and selects the command.
  Selection select(NavState & nav_state);

  /// @brief Forgets the last target (e.g. on activation).
  void reset() {last_target_ = geometry_msgs::msg::TwistStamped();}

private:
  geometry_msgs::msg::TwistStamped last_target_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__VELOCITYMUX_HPP_
