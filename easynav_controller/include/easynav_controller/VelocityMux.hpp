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
/// \brief Declaration of the VelocityMux class.

#ifndef EASYNAV_CONTROLLER__VELOCITYMUX_HPP_
#define EASYNAV_CONTROLLER__VELOCITYMUX_HPP_

#include <optional>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/time.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/VelocityCommand.hpp"

namespace easynav
{

/**
 * @class VelocityMux
 * @brief Decides, every RT cycle, which velocity command is sent to the robot.
 *
 * Takes the commands proposed this cycle (see velocity_command) and picks, by priority:
 * 1. an emergency override (VelocitySource::OVERRIDE, published as is, not smoothed);
 * 2. a command that takes over the robot's motion (VelocitySource::TAKEOVER);
 * 3. zero, while navigation is paused ("navigation_paused");
 * 4. the nominal controller's command (VelocitySource::CONTROLLER).
 * With no new proposal, the last target is kept, so the smoother can keep ramping towards it,
 * unless no proposal arrives within the timeout: then the target becomes zero (TIMEOUT).
 * Proposals with non-finite values are discarded.
 */
class VelocityMux
{
public:
  /// @brief Who the selected command comes from.
  enum class Choice {NONE, CONTROLLER, PAUSED, TAKEOVER, OVERRIDE, TIMEOUT};

  /// @brief This cycle's selection.
  struct Selection
  {
    geometry_msgs::msg::TwistStamped cmd;  ///< Target command.
    Choice choice {Choice::NONE};          ///< Who it comes from.
    bool fresh {false};                    ///< A new command was proposed this cycle.
    bool smooth {true};                    ///< Whether it must go through the smoother.
    bool discarded {false};                ///< A non-finite proposal was discarded this cycle.
  };

  /// @brief Takes this cycle's proposals from \p nav_state and selects the command at \p now.
  Selection select(NavState & nav_state, const rclcpp::Time & now);

  /// @brief Longest time (s) a non-zero target is kept without new proposals; 0 disables it.
  void set_timeout(double seconds) {timeout_ = seconds;}

  /// @brief Whether the target was zeroed because no proposal arrived in time.
  [[nodiscard]] bool timed_out() const {return timed_out_;}

  /// @brief Forgets the last target (e.g. on activation).
  void reset();

private:
  geometry_msgs::msg::TwistStamped last_target_;
  std::optional<rclcpp::Time> last_proposal_;
  double timeout_ {0.0};
  bool timed_out_ {false};
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__VELOCITYMUX_HPP_
