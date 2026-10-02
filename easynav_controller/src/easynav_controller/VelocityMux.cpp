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
/// \brief Implementation of the VelocityMux class.

#include <cmath>
#include <optional>
#include <string>

#include "easynav_controller/VelocityMux.hpp"

namespace easynav
{

namespace
{
const std::string kNavigationPaused {"navigation_paused"};

bool is_finite(const geometry_msgs::msg::Twist & t)
{
  return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
         std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z);
}

bool is_zero(const geometry_msgs::msg::Twist & t)
{
  return t == geometry_msgs::msg::Twist();
}
}  // namespace

VelocityMux::Selection
VelocityMux::select(NavState & nav_state, const rclcpp::Time & now)
{
  Selection selection;

  // Take every proposal, so none of them lingers into the next cycle.
  auto take = [&](VelocitySource source) {
      auto cmd = velocity_command::take(nav_state, source);
      if (cmd && !is_finite(cmd->twist)) {
        selection.discarded = true;
        return std::optional<geometry_msgs::msg::TwistStamped>();
      }
      return cmd;
    };
  const auto override_cmd = take(VelocitySource::OVERRIDE);
  const auto takeover = take(VelocitySource::TAKEOVER);
  const auto controller = take(VelocitySource::CONTROLLER);
  const bool paused = nav_state.has(kNavigationPaused) &&
    nav_state.get_safe<bool>(kNavigationPaused);

  if (override_cmd || takeover || controller) {
    last_proposal_ = now;
    timed_out_ = false;
  } else if (last_proposal_ && now < *last_proposal_) {
    last_proposal_ = now;  // The clock jumped back: count the timeout from now.
  }

  const bool discarded = selection.discarded;
  if (override_cmd) {
    selection = {*override_cmd, Choice::OVERRIDE, true, false};
  } else if (takeover) {
    selection = {*takeover, Choice::TAKEOVER, true, true};
  } else if (paused) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header = controller ? controller->header : last_target_.header;
    selection = {stop, Choice::PAUSED, controller.has_value(), true};
  } else if (controller) {
    selection = {*controller, Choice::CONTROLLER, true, true};
  } else if (timed_out_) {
    selection = {last_target_, Choice::TIMEOUT, false, true};
  } else if (timeout_ > 0.0 && last_proposal_ && !is_zero(last_target_.twist) &&
    (now - *last_proposal_).seconds() > timeout_)
  {
    // Nobody keeps commanding the robot: stop it instead of holding a stale command.
    timed_out_ = true;
    geometry_msgs::msg::TwistStamped stop;
    stop.header = last_target_.header;
    selection = {stop, Choice::TIMEOUT, true, true};
  } else {
    selection = {last_target_, Choice::NONE, false, true};
  }
  selection.discarded = discarded;

  last_target_ = selection.cmd;
  return selection;
}

void
VelocityMux::reset()
{
  last_target_ = geometry_msgs::msg::TwistStamped();
  last_proposal_.reset();
  timed_out_ = false;
}

}  // namespace easynav
