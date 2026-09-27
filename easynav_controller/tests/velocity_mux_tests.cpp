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

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_controller/VelocityMux.hpp"

using easynav::VelocityMux;
using easynav::VelocitySource;

namespace
{

geometry_msgs::msg::TwistStamped cmd(double vx)
{
  geometry_msgs::msg::TwistStamped c;
  c.twist.linear.x = vx;
  return c;
}

}  // namespace

TEST(VelocityMuxTest, PriorityIsReflexThenRecoveryThenController)
{
  VelocityMux mux;
  easynav::NavState nav_state;

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  auto sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);
  EXPECT_TRUE(sel.smooth);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::RECOVERY, cmd(-0.1));
  sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::RECOVERY);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, -0.1);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::RECOVERY, cmd(-0.1));
  easynav::velocity_command::propose(nav_state, VelocitySource::REFLEX, cmd(0.0));
  sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::REFLEX);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_FALSE(sel.smooth) << "a reflex override must not be smoothed";
}

TEST(VelocityMuxTest, PauseCommandsZeroOverTheController)
{
  VelocityMux mux;
  easynav::NavState nav_state;
  nav_state.set("navigation_paused", true);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  const auto sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::PAUSED);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(sel.smooth) << "pausing must brake within the limits";
}

TEST(VelocityMuxTest, ProposalsAreConsumedAndTheLastTargetIsKept)
{
  VelocityMux mux;
  easynav::NavState nav_state;

  easynav::velocity_command::propose(nav_state, VelocitySource::RECOVERY, cmd(-0.1));
  EXPECT_TRUE(mux.select(nav_state).fresh);

  // Nothing new this cycle: same target, not fresh, and the proposal did not linger.
  const auto sel = mux.select(nav_state);
  EXPECT_FALSE(sel.fresh);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, -0.1);
  EXPECT_FALSE(
    easynav::velocity_command::peek(nav_state, VelocitySource::RECOVERY).has_value());
}
