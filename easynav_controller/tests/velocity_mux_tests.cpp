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

#include <cmath>
#include <limits>

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

TEST(VelocityMuxTest, PriorityIsOverrideThenTakeoverThenController)
{
  VelocityMux mux;
  easynav::NavState nav_state;

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  auto sel = mux.select(nav_state, rclcpp::Time());
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);
  EXPECT_TRUE(sel.smooth);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  sel = mux.select(nav_state, rclcpp::Time());
  EXPECT_EQ(sel.choice, VelocityMux::Choice::TAKEOVER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, -0.1);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  easynav::velocity_command::propose(nav_state, VelocitySource::OVERRIDE, cmd(0.0));
  sel = mux.select(nav_state, rclcpp::Time());
  EXPECT_EQ(sel.choice, VelocityMux::Choice::OVERRIDE);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_FALSE(sel.smooth) << "an override must not be smoothed";
}

TEST(VelocityMuxTest, PauseCommandsZeroOverTheController)
{
  VelocityMux mux;
  easynav::NavState nav_state;
  nav_state.set("navigation_paused", true);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  const auto sel = mux.select(nav_state, rclcpp::Time());
  EXPECT_EQ(sel.choice, VelocityMux::Choice::PAUSED);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(sel.smooth) << "pausing must brake within the limits";
}

TEST(VelocityMuxTest, ProposalsAreConsumedAndTheLastTargetIsKept)
{
  VelocityMux mux;
  easynav::NavState nav_state;

  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  EXPECT_TRUE(mux.select(nav_state, rclcpp::Time()).fresh);

  // Nothing new this cycle: same target, not fresh, and the proposal did not linger.
  const auto sel = mux.select(nav_state, rclcpp::Time());
  EXPECT_FALSE(sel.fresh);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, -0.1);
  EXPECT_FALSE(
    easynav::velocity_command::peek(nav_state, VelocitySource::TAKEOVER).has_value());
}

TEST(VelocityCommandTest, PeekDoesNotConsumeTakeDoes)
{
  easynav::NavState nav_state;
  EXPECT_FALSE(easynav::velocity_command::peek(nav_state, VelocitySource::CONTROLLER));
  EXPECT_FALSE(easynav::velocity_command::take(nav_state, VelocitySource::CONTROLLER));

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.4));
  ASSERT_TRUE(easynav::velocity_command::peek(nav_state, VelocitySource::CONTROLLER));
  ASSERT_TRUE(easynav::velocity_command::peek(nav_state, VelocitySource::CONTROLLER));
  const auto taken = easynav::velocity_command::take(nav_state, VelocitySource::CONTROLLER);
  ASSERT_TRUE(taken);
  EXPECT_DOUBLE_EQ(taken->twist.linear.x, 0.4);
  EXPECT_FALSE(easynav::velocity_command::take(nav_state, VelocitySource::CONTROLLER));
  EXPECT_FALSE(easynav::velocity_command::peek(nav_state, VelocitySource::CONTROLLER));
}

TEST(VelocityCommandTest, EachSourceHasItsOwnSlotAndTheLatestProposalWins)
{
  easynav::NavState nav_state;
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.1));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(0.2));
  easynav::velocity_command::propose(nav_state, VelocitySource::OVERRIDE, cmd(0.3));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(0.25));

  EXPECT_DOUBLE_EQ(
    easynav::velocity_command::take(nav_state, VelocitySource::CONTROLLER)->twist.linear.x, 0.1);
  EXPECT_DOUBLE_EQ(
    easynav::velocity_command::take(nav_state, VelocitySource::TAKEOVER)->twist.linear.x, 0.25);
  EXPECT_DOUBLE_EQ(
    easynav::velocity_command::take(nav_state, VelocitySource::OVERRIDE)->twist.linear.x, 0.3);
}

TEST(VelocityMuxTest, TakeoverWinsOverPause)
{
  easynav::NavState nav_state;
  nav_state.set("navigation_paused", true);
  VelocityMux mux;
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(1.0));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.2));

  const auto sel = mux.select(nav_state, rclcpp::Time());
  EXPECT_EQ(sel.choice, VelocityMux::Choice::TAKEOVER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, -0.2);
  EXPECT_TRUE(sel.smooth);
}

TEST(VelocityMuxTest, SequenceOfSources)
{
  // controller -> takeover -> override -> nothing -> controller
  easynav::NavState nav_state;
  VelocityMux mux;

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  EXPECT_EQ(mux.select(nav_state, rclcpp::Time()).choice, VelocityMux::Choice::CONTROLLER);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  EXPECT_EQ(mux.select(nav_state, rclcpp::Time()).choice, VelocityMux::Choice::TAKEOVER);

  easynav::velocity_command::propose(nav_state, VelocitySource::OVERRIDE, cmd(0.0));
  const auto over = mux.select(nav_state, rclcpp::Time());
  EXPECT_EQ(over.choice, VelocityMux::Choice::OVERRIDE);
  EXPECT_FALSE(over.smooth);

  // Nothing proposed: the last target is kept, not fresh.
  const auto none = mux.select(nav_state, rclcpp::Time());
  EXPECT_EQ(none.choice, VelocityMux::Choice::NONE);
  EXPECT_FALSE(none.fresh);
  EXPECT_DOUBLE_EQ(none.cmd.twist.linear.x, 0.0);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  const auto back = mux.select(nav_state, rclcpp::Time());
  EXPECT_EQ(back.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(back.cmd.twist.linear.x, 0.5);
}

namespace
{

rclcpp::Time at(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

}  // namespace

TEST(VelocityMuxTimeoutTest, AHeldNonZeroTargetIsZeroedAfterTheTimeout)
{
  easynav::NavState nav_state;
  VelocityMux mux;
  mux.set_timeout(0.5);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  EXPECT_EQ(mux.select(nav_state, at(10.0)).choice, VelocityMux::Choice::CONTROLLER);

  auto sel = mux.select(nav_state, at(10.3));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);
  EXPECT_FALSE(mux.timed_out());

  // Exactly at the timeout: still kept.
  sel = mux.select(nav_state, at(10.5));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_FALSE(mux.timed_out());

  sel = mux.select(nav_state, at(10.51));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::TIMEOUT);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(sel.fresh) << "the new zero target must be published";
  EXPECT_TRUE(sel.smooth) << "it must brake within the limits";
  EXPECT_TRUE(mux.timed_out());

  // Stays timed out, without a new target each cycle.
  for (double t = 10.6; t < 12.0; t += 0.1) {
    sel = mux.select(nav_state, at(t));
    EXPECT_EQ(sel.choice, VelocityMux::Choice::TIMEOUT);
    EXPECT_FALSE(sel.fresh);
    EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  }

  // A new command ends it.
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.3));
  sel = mux.select(nav_state, at(12.0));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.3);
  EXPECT_FALSE(mux.timed_out());

  // And the timeout counts again from it.
  EXPECT_EQ(mux.select(nav_state, at(12.4)).choice, VelocityMux::Choice::NONE);
  EXPECT_EQ(mux.select(nav_state, at(12.6)).choice, VelocityMux::Choice::TIMEOUT);
}

TEST(VelocityMuxTimeoutTest, AZeroTargetNeverTimesOut)
{
  easynav::NavState nav_state;
  VelocityMux mux;
  mux.set_timeout(0.1);

  // Nothing ever proposed.
  EXPECT_EQ(mux.select(nav_state, at(0.0)).choice, VelocityMux::Choice::NONE);
  EXPECT_EQ(mux.select(nav_state, at(100.0)).choice, VelocityMux::Choice::NONE);

  // A zero command, then silence.
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.0));
  mux.select(nav_state, at(100.0));
  EXPECT_EQ(mux.select(nav_state, at(200.0)).choice, VelocityMux::Choice::NONE);
  EXPECT_FALSE(mux.timed_out());
}

TEST(VelocityMuxTimeoutTest, ZeroTimeoutDisablesIt)
{
  easynav::NavState nav_state;
  VelocityMux mux;
  mux.set_timeout(0.0);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  mux.select(nav_state, at(0.0));
  const auto sel = mux.select(nav_state, at(1000.0));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);
}

TEST(VelocityMuxTimeoutTest, AnySourceKeepsTheCommandAlive)
{
  easynav::NavState nav_state;
  VelocityMux mux;
  mux.set_timeout(0.5);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  mux.select(nav_state, at(0.0));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  mux.select(nav_state, at(0.4));
  easynav::velocity_command::propose(nav_state, VelocitySource::OVERRIDE, cmd(0.2));
  mux.select(nav_state, at(0.8));

  auto sel = mux.select(nav_state, at(1.2));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.2);

  sel = mux.select(nav_state, at(1.31));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::TIMEOUT);
}

TEST(VelocityMuxTimeoutTest, PauseWhileTimedOutCommandsZero)
{
  easynav::NavState nav_state;
  VelocityMux mux;
  mux.set_timeout(0.1);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  mux.select(nav_state, at(0.0));
  ASSERT_EQ(mux.select(nav_state, at(1.0)).choice, VelocityMux::Choice::TIMEOUT);

  nav_state.set("navigation_paused", true);
  const auto sel = mux.select(nav_state, at(1.1));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::PAUSED);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(mux.timed_out()) << "pausing is not a new command";
}

TEST(VelocityMuxTimeoutTest, TimeGoingBackwardsDoesNotTimeOut)
{
  // E.g. a simulation restarted: the clock jumps back.
  easynav::NavState nav_state;
  VelocityMux mux;
  mux.set_timeout(0.5);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  mux.select(nav_state, at(100.0));
  EXPECT_EQ(mux.select(nav_state, at(1.0)).choice, VelocityMux::Choice::NONE);
}

TEST(VelocityMuxTimeoutTest, ResetForgetsTheTimeout)
{
  easynav::NavState nav_state;
  VelocityMux mux;
  mux.set_timeout(0.1);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  mux.select(nav_state, at(0.0));
  ASSERT_EQ(mux.select(nav_state, at(1.0)).choice, VelocityMux::Choice::TIMEOUT);

  mux.reset();
  EXPECT_FALSE(mux.timed_out());
  const auto sel = mux.select(nav_state, at(2.0));
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
}

TEST(VelocityMuxNonFiniteTest, NonFiniteProposalsAreDiscarded)
{
  easynav::NavState nav_state;
  VelocityMux mux;

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  mux.select(nav_state, at(0.0));

  // A NaN from the controller: discarded, the last valid target is kept.
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(std::nan("")));
  auto sel = mux.select(nav_state, at(0.01));
  EXPECT_TRUE(sel.discarded);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_FALSE(sel.fresh);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);

  // An infinite override in any axis falls back to the next valid source.
  auto inf = cmd(0.0);
  inf.twist.angular.z = std::numeric_limits<double>::infinity();
  easynav::velocity_command::propose(nav_state, VelocitySource::OVERRIDE, inf);
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.2));
  sel = mux.select(nav_state, at(0.02));
  EXPECT_TRUE(sel.discarded);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.2);

  auto nan_y = cmd(0.0);
  nan_y.twist.linear.y = std::nan("");
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, nan_y);
  sel = mux.select(nav_state, at(0.03));
  EXPECT_TRUE(sel.discarded);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);

  // Valid again: nothing discarded.
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.1));
  sel = mux.select(nav_state, at(0.04));
  EXPECT_FALSE(sel.discarded);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
}

TEST(VelocityMuxNonFiniteTest, OnlyNonFiniteProposalsEndInATimeout)
{
  // A controller producing only NaN is as silent as one producing nothing.
  easynav::NavState nav_state;
  VelocityMux mux;
  mux.set_timeout(0.5);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  mux.select(nav_state, at(0.0));

  VelocityMux::Selection sel;
  for (double t = 0.1; t < 1.0; t += 0.1) {
    easynav::velocity_command::propose(
      nav_state, VelocitySource::CONTROLLER, cmd(std::nan("")));
    sel = mux.select(nav_state, at(t));
  }
  EXPECT_EQ(sel.choice, VelocityMux::Choice::TIMEOUT);
  EXPECT_TRUE(sel.discarded);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
}
