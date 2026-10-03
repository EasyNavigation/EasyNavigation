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

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_controller/VelocityMux.hpp"
#include "easynav_core/SafetyChannel.hpp"

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
  auto sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);
  EXPECT_TRUE(sel.smooth);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::TAKEOVER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, -0.1);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  easynav::velocity_command::propose(nav_state, VelocitySource::OVERRIDE, cmd(0.0));
  sel = mux.select(nav_state);
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
  const auto sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::PAUSED);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(sel.smooth) << "pausing must brake within the limits";
}

TEST(VelocityMuxTest, ProposalsAreConsumedAndTheLastTargetIsKept)
{
  VelocityMux mux;
  easynav::NavState nav_state;

  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  EXPECT_TRUE(mux.select(nav_state).fresh);

  // Nothing new this cycle: same target, not fresh, and the proposal did not linger.
  const auto sel = mux.select(nav_state);
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

  const auto sel = mux.select(nav_state);
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
  EXPECT_EQ(mux.select(nav_state).choice, VelocityMux::Choice::CONTROLLER);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  EXPECT_EQ(mux.select(nav_state).choice, VelocityMux::Choice::TAKEOVER);

  easynav::velocity_command::propose(nav_state, VelocitySource::OVERRIDE, cmd(0.0));
  const auto over = mux.select(nav_state);
  EXPECT_EQ(over.choice, VelocityMux::Choice::OVERRIDE);
  EXPECT_FALSE(over.smooth);

  // Nothing proposed: the last target is kept, not fresh.
  const auto none = mux.select(nav_state);
  EXPECT_EQ(none.choice, VelocityMux::Choice::NONE);
  EXPECT_FALSE(none.fresh);
  EXPECT_DOUBLE_EQ(none.cmd.twist.linear.x, 0.0);

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  const auto back = mux.select(nav_state);
  EXPECT_EQ(back.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(back.cmd.twist.linear.x, 0.5);
}

namespace
{

void set_protective_stop(easynav::NavState & nav_state, bool stop)
{
  easynav::SafetyChannelState state;
  state.protective_stop = stop;
  nav_state.set(easynav::kSafetyStatusKey, state);
}

void propose_all(easynav::NavState & nav_state)
{
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  easynav::velocity_command::propose(nav_state, VelocitySource::TAKEOVER, cmd(-0.1));
  easynav::velocity_command::propose(nav_state, VelocitySource::OVERRIDE, cmd(0.2));
}

}  // namespace

TEST(VelocityMuxTest, AProtectiveStopWinsOverEverySourceAndConsumesThem)
{
  VelocityMux mux;
  easynav::NavState nav_state;
  nav_state.set("navigation_paused", true);
  set_protective_stop(nav_state, true);
  propose_all(nav_state);

  const auto sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::SAFETY_STOP);
  EXPECT_EQ(sel.cmd.twist, geometry_msgs::msg::Twist());
  EXPECT_TRUE(sel.fresh);
  for (const auto source :
    {VelocitySource::CONTROLLER, VelocitySource::TAKEOVER, VelocitySource::OVERRIDE})
  {
    EXPECT_FALSE(easynav::velocity_command::peek(nav_state, source).has_value())
      << "no proposal may linger until the stop is released";
  }
}

TEST(VelocityMuxTest, AProtectiveStopIsFreshOnEntryAndWhileTheControllerProposes)
{
  VelocityMux mux;
  easynav::NavState nav_state;
  set_protective_stop(nav_state, true);

  EXPECT_TRUE(mux.select(nav_state).fresh) << "entering the stop is a new (zero) target";
  EXPECT_FALSE(mux.select(nav_state).fresh) << "nothing new";

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  EXPECT_TRUE(mux.select(nav_state).fresh) << "the controller is alive: no command timeout";
}

TEST(VelocityMuxTest, NoRestrictionWithoutStopOrWithoutStatus)
{
  VelocityMux mux;
  easynav::NavState nav_state;
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  EXPECT_EQ(mux.select(nav_state).choice, VelocityMux::Choice::CONTROLLER);

  set_protective_stop(nav_state, false);
  easynav::SafetyChannelState limited;
  limited.max_linear_vel = 0.1;  // A speed limit is the smoother's business, not the mux's.
  nav_state.set(easynav::kSafetyStatusKey, limited);
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  const auto sel = mux.select(nav_state);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);
}

TEST(VelocityMuxTest, AfterAProtectiveStopNothingOldIsResumed)
{
  // controller -> stop -> released with no new proposal -> controller
  VelocityMux mux;
  easynav::NavState nav_state;
  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.5));
  ASSERT_EQ(mux.select(nav_state).choice, VelocityMux::Choice::CONTROLLER);

  set_protective_stop(nav_state, true);
  propose_all(nav_state);
  ASSERT_EQ(mux.select(nav_state).choice, VelocityMux::Choice::SAFETY_STOP);

  set_protective_stop(nav_state, false);
  const auto released = mux.select(nav_state);
  EXPECT_EQ(released.choice, VelocityMux::Choice::NONE);
  EXPECT_EQ(released.cmd.twist, geometry_msgs::msg::Twist()) << "the last target is the stop";

  easynav::velocity_command::propose(nav_state, VelocitySource::CONTROLLER, cmd(0.4));
  const auto back = mux.select(nav_state);
  EXPECT_EQ(back.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(back.cmd.twist.linear.x, 0.4);
}

TEST(VelocityMuxTest, ResetForgetsTheStop)
{
  VelocityMux mux;
  easynav::NavState nav_state;
  set_protective_stop(nav_state, true);
  ASSERT_TRUE(mux.select(nav_state).fresh);
  ASSERT_FALSE(mux.select(nav_state).fresh);

  mux.reset();
  EXPECT_TRUE(mux.select(nav_state).fresh) << "after a reset, the stop is entered again";
}
