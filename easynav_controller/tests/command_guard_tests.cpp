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
/// \brief safety::CommandGuard around the VelocityMux: timeout, non-finite commands, new
/// controller commands, keepalive, QoS and diagnostics.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_controller/VelocityMux.hpp"
#include "easynav_controller/safety/CommandGuard.hpp"
#include "easynav_core/VelocityCommand.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
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

rclcpp::Time at(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

}  // namespace

class CommandGuardTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  // A guard configured with these parameters.
  bool configure(double cmd_timeout, double keepalive_period = 0.0)
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "guard_test_node", rclcpp::NodeOptions().parameter_overrides(
    {
      {"cmd_timeout", cmd_timeout},
      {"cmd_vel_keepalive_period", keepalive_period}}));
    guard_.declare_parameters(*node_);
    return guard_.configure(*node_);
  }

  // One RT cycle of the velocity output, as ControllerNode runs it.
  VelocityMux::Selection step(double t)
  {
    discarded_ = guard_.discard_non_finite(nav_state_);
    auto selection = guard_.supervise(mux_.select(nav_state_), at(t));
    guard_.report(nav_state_, discarded_, selection.fresh);
    return selection;
  }

  void propose(VelocitySource source, const geometry_msgs::msg::TwistStamped & c)
  {
    easynav::velocity_command::propose(nav_state_, source, c);
  }

  std::optional<DiagnosticStatus> diagnostic() const
  {
    if (!nav_state_.has("diagnostics.cmd_vel")) {return std::nullopt;}
    return nav_state_.get_safe<DiagnosticStatus>("diagnostics.cmd_vel");
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::safety::CommandGuard guard_;
  VelocityMux mux_;
  easynav::NavState nav_state_;
  bool discarded_ {false};
};

// ─── Timeout ─────────────────────────────────────────────────────────────────────────────────

TEST_F(CommandGuardTest, AHeldNonZeroTargetIsZeroedAfterTheTimeout)
{
  ASSERT_TRUE(configure(0.5));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  EXPECT_EQ(step(10.0).choice, VelocityMux::Choice::CONTROLLER);

  auto sel = step(10.3);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);
  EXPECT_FALSE(guard_.timed_out());

  // Exactly at the timeout: still kept.
  sel = step(10.5);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);
  EXPECT_FALSE(guard_.timed_out());

  sel = step(10.51);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(sel.fresh) << "the new zero target must be published";
  EXPECT_TRUE(sel.smooth) << "it must brake within the limits";
  EXPECT_TRUE(guard_.timed_out());

  // Stays timed out, without a new target each cycle.
  for (double t = 10.6; t < 12.0; t += 0.1) {
    sel = step(t);
    EXPECT_FALSE(sel.fresh);
    EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
    EXPECT_TRUE(guard_.timed_out());
  }

  // A new command ends it, and the timeout counts again from it.
  propose(VelocitySource::CONTROLLER, cmd(0.3));
  sel = step(12.0);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.3);
  EXPECT_FALSE(guard_.timed_out());
  EXPECT_DOUBLE_EQ(step(12.4).cmd.twist.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(step(12.6).cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(guard_.timed_out());
}

TEST_F(CommandGuardTest, AZeroTargetNeverTimesOut)
{
  ASSERT_TRUE(configure(0.1));

  step(0.0);  // Nothing ever proposed.
  step(100.0);
  EXPECT_FALSE(guard_.timed_out());

  propose(VelocitySource::CONTROLLER, cmd(0.0));  // A zero command, then silence.
  step(100.0);
  step(200.0);
  EXPECT_FALSE(guard_.timed_out());
}

TEST_F(CommandGuardTest, ZeroTimeoutDisablesIt)
{
  ASSERT_TRUE(configure(0.0));
  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(0.0);
  EXPECT_DOUBLE_EQ(step(1000.0).cmd.twist.linear.x, 0.5);
  EXPECT_FALSE(guard_.timed_out());
}

TEST_F(CommandGuardTest, AnySourceKeepsTheCommandAlive)
{
  ASSERT_TRUE(configure(0.5));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(0.0);
  propose(VelocitySource::TAKEOVER, cmd(-0.1));
  step(0.4);
  propose(VelocitySource::OVERRIDE, cmd(0.2));
  step(0.8);

  EXPECT_DOUBLE_EQ(step(1.2).cmd.twist.linear.x, 0.2);
  EXPECT_FALSE(guard_.timed_out());
  EXPECT_DOUBLE_EQ(step(1.31).cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(guard_.timed_out());
}

TEST_F(CommandGuardTest, PauseWhileTimedOutCommandsZero)
{
  ASSERT_TRUE(configure(0.1));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(0.0);
  step(1.0);
  ASSERT_TRUE(guard_.timed_out());

  nav_state_.set("navigation_paused", true);
  const auto sel = step(1.1);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::PAUSED);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(guard_.timed_out()) << "pausing is not a new command";
}

TEST_F(CommandGuardTest, AfterTheClockJumpsBackTheTimeoutCountsFromTheJump)
{
  // E.g. a simulation restarted.
  ASSERT_TRUE(configure(0.5));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(100.0);

  // Not stopped by the jump itself...
  EXPECT_DOUBLE_EQ(step(1.0).cmd.twist.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(step(1.5).cmd.twist.linear.x, 0.5);
  // ...but not held until the clock catches up (~99 s) either.
  EXPECT_DOUBLE_EQ(step(1.51).cmd.twist.linear.x, 0.0);
  EXPECT_TRUE(guard_.timed_out());
}

TEST_F(CommandGuardTest, RepeatedJumpsBackStillTimeOut)
{
  ASSERT_TRUE(configure(0.5));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(100.0);
  step(50.0);
  step(10.0);
  EXPECT_DOUBLE_EQ(step(10.4).cmd.twist.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(step(10.6).cmd.twist.linear.x, 0.0);
}

TEST_F(CommandGuardTest, ACommandAfterTheJumpRestartsTheTimeout)
{
  ASSERT_TRUE(configure(0.5));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(100.0);
  propose(VelocitySource::CONTROLLER, cmd(0.3));
  EXPECT_EQ(step(2.0).choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(step(2.4).cmd.twist.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(step(2.6).cmd.twist.linear.x, 0.0);
}

TEST_F(CommandGuardTest, ResetForgetsTheTimeout)
{
  ASSERT_TRUE(configure(0.1));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(0.0);
  step(1.0);
  ASSERT_TRUE(guard_.timed_out());

  guard_.reset();
  mux_.reset();
  EXPECT_FALSE(guard_.timed_out());
  EXPECT_DOUBLE_EQ(step(2.0).cmd.twist.linear.x, 0.0);
  EXPECT_FALSE(guard_.timed_out());
}

// ─── Non-finite commands ────────────────────────────────────────────────────────────────────

TEST_F(CommandGuardTest, NonFiniteProposalsAreDiscarded)
{
  ASSERT_TRUE(configure(0.0));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(0.0);

  // A NaN from the controller: discarded, the last valid target is kept.
  propose(VelocitySource::CONTROLLER, cmd(std::nan("")));
  auto sel = step(0.01);
  EXPECT_TRUE(discarded_);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);
  EXPECT_FALSE(sel.fresh);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.5);

  // An infinite override in any axis falls back to the next valid source.
  auto inf = cmd(0.0);
  inf.twist.angular.z = std::numeric_limits<double>::infinity();
  propose(VelocitySource::OVERRIDE, inf);
  propose(VelocitySource::CONTROLLER, cmd(0.2));
  sel = step(0.02);
  EXPECT_TRUE(discarded_);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.2);

  auto nan_y = cmd(0.0);
  nan_y.twist.linear.y = std::nan("");
  propose(VelocitySource::TAKEOVER, nan_y);
  sel = step(0.03);
  EXPECT_TRUE(discarded_);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::NONE);

  // Valid again: nothing discarded.
  propose(VelocitySource::CONTROLLER, cmd(0.1));
  sel = step(0.04);
  EXPECT_FALSE(discarded_);
  EXPECT_EQ(sel.choice, VelocityMux::Choice::CONTROLLER);
}

TEST_F(CommandGuardTest, OnlyNonFiniteProposalsEndInATimeout)
{
  // A controller producing only NaN is as silent as one producing nothing.
  ASSERT_TRUE(configure(0.5));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(0.0);

  VelocityMux::Selection sel;
  for (double t = 0.1; t < 1.0; t += 0.1) {
    propose(VelocitySource::CONTROLLER, cmd(std::nan("")));
    sel = step(t);
  }
  EXPECT_TRUE(guard_.timed_out());
  EXPECT_TRUE(discarded_);
  EXPECT_DOUBLE_EQ(sel.cmd.twist.linear.x, 0.0);
}

// ─── New controller commands ────────────────────────────────────────────────────────────────

TEST_F(CommandGuardTest, OnlyNewControllerCommandsAreProposed)
{
  ASSERT_TRUE(configure(0.5));
  auto c = cmd(0.5);
  c.header.stamp = at(1.0);

  EXPECT_TRUE(guard_.is_new(c));
  EXPECT_FALSE(guard_.is_new(c)) << "same stamp, same value";

  c.header.stamp = at(1.1);
  EXPECT_TRUE(guard_.is_new(c)) << "a new stamp";
  c.twist.linear.x = 0.4;
  EXPECT_TRUE(guard_.is_new(c)) << "a new value, same stamp";
  EXPECT_FALSE(guard_.is_new(c));

  guard_.reset();
  EXPECT_TRUE(guard_.is_new(c)) << "forgotten on reset";
}

// ─── Parameters and QoS ─────────────────────────────────────────────────────────────────────

TEST_F(CommandGuardTest, InvalidParametersAreRejected)
{
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto & [timeout, keepalive] : std::vector<std::pair<double, double>>{
    {-0.1, 0.0}, {std::nan(""), 0.0}, {inf, 0.0}, {0.5, -1.0}, {0.5, std::nan("")},
    {0.5, inf}})
  {
    EXPECT_FALSE(configure(timeout, keepalive)) << timeout << " " << keepalive;
  }
  EXPECT_TRUE(configure(0.0, 0.0));
  EXPECT_TRUE(configure(0.5, 0.1));
  EXPECT_DOUBLE_EQ(guard_.cmd_timeout(), 0.5);
  EXPECT_DOUBLE_EQ(guard_.keepalive_period(), 0.1);
}

TEST_F(CommandGuardTest, TimeoutMustBeLongerThanTheControllerPeriod)
{
  ASSERT_TRUE(configure(0.5));
  EXPECT_FALSE(guard_.check_controller_period(0.5, "ctrl"));
  EXPECT_FALSE(guard_.check_controller_period(1.0, "ctrl"));
  EXPECT_TRUE(guard_.check_controller_period(0.49, "ctrl"));

  ASSERT_TRUE(configure(0.0));
  EXPECT_TRUE(guard_.check_controller_period(10.0, "ctrl")) << "no timeout, nothing to check";
}

TEST_F(CommandGuardTest, QosKeepsOnlyTheLatestCommandAndOffersADeadlineWithAKeepalive)
{
  ASSERT_TRUE(configure(0.5, 0.0));
  EXPECT_EQ(guard_.qos().depth(), 1u);
  EXPECT_EQ(guard_.qos().deadline(), rclcpp::QoS(1).deadline()) << "the default: none";

  ASSERT_TRUE(configure(0.5, 0.05));
  EXPECT_EQ(guard_.qos().depth(), 1u);
  EXPECT_EQ(guard_.qos().deadline(), rclcpp::Duration::from_seconds(0.1));
  EXPECT_EQ(guard_.qos().liveliness(), rclcpp::LivelinessPolicy::Automatic);
  EXPECT_EQ(guard_.qos().liveliness_lease_duration(), rclcpp::Duration::from_seconds(0.1));
}

// ─── Keepalive ──────────────────────────────────────────────────────────────────────────────

TEST_F(CommandGuardTest, KeepaliveIsDueEveryPeriodAfterThePublication)
{
  ASSERT_TRUE(configure(0.5, 0.1));
  EXPECT_TRUE(guard_.keepalive_due(at(5.0))) << "nothing published yet";

  guard_.published(at(5.0));
  EXPECT_FALSE(guard_.keepalive_due(at(5.0)));
  EXPECT_FALSE(guard_.keepalive_due(at(5.09)));
  EXPECT_TRUE(guard_.keepalive_due(at(5.1)));
  EXPECT_TRUE(guard_.keepalive_due(at(7.0)));

  guard_.reset();
  EXPECT_TRUE(guard_.keepalive_due(at(5.01))) << "forgotten on reset";
}

TEST_F(CommandGuardTest, KeepaliveIsDueRightAfterTheClockJumpsBack)
{
  ASSERT_TRUE(configure(0.5, 0.1));
  guard_.published(at(100.0));
  EXPECT_TRUE(guard_.keepalive_due(at(1.0))) << "not ~99 s later";
}

TEST_F(CommandGuardTest, NoKeepaliveIfDisabled)
{
  ASSERT_TRUE(configure(0.5, 0.0));
  EXPECT_FALSE(guard_.keepalive_due(at(0.0)));
  guard_.published(at(0.0));
  EXPECT_FALSE(guard_.keepalive_due(at(1000.0)));
}

// ─── Diagnostics ────────────────────────────────────────────────────────────────────────────

TEST_F(CommandGuardTest, DiagnosticIsWrittenOnlyAfterAProblemAndOnChanges)
{
  ASSERT_TRUE(configure(0.1));

  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(0.0);
  EXPECT_FALSE(diagnostic()) << "nothing to report while all is fine";

  step(1.0);  // Timed out.
  auto diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR);
  EXPECT_EQ(diag->name, "cmd_vel");
  EXPECT_EQ(diag->hardware_id, "guard_test_node");
  EXPECT_NE(diag->message.find("0.1 s"), std::string::npos) << diag->message;
  const auto group = nav_state_.get_group_keys("diagnostics");
  EXPECT_EQ(std::count(group.begin(), group.end(), "diagnostics.cmd_vel"), 1);

  propose(VelocitySource::CONTROLLER, cmd(std::nan("")));
  step(1.1);
  diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR) << "still timed out";

  propose(VelocitySource::CONTROLLER, cmd(0.2));
  step(1.2);
  diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::OK);

  propose(VelocitySource::CONTROLLER, cmd(std::nan("")));
  step(1.21);
  diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR);
  EXPECT_NE(diag->message.find("Non-finite"), std::string::npos) << diag->message;

  // Writing it again does not duplicate the group entry.
  propose(VelocitySource::CONTROLLER, cmd(0.2));
  step(1.22);
  const auto keys = nav_state_.get_group_keys("diagnostics");
  EXPECT_EQ(std::count(keys.begin(), keys.end(), "diagnostics.cmd_vel"), 1);
}

TEST_F(CommandGuardTest, AnErrorReportedBeforeAResetIsClearedAfterIt)
{
  // NavState outlives a reconfiguration: its ERROR must be cleared by the next OK.
  ASSERT_TRUE(configure(0.1));
  propose(VelocitySource::CONTROLLER, cmd(0.5));
  step(0.0);
  step(1.0);
  ASSERT_EQ(diagnostic()->level, DiagnosticStatus::ERROR);

  ASSERT_TRUE(guard_.configure(*node_));  // As on a new configure (resets).
  mux_.reset();
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::ERROR);

  propose(VelocitySource::CONTROLLER, cmd(0.3));
  step(2.0);
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::OK);
}
