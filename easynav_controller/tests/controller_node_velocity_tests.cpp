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
/// \brief ControllerNode as the single exit of the velocity command: robot limits for the
/// controller plugins, selection among sources (mux), smoothing, publication, and braking on
/// deactivation.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_core/SafetyChannel.hpp"
#include "easynav_core/VelocityCommand.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{

class LimitsReadingController : public easynav::ControllerMethodBase {};

rclcpp::NodeOptions limits_options(std::vector<rclcpp::Parameter> extra = {})
{
  std::vector<rclcpp::Parameter> params {
    {"robot_limits.max_linear_vel", 1.0},
    {"robot_limits.min_linear_vel", -0.2},
    {"robot_limits.max_angular_vel", 1.5},
    {"robot_limits.max_linear_acc", 2.0},
    {"robot_limits.max_linear_decel", 4.0},
    {"robot_limits.max_angular_acc", 3.0},
    {"robot_limits.max_angular_decel", 6.0},
    // Stamped, so each published command carries the time it was computed at.
    {"use_cmd_vel_stamped", true},
  };
  params.insert(params.end(), extra.begin(), extra.end());
  return rclcpp::NodeOptions().parameter_overrides(params);
}

constexpr double kMaxLinearAcc = 2.0;
constexpr double kMaxLinearDecel = 4.0;
constexpr double kTolerance = 1e-6;

geometry_msgs::msg::TwistStamped cmd(double vx)
{
  geometry_msgs::msg::TwistStamped c;
  c.twist.linear.x = vx;
  return c;
}

}  // namespace

class ControllerNodeVelocityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void make_active_node(std::vector<rclcpp::Parameter> extra = {})
  {
    node_ = std::make_shared<easynav::ControllerNode>(limits_options(extra));
    node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

    listener_ = rclcpp::Node::make_shared("velocity_listener");
    sub_ = listener_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_stamped", 1000,
      [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) {
        received_.push_back(msg->twist.linear.x);
        stamps_.push_back(rclcpp::Time(msg->header.stamp));
      });
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_node(listener_);
    const auto start = std::chrono::steady_clock::now();
    while (sub_->get_publisher_count() == 0 && std::chrono::steady_clock::now() - start < 2s) {
      exe_->spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(sub_->get_publisher_count(), 0u);
  }

  // One RT cycle: the commands proposed by each source, then the velocity output.
  void cycle(
    std::optional<double> controller, std::optional<double> takeover = std::nullopt,
    std::optional<double> override_vel = std::nullopt)
  {
    if (controller) {
      easynav::velocity_command::propose(
        *nav_state_, easynav::VelocitySource::CONTROLLER, cmd(*controller));
    }
    if (takeover) {
      easynav::velocity_command::propose(
        *nav_state_, easynav::VelocitySource::TAKEOVER, cmd(*takeover));
    }
    if (override_vel) {
      easynav::velocity_command::propose(
        *nav_state_, easynav::VelocitySource::OVERRIDE, cmd(*override_vel));
    }
    node_->publish_cmd_vel_rt(nav_state_);
    rclcpp::sleep_for(10ms);
    exe_->spin_some();
  }

  void spin_for(std::chrono::milliseconds d)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < d) {
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
  }

  easynav::ControllerNode::SharedPtr node_;
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::shared_ptr<easynav::NavState> nav_state_ = std::make_shared<easynav::NavState>();
  std::vector<double> received_;
  std::vector<rclcpp::Time> stamps_;

  // Every change between consecutive commands respects the acceleration (speeding up) or the
  // deceleration (slowing down) limit, over the time between them.
  void expect_within_acceleration_limits(size_t from = 0) const
  {
    for (size_t i = from + 1; i < received_.size(); ++i) {
      const double dv = received_[i] - received_[i - 1];
      // Difference first: absolute times as doubles lose ~0.2 us of resolution.
      const double dt = (stamps_[i] - stamps_[i - 1]).seconds();
      const bool slowing = std::abs(received_[i]) < std::abs(received_[i - 1]);
      const double limit = slowing ? kMaxLinearDecel : kMaxLinearAcc;
      EXPECT_LE(std::abs(dv), limit * dt + kTolerance) <<
        "step " << i << ": " << received_[i - 1] << " -> " << received_[i] << " in " << dt <<
        " s exceeds " << (slowing ? "deceleration" : "acceleration") << " limit " << limit;
    }
  }
};

TEST_F(ControllerNodeVelocityTest, ControllerPluginsQueryTheRobotLimitsFromTheNode)
{
  auto node = std::make_shared<easynav::ControllerNode>(limits_options());
  LimitsReadingController controller;
  controller.initialize(node, "limits_reader");

  const auto limits = controller.get_robot_limits();
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, 1.0);
  EXPECT_DOUBLE_EQ(limits.min_linear_vel, -0.2);
  EXPECT_DOUBLE_EQ(limits.max_angular_vel, 1.5);
  EXPECT_DOUBLE_EQ(limits.max_linear_acc, 2.0);
  EXPECT_DOUBLE_EQ(limits.max_linear_decel, 4.0);
  EXPECT_DOUBLE_EQ(limits.max_angular_acc, 3.0);
  EXPECT_DOUBLE_EQ(limits.max_angular_decel, 6.0);

  // Outside a ControllerNode (e.g. a plain node in a test), the defaults.
  auto plain = std::make_shared<rclcpp_lifecycle::LifecycleNode>("plain_parent");
  LimitsReadingController orphan;
  orphan.initialize(plain, "limits_reader");
  EXPECT_DOUBLE_EQ(orphan.get_robot_limits().max_linear_vel, easynav::RobotLimits{}.max_linear_vel);
}

TEST_F(ControllerNodeVelocityTest, PublishesTheCommandRampingWithinTheLimits)
{
  make_active_node();

  // Asking for 5 m/s from standstill: never above 1.0 m/s, and never a jump above acc * dt.
  for (int i = 0; i < 100; ++i) {
    cycle(5.0);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 5u);
  EXPECT_LT(received_[1], 0.5) << "the first commands must be a ramp, not the target";
  EXPECT_DOUBLE_EQ(received_.back(), 1.0);
  for (size_t i = 1; i < received_.size(); ++i) {
    EXPECT_GE(received_[i], received_[i - 1]);
    EXPECT_LE(received_[i], 1.0);
  }
  expect_within_acceleration_limits();
}

TEST_F(ControllerNodeVelocityTest, ControllerAskingToStopDeadIsBrakedWithinTheDecelerationLimit)
{
  // The case that makes a robot "stick": the controller jumps from full speed to zero.
  make_active_node();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(received_.back(), 1.0);

  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 60; ++i) {
    cycle(0.0);
  }
  spin_for(100ms);

  ASSERT_GT(received_.size(), 3u);
  EXPECT_GT(received_.front(), 0.0) << "it must not stop dead in one step";
  EXPECT_DOUBLE_EQ(received_.back(), 0.0);
  expect_within_acceleration_limits();
}

TEST_F(ControllerNodeVelocityTest, DeactivationBrakesInARampAndEndsWithAnExactZero)
{
  make_active_node();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(received_.back(), 1.0);

  received_.clear();
  stamps_.clear();
  node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  spin_for(300ms);

  // 1.0 m/s at 4.0 m/s^2 takes 0.25 s: several decreasing steps, not a dead stop...
  ASSERT_GT(received_.size(), 3u);
  EXPECT_GT(received_.front(), 0.0);
  for (size_t i = 1; i < received_.size(); ++i) {
    EXPECT_LE(received_[i], received_[i - 1]);
  }
  // ...and the very last command is an exact zero.
  EXPECT_EQ(received_.back(), 0.0);
  // The ramp itself respects the deceleration limit (the final zero may come from a timeout,
  // so it is excluded).
  received_.pop_back();
  stamps_.pop_back();
  expect_within_acceleration_limits();
}

TEST_F(ControllerNodeVelocityTest, PauseBrakesInARampAndResumeRampsUp)
{
  make_active_node();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(received_.back(), 1.0);

  // Paused: the controller keeps asking for 1.0, the robot brakes within the limit.
  nav_state_->set("navigation_paused", true);
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 60; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  EXPECT_GT(received_.front(), 0.0) << "it must not stop dead in one step";
  EXPECT_DOUBLE_EQ(received_.back(), 0.0);
  expect_within_acceleration_limits();

  // Resumed: back to 1.0 in a ramp.
  nav_state_->set("navigation_paused", false);
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  EXPECT_LT(received_.front(), 0.5);
  EXPECT_DOUBLE_EQ(received_.back(), 1.0);
  expect_within_acceleration_limits();
}

TEST_F(ControllerNodeVelocityTest, KeepsRampingWithoutNewCommandsAndThenStopsPublishing)
{
  make_active_node({{"cmd_timeout", 0.0}});  // No timeout: the last target is kept.
  cycle(1.0);  // A single command...
  for (int i = 0; i < 100; ++i) {
    cycle(std::nullopt);  // ...then no new ones: the ramp still reaches it.
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  EXPECT_DOUBLE_EQ(received_.back(), 1.0);

  // Target reached and nothing new: nothing else is published.
  received_.clear();
  for (int i = 0; i < 20; ++i) {
    cycle(std::nullopt);
  }
  spin_for(100ms);
  EXPECT_TRUE(received_.empty());
}

TEST_F(ControllerNodeVelocityTest, SetRobotLimitsAppliesToTheSmoother)
{
  make_active_node();
  auto limits = node_->get_robot_limits();
  limits.max_linear_vel = 0.3;
  node_->set_robot_limits(limits);
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().max_linear_vel, 0.3);

  for (int i = 0; i < 100; ++i) {
    cycle(5.0);
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  EXPECT_DOUBLE_EQ(received_.back(), 0.3);
}

TEST_F(ControllerNodeVelocityTest, KnowsWhichLimitsWereConfigured)
{
  // Only some limits given: the rest are defaults, not configured.
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions().parameter_overrides({
    {"robot_limits.max_linear_vel", 0.8},
    {"robot_limits.max_angular_acc", easynav::RobotLimits{}.max_angular_acc},
    }));
  EXPECT_TRUE(node->is_robot_limit_configured("max_linear_vel"));
  EXPECT_TRUE(node->is_robot_limit_configured("max_angular_acc")) << "given, even if default";
  EXPECT_FALSE(node->is_robot_limit_configured("min_linear_vel"));
  EXPECT_FALSE(node->is_robot_limit_configured("max_linear_decel"));

  // Changed at runtime while unconfigured: applied and configured on the next configure.
  node->set_parameter(rclcpp::Parameter("robot_limits.max_linear_decel", 3.0));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_TRUE(node->is_robot_limit_configured("max_linear_decel"));
  EXPECT_DOUBLE_EQ(node->get_robot_limits().max_linear_decel, 3.0);
}

TEST_F(ControllerNodeVelocityTest, ReconfigureRestoresTheConfiguredLimits)
{
  // Limits changed by set_robot_limits() (e.g. deprecated values) last until the next configure.
  auto node = std::make_shared<easynav::ControllerNode>(limits_options());
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  auto limits = node->get_robot_limits();
  limits.max_linear_vel = 0.1;
  node->set_robot_limits(limits);

  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_DOUBLE_EQ(node->get_robot_limits().max_linear_vel, 1.0);
}

// Deprecated per-controller limit parameters.
namespace
{
const easynav::LegacyRobotLimitNames kLegacy{
  "old_max_speed", "", "old_max_turn", "old_max_acc", "", "", ""};
}  // namespace

TEST_F(ControllerNodeVelocityTest, DeprecatedLimitsApplyWhenRobotLimitsAreNotConfigured)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions().parameter_overrides({
    {"ctrl.old_max_speed", 0.8},
    {"ctrl.old_max_acc", 1.5},
    }));
  LimitsReadingController controller;
  controller.initialize(node, "ctrl");

  const auto limits = controller.get_robot_limits(kLegacy);
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, 0.8);
  EXPECT_DOUBLE_EQ(limits.max_linear_acc, 1.5);
  EXPECT_DOUBLE_EQ(limits.max_angular_vel, easynav::RobotLimits{}.max_angular_vel) <<
    "not configured under its old name either: default";
  EXPECT_FALSE(node->has_parameter("ctrl.old_max_turn")) << "unconfigured old names not declared";
  // The node enforces the same limits (smoother).
  EXPECT_DOUBLE_EQ(node->get_robot_limits().max_linear_vel, 0.8);
  EXPECT_DOUBLE_EQ(node->get_robot_limits().max_linear_acc, 1.5);
}

TEST_F(ControllerNodeVelocityTest, RobotLimitsTakePrecedenceOverDeprecatedOnes)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions().parameter_overrides({
    {"robot_limits.max_linear_vel", 0.6},
    {"ctrl.old_max_speed", 0.8},
    {"ctrl.old_max_turn", 2.0},
    }));
  LimitsReadingController controller;
  controller.initialize(node, "ctrl");

  const auto limits = controller.get_robot_limits(kLegacy);
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, 0.6) << "both given: the new one wins";
  EXPECT_DOUBLE_EQ(limits.max_angular_vel, 2.0) << "only the old one given: applied";
  EXPECT_DOUBLE_EQ(node->get_robot_limits().max_linear_vel, 0.6);
  EXPECT_DOUBLE_EQ(node->get_robot_limits().max_angular_vel, 2.0);
}

TEST_F(ControllerNodeVelocityTest, NoDeprecatedLimitsMeansRobotLimits)
{
  auto node = std::make_shared<easynav::ControllerNode>(limits_options());
  LimitsReadingController controller;
  controller.initialize(node, "ctrl");

  const auto limits = controller.get_robot_limits(kLegacy);
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, 1.0);
  EXPECT_DOUBLE_EQ(limits.max_linear_acc, 2.0);
  EXPECT_FALSE(node->has_parameter("ctrl.old_max_speed"));
}

TEST_F(ControllerNodeVelocityTest, DeprecatedLimitsSurviveReconfiguration)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions().parameter_overrides({{"ctrl.old_max_speed", 0.8}}));
  for (int i = 0; i < 3; ++i) {
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);  // Re-reads robot_limits.
    LimitsReadingController controller;
    controller.initialize(node, "ctrl");
    EXPECT_DOUBLE_EQ(controller.get_robot_limits(kLegacy).max_linear_vel, 0.8) << "round " << i;
    EXPECT_DOUBLE_EQ(node->get_robot_limits().max_linear_vel, 0.8) << "round " << i;
    node->trigger_transition(Transition::TRANSITION_CLEANUP);
  }
}

TEST_F(ControllerNodeVelocityTest, DeprecatedLimitsOutsideAControllerNode)
{
  auto plain = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "plain_legacy_parent",
    rclcpp::NodeOptions().parameter_overrides({{"ctrl.old_max_speed", 0.8}}));
  LimitsReadingController controller;
  controller.initialize(plain, "ctrl");
  EXPECT_DOUBLE_EQ(controller.get_robot_limits(kLegacy).max_linear_vel, 0.8);
}

// Selection among sources (VelocityMux), end to end through ControllerNode.

TEST_F(ControllerNodeVelocityTest, OverrideBypassesTheSmoother)
{
  make_active_node();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(received_.back(), 1.0);

  // Emergency: from 1.0 m/s to 0 in a single cycle, beyond the deceleration limit.
  received_.clear();
  stamps_.clear();
  cycle(1.0, std::nullopt, 0.0);
  spin_for(100ms);
  ASSERT_EQ(received_.size(), 1u);
  EXPECT_DOUBLE_EQ(received_.back(), 0.0);
}

TEST_F(ControllerNodeVelocityTest, AfterAnOverrideTheRampStartsFromIt)
{
  make_active_node();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  cycle(1.0, std::nullopt, 0.0);  // Emergency stop.
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(received_.back(), 0.0);

  // The controller asks for 1.0 again: from 0, not from where it was before the override.
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  EXPECT_LT(received_.front(), 0.5);
  EXPECT_DOUBLE_EQ(received_.back(), 1.0);
  expect_within_acceleration_limits();
}

TEST_F(ControllerNodeVelocityTest, TakeoverIsPreferredAndSmoothedThenControlReturns)
{
  make_active_node();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(received_.back(), 1.0);

  // A takeover (e.g. a recovery backing up) wins over the controller, within the limits.
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0, -0.2);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  EXPECT_GT(received_.front(), 0.0) << "smoothed: no jump to the takeover's command";
  EXPECT_DOUBLE_EQ(received_.back(), -0.2);
  expect_within_acceleration_limits();

  // The takeover ends: the controller's command applies again, also in a ramp.
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  EXPECT_LE(received_.front(), 0.0) << "no sign jump: through zero first";
  EXPECT_DOUBLE_EQ(received_.back(), 1.0);
  expect_within_acceleration_limits();
}

TEST_F(ControllerNodeVelocityTest, TakeoverWinsOverPauseAndOverrideOverEverything)
{
  make_active_node();
  nav_state_->set("navigation_paused", true);

  // Paused, but something takes over the motion: it moves.
  for (int i = 0; i < 100; ++i) {
    cycle(1.0, 0.3);
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  EXPECT_DOUBLE_EQ(received_.back(), 0.3);

  // An override beats the takeover, the pause and the controller.
  cycle(1.0, 0.3, 0.05);
  spin_for(100ms);
  EXPECT_DOUBLE_EQ(received_.back(), 0.05);

  // Nothing but the controller while paused: back to zero, in a ramp.
  for (int i = 0; i < 60; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  EXPECT_DOUBLE_EQ(received_.back(), 0.0);
}

TEST_F(ControllerNodeVelocityTest, ProposalsArePrintedInTheNavStateDump)
{
  make_active_node();  // Registers the printers.
  easynav::velocity_command::propose(
    *nav_state_, easynav::VelocitySource::CONTROLLER, cmd(0.5));

  auto line = [this](const std::string & key) {
      std::istringstream lines(nav_state_->debug_string());
      std::string l;
      while (std::getline(lines, l)) {
        if (l.rfind(key + " = ", 0) == 0) {return l;}
      }
      return std::string();
    };
  EXPECT_NE(line("cmd_vel.proposal.controller").find("pending Twist with (0.5, 0, 0)"),
    std::string::npos) << line("cmd_vel.proposal.controller");

  node_->publish_cmd_vel_rt(nav_state_);  // The mux takes it.
  EXPECT_NE(line("cmd_vel.proposal.controller").find("taken Twist with (0.5, 0, 0)"),
    std::string::npos) << line("cmd_vel.proposal.controller");
}

namespace
{

std::optional<diagnostic_msgs::msg::DiagnosticStatus> cmd_vel_diagnostic(
  const easynav::NavState & nav_state)
{
  if (!nav_state.has("diagnostics.cmd_vel")) {return std::nullopt;}
  return nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.cmd_vel");
}

}  // namespace

TEST_F(ControllerNodeVelocityTest, NoNewCommandsBrakesToZeroAfterTheTimeoutAndResumes)
{
  make_active_node({{"cmd_timeout", 0.2}});
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(received_.back(), 1.0);
  EXPECT_FALSE(cmd_vel_diagnostic(*nav_state_)) << "nothing to report while all is fine";

  // The controller goes silent: still 1.0 before the timeout, then a ramp down to zero.
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 60; ++i) {
    cycle(std::nullopt);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  EXPECT_DOUBLE_EQ(received_.back(), 0.0);
  EXPECT_GT(received_.front(), 0.0) << "it must not stop dead in one step";
  expect_within_acceleration_limits();

  auto diag = cmd_vel_diagnostic(*nav_state_);
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(diag->hardware_id, "controller_node");
  const auto group = nav_state_->get_group_keys("diagnostics");
  EXPECT_NE(std::find(group.begin(), group.end(), "diagnostics.cmd_vel"), group.end());

  // Commands again: it moves and the diagnostic goes back to OK.
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 100; ++i) {
    cycle(0.5);
  }
  spin_for(100ms);
  EXPECT_DOUBLE_EQ(received_.back(), 0.5);
  diag = cmd_vel_diagnostic(*nav_state_);
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ControllerNodeVelocityTest, TheDefaultTimeoutStopsAStaleCommand)
{
  make_active_node();  // cmd_timeout: 0.5 s by default.
  cycle(1.0);
  for (int i = 0; i < 100; ++i) {  // ~1 s without new commands.
    cycle(std::nullopt);
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  EXPECT_DOUBLE_EQ(received_.back(), 0.0);
}

TEST_F(ControllerNodeVelocityTest, NonFiniteCommandsAreNeverPublished)
{
  make_active_node({{"cmd_timeout", 0.0}});
  for (int i = 0; i < 100; ++i) {
    cycle(0.5);
  }
  for (int i = 0; i < 20; ++i) {
    cycle(std::nan(""));
  }
  for (int i = 0; i < 20; ++i) {
    cycle(std::numeric_limits<double>::infinity());
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  for (const auto v : received_) {
    EXPECT_TRUE(std::isfinite(v));
  }
  EXPECT_DOUBLE_EQ(received_.back(), 0.5) << "the last valid command is kept";
  auto diag = cmd_vel_diagnostic(*nav_state_);
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  cycle(0.4);
  diag = cmd_vel_diagnostic(*nav_state_);
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(ControllerNodeVelocityTest, KeepaliveRepublishesTheHeldCommand)
{
  make_active_node({{"cmd_timeout", 0.0}, {"cmd_vel_keepalive_period", 0.05}});
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(received_.back(), 1.0);

  // ~0.5 s with nothing new: republished about every 0.05 s.
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 50; ++i) {
    cycle(std::nullopt);
  }
  spin_for(50ms);
  EXPECT_GE(received_.size(), 5u);
  EXPECT_LE(received_.size(), 25u) << "not every cycle, only when due";
  for (const auto v : received_) {
    EXPECT_DOUBLE_EQ(v, 1.0);
  }
  for (size_t i = 1; i < stamps_.size(); ++i) {
    EXPECT_GE((stamps_[i] - stamps_[i - 1]).seconds(), 0.05 - 1e-3);
  }
}

TEST_F(ControllerNodeVelocityTest, KeepalivePublishesZeroAtRest)
{
  make_active_node({{"cmd_vel_keepalive_period", 0.05}});
  for (int i = 0; i < 30; ++i) {
    cycle(std::nullopt);
  }
  spin_for(50ms);
  EXPECT_GE(received_.size(), 3u);
  for (const auto v : received_) {
    EXPECT_DOUBLE_EQ(v, 0.0);
  }
}

TEST_F(ControllerNodeVelocityTest, VelocityQosKeepsOnlyTheLatestCommandAndOffersADeadline)
{
  {
    make_active_node();
    const auto info = node_->get_publishers_info_by_topic("cmd_vel_stamped");
    ASSERT_EQ(info.size(), 1u);
    EXPECT_EQ(info[0].qos_profile().depth(), 1u);
    // Infinite (the middleware reports it as a huge value): no keepalive, no deadline promise.
    EXPECT_GT(info[0].qos_profile().deadline().seconds(), 1e6);
    node_.reset();
  }
  {
    make_active_node({{"cmd_vel_keepalive_period", 0.05}});
    const auto info = node_->get_publishers_info_by_topic("cmd_vel_stamped");
    ASSERT_EQ(info.size(), 1u);
    EXPECT_EQ(info[0].qos_profile().deadline(), rclcpp::Duration(100ms));
    EXPECT_EQ(info[0].qos_profile().liveliness(), rclcpp::LivelinessPolicy::Automatic);
    EXPECT_EQ(info[0].qos_profile().liveliness_lease_duration(), rclcpp::Duration(100ms));
  }
}

TEST_F(ControllerNodeVelocityTest, ReceiverDetectsThatCommandsStopped)
{
  // E.g. the RT cycle hangs: the receiver's deadline is missed.
  make_active_node({{"cmd_vel_keepalive_period", 0.05}});
  std::atomic<int> missed {0};
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.deadline_callback =
    [&missed](rclcpp::QOSDeadlineRequestedInfo &) {++missed;};
  auto watchdog = listener_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_stamped", rclcpp::QoS(1).deadline(rclcpp::Duration(150ms)),
    [](geometry_msgs::msg::TwistStamped::UniquePtr) {}, options);
  spin_for(100ms);

  for (int i = 0; i < 40; ++i) {
    cycle(std::nullopt);
  }
  EXPECT_EQ(missed.load(), 0) << "commands keep arriving in time";

  spin_for(500ms);  // No RT cycles.
  EXPECT_GT(missed.load(), 0);
}

TEST_F(ControllerNodeVelocityTest, NegativeOrNonFiniteTimeoutsFailToConfigure)
{
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto & param : {rclcpp::Parameter("cmd_timeout", -0.1),
      rclcpp::Parameter("cmd_timeout", std::nan("")),
      rclcpp::Parameter("cmd_timeout", inf),
      rclcpp::Parameter("cmd_vel_keepalive_period", -1.0),
      rclcpp::Parameter("cmd_vel_keepalive_period", std::nan("")),
      rclcpp::Parameter("cmd_vel_keepalive_period", inf)})
  {
    auto node = std::make_shared<easynav::ControllerNode>(limits_options({param}));
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED) <<
      param.get_name();

    // Fixed, it configures.
    node->set_parameter(rclcpp::Parameter(param.get_name(), 0.0));
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE) << param.get_name();
  }
}

TEST_F(ControllerNodeVelocityTest, TimeoutMustBeLongerThanTheControllerPeriod)
{
  auto make = [](double cmd_timeout) {
      return std::make_shared<easynav::ControllerNode>(
        limits_options({
      {"controller_types", std::vector<std::string>{"ctrl"}},
      {"ctrl.plugin", std::string("easynav_controller/DummyController")},
      {"ctrl.rt_freq", 2.0},      // 0.5 s
      {"cmd_timeout", cmd_timeout}}));
    };

  // Equal to the period, or shorter: every command would time out.
  for (const double timeout : {0.5, 0.3}) {
    auto node = make(timeout);
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED) << timeout;
    EXPECT_EQ(node->get_loaded_controller(), "") << "the plugin is released on failure";
  }

  for (const double timeout : {0.0, 0.51}) {
    auto node = make(timeout);
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE) << timeout;
  }
}

TEST_F(ControllerNodeVelocityTest, ReconfigurationStartsWithNoTimeoutState)
{
  make_active_node({{"cmd_timeout", 0.1}});
  for (int i = 0; i < 50; ++i) {
    cycle(1.0);
  }
  for (int i = 0; i < 30; ++i) {
    cycle(std::nullopt);
  }
  ASSERT_TRUE(cmd_vel_diagnostic(*nav_state_));

  node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  // The ERROR reported before the reconfiguration is still in NavState until commands flow.
  auto diag = cmd_vel_diagnostic(*nav_state_);
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  received_.clear();
  for (int i = 0; i < 50; ++i) {
    cycle(0.5);
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  EXPECT_DOUBLE_EQ(received_.back(), 0.5);
  diag = cmd_vel_diagnostic(*nav_state_);
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, diagnostic_msgs::msg::DiagnosticStatus::OK) << "no stale ERROR";
}

TEST_F(ControllerNodeVelocityTest, InvalidRobotLimitsFailToConfigure)
{
  const double nan = std::nan("");
  const std::vector<rclcpp::Parameter> invalid {
    {"robot_limits.max_linear_vel", -0.1},
    {"robot_limits.max_linear_vel", nan},
    {"robot_limits.min_linear_vel", 0.1},
    {"robot_limits.max_angular_vel", -1.0},
    {"robot_limits.max_angular_vel", std::numeric_limits<double>::infinity()},
    {"robot_limits.max_linear_acc", 0.0},
    {"robot_limits.max_linear_decel", -1.0},
    {"robot_limits.max_angular_acc", 0.0},
    {"robot_limits.max_angular_decel", nan},
  };
  for (const auto & param : invalid) {
    auto node = std::make_shared<easynav::ControllerNode>(limits_options({param}));
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED) <<
      param.get_name() << " = " << param.value_to_string();

    // Fixed, it configures.
    node->set_parameter(rclcpp::Parameter(param.get_name(), param.get_name() ==
      "robot_limits.min_linear_vel" ? -0.1 : 0.5));
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE) << param.get_name();
  }
}

TEST_F(ControllerNodeVelocityTest, ZeroVelocityLimitsAreValid)
{
  // E.g. a robot that cannot reverse, or that only rotates in place.
  auto node = std::make_shared<easynav::ControllerNode>(
    limits_options({
    {"robot_limits.max_linear_vel", 0.0},
    {"robot_limits.min_linear_vel", 0.0},
    {"robot_limits.max_angular_vel", 0.0},
    {"robot_limits.max_linear_acc", 1e-6}}));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

namespace
{

easynav::SafetyChannelState protective_stop(bool lost = false)
{
  easynav::SafetyChannelState state;
  state.protective_stop = true;
  state.status_lost = lost;
  return state;
}

easynav::SafetyChannelState speed_limit(double linear, double angular)
{
  easynav::SafetyChannelState state;
  state.max_linear_vel = linear;
  state.max_angular_vel = angular;
  return state;
}

}  // namespace

class ControllerNodeSafetyChannelTest : public ControllerNodeVelocityTest
{
protected:
  void reach(double vx)
  {
    for (int i = 0; i < 100; ++i) {
      cycle(vx);
    }
    spin_for(100ms);
    ASSERT_DOUBLE_EQ(received_.back(), vx);
    received_.clear();
    stamps_.clear();
  }
};

TEST_F(ControllerNodeSafetyChannelTest, AProtectiveStopCommandsZeroAtOnceAndResumesFromZero)
{
  make_active_node();
  reach(1.0);

  // The safety channel stops the robot: zero at once, not a ramp from 1.0.
  nav_state_->set(easynav::kSafetyStatusKey, protective_stop());
  for (int i = 0; i < 20; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  for (const double v : received_) {
    EXPECT_DOUBLE_EQ(v, 0.0);
  }

  // Released: from zero, within the acceleration limit.
  nav_state_->set(easynav::kSafetyStatusKey, easynav::SafetyChannelState());
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 100; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  // At most one smoother step (up to 0.1 s at 2 m/s^2) above zero, not back to 1.0.
  EXPECT_LE(received_.front(), kMaxLinearAcc * 0.1 + kTolerance);
  EXPECT_DOUBLE_EQ(received_.back(), 1.0);
  expect_within_acceleration_limits();
}

TEST_F(ControllerNodeSafetyChannelTest, AProtectiveStopIsPublishedEvenWithoutCommands)
{
  make_active_node({{"cmd_timeout", 0.0}});
  reach(1.0);

  // A stop with no source commanding (e.g. the controller died): the zero still goes out.
  nav_state_->set(easynav::kSafetyStatusKey, protective_stop());
  cycle(std::nullopt);
  spin_for(100ms);
  ASSERT_EQ(received_.size(), 1u);
  EXPECT_DOUBLE_EQ(received_.front(), 0.0);
}

TEST_F(ControllerNodeSafetyChannelTest, NoSourceMovesTheRobotDuringAProtectiveStop)
{
  make_active_node();
  nav_state_->set(easynav::kSafetyStatusKey, protective_stop(true));
  for (int i = 0; i < 20; ++i) {
    cycle(1.0, -0.1, 0.2);  // Controller, recovery takeover and override.
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  for (const double v : received_) {
    EXPECT_DOUBLE_EQ(v, 0.0);
  }
}

TEST_F(ControllerNodeSafetyChannelTest, ASpeedLimitCutsTheLimitsAndIsReachedBraking)
{
  make_active_node();
  reach(1.0);

  nav_state_->set(easynav::kSafetyStatusKey, speed_limit(0.4, 0.5));
  for (int i = 0; i < 60; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_GT(received_.size(), 3u);
  EXPECT_GT(received_.front(), 0.4) << "it must not drop to the limit in one step";
  EXPECT_DOUBLE_EQ(received_.back(), 0.4);
  expect_within_acceleration_limits();

  const auto limits = node_->get_robot_limits();
  EXPECT_DOUBLE_EQ(limits.max_linear_vel, 0.4);
  EXPECT_DOUBLE_EQ(limits.min_linear_vel, -0.2) << "already below the limit";
  EXPECT_DOUBLE_EQ(limits.max_angular_vel, 0.5);
  EXPECT_DOUBLE_EQ(limits.max_linear_decel, kMaxLinearDecel) << "accelerations are not cut";

  // Lifted: back to the configured limits.
  nav_state_->set(easynav::kSafetyStatusKey, easynav::SafetyChannelState());
  received_.clear();
  stamps_.clear();
  for (int i = 0; i < 60; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  EXPECT_DOUBLE_EQ(received_.back(), 1.0);
  expect_within_acceleration_limits();
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().max_linear_vel, 1.0);
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().max_angular_vel, 1.5);
}

TEST_F(ControllerNodeSafetyChannelTest, ASpeedLimitAlsoCutsReversing)
{
  make_active_node();
  nav_state_->set(easynav::kSafetyStatusKey, speed_limit(0.1, 0.5));
  for (int i = 0; i < 60; ++i) {
    cycle(-1.0);
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  EXPECT_DOUBLE_EQ(received_.back(), -0.1);
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().min_linear_vel, -0.1);
}

TEST_F(ControllerNodeSafetyChannelTest, ASpeedLimitOfZeroHoldsTheRobot)
{
  make_active_node();
  reach(1.0);
  nav_state_->set(easynav::kSafetyStatusKey, speed_limit(0.0, 0.0));
  for (int i = 0; i < 60; ++i) {
    cycle(1.0);
  }
  spin_for(100ms);
  ASSERT_FALSE(received_.empty());
  EXPECT_DOUBLE_EQ(received_.back(), 0.0);
  expect_within_acceleration_limits();
}

TEST_F(ControllerNodeSafetyChannelTest, ASpeedLimitAboveTheRobotLimitsChangesNothing)
{
  make_active_node();
  const auto configured = node_->get_robot_limits();
  nav_state_->set(easynav::kSafetyStatusKey, speed_limit(5.0, 5.0));
  cycle(1.0);
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().max_linear_vel, configured.max_linear_vel);
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().min_linear_vel, configured.min_linear_vel);
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().max_angular_vel, configured.max_angular_vel);
}

TEST_F(ControllerNodeSafetyChannelTest, NewRobotLimitsKeepTheSpeedLimit)
{
  make_active_node();
  nav_state_->set(easynav::kSafetyStatusKey, speed_limit(0.4, 0.5));
  cycle(1.0);

  auto limits = node_->get_robot_limits();
  limits.max_linear_vel = 2.0;  // E.g. a deprecated per-controller limit.
  node_->set_robot_limits(limits);
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().max_linear_vel, 0.4);
}

TEST_F(ControllerNodeSafetyChannelTest, ReconfiguringStartsWithoutTheSpeedLimit)
{
  make_active_node();
  nav_state_->set(easynav::kSafetyStatusKey, speed_limit(0.4, 0.5));
  cycle(1.0);
  ASSERT_DOUBLE_EQ(node_->get_robot_limits().max_linear_vel, 0.4);

  node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(node_->get_robot_limits().max_linear_vel, 1.0)
    << "the configured limits, until the RT cycle applies the safety channel again";
}
