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
/// controller plugins, smoothing, publication, and braking on deactivation.

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_core/ControllerMethodBase.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{

class LimitsReadingController : public easynav::ControllerMethodBase {};

rclcpp::NodeOptions limits_options()
{
  return rclcpp::NodeOptions().parameter_overrides({
      {"robot_limits.max_linear_vel", 1.0},
      {"robot_limits.min_linear_vel", -0.2},
      {"robot_limits.max_angular_vel", 1.5},
      {"robot_limits.max_linear_acc", 2.0},
      {"robot_limits.max_linear_decel", 4.0},
      {"robot_limits.max_angular_acc", 3.0},
      {"robot_limits.max_angular_decel", 6.0},
      // Stamped, so each published command carries the time it was computed at.
      {"use_cmd_vel_stamped", true},
  });
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

  void make_active_node()
  {
    node_ = std::make_shared<easynav::ControllerNode>(limits_options());
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

  // One RT cycle: optionally a new controller command, then the velocity output.
  void cycle(std::optional<double> controller)
  {
    if (controller) {
      nav_state_->set("cmd_vel", cmd(*controller));
    }
    node_->publish_cmd_vel_rt(nav_state_, controller.has_value());
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
  make_active_node();
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
