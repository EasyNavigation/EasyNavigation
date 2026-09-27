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
/// controller plugins, mux + smoother + publication, and braking on deactivation.

#include <chrono>
#include <cmath>
#include <memory>
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
#include "easynav_core/VelocityCommand.hpp"

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
        stamps_.push_back(rclcpp::Time(msg->header.stamp).seconds());
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

  // One RT cycle: optionally a controller/reflex proposal, then the velocity output.
  void cycle(std::optional<double> controller, std::optional<double> reflex = std::nullopt)
  {
    if (controller) {
      easynav::velocity_command::propose(
        *nav_state_, easynav::VelocitySource::CONTROLLER, cmd(*controller));
    }
    if (reflex) {
      easynav::velocity_command::propose(
        *nav_state_, easynav::VelocitySource::REFLEX, cmd(*reflex));
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
  std::vector<double> stamps_;

  // Every change between consecutive commands respects the acceleration (speeding up) or the
  // deceleration (slowing down) limit, over the time between them.
  void expect_within_acceleration_limits(size_t from = 0) const
  {
    for (size_t i = from + 1; i < received_.size(); ++i) {
      const double dv = received_[i] - received_[i - 1];
      const double dt = stamps_[i] - stamps_[i - 1];
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

TEST_F(ControllerNodeVelocityTest, ReflexOverrideBypassesTheSmoother)
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
  cycle(1.0, 0.0);
  spin_for(100ms);
  ASSERT_EQ(received_.size(), 1u);
  EXPECT_DOUBLE_EQ(received_.back(), 0.0);
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
