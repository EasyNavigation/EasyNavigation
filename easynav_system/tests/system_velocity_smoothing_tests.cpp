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
/// \brief End to end through SystemNode: with very low accelerations, the published commands
/// are smoothed, never a jump to the controller's command.

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_system/SystemNode.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{
constexpr double kLinearAcc = 0.2;     // m/s^2
constexpr double kLinearDecel = 0.3;   // m/s^2
constexpr double kAngularAcc = 0.4;    // rad/s^2
constexpr double kAngularDecel = 0.6;  // rad/s^2
constexpr double kTolerance = 1e-6;
}  // namespace

class SystemVelocitySmoothingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      std::vector<const char *> argv{
        "system_velocity_smoothing_tests",
        "--ros-args",
        "-p", "controller_types:=['dummy']",
        "-p", "dummy.plugin:=easynav_controller/DummyController",
        "-p", "robot_limits.max_linear_vel:=1.0",
        "-p", "robot_limits.min_linear_vel:=-1.0",
        "-p", "robot_limits.max_angular_vel:=2.0",
        "-p", "robot_limits.max_linear_acc:=0.2",
        "-p", "robot_limits.max_linear_decel:=0.3",
        "-p", "robot_limits.max_angular_acc:=0.4",
        "-p", "robot_limits.max_angular_decel:=0.6",
        // Stamped, so each command carries the time it was computed at.
        "-p", "use_cmd_vel_stamped:=true",
      };
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }

    system_node_ = std::make_shared<easynav::SystemNode>();
    ASSERT_EQ(
      system_node_->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(
      system_node_->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
      State::PRIMARY_STATE_ACTIVE);

    listener_ = rclcpp::Node::make_shared("smoothing_listener");
    sub_ = listener_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_stamped", 1000,
      [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) {received_.push_back(*msg);});
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_node(listener_);
    const auto start = std::chrono::steady_clock::now();
    while (sub_->get_publisher_count() == 0 && std::chrono::steady_clock::now() - start < 2s) {
      exe_->spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(sub_->get_publisher_count(), 0u);
  }

  // The controller asks for (vx, wz); RT cycles at 200 Hz for \p duration.
  void command(double vx, double wz, std::chrono::milliseconds duration)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = vx;
    cmd.twist.angular.z = wz;
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      system_node_->get_nav_state()->set("cmd_vel", cmd);
      system_node_->system_cycle_rt();
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
    const auto end = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - end < 100ms) {
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
  }

  // Every change between consecutive commands, on both axes, is within the acceleration
  // (speeding up) or deceleration (slowing down) limit over the time between them.
  void expect_smoothed(std::size_t from = 0) const
  {
    ASSERT_GT(received_.size(), from + 2);
    for (std::size_t i = from + 1; i < received_.size(); ++i) {
      const auto & prev = received_[i - 1];
      const auto & curr = received_[i];
      const double dt = (rclcpp::Time(curr.header.stamp) -
        rclcpp::Time(prev.header.stamp)).seconds();
      auto check = [&](double a, double b, double acc, double decel, const char * axis) {
          const double limit = (std::abs(b) < std::abs(a) ? decel : acc) * dt;
          EXPECT_LE(std::abs(b - a), limit + kTolerance) <<
            axis << " step " << i << ": " << a << " -> " << b << " in " << dt << " s";
        };
      check(prev.twist.linear.x, curr.twist.linear.x, kLinearAcc, kLinearDecel, "linear");
      check(prev.twist.angular.z, curr.twist.angular.z, kAngularAcc, kAngularDecel, "angular");
    }
  }

  easynav::SystemNode::SharedPtr system_node_;
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::vector<geometry_msgs::msg::TwistStamped> received_;
};

TEST_F(SystemVelocitySmoothingTest, SpeedingUpIsSmoothed)
{
  // Full speed asked from standstill: after 1 s, at most acc * 1 s, far from the target.
  command(1.0, 2.0, 1000ms);

  ASSERT_GT(received_.size(), 50u);
  expect_smoothed();
  const auto & last = received_.back().twist;
  EXPECT_LE(last.linear.x, kLinearAcc * 1.2);
  EXPECT_GT(last.linear.x, kLinearAcc * 0.7) << "it must actually be moving towards the target";
  EXPECT_LE(last.angular.z, kAngularAcc * 1.2);
  EXPECT_GT(last.angular.z, kAngularAcc * 0.7);
  for (std::size_t i = 1; i < received_.size(); ++i) {
    EXPECT_GE(received_[i].twist.linear.x, received_[i - 1].twist.linear.x) << "monotonic";
  }
}

TEST_F(SystemVelocitySmoothingTest, StoppingIsSmoothed)
{
  command(1.0, 2.0, 1000ms);
  const double reached = received_.back().twist.linear.x;
  ASSERT_GT(reached, 0.1);

  // The controller asks to stop dead: it brakes within the deceleration limit.
  const auto from = received_.size() - 1;
  command(0.0, 0.0, 1500ms);
  expect_smoothed(from);
  EXPECT_GT(received_[from + 1].twist.linear.x, 0.0) << "not a dead stop in one step";
  EXPECT_DOUBLE_EQ(received_.back().twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(received_.back().twist.angular.z, 0.0);
}

TEST_F(SystemVelocitySmoothingTest, ChangeOfDirectionGoesThroughZero)
{
  command(1.0, 2.0, 1000ms);
  const auto from = received_.size() - 1;

  // Straight to reversing: decelerate to zero, then accelerate backwards, never jumping sign.
  command(-1.0, -2.0, 2500ms);
  expect_smoothed(from);
  bool passed_through_zero = false;
  for (std::size_t i = from + 1; i < received_.size(); ++i) {
    const double a = received_[i - 1].twist.linear.x;
    const double b = received_[i].twist.linear.x;
    EXPECT_FALSE(a > 0.0 && b < 0.0) << "sign jump at step " << i << ": " << a << " -> " << b;
    passed_through_zero |= (b == 0.0);
  }
  EXPECT_TRUE(passed_through_zero);
  EXPECT_LT(received_.back().twist.linear.x, 0.0) << "it ends up reversing";
}
