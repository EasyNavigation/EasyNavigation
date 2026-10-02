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
/// \brief How ControllerNode copes with a controller plugin that misbehaves (FaultyController).

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_controller/ControllerNode.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class FaultyControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static rclcpp::NodeOptions options(
    const std::string & fault, int fault_after, std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> params {
      {"controller_types", std::vector<std::string>{"ctrl"}},
      {"ctrl.plugin", std::string("easynav_controller/FaultyController")},
      {"ctrl.fault", fault},
      {"ctrl.fault_after", fault_after},
      {"ctrl.linear_vel", 0.5},
      {"ctrl.angular_vel", 0.0},
      {"robot_limits.max_linear_vel", 1.0},
      {"robot_limits.max_angular_vel", 1.5},
      {"robot_limits.max_linear_acc", 2.0},
      {"robot_limits.max_linear_decel", 4.0},
      {"use_cmd_vel_stamped", true},
      {"cmd_timeout", 0.3},
    };
    params.insert(params.end(), extra.begin(), extra.end());
    return rclcpp::NodeOptions().parameter_overrides(params);
  }

  void make_active_node(
    const std::string & fault, int fault_after, std::vector<rclcpp::Parameter> extra = {})
  {
    node_ = std::make_shared<easynav::ControllerNode>(options(fault, fault_after, extra));
    node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

    listener_ = rclcpp::Node::make_shared("faulty_listener");
    sub_ = listener_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_stamped", 1000,
      [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) {
        linear_.push_back(msg->twist.linear.x);
        angular_.push_back(msg->twist.angular.z);
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

  // One RT cycle, as SystemNode runs it: the controller, then the velocity output.
  void cycle()
  {
    node_->cycle_rt(nav_state_, true);
    node_->publish_cmd_vel_rt(nav_state_);
    rclcpp::sleep_for(10ms);
    exe_->spin_some();
  }

  void cycles(int n)
  {
    for (int i = 0; i < n; ++i) {
      cycle();
    }
  }

  void spin_for(std::chrono::milliseconds d)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < d) {
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
  }

  std::optional<DiagnosticStatus> diagnostic() const
  {
    if (!nav_state_->has("diagnostics.cmd_vel")) {return std::nullopt;}
    return nav_state_->get_safe<DiagnosticStatus>("diagnostics.cmd_vel");
  }

  easynav::ControllerNode::SharedPtr node_;
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::shared_ptr<easynav::NavState> nav_state_ = std::make_shared<easynav::NavState>();
  std::vector<double> linear_;
  std::vector<double> angular_;
};

TEST_F(FaultyControllerTest, NoFaultReachesTheCommandedVelocity)
{
  make_active_node("none", 0);
  cycles(100);
  spin_for(100ms);
  ASSERT_FALSE(linear_.empty());
  EXPECT_DOUBLE_EQ(linear_.back(), 0.5);
  EXPECT_FALSE(diagnostic());
}

TEST_F(FaultyControllerTest, UnknownFaultFailsToConfigure)
{
  auto node = std::make_shared<easynav::ControllerNode>(options("melt", 0));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

// Regression: ControllerNode used to repropose the last "cmd_vel" in NavState, so a silent
// controller kept the robot moving forever.
TEST_F(FaultyControllerTest, ControllerThatStopsProposingIsStopped)
{
  make_active_node("stop_proposing", 60);
  cycles(60);
  spin_for(50ms);
  ASSERT_DOUBLE_EQ(linear_.back(), 0.5);

  cycles(60);  // ~0.6 s, longer than cmd_timeout
  spin_for(100ms);
  EXPECT_DOUBLE_EQ(linear_.back(), 0.0);
  const auto diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR);
}

TEST_F(FaultyControllerTest, ControllerThatFreezesIsStopped)
{
  // The same command, with the same stamp, again and again.
  make_active_node("freeze", 60);
  cycles(60);
  spin_for(50ms);
  ASSERT_DOUBLE_EQ(linear_.back(), 0.5);

  cycles(60);
  spin_for(100ms);
  EXPECT_DOUBLE_EQ(linear_.back(), 0.0);
  const auto diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR);
}

TEST_F(FaultyControllerTest, ControllerThatThrowsIsStoppedAndEasyNavSurvives)
{
  make_active_node("throw", 60);
  cycles(60);
  spin_for(50ms);
  ASSERT_DOUBLE_EQ(linear_.back(), 0.5);

  ASSERT_NO_THROW(cycles(60));
  spin_for(100ms);
  EXPECT_DOUBLE_EQ(linear_.back(), 0.0);
  EXPECT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(FaultyControllerTest, NaNCommandsAreNeverPublishedAndEndInAStop)
{
  make_active_node("nan", 60);
  cycles(120);
  spin_for(100ms);
  ASSERT_FALSE(linear_.empty());
  for (size_t i = 0; i < linear_.size(); ++i) {
    EXPECT_TRUE(std::isfinite(linear_[i]) && std::isfinite(angular_[i])) << i;
  }
  EXPECT_DOUBLE_EQ(linear_.back(), 0.0) << "only NaN: as silent as no command";
  const auto diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR);
}

TEST_F(FaultyControllerTest, HugeCommandsAreClampedToTheRobotLimits)
{
  make_active_node("max_velocity", 0);
  cycles(150);
  spin_for(100ms);
  ASSERT_FALSE(linear_.empty());
  for (size_t i = 0; i < linear_.size(); ++i) {
    EXPECT_LE(std::abs(linear_[i]), 1.0) << i;
    EXPECT_LE(std::abs(angular_[i]), 1.5) << i;
  }
  EXPECT_DOUBLE_EQ(linear_.back(), 1.0);
  EXPECT_DOUBLE_EQ(angular_.back(), 1.5);
}

TEST_F(FaultyControllerTest, HangingControllerIsDetectedByTheReceiverDeadline)
{
  // The controller blocks the RT cycle: nothing is published, the receiver notices.
  make_active_node(
    "hang", 30, {{"ctrl.hang_time", 0.5}, {"cmd_vel_keepalive_period", 0.05}});
  std::atomic<int> missed {0};
  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.deadline_callback =
    [&missed](rclcpp::QOSDeadlineRequestedInfo &) {++missed;};
  auto watchdog = listener_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_stamped", rclcpp::QoS(1).deadline(rclcpp::Duration(150ms)),
    [](geometry_msgs::msg::TwistStamped::UniquePtr) {}, sub_options);
  spin_for(100ms);

  cycles(30);
  EXPECT_EQ(missed.load(), 0) << "in time before the fault";

  // The RT cycle runs in its own thread, as in EasyNav.
  std::atomic<bool> done {false};
  std::thread rt([this, &done]() {
      for (int i = 0; i < 3; ++i) {
        node_->cycle_rt(nav_state_, true);
        node_->publish_cmd_vel_rt(nav_state_);
      }
      done = true;
    });
  while (!done) {
    exe_->spin_some();
    rclcpp::sleep_for(5ms);
  }
  rt.join();
  EXPECT_GT(missed.load(), 0);
}

TEST_F(FaultyControllerTest, FaultAfterLetsTheFirstUpdatesThrough)
{
  make_active_node("stop_proposing", 1000);
  cycles(100);
  spin_for(100ms);
  EXPECT_DOUBLE_EQ(linear_.back(), 0.5);
  EXPECT_FALSE(diagnostic());
}

TEST_F(FaultyControllerTest, ReconfiguringRestartsTheFaultCount)
{
  make_active_node("stop_proposing", 30);
  cycles(80);
  spin_for(100ms);
  ASSERT_DOUBLE_EQ(linear_.back(), 0.0);

  node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  linear_.clear();
  cycles(30);
  spin_for(100ms);
  ASSERT_FALSE(linear_.empty());
  EXPECT_GT(linear_.back(), 0.0) << "it moves again until the fault comes back";
}
