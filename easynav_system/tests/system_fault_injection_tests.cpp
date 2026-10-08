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
/// \brief The whole RT cycle of SystemNode with a controller that misbehaves (FaultyController).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_system/SystemNode.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class SystemFaultInjectionTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    exe_.reset();
    sub_.reset();
    listener_.reset();
    system_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // The subnodes take their parameters from the global arguments: a new context per fault.
  void start(const std::string & fault, int fault_after)
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    const std::vector<std::string> args {
      "system_fault_injection_tests", "--ros-args",
      "-p", "controller_types:=['ctrl']",
      "-p", "ctrl.plugin:=easynav_controller/FaultyController",
      "-p", "ctrl.fault:='" + fault + "'",  // Quoted: YAML reads nan as a number
      "-p", "ctrl.fault_after:=" + std::to_string(fault_after),
      "-p", "ctrl.linear_vel:=0.5",
      "-p", "ctrl.rt_freq:=50.0",
      "-p", "robot_limits.max_linear_vel:=1.0",
      "-p", "robot_limits.max_angular_vel:=1.5",
      "-p", "robot_limits.max_linear_acc:=2.0",
      "-p", "robot_limits.max_linear_decel:=4.0",
      "-p", "use_cmd_vel_stamped:=true",
      "-p", "cmd_timeout:=0.3",
    };
    std::vector<const char *> argv;
    for (const auto & arg : args) {
      argv.push_back(arg.c_str());
    }
    rclcpp::init(static_cast<int>(argv.size()), argv.data());

    system_node_ = std::make_shared<easynav::SystemNode>();
    ASSERT_EQ(
      system_node_->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(
      system_node_->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
      State::PRIMARY_STATE_ACTIVE);

    listener_ = rclcpp::Node::make_shared("fault_listener");
    sub_ = listener_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_stamped", 1000,
      [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) {
        linear_.push_back(msg->twist.linear.x);
        angular_.push_back(msg->twist.angular.z);
      });
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_node(listener_);
    const auto begin = std::chrono::steady_clock::now();
    while (sub_->get_publisher_count() == 0 && std::chrono::steady_clock::now() - begin < 2s) {
      exe_->spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(sub_->get_publisher_count(), 0u);
  }

  // RT cycles of the whole system (sensors, localizer, controller, recovery, output).
  void run_for(std::chrono::milliseconds duration)
  {
    const auto begin = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - begin < duration) {
      system_node_->system_cycle_rt();
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
    const auto end = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - end < 50ms) {
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
  }

  std::optional<DiagnosticStatus> diagnostic() const
  {
    auto nav_state = system_node_->get_nav_state();
    if (!nav_state->has("diagnostics.cmd_vel")) {return std::nullopt;}
    return nav_state->get_safe<DiagnosticStatus>("diagnostics.cmd_vel");
  }

  bool in_diagnostics_group() const
  {
    const auto keys = system_node_->get_nav_state()->get_group_keys("diagnostics");
    return std::find(keys.begin(), keys.end(), "diagnostics.cmd_vel") != keys.end();
  }

  easynav::SystemNode::SharedPtr system_node_;
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::vector<double> linear_;
  std::vector<double> angular_;
};

TEST_F(SystemFaultInjectionTest, HealthyControllerMovesTheRobotWithoutDiagnostics)
{
  start("none", 0);
  run_for(600ms);
  ASSERT_FALSE(linear_.empty());
  EXPECT_DOUBLE_EQ(linear_.back(), 0.5);
  EXPECT_FALSE(diagnostic());
}

TEST_F(SystemFaultInjectionTest, SilentControllerStopsTheRobotAndIsDiagnosed)
{
  start("stop_proposing", 25);  // ~0.5 s at 50 Hz
  run_for(450ms);
  ASSERT_FALSE(linear_.empty());
  ASSERT_DOUBLE_EQ(linear_.back(), 0.5);

  run_for(800ms);
  EXPECT_DOUBLE_EQ(linear_.back(), 0.0);
  const auto diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR);
  EXPECT_TRUE(in_diagnostics_group()) << "visible to the recovery system";
  EXPECT_EQ(system_node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(SystemFaultInjectionTest, FrozenControllerStopsTheRobot)
{
  start("freeze", 25);
  run_for(450ms);
  ASSERT_DOUBLE_EQ(linear_.back(), 0.5);

  run_for(800ms);
  EXPECT_DOUBLE_EQ(linear_.back(), 0.0);
  const auto diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR);
}

TEST_F(SystemFaultInjectionTest, ThrowingControllerStopsTheRobotAndEasyNavKeepsRunning)
{
  start("throw", 25);
  run_for(450ms);
  ASSERT_DOUBLE_EQ(linear_.back(), 0.5);

  ASSERT_NO_THROW(run_for(800ms));
  EXPECT_DOUBLE_EQ(linear_.back(), 0.0);
  EXPECT_EQ(system_node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(SystemFaultInjectionTest, NaNFromTheStartNeverReachesTheRobot)
{
  start("nan", 0);
  run_for(800ms);
  for (size_t i = 0; i < linear_.size(); ++i) {
    EXPECT_TRUE(std::isfinite(linear_[i]) && std::isfinite(angular_[i])) << i;
    EXPECT_DOUBLE_EQ(linear_[i], 0.0) << i;
  }
  const auto diag = diagnostic();
  ASSERT_TRUE(diag);
  EXPECT_EQ(diag->level, DiagnosticStatus::ERROR);
}

TEST_F(SystemFaultInjectionTest, HugeCommandsAreClampedToTheRobotLimits)
{
  start("max_velocity", 0);
  run_for(1000ms);
  ASSERT_FALSE(linear_.empty());
  for (size_t i = 0; i < linear_.size(); ++i) {
    EXPECT_LE(std::abs(linear_[i]), 1.0) << i;
    EXPECT_LE(std::abs(angular_[i]), 1.5) << i;
  }
  EXPECT_DOUBLE_EQ(linear_.back(), 1.0);
}

TEST_F(SystemFaultInjectionTest, DeactivatingAfterAFaultLeavesTheRobotStopped)
{
  start("nan", 25);
  run_for(450ms);
  ASSERT_DOUBLE_EQ(linear_.back(), 0.5);
  run_for(100ms);  // Faulty, not timed out yet: the last valid command is held.

  ASSERT_EQ(
    system_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE).id(),
    State::PRIMARY_STATE_INACTIVE);
  run_for(300ms);  // No RT cycles run while inactive.
  ASSERT_FALSE(linear_.empty());
  EXPECT_EQ(linear_.back(), 0.0) << "the last command is an exact zero";
}
