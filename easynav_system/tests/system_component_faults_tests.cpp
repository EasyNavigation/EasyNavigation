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
/// \brief SystemNode with a localizer, planner or maps manager that misbehaves
/// (FaultyLocalizer, FaultyPlanner, FaultyMapsManager).

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_system/RealTime.hpp"
#include "easynav_system/SystemNode.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class SystemComponentFaultsTest : public ::testing::Test
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

  // A controller commanding 0.5 m/s, plus \p params. The subnodes take their parameters from
  // the global arguments: a new context each time.
  void start(std::vector<std::string> params)
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::vector<std::string> args {
      "system_component_faults_tests", "--ros-args",
      "-p", "controller_types:=['ctrl']",
      "-p", "ctrl.plugin:=easynav_controller/FaultyController",
      "-p", "ctrl.fault:='none'",
      "-p", "robot_limits.max_linear_acc:=10.0",
    };
    for (const auto & param : params) {
      args.push_back("-p");
      args.push_back(param);
    }
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

    listener_ = rclcpp::Node::make_shared("component_faults_listener");
    sub_ = listener_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1000,
      [this](geometry_msgs::msg::Twist::UniquePtr msg) {linear_.push_back(msg->linear.x);});
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_node(listener_);
    const auto begin = std::chrono::steady_clock::now();
    while (sub_->get_publisher_count() == 0 && std::chrono::steady_clock::now() - begin < 2s) {
      exe_->spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(sub_->get_publisher_count(), 0u);
  }

  // RT cycles at 200 Hz for \p duration, and a non-RT cycle every \p nort_every RT cycles.
  void run_for(std::chrono::milliseconds duration, int nort_every = 0)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    for (int i = 0; std::chrono::steady_clock::now() < end; ++i) {
      system_node_->system_cycle_rt();
      if (nort_every > 0 && i % nort_every == 0) {
        system_node_->system_cycle();
      }
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
    spin_some_for(50ms);
  }

  void spin_some_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
  }

  std::optional<DiagnosticStatus> diagnostic(const std::string & key) const
  {
    auto nav_state = system_node_->get_nav_state();
    if (!nav_state->has(key)) {return std::nullopt;}
    return nav_state->get_safe<DiagnosticStatus>(key);
  }

  void expect_still_running() const
  {
    EXPECT_EQ(system_node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
    EXPECT_FALSE(system_node_->is_shutdown_requested());
  }

  easynav::SystemNode::SharedPtr system_node_;
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::vector<double> linear_;
};

// ─── Localizer ──────────────────────────────────────────────────────────────────────────────

// A localizer updating every RT cycle, failing with \p fault after \p fault_after updates.
std::vector<std::string> faulty_localizer(
  const std::string & fault, int fault_after, std::vector<std::string> extra = {})
{
  std::vector<std::string> params {
    "localizer_types:=['loc']",
    "loc.plugin:=easynav_localizer/FaultyLocalizer",
    "loc.fault:='" + fault + "'",
    "loc.fault_after:=" + std::to_string(fault_after),
    "loc.rt_freq:=200.0",
    "safety.max_pose_age:=0.3",
  };
  params.insert(params.end(), extra.begin(), extra.end());
  return params;
}

TEST_F(SystemComponentFaultsTest, AHealthyLocalizerIsNotReported)
{
  start(faulty_localizer("none", 0));
  run_for(600ms);
  EXPECT_FALSE(diagnostic("diagnostics.robot_pose"));
  ASSERT_FALSE(linear_.empty());
  EXPECT_DOUBLE_EQ(linear_.back(), 0.5);
}

TEST_F(SystemComponentFaultsTest, AThrowingLocalizerIsContainedAndItsOldPoseReported)
{
  start(faulty_localizer("throw", 20));
  run_for(600ms);
  expect_still_running();
  ASSERT_TRUE(diagnostic("diagnostics.robot_pose"));
  EXPECT_EQ(diagnostic("diagnostics.robot_pose")->level, DiagnosticStatus::ERROR);
  EXPECT_NE(diagnostic("diagnostics.robot_pose")->message.find("s old"), std::string::npos);
}

TEST_F(SystemComponentFaultsTest, ALocalizerThatStopsPublishingIsReported)
{
  start(faulty_localizer("stop_publishing", 20));
  run_for(600ms);
  ASSERT_TRUE(diagnostic("diagnostics.robot_pose"));
  EXPECT_EQ(diagnostic("diagnostics.robot_pose")->level, DiagnosticStatus::ERROR);
}

TEST_F(SystemComponentFaultsTest, AFrozenLocalizerIsReportedButOnlyStopsTheRobotInSafetyMode)
{
  start(faulty_localizer("freeze", 20));
  run_for(600ms);
  ASSERT_TRUE(diagnostic("diagnostics.robot_pose"));
  EXPECT_EQ(diagnostic("diagnostics.robot_pose")->level, DiagnosticStatus::ERROR);
  ASSERT_FALSE(linear_.empty());
  EXPECT_DOUBLE_EQ(linear_.back(), 0.5) << "outside safety mode, it is only reported";
  expect_still_running();
}

TEST_F(SystemComponentFaultsTest, ANanPoseIsReported)
{
  start(faulty_localizer("nan", 0));
  run_for(200ms);
  ASSERT_TRUE(diagnostic("diagnostics.robot_pose"));
  EXPECT_EQ(diagnostic("diagnostics.robot_pose")->level, DiagnosticStatus::ERROR);
  EXPECT_NE(
    diagnostic("diagnostics.robot_pose")->message.find("not finite"), std::string::npos);
  expect_still_running();
}

TEST_F(SystemComponentFaultsTest, AJumpingPoseIsStillUpToDate)
{
  // Detecting jumps is the localizer's own business (e.g. AMCL's convergence evaluator).
  start(faulty_localizer("jump", 0));
  run_for(300ms);
  EXPECT_FALSE(diagnostic("diagnostics.robot_pose"));
}

TEST_F(SystemComponentFaultsTest, AHangingLocalizerMakesTheRtCyclesLate)
{
  start(
    faulty_localizer(
      "hang", 20, {"loc.hang_time:=0.03", "safety.rt_monitor.max_late_cycles:=3"}));
  run_for(500ms);
  ASSERT_TRUE(diagnostic("diagnostics.rt_cycle"));
  EXPECT_EQ(diagnostic("diagnostics.rt_cycle")->level, DiagnosticStatus::WARN);
  expect_still_running();
}

// ─── Planner and maps manager (non-RT) ──────────────────────────────────────────────────────

TEST_F(SystemComponentFaultsTest, AThrowingPlannerOrMapsManagerDoesNotStopEasyNav)
{
  start(
  {
    "planner_types:=['plan']",
    "plan.plugin:=easynav_planner/FaultyPlanner",
    "plan.fault:='throw'",
    "plan.freq:=100.0",
    "map_types:=['maps']",
    "maps.plugin:=easynav_maps_manager/FaultyMapsManager",
    "maps.fault:='throw'",
    "maps.freq:=100.0"});
  EXPECT_NO_THROW(run_for(500ms, 2));
  expect_still_running();
  ASSERT_FALSE(linear_.empty());
  EXPECT_DOUBLE_EQ(linear_.back(), 0.5) << "the RT cycle and the controller go on";
}

TEST_F(SystemComponentFaultsTest, ANanPathIsDiscardedSoTheControllerHasNothingToFollow)
{
  start(
  {
    "planner_types:=['plan']",
    "plan.plugin:=easynav_planner/FaultyPlanner",
    "plan.fault:='nan'",
    "plan.freq:=100.0"});
  // A goal ahead, and a pose: the planner has something to plan.
  auto nav_state = system_node_->get_nav_state();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());
  nav_msgs::msg::Goals goals;
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 2.0;
  goal.pose.orientation.w = 1.0;
  goals.goals.push_back(goal);
  nav_state->set("goals", goals);

  run_for(100ms, 2);
  ASSERT_TRUE(nav_state->has("path"));
  EXPECT_TRUE(nav_state->get_safe<nav_msgs::msg::Path>("path").poses.empty());
  ASSERT_TRUE(diagnostic("diagnostics.path"));
  EXPECT_EQ(diagnostic("diagnostics.path")->level, DiagnosticStatus::ERROR);
  expect_still_running();
}

// The non-RT cycle runs in its own thread, hung by a plugin, while the RT cycle runs at 200 Hz
// with EasyNav's real-time priority for 1 s. Returns the non-RT cycles completed.
int run_rt_while_nort_hangs(const easynav::SystemNode::SharedPtr & system_node)
{
  std::atomic<bool> stop {false};
  std::atomic<int> nort_cycles {0};
  std::thread nort([&]() {
      while (!stop) {
        system_node->system_cycle();
        ++nort_cycles;
      }
    });
  std::thread rt([&]() {
      if (!easynav::set_real_time_priority(easynav::kRealTimePriority).empty()) {return;}
      auto next = std::chrono::steady_clock::now();
      const auto end = next + 1s;
      while (std::chrono::steady_clock::now() < end) {
        system_node->system_cycle_rt();
        next += 5ms;
        std::this_thread::sleep_until(next);
      }
    });
  rt.join();
  stop = true;
  nort.join();
  return nort_cycles.load();
}

TEST_F(SystemComponentFaultsTest, AHangingPlannerDoesNotDelayTheRtCycle)
{
  // That nothing is late can only be asserted if the load of the machine cannot delay the cycle.
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "needs real-time scheduling, not allowed here";
  }
  start(
  {
    "planner_types:=['plan']",
    "plan.plugin:=easynav_planner/FaultyPlanner",
    "plan.fault:='hang'",
    "plan.hang_time:=0.2",
    "plan.freq:=100.0"});

  EXPECT_GE(run_rt_while_nort_hangs(system_node_), 2) << "the planner did hang the non-RT cycle";
  EXPECT_EQ(system_node_->get_safety().get_rt_monitor().late_cycles(), 0u);
  EXPECT_FALSE(diagnostic("diagnostics.rt_cycle")) << "no RT cycle was late";
}

TEST_F(SystemComponentFaultsTest, AHangingMapsManagerDoesNotDelayTheRtCycle)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "needs real-time scheduling, not allowed here";
  }
  start(
  {
    "map_types:=['maps']",
    "maps.plugin:=easynav_maps_manager/FaultyMapsManager",
    "maps.fault:='hang'",
    "maps.hang_time:=0.2",
    "maps.freq:=100.0"});

  EXPECT_GE(
    run_rt_while_nort_hangs(system_node_),
    2) << "the maps manager did hang the non-RT cycle";
  EXPECT_EQ(system_node_->get_safety().get_rt_monitor().late_cycles(), 0u);
  EXPECT_FALSE(diagnostic("diagnostics.rt_cycle")) << "no RT cycle was late";
}
