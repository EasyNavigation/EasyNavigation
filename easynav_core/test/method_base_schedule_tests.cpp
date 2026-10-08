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

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rcl/time.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;

// A plugin on a node whose clock the test sets (simulated time).
class MethodBaseScheduleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    method_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void start(double rt_freq, double freq = 10.0, double t0 = 100.0)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {{"use_sim_time", true}, {"plugin.rt_freq", rt_freq}, {"plugin.freq", freq}});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("schedule_test_node", options);
    set_time(t0);
    method_ = std::make_unique<easynav::MethodBase>();
    method_->initialize(node_, "plugin");
  }

  void set_time(double t)
  {
    now_ = t;
    ASSERT_EQ(
      rcl_set_ros_time_override(
        node_->get_clock()->get_clock_handle(), static_cast<rcl_time_point_value_t>(t * 1e9)),
      RCL_RET_OK);
  }

  // What the base classes do every RT cycle; returns whether it ran.
  bool rt_cycle(bool trigger = false)
  {
    method_->report_rt_rate(nav_state_);
    if (method_->isTime2RunRT() || trigger) {
      method_->setRunRT();
      ++runs_;
      return true;
    }
    return false;
  }

  bool cycle()
  {
    method_->report_rate(nav_state_);
    if (method_->isTime2Run()) {
      method_->setRun();
      ++runs_;
      return true;
    }
    return false;
  }

  // RT cycles every period (s), plus a jitter in [0, jitter), for duration; returns the runs.
  int run_rt_cycles(double period, double duration, double jitter = 0.0)
  {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> noise(0.0, jitter > 0.0 ? jitter : 1e-12);
    const int before = runs_;
    const double end = now_ + duration;
    for (double tick = now_ + period; tick <= end + 1e-9; tick += period) {
      set_time(tick + (jitter > 0.0 ? noise(rng) : 0.0));
      rt_cycle();
    }
    set_time(end);
    return runs_ - before;
  }

  std::optional<DiagnosticStatus> diagnostic(const std::string & key) const
  {
    if (!nav_state_.has(key)) {return std::nullopt;}
    return nav_state_.get<DiagnosticStatus>(key);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<easynav::MethodBase> method_;
  easynav::NavState nav_state_;
  double now_ {0.0};
  int runs_ {0};
};

// ─── Schedule ───────────────────────────────────────────────────────────────────────────────

TEST_F(MethodBaseScheduleTest, KeepsItsFrequencyWhenTheCycleIsNotAMultiple)
{
  // 30 Hz checked at 50 Hz: before, it ran every other cycle (25 Hz).
  start(30.0);
  EXPECT_NEAR(run_rt_cycles(0.02, 10.0), 300, 1);
}

TEST_F(MethodBaseScheduleTest, KeepsItsFrequencyAtTheCycleFrequencyWithJitter)
{
  // 50 Hz checked at 50 Hz, each cycle up to 3 ms late: before, any delay lost a run.
  start(50.0);
  EXPECT_NEAR(run_rt_cycles(0.02, 10.0, 0.003), 500, 2);
}

TEST_F(MethodBaseScheduleTest, RunsExactlyAtItsFrequencyWithAFastCycle)
{
  for (const double freq : {1.0, 10.0, 20.0, 30.0, 100.0}) {
    start(freq);
    runs_ = 0;
    EXPECT_NEAR(run_rt_cycles(0.001, 5.0), 5.0 * freq, 1) << freq << " Hz";
    method_.reset();
    node_.reset();
  }
}

TEST_F(MethodBaseScheduleTest, CannotRunFasterThanTheCycle)
{
  // 100 Hz checked at 50 Hz: once per cycle at most, never twice to catch up.
  start(100.0);
  EXPECT_NEAR(run_rt_cycles(0.02, 5.0), 250, 1);
}

TEST_F(MethodBaseScheduleTest, AStallDoesNotCauseABurstOfRuns)
{
  start(10.0);
  run_rt_cycles(0.01, 1.0);
  set_time(now_ + 2.0);  // E.g. the RT cycle blocked for 2 s
  EXPECT_TRUE(rt_cycle()) << "runs right away";
  EXPECT_FALSE(rt_cycle()) << "but only once: the missed runs are lost";
  set_time(now_ + 0.05);
  EXPECT_FALSE(rt_cycle());
  set_time(now_ + 0.05);
  EXPECT_TRUE(rt_cycle()) << "a period after the restart";
}

TEST_F(MethodBaseScheduleTest, ATriggeredRunRestartsTheSchedule)
{
  start(10.0);
  set_time(now_ + 0.1);
  ASSERT_TRUE(rt_cycle());  // Scheduled, at 100.1
  set_time(now_ + 0.07);
  ASSERT_TRUE(rt_cycle(true));  // Triggered, at 100.17
  set_time(100.2);
  EXPECT_FALSE(rt_cycle()) << "not twice in 30 ms";
  set_time(100.269);
  EXPECT_FALSE(rt_cycle());
  set_time(100.271);
  EXPECT_TRUE(rt_cycle()) << "a period after the triggered run";
}

TEST_F(MethodBaseScheduleTest, AScheduledRunThatIsAlsoTriggeredKeepsTheSchedule)
{
  start(10.0);
  set_time(now_ + 0.13);
  ASSERT_TRUE(rt_cycle(true));  // Due at 100.1 and triggered: one run
  set_time(100.2);
  EXPECT_TRUE(rt_cycle()) << "next at 100.2, not 100.23";
}

TEST_F(MethodBaseScheduleTest, AJumpBackInTimeRunsRightAway)
{
  start(10.0, 10.0, 500.0);
  run_rt_cycles(0.01, 1.0);
  set_time(10.0);  // A simulation reset
  EXPECT_TRUE(rt_cycle());
  runs_ = 0;
  EXPECT_NEAR(run_rt_cycles(0.01, 2.0), 20, 1);
}

TEST_F(MethodBaseScheduleTest, RtAndNonRtSchedulesAreIndependent)
{
  start(20.0, 5.0);
  int rt_runs = 0;
  int runs = 0;
  for (int i = 1; i <= 1000; ++i) {
    set_time(100.0 + i * 0.005);
    rt_runs += rt_cycle();
    runs += cycle();
  }
  EXPECT_NEAR(rt_runs, 100, 1);
  EXPECT_NEAR(runs, 25, 1);
}

TEST_F(MethodBaseScheduleTest, InvalidFrequenciesAreRejected)
{
  for (const double bad : std::vector<double>{0.0, -1.0, std::nan(""), INFINITY}) {
    for (const std::string param : {"rt_freq", "freq"}) {
      rclcpp::NodeOptions options;
      options.parameter_overrides({{"bad." + param, bad}});
      auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("invalid_freq_node", options);
      easynav::MethodBase method;
      EXPECT_THROW(method.initialize(node, "bad"), std::runtime_error) << param << " " << bad;
    }
  }
}

// ─── Rate diagnostics ───────────────────────────────────────────────────────────────────────

TEST_F(MethodBaseScheduleTest, OnRateReportsOkOnceAndNothingElse)
{
  start(30.0);
  run_rt_cycles(0.02, 0.1);
  ASSERT_TRUE(diagnostic("diagnostics.plugin.rt_rate"));
  auto status = *diagnostic("diagnostics.plugin.rt_rate");
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.name, "plugin.rt_rate");
  EXPECT_EQ(status.hardware_id, "schedule_test_node");
  EXPECT_NE(status.message.find("30.0 Hz"), std::string::npos) << status.message;
  const auto keys = nav_state_.get_group_keys("diagnostics");
  EXPECT_NE(std::find(keys.begin(), keys.end(), "diagnostics.plugin.rt_rate"), keys.end());

  run_rt_cycles(0.02, 10.0);
  EXPECT_EQ(diagnostic("diagnostics.plugin.rt_rate")->message, status.message) << "unchanged";
  EXPECT_EQ(method_->get_rt_rate_monitor().status(), easynav::RateMonitor::Status::OK);
  EXPECT_FALSE(diagnostic("diagnostics.plugin.rate")) << "the non-RT update never checked";
}

TEST_F(MethodBaseScheduleTest, ASlowCycleIsAWarningThatSaysForHowLongAndOkAgain)
{
  start(50.0);
  run_rt_cycles(0.02, 2.0);
  ASSERT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::OK);

  // The RT cycle only manages 25 Hz: the component, configured at 50 Hz, cannot keep it.
  run_rt_cycles(0.04, 1.1);
  auto status = *diagnostic("diagnostics.plugin.rt_rate");
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_NE(status.message.find("rt_freq 50.0 Hz not kept"), std::string::npos) << status.message;
  EXPECT_NEAR(method_->get_rt_rate_monitor().rate(), 25.0, 1.5);

  run_rt_cycles(0.04, 2.0);
  status = *diagnostic("diagnostics.plugin.rt_rate");
  EXPECT_EQ(status.level, DiagnosticStatus::WARN) << "only reported, never an ERROR";
  EXPECT_EQ(method_->get_rt_rate_monitor().status(), easynav::RateMonitor::Status::ERROR);
  EXPECT_NE(status.message.find("not kept for"), std::string::npos) << status.message;

  run_rt_cycles(0.02, 1.1);
  EXPECT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::OK);
}

TEST_F(MethodBaseScheduleTest, ACycleSlowerThanTwoPeriodsButAMultipleIsNotReported)
{
  // 10 Hz in a 50 Hz cycle that runs late (30 ms instead of 20): still 10 runs per second.
  start(10.0);
  run_rt_cycles(0.03, 10.0);
  EXPECT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::OK);
}

TEST_F(MethodBaseScheduleTest, NonRtRateIsItsOwnDiagnostic)
{
  start(10.0, 5.0);
  for (int i = 1; i <= 400; ++i) {  // Checked at 10 Hz: 5 Hz kept
    set_time(100.0 + i * 0.1);
    cycle();
  }
  ASSERT_TRUE(diagnostic("diagnostics.plugin.rate"));
  EXPECT_EQ(diagnostic("diagnostics.plugin.rate")->level, DiagnosticStatus::OK);
  EXPECT_NE(diagnostic("diagnostics.plugin.rate")->message.find("freq 5.0 Hz"), std::string::npos);

  for (int i = 1; i <= 50; ++i) {  // Checked at 2 Hz: 2 Hz instead of 5
    set_time(now_ + 0.5);
    cycle();
  }
  EXPECT_EQ(diagnostic("diagnostics.plugin.rate")->level, DiagnosticStatus::WARN);
  EXPECT_NE(diagnostic("diagnostics.plugin.rate")->message.find("not kept for"), std::string::npos);
  EXPECT_FALSE(diagnostic("diagnostics.plugin.rt_rate"));
}

TEST_F(MethodBaseScheduleTest, BlockingLongerThanAWindowIsReported)
{
  // Each update blocks the cycle 1.5 s: no checks in between
  start(10.0);
  run_rt_cycles(0.01, 2.0);
  for (int i = 0; i < 4; ++i) {
    set_time(now_ + 1.5);
    rt_cycle();
  }
  ASSERT_TRUE(diagnostic("diagnostics.plugin.rt_rate"));
  EXPECT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::WARN);
  EXPECT_NE(
    diagnostic("diagnostics.plugin.rt_rate")->message.find("not kept for"),
    std::string::npos);
}

TEST_F(MethodBaseScheduleTest, ResetOnActivationForgetsTheTimeInactive)
{
  start(20.0);
  run_rt_cycles(0.01, 2.0);
  set_time(now_ + 10.0);  // No cycles: EasyNav inactive
  method_->reset_rate_monitors();  // What the nodes do in on_activate()
  run_rt_cycles(0.01, 3.0);
  EXPECT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::OK);
  EXPECT_EQ(method_->get_rt_rate_monitor().slow_windows(), 0);
}

TEST_F(MethodBaseScheduleTest, ResetClearsAReportedSlowness)
{
  start(50.0);
  run_rt_cycles(0.1, 4.0);
  ASSERT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::WARN);
  ASSERT_NE(
    diagnostic("diagnostics.plugin.rt_rate")->message.find("not kept for"),
    std::string::npos);
  method_->reset_rate_monitors();
  rt_cycle();
  EXPECT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::OK);
}

TEST_F(MethodBaseScheduleTest, WithoutResetTheTimeInactiveIsSlow)
{
  start(20.0);
  run_rt_cycles(0.01, 2.0);
  set_time(now_ + 10.0);
  rt_cycle();
  EXPECT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::WARN);
  EXPECT_NE(
    diagnostic("diagnostics.plugin.rt_rate")->message.find("not kept for"),
    std::string::npos);
}

TEST_F(MethodBaseScheduleTest, ANewInstanceReplacesAStaleWarning)
{
  start(50.0);
  run_rt_cycles(0.1, 4.0);
  ASSERT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::WARN);
  ASSERT_NE(
    diagnostic("diagnostics.plugin.rt_rate")->message.find("not kept for"),
    std::string::npos);

  // Reconfigured: the same plugin, initialized again
  method_ = std::make_unique<easynav::MethodBase>();
  method_->initialize(node_, "plugin");
  rt_cycle();
  EXPECT_EQ(diagnostic("diagnostics.plugin.rt_rate")->level, DiagnosticStatus::OK);
}
