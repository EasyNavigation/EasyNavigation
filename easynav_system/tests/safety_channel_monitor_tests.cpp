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
/// \brief SafetyChannelMonitor, with simulated times.

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "gtest/gtest.h"

#include "easynav_system/safety/SafetyChannelMonitor.hpp"

using easynav::safety::SafetyChannelMonitor;
using Condition = SafetyChannelMonitor::Condition;
using namespace std::chrono_literals;

namespace
{

SafetyChannelMonitor::Status status(
  bool stop, bool limited = false, double linear = 0.0, double angular = 0.0)
{
  SafetyChannelMonitor::Status msg;
  msg.protective_stop = stop;
  msg.speed_limited = limited;
  msg.max_linear_vel = linear;
  msg.max_angular_vel = angular;
  return msg;
}

const SafetyChannelMonitor::Clock::time_point t0 {std::chrono::seconds(1000)};

}  // namespace

TEST(SafetyChannelMonitorTest, WithoutStatusItIsAProtectiveStop)
{
  SafetyChannelMonitor monitor;
  monitor.configure(0.5);
  const auto evaluation = monitor.evaluate(t0);
  EXPECT_EQ(evaluation.condition, Condition::NO_STATUS);
  EXPECT_TRUE(evaluation.state.protective_stop);
  EXPECT_TRUE(evaluation.state.status_lost);
  EXPECT_FALSE(monitor.last_status());
}

TEST(SafetyChannelMonitorTest, AValidStatusIsAppliedAsIs)
{
  SafetyChannelMonitor monitor;
  monitor.configure(0.5);

  monitor.received(status(false), t0);
  auto evaluation = monitor.evaluate(t0 + 100ms);
  EXPECT_EQ(evaluation.condition, Condition::VALID);
  EXPECT_FALSE(evaluation.state.protective_stop);
  EXPECT_FALSE(evaluation.state.status_lost);
  EXPECT_TRUE(std::isinf(evaluation.state.max_linear_vel)) << "no speed limit";
  EXPECT_TRUE(std::isinf(evaluation.state.max_angular_vel));

  monitor.received(status(true), t0 + 200ms);
  evaluation = monitor.evaluate(t0 + 200ms);
  EXPECT_TRUE(evaluation.state.protective_stop);
  EXPECT_FALSE(evaluation.state.status_lost);
}

TEST(SafetyChannelMonitorTest, TheSpeedLimitOnlyAppliesWhenFlagged)
{
  SafetyChannelMonitor monitor;
  monitor.configure(0.5);

  monitor.received(status(false, false, 0.1, 0.2), t0);
  EXPECT_TRUE(std::isinf(monitor.evaluate(t0).state.max_linear_vel)) << "not speed_limited";

  monitor.received(status(false, true, 0.1, 0.2), t0);
  auto state = monitor.evaluate(t0).state;
  EXPECT_DOUBLE_EQ(state.max_linear_vel, 0.1);
  EXPECT_DOUBLE_EQ(state.max_angular_vel, 0.2);

  monitor.received(status(false, true, 0.0, 0.0), t0);  // Zero is a valid limit.
  state = monitor.evaluate(t0).state;
  EXPECT_EQ(monitor.evaluate(t0).condition, Condition::VALID);
  EXPECT_DOUBLE_EQ(state.max_linear_vel, 0.0);
}

TEST(SafetyChannelMonitorTest, AStatusIsValidUpToTheTimeoutAndStaleAfter)
{
  SafetyChannelMonitor monitor;
  monitor.configure(0.5);
  monitor.received(status(false), t0);

  EXPECT_EQ(monitor.evaluate(t0 + 500ms).condition, Condition::VALID) << "boundary";
  const auto stale = monitor.evaluate(t0 + 501ms);
  EXPECT_EQ(stale.condition, Condition::STALE);
  EXPECT_TRUE(stale.state.protective_stop);
  EXPECT_TRUE(stale.state.status_lost);
  EXPECT_TRUE(std::isinf(stale.state.max_linear_vel));

  monitor.received(status(false), t0 + 600ms);  // Back.
  EXPECT_EQ(monitor.evaluate(t0 + 600ms).condition, Condition::VALID);
}

TEST(SafetyChannelMonitorTest, InvalidStatusesAreAProtectiveStop)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto & msg : {status(false, true, -0.1, 1.0), status(false, true, 1.0, -0.1),
      status(false, true, nan, 1.0), status(false, true, 1.0, nan),
      status(false, true, inf, 1.0)})
  {
    SafetyChannelMonitor monitor;
    monitor.configure(0.5);
    monitor.received(msg, t0);
    const auto evaluation = monitor.evaluate(t0);
    EXPECT_EQ(evaluation.condition, Condition::INVALID);
    EXPECT_TRUE(evaluation.state.protective_stop);
    EXPECT_FALSE(SafetyChannelMonitor::invalid_reason(msg).empty());
  }
  EXPECT_TRUE(SafetyChannelMonitor::invalid_reason(status(false, false, nan, -1.0)).empty())
    << "the limits are ignored when not speed_limited";
}

TEST(SafetyChannelMonitorTest, AValidStatusAfterAnInvalidOneIsApplied)
{
  SafetyChannelMonitor monitor;
  monitor.configure(0.5);
  monitor.received(status(false, true, -1.0, 1.0), t0);
  ASSERT_EQ(monitor.evaluate(t0).condition, Condition::INVALID);
  monitor.received(status(false), t0 + 10ms);
  EXPECT_EQ(monitor.evaluate(t0 + 10ms).condition, Condition::VALID);
}

TEST(SafetyChannelMonitorTest, ConfigureForgetsTheLastStatus)
{
  SafetyChannelMonitor monitor;
  monitor.configure(0.5);
  monitor.received(status(false), t0);
  ASSERT_TRUE(monitor.last_status());

  monitor.configure(1.0);
  EXPECT_FALSE(monitor.last_status());
  EXPECT_EQ(monitor.evaluate(t0).condition, Condition::NO_STATUS);
}

TEST(SafetyChannelMonitorTest, EvaluationsCompareByStateAndCondition)
{
  SafetyChannelMonitor monitor;
  monitor.configure(0.5);
  monitor.received(status(false, true, 0.3, 0.5), t0);
  const auto a = monitor.evaluate(t0);
  EXPECT_TRUE(a == monitor.evaluate(t0 + 100ms));
  monitor.received(status(false, true, 0.2, 0.5), t0 + 200ms);
  EXPECT_FALSE(a == monitor.evaluate(t0 + 200ms)) << "another speed limit";
  EXPECT_FALSE(a == monitor.evaluate(t0 + 2s)) << "stale";
}

TEST(SafetyChannelMonitorTest, ReceivingAndEvaluatingFromDifferentThreads)
{
  SafetyChannelMonitor monitor;
  monitor.configure(10.0);
  std::thread receiver([&monitor]() {
      for (int i = 0; i < 10000; ++i) {
        monitor.received(status(i % 2 == 0, true, 0.1 * (i % 5), 0.2), t0);
      }
    });
  for (int i = 0; i < 10000; ++i) {
    const auto evaluation = monitor.evaluate(t0);
    if (evaluation.condition == Condition::VALID) {
      EXPECT_LE(evaluation.state.max_linear_vel, 0.4 + 1e-9);
      EXPECT_DOUBLE_EQ(evaluation.state.max_angular_vel, 0.2);
    }
  }
  receiver.join();
}
