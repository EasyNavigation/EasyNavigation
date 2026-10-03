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
/// \brief safety::RtMonitor, with simulated cycle start times.

#include <chrono>

#include "gtest/gtest.h"

#include "easynav_system/safety/RtMonitor.hpp"

using easynav::safety::RtMonitor;
using Status = RtMonitor::Status;
using namespace std::chrono_literals;

namespace
{

// 200 Hz: late if more than 2 periods (10 ms) after the previous cycle.
RtMonitor monitor(int max_late_cycles = 3)
{
  RtMonitor m;
  m.configure(0.005, 2.0, max_late_cycles);
  return m;
}

const RtMonitor::Clock::time_point t0 = RtMonitor::Clock::time_point{} + 1h;

}  // namespace

TEST(RtMonitorTest, TheFirstCycleIsNeverLate)
{
  auto m = monitor();
  EXPECT_EQ(m.cycle_started(t0), Status::OK);
  EXPECT_EQ(m.late_cycles(), 0u);
  EXPECT_DOUBLE_EQ(m.last_period(), 0.0);
}

TEST(RtMonitorTest, CyclesOnTimeStayOk)
{
  auto m = monitor();
  auto t = t0;
  for (int i = 0; i < 1000; ++i, t += 5ms) {
    EXPECT_EQ(m.cycle_started(t), Status::OK) << i;
  }
  EXPECT_EQ(m.late_cycles(), 0u);
  EXPECT_DOUBLE_EQ(m.last_period(), 0.005);
}

TEST(RtMonitorTest, LateIsMoreThanTheMaximumPeriod)
{
  auto m = monitor();
  m.cycle_started(t0);
  EXPECT_EQ(m.cycle_started(t0 + 10ms), Status::OK) << "exactly the maximum: on time";
  EXPECT_EQ(m.cycle_started(t0 + 10ms + 10ms + 1us), Status::LATE);
  EXPECT_EQ(m.late_cycles(), 1u);
}

TEST(RtMonitorTest, ALateCycleThenOnTimeAgain)
{
  auto m = monitor();
  m.cycle_started(t0);
  EXPECT_EQ(m.cycle_started(t0 + 50ms), Status::LATE);
  EXPECT_EQ(m.consecutive_late_cycles(), 1);
  EXPECT_DOUBLE_EQ(m.last_period(), 0.05);
  EXPECT_EQ(m.cycle_started(t0 + 55ms), Status::OK);
  EXPECT_EQ(m.consecutive_late_cycles(), 0);
  EXPECT_EQ(m.late_cycles(), 1u) << "the total is kept";
}

TEST(RtMonitorTest, TooManyLateCyclesInARowAreAnError)
{
  auto m = monitor(3);
  auto t = t0;
  m.cycle_started(t);
  EXPECT_EQ(m.cycle_started(t += 20ms), Status::LATE);
  EXPECT_EQ(m.cycle_started(t += 20ms), Status::LATE);
  EXPECT_EQ(m.cycle_started(t += 20ms), Status::ERROR);
  EXPECT_EQ(m.cycle_started(t += 20ms), Status::ERROR);
  EXPECT_EQ(m.consecutive_late_cycles(), 4);

  // One on time ends it.
  EXPECT_EQ(m.cycle_started(t += 5ms), Status::OK);
  EXPECT_EQ(m.late_cycles(), 4u);

  // Late cycles not in a row never add up to an error.
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(m.cycle_started(t += 20ms), Status::LATE);
    EXPECT_EQ(m.cycle_started(t += 5ms), Status::OK);
  }
}

TEST(RtMonitorTest, OneLateCycleIsAnErrorIfSoConfigured)
{
  auto m = monitor(1);
  m.cycle_started(t0);
  EXPECT_EQ(m.cycle_started(t0 + 20ms), Status::ERROR);
}

TEST(RtMonitorTest, ResetForgetsThePreviousCycle)
{
  // E.g. EasyNav was inactive for a while: no RT cycles then.
  auto m = monitor(1);
  m.cycle_started(t0);
  ASSERT_EQ(m.cycle_started(t0 + 20ms), Status::ERROR);

  m.reset();
  EXPECT_EQ(m.status(), Status::OK);
  EXPECT_EQ(m.late_cycles(), 0u);
  EXPECT_EQ(m.cycle_started(t0 + 10s), Status::OK) << "not late from before the reset";
  EXPECT_EQ(m.cycle_started(t0 + 10s + 5ms), Status::OK);
}

TEST(RtMonitorTest, ConfigureChangesThePeriodAndStartsOver)
{
  auto m = monitor(1);
  m.cycle_started(t0);
  ASSERT_EQ(m.cycle_started(t0 + 20ms), Status::ERROR);

  m.configure(0.1, 2.0, 1);  // 10 Hz: late if more than 200 ms
  EXPECT_EQ(m.status(), Status::OK);
  m.cycle_started(t0 + 1s);
  EXPECT_EQ(m.cycle_started(t0 + 1s + 150ms), Status::OK);
  EXPECT_EQ(m.cycle_started(t0 + 1s + 400ms), Status::ERROR);
}
