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

#include <cmath>

#include "gtest/gtest.h"

#include "easynav_core/RateMonitor.hpp"

using easynav::RateMonitor;
using Status = RateMonitor::Status;

namespace
{

// Runs at rate Hz while checking at check Hz, from t to t + duration; returns the end time.
double simulate(RateMonitor & monitor, double t, double duration, double rate, double check)
{
  const double end = t + duration;
  double next_run = t;
  for (; t < end; t += 1.0 / check) {
    if (rate > 0.0 && t >= next_run) {
      monitor.run(t);
      next_run += 1.0 / rate;
    }
    monitor.update(t);
  }
  return t;
}

}  // namespace

TEST(RateMonitorTest, WindowIsOneSecondOrTenPeriods)
{
  RateMonitor monitor;
  monitor.configure(30.0);
  EXPECT_DOUBLE_EQ(monitor.window(), 1.0);
  monitor.configure(10.0);
  EXPECT_DOUBLE_EQ(monitor.window(), 1.0);
  monitor.configure(2.0);
  EXPECT_DOUBLE_EQ(monitor.window(), 5.0);
  monitor.configure(0.5);
  EXPECT_DOUBLE_EQ(monitor.window(), 20.0);
}

TEST(RateMonitorTest, OkWhileNoWindowClosed)
{
  RateMonitor monitor;
  monitor.configure(10.0);
  EXPECT_EQ(monitor.update(0.0), Status::OK);
  EXPECT_EQ(monitor.update(0.5), Status::OK) << "no run, but the window is still open";
  EXPECT_DOUBLE_EQ(monitor.rate(), 0.0);
}

TEST(RateMonitorTest, OnRateIsOk)
{
  for (const double freq : {1.0, 10.0, 30.0, 50.0, 200.0}) {
    RateMonitor monitor;
    monitor.configure(freq);
    simulate(monitor, 0.0, 30.0, freq, 1000.0);
    EXPECT_EQ(monitor.status(), Status::OK) << freq << " Hz";
    EXPECT_NEAR(monitor.rate(), freq, 0.1 * freq + 0.2) << freq << " Hz";
    EXPECT_EQ(monitor.slow_windows(), 0) << freq << " Hz";
  }
}

TEST(RateMonitorTest, FasterThanConfiguredIsOk)
{
  // E.g. a controller also triggered by the sensors
  RateMonitor monitor;
  monitor.configure(10.0);
  simulate(monitor, 0.0, 5.0, 25.0, 200.0);
  EXPECT_EQ(monitor.status(), Status::OK);
  EXPECT_GT(monitor.rate(), 20.0);
}

TEST(RateMonitorTest, WithinTheToleranceIsOk)
{
  RateMonitor monitor;
  monitor.configure(30.0);
  simulate(monitor, 0.0, 10.0, 28.0, 1000.0);  // 93 %
  EXPECT_EQ(monitor.status(), Status::OK);
}

TEST(RateMonitorTest, SlowIsWarnThenErrorAfterSlowWindowsInARow)
{
  RateMonitor monitor;
  monitor.configure(50.0);
  double t = simulate(monitor, 0.0, 2.0, 50.0, 200.0);
  ASSERT_EQ(monitor.status(), Status::OK);

  // Half the rate, e.g. 50 Hz in an RT cycle that only manages 25 Hz
  t = simulate(monitor, t, 1.05, 25.0, 200.0);
  EXPECT_EQ(monitor.status(), Status::WARN);
  EXPECT_EQ(monitor.slow_windows(), 1);
  EXPECT_NEAR(monitor.rate(), 25.0, 2.0);

  t = simulate(monitor, t, 1.0, 25.0, 200.0);
  EXPECT_EQ(monitor.status(), Status::WARN);
  t = simulate(monitor, t, 1.0, 25.0, 200.0);
  EXPECT_EQ(monitor.status(), Status::ERROR);
  EXPECT_EQ(monitor.slow_windows(), RateMonitor::kMaxSlowWindows);

  t = simulate(monitor, t, 3.0, 25.0, 200.0);
  EXPECT_EQ(monitor.status(), Status::ERROR) << "stays in error while slow";

  simulate(monitor, t, 1.05, 50.0, 200.0);
  EXPECT_EQ(monitor.status(), Status::OK) << "one window on rate";
  EXPECT_EQ(monitor.slow_windows(), 0);
}

TEST(RateMonitorTest, OneSlowWindowBetweenGoodOnesIsOnlyAWarn)
{
  RateMonitor monitor;
  monitor.configure(20.0);
  double t = simulate(monitor, 0.0, 2.0, 20.0, 100.0);
  for (int i = 0; i < 5; ++i) {
    t = simulate(monitor, t, 1.0, 10.0, 100.0);
    EXPECT_NE(monitor.status(), Status::ERROR) << i;
    t = simulate(monitor, t, 2.05, 20.0, 100.0);  // At least a whole window on rate
    EXPECT_EQ(monitor.status(), Status::OK) << i;
  }
}

TEST(RateMonitorTest, NoRunsWhileCheckedIsAnError)
{
  // Checked, but never runs: e.g. a plugin whose schedule is broken
  RateMonitor monitor;
  monitor.configure(10.0);
  simulate(monitor, 0.0, 3.5, 0.0, 100.0);
  EXPECT_EQ(monitor.status(), Status::ERROR);
  EXPECT_DOUBLE_EQ(monitor.rate(), 0.0);
}

TEST(RateMonitorTest, AGapWithoutChecksCountsAsEveryWindowItSpans)
{
  // E.g. the component blocked its cycle for 5 s: no checks, no runs
  RateMonitor monitor;
  monitor.configure(10.0);
  double t = simulate(monitor, 0.0, 2.0, 10.0, 100.0);
  ASSERT_EQ(monitor.status(), Status::OK);
  t += 5.0;
  EXPECT_EQ(monitor.update(t), Status::ERROR);
  EXPECT_GE(monitor.slow_windows(), 5);
}

TEST(RateMonitorTest, AShortGapIsOnlyAWarning)
{
  RateMonitor monitor;
  monitor.configure(10.0);
  double t = simulate(monitor, 0.0, 2.0, 10.0, 100.0);
  t += 1.5;
  EXPECT_EQ(monitor.update(t), Status::WARN);
  // The gap, plus the part of the window already open: 1 or 2 windows, never 3
  EXPECT_GE(monitor.slow_windows(), 1);
  EXPECT_LE(monitor.slow_windows(), 2);
  simulate(monitor, t, 2.05, 10.0, 100.0);
  EXPECT_EQ(monitor.status(), Status::OK) << "on rate again";
}

TEST(RateMonitorTest, BlockingLongerThanAWindowOnEveryRunIsAnError)
{
  // Each run blocks the cycle 1.5 s: 0.67 Hz instead of 10
  RateMonitor monitor;
  monitor.configure(10.0);
  double t = 0.0;
  for (int i = 0; i < 4; ++i, t += 1.5) {
    monitor.update(t);
    monitor.run(t);
  }
  monitor.update(t);
  EXPECT_EQ(monitor.status(), Status::ERROR);
}

TEST(RateMonitorTest, ResetBeforeResumingForgetsTheGap)
{
  // What the nodes do on activation, after being inactive
  RateMonitor monitor;
  monitor.configure(10.0);
  double t = simulate(monitor, 0.0, 2.0, 10.0, 100.0);
  t += 5.0;
  monitor.reset();
  simulate(monitor, t, 3.0, 10.0, 100.0);
  EXPECT_EQ(monitor.status(), Status::OK);
  EXPECT_EQ(monitor.slow_windows(), 0);
}

TEST(RateMonitorTest, AJumpBackInTimeRestartsTheWindow)
{
  RateMonitor monitor;
  monitor.configure(10.0);
  simulate(monitor, 100.0, 2.0, 10.0, 100.0);
  // A simulation reset: the clock starts again
  simulate(monitor, 0.0, 3.0, 10.0, 100.0);
  EXPECT_EQ(monitor.status(), Status::OK);
  EXPECT_NEAR(monitor.rate(), 10.0, 1.0);
}

TEST(RateMonitorTest, LowFrequencyNeedsItsLongerWindow)
{
  // 1 Hz: windows of 10 s, so one late run is within the margin
  RateMonitor monitor;
  monitor.configure(1.0);
  double t = 0.0;
  double next_run = 0.0;
  for (; t < 30.0; t += 0.01) {
    if (t >= next_run) {
      monitor.run(t);
      next_run += (std::fmod(t, 7.0) < 0.01) ? 1.4 : 1.0;  // Now and then 0.4 s late
    }
    monitor.update(t);
  }
  EXPECT_EQ(monitor.status(), Status::OK);
}

TEST(RateMonitorTest, ResetForgetsTheStatus)
{
  RateMonitor monitor;
  monitor.configure(10.0);
  simulate(monitor, 0.0, 3.5, 0.0, 100.0);
  ASSERT_EQ(monitor.status(), Status::ERROR);
  monitor.reset();
  EXPECT_EQ(monitor.status(), Status::OK);
  EXPECT_EQ(monitor.slow_windows(), 0);
  EXPECT_DOUBLE_EQ(monitor.rate(), 0.0);
}

TEST(RateMonitorTest, ConfigureChangesTheFrequencyAndResets)
{
  RateMonitor monitor;
  monitor.configure(50.0);
  double t = simulate(monitor, 0.0, 3.5, 20.0, 100.0);
  ASSERT_EQ(monitor.status(), Status::ERROR);
  monitor.configure(20.0);
  EXPECT_EQ(monitor.status(), Status::OK);
  simulate(monitor, t, 3.5, 20.0, 100.0);
  EXPECT_EQ(monitor.status(), Status::OK);
  EXPECT_DOUBLE_EQ(monitor.frequency(), 20.0);
}
