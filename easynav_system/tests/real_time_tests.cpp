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
/// \brief Real-time setup of the EasyNav process.

#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "easynav_system/RealTime.hpp"

TEST(RealTimeTest, LockMemoryRequiresAnUnlimitedMemlockLimit)
{
  rlimit original {};
  ASSERT_EQ(getrlimit(RLIMIT_MEMLOCK, &original), 0);

  // Lowering the soft limit is always allowed.
  rlimit finite = original;
  finite.rlim_cur = 64 * 1024;
  ASSERT_EQ(setrlimit(RLIMIT_MEMLOCK, &finite), 0);

  const auto error = easynav::lock_memory();
  EXPECT_NE(error.find("RLIMIT_MEMLOCK"), std::string::npos) << error;
  EXPECT_NE(error.find("unlimited"), std::string::npos) << error;

  ASSERT_EQ(setrlimit(RLIMIT_MEMLOCK, &original), 0);
}

TEST(RealTimeTest, InvalidPriorityIsReported)
{
  std::string error;
  std::thread([&error]() {error = easynav::set_real_time_priority(1000);}).join();
  EXPECT_NE(error.find("SCHED_FIFO priority 1000"), std::string::npos) << error;
}

TEST(RealTimeTest, PriorityWithinTheAllowedRangeIsSet)
{
  rlimit rtprio {};
  ASSERT_EQ(getrlimit(RLIMIT_RTPRIO, &rtprio), 0);
  const bool allowed = geteuid() == 0 || rtprio.rlim_cur >= 1;

  // In its own thread: the test's threads keep their scheduling.
  std::string error;
  int policy = -1;
  std::thread([&error, &policy]() {
      error = easynav::set_real_time_priority(1);
      policy = sched_getscheduler(0);
    }).join();

  if (allowed) {
    EXPECT_EQ(error, "");
    EXPECT_EQ(policy, SCHED_FIFO);
  } else {
    EXPECT_NE(error.find("RLIMIT_RTPRIO"), std::string::npos) << error;
    EXPECT_NE(policy, SCHED_FIFO);
  }
}
