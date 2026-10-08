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

TEST(RealTimeTest, PrioritySetOrWhyNot)
{
  // Whether SCHED_FIFO is allowed depends on more than RLIMIT_RTPRIO (e.g. CAP_SYS_NICE): the
  // result itself tells.
  std::string error;
  int policy = -1;
  std::thread(
    [&error, &policy]() {            // Its own thread: the test's keep their scheduling.
      error = easynav::set_real_time_priority(1);
      policy = sched_getscheduler(0);
    }).join();

  if (error.empty()) {
    EXPECT_EQ(policy, SCHED_FIFO);
  } else {
    EXPECT_NE(error.find("SCHED_FIFO priority 1"), std::string::npos) << error;
    EXPECT_NE(policy, SCHED_FIFO);
  }
}

TEST(RealTimeTest, CheckingThePriorityLeavesTheCallerUntouched)
{
  const int before = sched_getscheduler(0);
  const auto error = easynav::check_real_time_priority(easynav::kRealTimePriority);
  EXPECT_EQ(sched_getscheduler(0), before);

  std::string direct;
  std::thread(
    [&direct]() {
      direct = easynav::set_real_time_priority(easynav::kRealTimePriority);
    }).join();
  EXPECT_EQ(error.empty(), direct.empty()) << "the same answer as trying it";
}

TEST(RealTimeTest, CheckingTheMemoryLockDoesNotLock)
{
  rlimit original {};
  ASSERT_EQ(getrlimit(RLIMIT_MEMLOCK, &original), 0);
  rlimit finite = original;
  finite.rlim_cur = 64 * 1024;
  ASSERT_EQ(setrlimit(RLIMIT_MEMLOCK, &finite), 0);
  const auto error = easynav::check_memory_lock();
  ASSERT_EQ(setrlimit(RLIMIT_MEMLOCK, &original), 0);
  EXPECT_NE(error.find("RLIMIT_MEMLOCK"), std::string::npos) << error;

  if (original.rlim_cur == RLIM_INFINITY) {
    EXPECT_EQ(easynav::check_memory_lock(), "");
  }
}
