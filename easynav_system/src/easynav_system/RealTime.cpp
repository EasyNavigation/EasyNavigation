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
/// \brief Implementation of the real-time setup of the EasyNav process.

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <thread>

#include "easynav_system/RealTime.hpp"

namespace easynav
{

std::string
set_real_time_priority(int priority)
{
  sched_param param {};
  param.sched_priority = priority;
  const int error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (error != 0) {
    return "SCHED_FIFO priority " + std::to_string(priority) + ": " + std::strerror(error) +
           " (check RLIMIT_RTPRIO, e.g. 'ulimit -r')";
  }
  return "";
}

std::string
check_real_time_priority(int priority)
{
  std::string error;
  std::thread([&error, priority]() {error = set_real_time_priority(priority);}).join();
  return error;
}

std::string
check_memory_lock()
{
  rlimit limit {};
  if (getrlimit(RLIMIT_MEMLOCK, &limit) != 0) {
    return std::string("getrlimit(RLIMIT_MEMLOCK): ") + std::strerror(errno);
  }
  if (limit.rlim_cur != RLIM_INFINITY) {
    return "RLIMIT_MEMLOCK is " + std::to_string(limit.rlim_cur) +
           " bytes, it must be unlimited (e.g. 'ulimit -l unlimited')";
  }
  return "";
}

std::string
lock_memory()
{
  if (const auto error = check_memory_lock(); !error.empty()) {
    return error;
  }
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    return std::string("mlockall: ") + std::strerror(errno);
  }
  return "";
}

}  // namespace easynav
