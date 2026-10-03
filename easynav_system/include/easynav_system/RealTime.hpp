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

#ifndef EASYNAV_SYSTEM__REALTIME_HPP_
#define EASYNAV_SYSTEM__REALTIME_HPP_

#include <string>

namespace easynav
{

/// @brief Gives the calling thread SCHED_FIFO \p priority.
/// @return "" on success, otherwise why not.
std::string set_real_time_priority(int priority);

/// @brief Locks the process memory, current and future, so the RT cycle never page-faults.
/// Requires an unlimited RLIMIT_MEMLOCK: with a finite one, later allocations could fail.
/// @return "" on success, otherwise why not.
std::string lock_memory();

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__REALTIME_HPP_
