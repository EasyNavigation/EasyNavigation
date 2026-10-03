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
/// \brief Implementation of the RtMonitor class.

#include "easynav_system/safety/RtMonitor.hpp"

namespace easynav::safety
{

void
RtMonitor::configure(double period, double max_period_factor, int max_late_cycles)
{
  max_period_ = period * max_period_factor;
  max_late_cycles_ = max_late_cycles;
  reset();
}

void
RtMonitor::reset()
{
  last_start_.reset();
  status_ = Status::OK;
  late_cycles_ = 0;
  consecutive_late_ = 0;
  last_period_ = 0.0;
}

RtMonitor::Status
RtMonitor::cycle_started(Clock::time_point now)
{
  if (last_start_) {
    last_period_ = std::chrono::duration<double>(now - *last_start_).count();
    if (last_period_ > max_period_) {
      ++late_cycles_;
      ++consecutive_late_;
    } else {
      consecutive_late_ = 0;
    }
  }
  last_start_ = now;

  if (consecutive_late_ >= max_late_cycles_) {
    status_ = Status::ERROR;
  } else if (consecutive_late_ > 0) {
    status_ = Status::LATE;
  } else {
    status_ = Status::OK;
  }
  return status_;
}

}  // namespace easynav::safety
