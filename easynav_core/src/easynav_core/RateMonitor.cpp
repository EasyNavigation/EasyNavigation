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
/// \brief Implementation of the RateMonitor class.

#include <algorithm>

#include "easynav_core/RateMonitor.hpp"

namespace easynav
{

void
RateMonitor::configure(double frequency)
{
  frequency_ = frequency;
  window_ = std::max(kMinWindow, kWindowPeriods / frequency);
  reset();
}

void
RateMonitor::reset()
{
  started_ = false;
  runs_ = 0;
  rate_ = 0.0;
  slow_windows_ = 0;
  status_ = Status::OK;
}

void
RateMonitor::start_window(double now)
{
  started_ = true;
  window_start_ = now;
  last_update_ = now;
  runs_ = 0;
}

void
RateMonitor::run(double now)
{
  if (!started_ || now < window_start_) {
    start_window(now);
  }
  ++runs_;
}

RateMonitor::Status
RateMonitor::update(double now)
{
  if (!started_ || now < last_update_) {
    start_window(now);  // First check, or the clock went back
    return status_;
  }
  last_update_ = now;

  const double elapsed = now - window_start_;
  if (elapsed < window_) {
    return status_;
  }

  rate_ = runs_ / elapsed;
  // +1: a window edge may cut a run that is on time
  if (runs_ + 1 < kMinRatio * frequency_ * elapsed) {
    // A long gap (e.g. blocked) counts as every window it spans
    slow_windows_ += static_cast<int>(elapsed / window_);
  } else {
    slow_windows_ = 0;
  }
  if (slow_windows_ >= kMaxSlowWindows) {
    status_ = Status::ERROR;
  } else if (slow_windows_ > 0) {
    status_ = Status::WARN;
  } else {
    status_ = Status::OK;
  }

  window_start_ = now;
  runs_ = 0;
  return status_;
}

}  // namespace easynav
