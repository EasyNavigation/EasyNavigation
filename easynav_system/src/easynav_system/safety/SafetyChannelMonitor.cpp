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
/// \brief Implementation of the SafetyChannelMonitor class.

#include <cmath>
#include <string>
#include <utility>

#include "easynav_system/safety/SafetyChannelMonitor.hpp"

namespace easynav::safety
{

void
SafetyChannelMonitor::configure(double timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
  last_.reset();
  last_valid_ = false;
}

void
SafetyChannelMonitor::received(const Status & msg, Clock::time_point now)
{
  const bool valid = invalid_reason(msg).empty();
  std::lock_guard<std::mutex> lock(mutex_);
  last_ = msg;
  last_time_ = now;
  last_valid_ = valid;
}

SafetyChannelMonitor::Evaluation
SafetyChannelMonitor::evaluate(Clock::time_point now) const
{
  Evaluation evaluation;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_) {
    evaluation.condition = Condition::NO_STATUS;
  } else if (!last_valid_) {
    evaluation.condition = Condition::INVALID;
  } else if (now - last_time_ > timeout_) {
    evaluation.condition = Condition::STALE;
  } else {
    evaluation.condition = Condition::VALID;
  }

  // Nothing says the robot may move: a protective stop.
  if (evaluation.condition != Condition::VALID) {
    evaluation.state.status_lost = true;
    evaluation.state.protective_stop = true;
    return evaluation;
  }
  evaluation.state.protective_stop = last_->protective_stop;
  if (last_->speed_limited) {
    evaluation.state.max_linear_vel = last_->max_linear_vel;
    evaluation.state.max_angular_vel = last_->max_angular_vel;
  }
  return evaluation;
}

std::optional<SafetyChannelMonitor::Status>
SafetyChannelMonitor::last_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_;
}

std::string
SafetyChannelMonitor::invalid_reason(const Status & msg)
{
  if (!msg.speed_limited) {
    return "";
  }
  for (const auto & [name, value] : {
      std::pair<const char *, double>{"max_linear_vel", msg.max_linear_vel},
      std::pair<const char *, double>{"max_angular_vel", msg.max_angular_vel}})
  {
    if (!std::isfinite(value) || value < 0.0) {
      return std::string(name) + " = " + std::to_string(value) + " (speed_limited: >= 0)";
    }
  }
  return "";
}

}  // namespace easynav::safety
