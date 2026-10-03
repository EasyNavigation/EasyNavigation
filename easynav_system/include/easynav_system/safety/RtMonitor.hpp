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
/// \brief Declaration of the RtMonitor class.

#ifndef EASYNAV_SYSTEM__SAFETY__RTMONITOR_HPP_
#define EASYNAV_SYSTEM__SAFETY__RTMONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace easynav::safety
{

/**
 * @class RtMonitor
 * @brief Detects real-time cycles that start late.
 *
 * A cycle is late if it starts more than max_period_factor periods after the previous one. The
 * status is LATE after a late cycle, ERROR after max_late_cycles late cycles in a row, and OK again
 * after a cycle on time.
 */
class RtMonitor
{
public:
  using Clock = std::chrono::steady_clock;

  enum class Status : uint8_t {OK = 0, LATE = 1, ERROR = 2};

  /// @brief Sets the expected \p period (s) and the thresholds, and starts over.
  void configure(double period, double max_period_factor, int max_late_cycles);

  /// @brief Starts over (e.g. on activation): the next cycle has no previous one to be late from.
  void reset();

  /// @brief A cycle starts at \p now. @return The status after it.
  Status cycle_started(Clock::time_point now);

  [[nodiscard]] Status status() const {return status_;}

  /// @brief Late cycles since the last reset().
  [[nodiscard]] uint64_t late_cycles() const {return late_cycles_;}

  /// @brief Late cycles in a row, up to now.
  [[nodiscard]] int consecutive_late_cycles() const {return consecutive_late_;}

  /// @brief Time (s) between the last two cycle starts, 0 if none yet.
  [[nodiscard]] double last_period() const {return last_period_;}

private:
  double max_period_ {0.0};
  int max_late_cycles_ {1};

  std::optional<Clock::time_point> last_start_;
  Status status_ {Status::OK};
  uint64_t late_cycles_ {0};
  int consecutive_late_ {0};
  double last_period_ {0.0};
};

}  // namespace easynav::safety

#endif  // EASYNAV_SYSTEM__SAFETY__RTMONITOR_HPP_
