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
/// \brief Declaration of the SafetyChannelMonitor class.

#ifndef EASYNAV_SYSTEM__SAFETY__SAFETYCHANNELMONITOR_HPP_
#define EASYNAV_SYSTEM__SAFETY__SAFETYCHANNELMONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "easynav_interfaces/msg/safety_status.hpp"

#include "easynav_core/SafetyChannel.hpp"

namespace easynav::safety
{

/**
 * @class SafetyChannelMonitor
 * @brief Turns the safety channel's SafetyStatus messages into what EasyNav applies.
 *
 * With no valid status received within the last timeout seconds, the status is lost, and that is
 * treated as a protective stop. Times are reception times on the monotonic clock, so they do not
 * depend on the publisher's clock. received() and evaluate() may run in different threads.
 */
class SafetyChannelMonitor
{
public:
  using Clock = std::chrono::steady_clock;
  using Status = easynav_interfaces::msg::SafetyStatus;

  /// @brief Why the state is what it is.
  enum class Condition : uint8_t {VALID, NO_STATUS, STALE, INVALID};

  struct Evaluation
  {
    SafetyChannelState state;
    Condition condition {Condition::NO_STATUS};
    bool operator==(const Evaluation & other) const
    {
      return state == other.state && condition == other.condition;
    }
  };

  /// @brief Sets the \p timeout (s, > 0) and forgets any status.
  void configure(double timeout);

  /// @brief A status \p msg was received at \p now.
  void received(const Status & msg, Clock::time_point now);

  /// @brief The state to apply at \p now.
  Evaluation evaluate(Clock::time_point now) const;

  /// @brief The last status received, if any.
  std::optional<Status> last_status() const;

  /// @brief Why \p msg is invalid, or "".
  static std::string invalid_reason(const Status & msg);

private:
  Clock::duration timeout_ {};
  mutable std::mutex mutex_;
  std::optional<Status> last_;
  Clock::time_point last_time_ {};
  bool last_valid_ {false};
};

}  // namespace easynav::safety

#endif  // EASYNAV_SYSTEM__SAFETY__SAFETYCHANNELMONITOR_HPP_
