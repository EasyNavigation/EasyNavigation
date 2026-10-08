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
/// \brief LogCapture: records the log messages emitted while it lives (for tests).

#ifndef EASYNAV_COMMON__TESTING__LOGCAPTURE_HPP_
#define EASYNAV_COMMON__TESTING__LOGCAPTURE_HPP_

#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include "rcutils/logging.h"

namespace easynav
{
namespace testing
{

/// @brief Records the log messages (and their severity) emitted while it lives. One at a time.
class LogCapture
{
public:
  struct Message
  {
    int severity;
    std::string text;
  };

  LogCapture()
  {
    previous_ = rcutils_logging_get_output_handler();
    std::lock_guard<std::mutex> lock(mutex());
    messages().clear();
    rcutils_logging_set_output_handler(&LogCapture::handler);
  }

  ~LogCapture() {rcutils_logging_set_output_handler(previous_);}

  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;

  /// @brief Messages containing every one of \p parts, with severity \p severity (-1: any).
  std::vector<std::string> find(
    const std::vector<std::string> & parts, int severity = RCUTILS_LOG_SEVERITY_WARN) const
  {
    std::lock_guard<std::mutex> lock(mutex());
    std::vector<std::string> found;
    for (const auto & message : messages()) {
      if (severity != -1 && message.severity != severity) {
        continue;
      }
      bool all = true;
      for (const auto & part : parts) {
        all = all && message.text.find(part) != std::string::npos;
      }
      if (all) {
        found.push_back(message.text);
      }
    }
    return found;
  }

  /// @brief Number of messages containing every one of \p parts (warnings by default).
  size_t count(
    const std::vector<std::string> & parts, int severity = RCUTILS_LOG_SEVERITY_WARN) const
  {
    return find(parts, severity).size();
  }

private:
  static void handler(
    const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
    const char * format, va_list * args)
  {
    va_list copy;
    va_copy(copy, *args);
    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    std::lock_guard<std::mutex> lock(mutex());
    messages().push_back({severity, buffer});
  }

  static std::mutex & mutex()
  {
    static std::mutex m;
    return m;
  }

  static std::vector<Message> & messages()
  {
    static std::vector<Message> m;
    return m;
  }

  rcutils_logging_output_handler_t previous_;
};

}  // namespace testing
}  // namespace easynav

#endif  // EASYNAV_COMMON__TESTING__LOGCAPTURE_HPP_
