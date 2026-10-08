// Copyright 2025 Intelligent Robotics Lab
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


#ifndef EASYNAV_COMMON_TYPES__YTSESSION_HPP_
#define EASYNAV_COMMON_TYPES__YTSESSION_HPP_

#include <algorithm>
#include <string>

#include "easynav_common/Singleton.hpp"

#include "yaets/tracing.hpp"

namespace easynav
{

/**
 * @class YTSession
 * @brief A Yaets Tracing Session
 */
class YTSession : public yaets::TraceSession, public Singleton<YTSession>
{
public:
  /// \param ros_namespace The owning node's ROS namespace (e.g. "/robot_1" or
  ///   "/"), used to pick this instance's trace log path -- see log_path().
  ///   Only the first call across getInstance()/get() overloads (from any
  ///   translation unit) actually constructs the singleton, so this must be
  ///   supplied before any EASYNAV_TRACE_EVENT/EASYNAV_TRACE_NAMED_EVENT
  ///   fires; system_main.cpp does this right after creating system_node.
  explicit YTSession(const std::string & ros_namespace = "")
  : yaets::TraceSession(log_path(ros_namespace))
  {}

  ~YTSession()
  {
    stop();
  }

  SINGLETON_DEFINITIONS(YTSession)

private:
  /// \brief Trace log path, keyed by ROS namespace so that multiple EasyNav
  /// instances (one per robot) each get their own file the TUI/CLI tools can
  /// find without needing to know a PID: "/tmp/easynav.log" for the root
  /// namespace ("" or "/"), "/tmp/easynav_<ns>.log" (leading/trailing
  /// slashes stripped, inner slashes turned into "_") otherwise -- e.g.
  /// "/robot_1" -> "/tmp/easynav_robot_1.log".
  static std::string log_path(const std::string & ros_namespace)
  {
    std::string ns = ros_namespace;
    while (!ns.empty() && ns.front() == '/') {
      ns.erase(ns.begin());
    }
    while (!ns.empty() && ns.back() == '/') {
      ns.pop_back();
    }
    std::replace(ns.begin(), ns.end(), '/', '_');

    if (ns.empty()) {
      return "/tmp/easynav.log";
    }
    return "/tmp/easynav_" + ns + ".log";
  }
};

}  // namespace easynav

#ifdef EASYNAV_DEBUG_WITH_YAETS
  #define EASYNAV_TRACE_EVENT TRACE_EVENT(easynav::YTSession::get())
  #define EASYNAV_TRACE_NAMED_EVENT(name) yaets::TraceGuard guard(easynav::YTSession::get(), name);
#else
  #define EASYNAV_TRACE_EVENT ((void)0)
  #define EASYNAV_TRACE_NAMED_EVENT(name) ((void)0)
#endif


#endif  // EASYNAV_COMMON_TYPES__YTSESSION_HPP_
