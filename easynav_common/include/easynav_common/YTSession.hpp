// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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
  explicit YTSession()
  : yaets::TraceSession("/tmp/easynav.log")
  {}

  ~YTSession()
  {
    stop();
  }

  SINGLETON_DEFINITIONS(YTSession)
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
