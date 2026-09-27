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
/// \brief Declaration of the DummySafetyReflex class.

#ifndef EASYNAV_RECOVERY__DUMMYSAFETYREFLEX_HPP_
#define EASYNAV_RECOVERY__DUMMYSAFETYREFLEX_HPP_

#include "easynav_core/SafetyReflexBase.hpp"

namespace easynav
{

/**
 * @class DummySafetyReflex
 * @brief A default "dummy" implementation for SafetyReflexBase.
 *
 * Never intervenes, unless its "trigger" parameter is true: then it stops the robot every RT
 * cycle. It serves as an example, and as a real, always-loadable reflex for tests that need one
 * without pulling in a specific reflex's dependencies.
 */
class DummySafetyReflex : public easynav::SafetyReflexBase
{
public:
  DummySafetyReflex() = default;
  ~DummySafetyReflex() = default;

  void on_initialize() override;

protected:
  bool check(NavState & nav_state) override;
  void mitigate(NavState & nav_state) override;

private:
  bool trigger_ {false};
};

}  // namespace easynav

#endif  // EASYNAV_RECOVERY__DUMMYSAFETYREFLEX_HPP_
