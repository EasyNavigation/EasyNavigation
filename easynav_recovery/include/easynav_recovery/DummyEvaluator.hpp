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
/// \brief Declaration of the DummyEvaluator plugin.

#ifndef EASYNAV_RECOVERY__DUMMYEVALUATOR_HPP_
#define EASYNAV_RECOVERY__DUMMYEVALUATOR_HPP_

#include "easynav_core/RecoveryEvaluatorBase.hpp"

namespace easynav
{

/**
 * @class DummyEvaluator
 * @brief A default "dummy" implementation for RecoveryEvaluatorBase.
 *
 * Always reports diagnostic_msgs::msg::DiagnosticStatus::OK. It serves as an example, a
 * reference no-op plugin, and a real, always-loadable evaluator for tests that need one
 * without pulling in a specific diagnosis's dependencies.
 */
class DummyEvaluator : public easynav::RecoveryEvaluatorBase
{
public:
  DummyEvaluator() = default;
  ~DummyEvaluator() = default;

  void on_initialize() override;

protected:
  void update(NavState & nav_state) override;
};

}  // namespace easynav

#endif  // EASYNAV_RECOVERY__DUMMYEVALUATOR_HPP_
