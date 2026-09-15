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
/// \brief Declaration of the DummyMitigation plugin.

#ifndef EASYNAV_RECOVERY__DUMMYMITIGATION_HPP_
#define EASYNAV_RECOVERY__DUMMYMITIGATION_HPP_

#include "easynav_core/RecoveryMitigationBase.hpp"

namespace easynav
{

/**
 * @class DummyMitigation
 * @brief A default "dummy" implementation for RecoveryMitigationBase.
 *
 * Accepts any non-OK diagnostic and immediately reports SUCCEEDED (or FAILED, if configured
 * to), without touching the robot unless configured to require control. It serves as an
 * example, a reference no-op plugin, and a real, always-loadable mitigation for tests that need
 * one without pulling in a specific mitigation's dependencies.
 */
class DummyMitigation : public easynav::RecoveryMitigationBase
{
public:
  DummyMitigation() = default;
  ~DummyMitigation() = default;

  void on_initialize() override;

  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus & status) const override;
  bool requires_control() const override {return requires_control_;}

protected:
  RecoveryStatus on_cycle(NavState & nav_state) override;

private:
  bool requires_control_ {false};

  /// @brief If true, on_cycle() reports FAILED instead of SUCCEEDED. For tests exercising
  /// RecoveryManagerNode's exclusion-on-FAILED behavior.
  bool should_fail_ {false};
};

}  // namespace easynav

#endif  // EASYNAV_RECOVERY__DUMMYMITIGATION_HPP_
