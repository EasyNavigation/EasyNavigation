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
/// \brief Declaration of the FaultyMapsManager class.

#ifndef EASYNAV_MAPS_MANAGER__FAULT_INJECTION__FAULTYMAPSMANAGER_HPP_
#define EASYNAV_MAPS_MANAGER__FAULT_INJECTION__FAULTYMAPSMANAGER_HPP_

#include <string>

#include "easynav_core/MapsManagerBase.hpp"

namespace easynav
{

/**
 * @class FaultyMapsManager
 * @brief Maps manager that misbehaves on purpose, to test how EasyNav copes with it.
 *
 * Manages no map (like DummyMapsManager) and, after "<name>.fault_after" updates, injects the
 * fault in "<name>.fault":
 * - "none": keeps working.
 * - "throw": throws from update().
 * - "hang": blocks update() for "<name>.hang_time" seconds on each update.
 */
class FaultyMapsManager : public MapsManagerBase
{
public:
  /// @brief Reads the parameters; throws on an unknown fault.
  void on_initialize() override;

  /// @brief Does nothing, or injects the fault.
  void update(NavState & nav_state) override;

  /// @brief Updates run so far (to check that it keeps being called).
  [[nodiscard]] int updates() const {return updates_;}

private:
  std::string fault_ {"none"};
  int fault_after_ {0};
  double hang_time_ {1.0};

  int updates_ {0};
};

}  // namespace easynav

#endif  // EASYNAV_MAPS_MANAGER__FAULT_INJECTION__FAULTYMAPSMANAGER_HPP_
