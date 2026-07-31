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

/// \file
/// \brief Declaration of the DummyMapsManager class, a default map manager plugin for Easy Navigation.

#ifndef EASYNAV_PLANNER__DUMMYMAPMANAGER_HPP_
#define EASYNAV_PLANNER__DUMMYMAPMANAGER_HPP_

#include "easynav_core/MapsManagerBase.hpp"

namespace easynav
{

/**
 * @class DummyMapsManager
 * @brief A default "do-nothing" implementation of MapsManagerBase.
 *
 * Serves as a placeholder or fallback when no real map manager is provided.
 */
class DummyMapsManager : public easynav::MapsManagerBase
{
public:
  /// @brief Default constructor.
  DummyMapsManager() = default;

  /// @brief Default destructor.
  ~DummyMapsManager() = default;

  /**
   * @brief Initialize the plugin.
   */
  virtual void on_initialize() override;

  /**
   * @brief Dummy update method.
   * @param nav_state The current navigation state.
   */
  virtual void update(NavState & nav_state) override;

private:
  double cycle_time_rt_ {0.0};
  double cycle_time_nort_ {0.0};
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__DUMMYMAPMANAGER_HPP_
