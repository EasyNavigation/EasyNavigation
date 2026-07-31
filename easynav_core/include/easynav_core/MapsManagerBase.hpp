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
/// \brief Declaration of the abstract base class MapsManagerBase.

#ifndef EASYNAV_CORE__MAPSMANAGERBASE_HPP_
#define EASYNAV_CORE__MAPSMANAGERBASE_HPP_

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class MapsManagerBase
 * @brief Abstract base class for map management in Easy Navigation.
 *
 * This class defines the interface for components responsible for generating or maintaining maps.
 * Derived classes must implement the update and get_maps methods.
 */
class MapsManagerBase : public MethodBase
{
public:
  /// @brief Default constructor.
  MapsManagerBase() = default;

  /// @brief Virtual destructor.
  virtual ~MapsManagerBase() = default;

  /**
   * @brief Helper to run the update method if it is time to do so.
   *
   * @param nav_state The current state of the navigation system.
   */
  void internal_update(NavState & nav_state);

protected:
  /**
   * @brief Run the map update logic.
   *
   * Called periodically by the system to update map data using the current navigation state.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__MAPSMANAGERBASE_HPP_
