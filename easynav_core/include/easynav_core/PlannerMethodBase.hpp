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
/// \brief Declaration of the abstract base class PlannerMethodBase.

#ifndef EASYNAV_CORE__PLANNERMETHODBASE_HPP_
#define EASYNAV_CORE__PLANNERMETHODBASE_HPP_

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class PlannerMethodBase
 * @brief Abstract base class for path planning methods in Easy Navigation.
 *
 * This class defines the interface for implementing planning algorithms.
 * Derived classes must implement the update and get_path methods.
 */
class PlannerMethodBase : public MethodBase
{
public:
  /// @brief Default constructor.
  PlannerMethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~PlannerMethodBase() = default;

  /**
   * @brief Helper to run the planner update if it is time.
   *
   * @param nav_state The current state of the navigation system.
   */
  void internal_update(NavState & nav_state);

  /**
   * @brief Helper to run the planner update independently if it is time.
   *
   * @param nav_state The current state of the navigation system.
   */
  void force_update(NavState & nav_state);

protected:
  /**
   * @brief Run the path planning algorithm and update the route.
   *
   * Called periodically by the system to compute or refine a navigation path.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__PLANNERMETHODBASE_HPP_
