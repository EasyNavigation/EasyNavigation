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
/// \brief Declaration of the abstract base class LocalizerMethodBase.

#ifndef EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_
#define EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class LocalizerMethodBase
 * @brief Abstract base class for localization methods in Easy Navigation.
 *
 * This class defines the interface for localization algorithm implementations.
 * Derived classes must implement the update methods.
 */
class LocalizerMethodBase : public MethodBase
{
public:
  /// @brief Default constructor.
  LocalizerMethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~LocalizerMethodBase() = default;

  /**
   * @brief Helper to run the real-time update if appropriate.
   *
   * @param nav_state The current state of the navigation system.
   * @param trigger Force execution regardless of timing.
   * @return True if update_rt() was called, false otherwise.
   */
  bool internal_update_rt(NavState & nav_state, bool trigger = false);

  /**
   * @brief Helper to run the non-real-time update if appropriate.
   *
   * @param nav_state The current state of the navigation system.
   */
  void internal_update(NavState & nav_state);

protected:
  /**
   * @brief Run the real-time localization update.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update_rt(NavState & nav_state) = 0;

  /**
   * @brief Run the non-real-time localization update.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_
