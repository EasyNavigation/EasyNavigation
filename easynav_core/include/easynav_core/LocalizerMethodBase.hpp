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

/// \file
/// \brief Declaration of the abstract base class LocalizerMethodBase.

#ifndef EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_
#define EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_

#include <atomic>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

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

  /**
   * @brief Called once, before this instance's first cycle, with the valid robot pose (map
   * frame, finite) that a previous localizer left in NavState, e.g. after a reconfiguration.
   *
   * Contract between localizers, so any can replace any other (e.g. AMCL <-> Fusion): each one
   * writes its estimate to "robot_pose" (nav_msgs/Odometry, map frame, with covariance), and
   * the next one starts from it here. Default: ignore it.
   */
  virtual void on_last_known_pose(
    [[maybe_unused]] const geometry_msgs::msg::PoseWithCovarianceStamped & pose) {}

private:
  /// @brief Hands the last known pose to on_last_known_pose(), on the first cycle only.
  void check_last_known_pose(const NavState & nav_state);

  std::atomic<bool> last_known_pose_checked_ {false};
};

}  // namespace easynav

#endif  // EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_
