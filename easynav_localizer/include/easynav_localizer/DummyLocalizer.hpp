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
/// \brief Declaration of the DummyLocalizer class, a default plugin implementation for localization.

#ifndef EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_
#define EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace easynav
{

/**
 * @class DummyLocalizer
 * @brief A default "do-nothing" implementation of LocalizerMethodBase.
 *
 * This class complies with the localization interface but does not perform real computation.
 * It is intended as a placeholder, fallback, or example plugin.
 */
class DummyLocalizer : public easynav::LocalizerMethodBase
{
public:
  /// @brief Default constructor.
  DummyLocalizer() = default;

  /// @brief Default destructor.
  ~DummyLocalizer() = default;

  /**
   * @brief Plugin-specific initialization logic.
   */
  virtual void on_initialize() override;

  /**
   * @brief Update the localization using the current navigation state.
   *
   * This dummy version performs no actual computation.
   *
   * @param nav_state The current navigation state.
   */
  virtual void update_rt(NavState & nav_state) override;

  /**
   * @brief Update the localization using the current navigation state.
   *
   * This dummy version performs no actual computation.
   *
   * @param nav_state The current navigation state.
   */
  virtual void update(NavState & nav_state) override;

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double cycle_time_rt_ {0.0};
  double cycle_time_nort_ {0.0};
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_
