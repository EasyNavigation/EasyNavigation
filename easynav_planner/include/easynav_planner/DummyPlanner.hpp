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
/// \brief Declaration of the DummyPlanner class, a default fallback planner plugin.

#ifndef EASYNAV_PLANNER__DUMMYPLANNER_HPP_
#define EASYNAV_PLANNER__DUMMYPLANNER_HPP_

#include "nav_msgs/msg/path.hpp"
#include "easynav_core/PlannerMethodBase.hpp"

namespace easynav
{

/**
 * @class DummyPlanner
 * @brief A default "do-nothing" implementation of PlannerMethodBase.
 *
 * Used as a fallback when no real planner is configured.
 */
class DummyPlanner : public easynav::PlannerMethodBase
{
public:
  /// @brief Default constructor.
  DummyPlanner() = default;

  /// @brief Destructor.
  ~DummyPlanner() = default;

  /**
   * @brief Initialization hook.
   */
  virtual void on_initialize() override;

  /**
   * @brief Dummy update method.
   * @param nav_state Current navigation state.
   */
  virtual void update(NavState & nav_state) override;

private:
  /// @brief Stored path message (unused in dummy).
  nav_msgs::msg::Path path_;

  double cycle_time_rt_ {0.0};
  double cycle_time_nort_ {0.0};
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__DUMMYPLANNER_HPP_
