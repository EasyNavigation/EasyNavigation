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
/// \brief Declaration of the FaultyPlanner class.

#ifndef EASYNAV_PLANNER__FAULT_INJECTION__FAULTYPLANNER_HPP_
#define EASYNAV_PLANNER__FAULT_INJECTION__FAULTYPLANNER_HPP_

#include <string>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"

namespace easynav
{

/**
 * @class FaultyPlanner
 * @brief Planner that misbehaves on purpose, to test how EasyNav copes with it.
 *
 * Every update writes "path": a straight line from "robot_pose" to the first goal in "goals"
 * (empty without them), and, after "<name>.fault_after" updates, injects the fault in
 * "<name>.fault":
 * - "none": keeps planning.
 * - "throw": throws from update().
 * - "hang": blocks update() for "<name>.hang_time" seconds on each update.
 * - "empty_path": writes a path with no poses.
 * - "freeze": keeps writing the last path, with its old stamp.
 * - "nan": writes a path of NaN poses.
 */
class FaultyPlanner : public PlannerMethodBase
{
public:
  /// @brief Reads the parameters; throws on an unknown fault.
  void on_initialize() override;

  /// @brief Plans or injects the fault.
  void update(NavState & nav_state) override;

private:
  /// @brief The straight path to the goal, stamped now (empty without pose or goal).
  nav_msgs::msg::Path plan(const NavState & nav_state) const;

  std::string fault_ {"none"};
  int fault_after_ {0};
  double hang_time_ {1.0};

  int updates_ {0};
  nav_msgs::msg::Path path_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__FAULT_INJECTION__FAULTYPLANNER_HPP_
