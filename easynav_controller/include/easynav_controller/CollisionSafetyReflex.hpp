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
/// \brief Declaration of the CollisionSafetyReflex plugin.

#ifndef EASYNAV_CONTROLLER__COLLISIONSAFETYREFLEX_HPP_
#define EASYNAV_CONTROLLER__COLLISIONSAFETYREFLEX_HPP_

#include "easynav_core/CollisionChecker.hpp"
#include "easynav_core/SafetyReflexBase.hpp"

namespace easynav
{

/**
 * @class CollisionSafetyReflex
 * @brief Reference level-0 safety reflex: stops the robot on imminent collision.
 *
 * Loaded and run directly by SystemNode on every RT cycle, right before "cmd_vel" is
 * published, regardless of whether it was produced by the active controller or by a movement
 * recovery mitigator. See docs/recoveries_easynav.md, level 0.
 *
 * This is the same forward-projection check historically built into ControllerMethodBase
 * (still available there, opt-in, for backward compatibility); both share the
 * CollisionChecker implementation so the safety-critical geometry exists only once.
 */
class CollisionSafetyReflex : public SafetyReflexBase
{
public:
  CollisionSafetyReflex() = default;
  ~CollisionSafetyReflex() = default;

  void on_initialize() override;

protected:
  bool check(NavState & nav_state) override;
  void mitigate(NavState & nav_state) override;

private:
  CollisionChecker collision_checker_;
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__COLLISIONSAFETYREFLEX_HPP_
