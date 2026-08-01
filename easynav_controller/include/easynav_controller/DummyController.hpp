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
/// \brief Declaration of the DummyController method.

#ifndef EASYNAV_CONTROLLER__DUMMYCONTROLLER_HPP_
#define EASYNAV_CONTROLLER__DUMMYCONTROLLER_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_core/ControllerMethodBase.hpp"

namespace easynav
{

/**
 * @class DummyController
 * @brief A default "dummy" implementation for the Control Method.
 *
 * This control method does nothing. It serves as an example, and will be used as a default plugin implementation
 * if the navigation system configuration does not specify one.
 */
class DummyController : public easynav::ControllerMethodBase
{
public:
  DummyController() = default;
  ~DummyController() = default;

  /**
   * @brief Initializes the control method plugin.
   *
   * This method is called once during the configuration phase of the controller node,
   * and can be optionally overridden by derived classes to perform custom setup logic.
   */
  virtual void on_initialize() override;

  /**
   * @brief Run the control method and update the control command.
   *
   * This method will be called by the system's ControllerNode to run the control algorithm.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update_rt(NavState & nav_state) override;

private:
  /**
   * @brief Current robot velocity command.
   */
  geometry_msgs::msg::TwistStamped cmd_vel_;

  double cycle_time_rt_ {0.0};
  double cycle_time_nort_ {0.0};
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__DUMMYCONTROLLER_HPP_
