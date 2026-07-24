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

#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_system/GoalManager.hpp"
#include "easynav_common/types/NavState.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  easynav::NavState nav_state;

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("goalmanager_test_node");
  auto manaer = easynav::GoalManager::make_shared(nav_state, node);


  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Unable to configure GoalManager");
    rclcpp::shutdown();
    return 1;
  }
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Unable to activate GoalManager");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
