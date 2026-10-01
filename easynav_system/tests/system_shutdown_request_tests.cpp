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
/// \brief A shutdown requested by a recovery mitigation takes SystemNode (and every EasyNav
/// node) out of Active through the lifecycle's error path, ending in Finalized.

#include <string>
#include <vector>

#include "easynav_system/SystemNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class SystemShutdownRequestTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      std::vector<const char *> argv{
        "system_shutdown_request_tests",
        "--ros-args",
        "-p", "controller_types:=['dummy_controller']",
        "-p", "dummy_controller.plugin:=easynav_controller/DummyController",
        "-p", "localizer_types:=['dummy_localizer']",
        "-p", "dummy_localizer.plugin:=easynav_localizer/DummyLocalizer",
        "-p", "planner_types:=['dummy_planner']",
        "-p", "dummy_planner.plugin:=easynav_planner/DummyPlanner",
        "-p", "map_types:=['dummy_map']",
        "-p", "dummy_map.plugin:=easynav_maps_manager/DummyMapsManager",
      };
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }
  }

  static easynav::SystemNode::SharedPtr make_active_system()
  {
    auto system_node = std::make_shared<easynav::SystemNode>();
    system_node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    system_node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    return system_node;
  }
};

TEST_F(SystemShutdownRequestTest, NoRequestByDefault)
{
  auto system_node = make_active_system();
  ASSERT_EQ(system_node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  system_node->system_cycle();
  EXPECT_FALSE(system_node->is_shutdown_requested());

  // A plain deactivation is still a normal one.
  system_node->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  EXPECT_EQ(system_node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SystemShutdownRequestTest, RequestEndsFinalizedThroughErrorProcessing)
{
  auto system_node = make_active_system();
  ASSERT_EQ(system_node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  auto listener_node = rclcpp::Node::make_shared("shutdown_cmd_vel_listener");
  std::vector<geometry_msgs::msg::Twist> received;
  auto sub = listener_node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [&received](geometry_msgs::msg::Twist::UniquePtr msg) {received.push_back(*msg);});
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(listener_node);

  // What a recovery system asks through SystemActions.
  system_node->request_shutdown("diagnostics.graph [ros_graph]: broken");

  system_node->system_cycle();
  ASSERT_TRUE(system_node->is_shutdown_requested());
  EXPECT_EQ(system_node->get_shutdown_reason(), "diagnostics.graph [ros_graph]: broken");

  // Let discovery match the listener with SystemNode's cmd_vel publisher.
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 2s &&
    listener_node->count_publishers("cmd_vel") == 0)
  {
    rclcpp::sleep_for(10ms);
  }

  easynav::SystemNode::CallbackReturnT cb_result;
  const auto & final_state =
    system_node->trigger_transition(Transition::TRANSITION_DEACTIVATE, cb_result);

  EXPECT_EQ(cb_result, easynav::SystemNode::CallbackReturnT::ERROR);
  EXPECT_EQ(final_state.id(), State::PRIMARY_STATE_FINALIZED);
  EXPECT_EQ(system_node->get_current_state().id(), State::PRIMARY_STATE_FINALIZED);

  for (auto & [name, info] : system_node->get_system_nodes()) {
    EXPECT_EQ(info.node_ptr->get_current_state().id(), State::PRIMARY_STATE_FINALIZED) << name;
  }

  // The robot was left with a zero velocity.
  const auto wait_start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - wait_start < 2s && received.empty()) {
    exe.spin_some();
    rclcpp::sleep_for(10ms);
  }
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(received.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(received.back().angular.z, 0.0);
}
