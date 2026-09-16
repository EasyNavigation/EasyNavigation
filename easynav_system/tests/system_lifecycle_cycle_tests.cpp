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
/// \brief Exercises SystemNode's own lifecycle state machine through
/// repeated full cycles, including active -> inactive -> unconfigured ->
/// inactive -> active.

#include <vector>

#include "easynav_system/SystemNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

class SystemLifecycleCycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!initialized_) {
      std::vector<const char *> argv{
        "system_lifecycle_cycle_tests",
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
      initialized_ = true;
    }
  }

  void TearDown() override
  {
  }

  static bool initialized_;
};

bool SystemLifecycleCycleTest::initialized_ = false;

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{

// Returns false (instead of asserting directly) so a failed transition aborts
// the calling TEST_F via ASSERT_TRUE at the call site, rather than letting the
// test continue with the node left in an unexpected lifecycle state.
[[nodiscard]] bool expect_transition(
  const easynav::SystemNode::SharedPtr & node, uint8_t transition_id, uint8_t expected_state_id)
{
  easynav::SystemNode::CallbackReturnT cb_result;
  const auto & new_state = node->trigger_transition(transition_id, cb_result);

  EXPECT_EQ(cb_result, easynav::SystemNode::CallbackReturnT::SUCCESS) <<
    "transition " << static_cast<int>(transition_id) << " callback failed";
  EXPECT_EQ(new_state.id(), expected_state_id) <<
    "transition " << static_cast<int>(transition_id) << " left node in unexpected state";

  return cb_result == easynav::SystemNode::CallbackReturnT::SUCCESS &&
         new_state.id() == expected_state_id;
}

}  // namespace

TEST_F(SystemLifecycleCycleTest, ActiveToUnconfiguredAndBackRepeatedly)
{
  auto system_node = std::make_shared<easynav::SystemNode>();

  ASSERT_TRUE(expect_transition(
    system_node, Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE));
  ASSERT_TRUE(expect_transition(
    system_node, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));

  auto listener_node = rclcpp::Node::make_shared("cmd_vel_cycle_listener");
  std::vector<geometry_msgs::msg::Twist> received;
  auto sub = listener_node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [&received](geometry_msgs::msg::Twist::UniquePtr msg) {
      received.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(listener_node);

  constexpr int kCycles = 4;
  for (int cycle = 0; cycle < kCycles; ++cycle) {
    SCOPED_TRACE(::testing::Message() << "cycle " << cycle);

    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE));
    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_UNCONFIGURED));

    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE));
    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));

    auto nav_state = system_node->get_nav_state();
    ASSERT_TRUE(nav_state->has("navigation_paused"));
    ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));

    geometry_msgs::msg::TwistStamped nonzero_cmd;
    nonzero_cmd.twist.linear.x = 1.5 + cycle;
    nonzero_cmd.twist.angular.z = 0.5;
    nav_state->set("cmd_vel", nonzero_cmd);

    received.clear();
    auto start = listener_node->now();
    while (listener_node->now() - start < 2s && received.empty()) {
      system_node->system_cycle();
      system_node->system_cycle_rt();
      exe.spin_some();
      rclcpp::sleep_for(10ms);
    }

    ASSERT_FALSE(received.empty()) << "no cmd_vel received after reconfiguring (cycle " <<
      cycle << ")";
    EXPECT_DOUBLE_EQ(received.back().linear.x, 1.5 + cycle);
    EXPECT_DOUBLE_EQ(received.back().angular.z, 0.5);
  }
}

TEST_F(SystemLifecycleCycleTest, AllPrimaryTransitionsRepeatedly)
{
  auto system_node = std::make_shared<easynav::SystemNode>();

  for (int cycle = 0; cycle < 3; ++cycle) {
    SCOPED_TRACE(::testing::Message() << "cycle " << cycle);

    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE));
    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));

    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE));
    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));
    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE));

    ASSERT_TRUE(expect_transition(
      system_node, Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_UNCONFIGURED));
  }

  ASSERT_TRUE(expect_transition(
    system_node, Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE));
  ASSERT_TRUE(expect_transition(
    system_node, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));

  ASSERT_NO_THROW(system_node->system_cycle());
  ASSERT_NO_THROW(system_node->system_cycle_rt());
}
