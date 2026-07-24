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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_planner/PlannerNode.hpp"
#include "easynav_common/types/NavState.hpp"

#include "gtest/gtest.h"

class PlannerNodeTestCase : public ::testing::Test
{
protected:
  ~PlannerNodeTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
};

// ---------------------------------------------------------------------------
// 1. Constructor produces a node with the expected name.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, node_name)
{
  auto node = std::make_shared<easynav::PlannerNode>();
  EXPECT_EQ(std::string(node->get_name()), "planner_node");
}

// ---------------------------------------------------------------------------
// 2. Empty planner_types list → configure succeeds.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, configure_no_plugins)
{
  auto node = std::make_shared<easynav::PlannerNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 3. More than one planner type → configure fails.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, configure_fails_when_more_than_one_plugin_type)
{
  auto node = std::make_shared<easynav::PlannerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "planner_types", std::vector<std::string>{"p1", "p2"}));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 4. Loading the built-in DummyPlanner plugin succeeds.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, configure_succeeds_with_dummy_plugin)
{
  auto node = std::make_shared<easynav::PlannerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "planner_types", std::vector<std::string>{"my_planner"})
    .append_parameter_override(
      "my_planner.plugin", std::string("easynav_planner/DummyPlanner")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 5. Non-existent plugin class → configure fails.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, configure_fails_with_nonexistent_plugin)
{
  auto node = std::make_shared<easynav::PlannerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "planner_types", std::vector<std::string>{"my_planner"})
    .append_parameter_override(
      "my_planner.plugin", std::string("easynav_planner/NoSuchPlanner")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 6. Full lifecycle: configure → activate → deactivate → cleanup.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, complete_lifecycle_configure_activate_deactivate_cleanup)
{
  auto node = std::make_shared<easynav::PlannerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "planner_types", std::vector<std::string>{"my_planner"})
    .append_parameter_override(
      "my_planner.plugin", std::string("easynav_planner/DummyPlanner")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

// ---------------------------------------------------------------------------
// 7. Shutdown transition from inactive state.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, lifecycle_shutdown_from_inactive)
{
  auto node = std::make_shared<easynav::PlannerNode>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

// ---------------------------------------------------------------------------
// 8. cycle() with no plugin loaded does not crash.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, cycle_with_no_plugin)
{
  auto node = std::make_shared<easynav::PlannerNode>();
  auto nav_state = std::make_shared<easynav::NavState>();

  EXPECT_NO_THROW(node->cycle(nav_state));
  EXPECT_NO_THROW(node->cycle(nav_state, true));
}

// ---------------------------------------------------------------------------
// 9. cycle() with loaded plugin runs without crashing.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, cycle_with_plugin_does_not_crash)
{
  auto node = std::make_shared<easynav::PlannerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "planner_types", std::vector<std::string>{"my_planner"})
    .append_parameter_override(
      "my_planner.plugin", std::string("easynav_planner/DummyPlanner")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_NO_THROW(node->cycle(nav_state));
  EXPECT_NO_THROW(node->cycle(nav_state, true));
}

// ---------------------------------------------------------------------------
// 10. get_last_rt/non-rt timestamps return default Time when no plugin loaded.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, get_last_timestamps_without_plugin)
{
  auto node = std::make_shared<easynav::PlannerNode>();

  EXPECT_NO_THROW(node->get_last_rt_execution_ts());
  EXPECT_NO_THROW(node->get_last_execution_ts());
}

// ---------------------------------------------------------------------------
// 11. get_last timestamps are accessible after loading a plugin.
// ---------------------------------------------------------------------------

TEST_F(PlannerNodeTestCase, get_last_timestamps_with_plugin)
{
  auto node = std::make_shared<easynav::PlannerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "planner_types", std::vector<std::string>{"my_planner"})
    .append_parameter_override(
      "my_planner.plugin", std::string("easynav_planner/DummyPlanner")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  EXPECT_NO_THROW(node->get_last_rt_execution_ts());
  EXPECT_NO_THROW(node->get_last_execution_ts());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
