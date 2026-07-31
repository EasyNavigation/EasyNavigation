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

#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_common/types/NavState.hpp"

#include "gtest/gtest.h"

class MapsManagerNodeTestCase : public ::testing::Test
{
protected:
  ~MapsManagerNodeTestCase()
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

TEST_F(MapsManagerNodeTestCase, node_name)
{
  auto node = std::make_shared<easynav::MapsManagerNode>();
  EXPECT_EQ(std::string(node->get_name()), "maps_manager_node");
}

// ---------------------------------------------------------------------------
// 2. Empty map_types list → configure succeeds (no maps is valid).
// ---------------------------------------------------------------------------

TEST_F(MapsManagerNodeTestCase, configure_no_plugins)
{
  auto node = std::make_shared<easynav::MapsManagerNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 3. Loading a single DummyMapsManager plugin succeeds.
// ---------------------------------------------------------------------------

TEST_F(MapsManagerNodeTestCase, configure_succeeds_with_one_plugin)
{
  auto node = std::make_shared<easynav::MapsManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "map_types", std::vector<std::string>{"my_map"})
    .append_parameter_override(
      "my_map.plugin", std::string("easynav_maps_manager/DummyMapsManager")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 4. MapsManagerNode allows more than one plugin (unlike controller/planner).
// ---------------------------------------------------------------------------

TEST_F(MapsManagerNodeTestCase, configure_succeeds_with_two_plugins)
{
  auto node = std::make_shared<easynav::MapsManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "map_types", std::vector<std::string>{"map_a", "map_b"})
    .append_parameter_override(
      "map_a.plugin", std::string("easynav_maps_manager/DummyMapsManager"))
    .append_parameter_override(
      "map_b.plugin", std::string("easynav_maps_manager/DummyMapsManager")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 5. Non-existent plugin class → configure fails.
// ---------------------------------------------------------------------------

TEST_F(MapsManagerNodeTestCase, configure_fails_with_nonexistent_plugin)
{
  auto node = std::make_shared<easynav::MapsManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "map_types", std::vector<std::string>{"my_map"})
    .append_parameter_override(
      "my_map.plugin", std::string("easynav_maps_manager/NoSuchMapsManager")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 6. Full lifecycle: configure → activate → deactivate → cleanup.
// ---------------------------------------------------------------------------

TEST_F(MapsManagerNodeTestCase, complete_lifecycle_configure_activate_deactivate_cleanup)
{
  auto node = std::make_shared<easynav::MapsManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "map_types", std::vector<std::string>{"my_map"})
    .append_parameter_override(
      "my_map.plugin", std::string("easynav_maps_manager/DummyMapsManager")));

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

TEST_F(MapsManagerNodeTestCase, lifecycle_shutdown_from_inactive)
{
  auto node = std::make_shared<easynav::MapsManagerNode>();

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
// 8. cycle() with no managers loaded does not crash.
// ---------------------------------------------------------------------------

TEST_F(MapsManagerNodeTestCase, cycle_with_no_managers)
{
  auto node = std::make_shared<easynav::MapsManagerNode>();
  auto nav_state = std::make_shared<easynav::NavState>();

  EXPECT_NO_THROW(node->cycle(nav_state));
}

// ---------------------------------------------------------------------------
// 9. cycle() with loaded managers runs without crashing.
// ---------------------------------------------------------------------------

TEST_F(MapsManagerNodeTestCase, cycle_with_managers_does_not_crash)
{
  auto node = std::make_shared<easynav::MapsManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "map_types", std::vector<std::string>{"my_map"})
    .append_parameter_override(
      "my_map.plugin", std::string("easynav_maps_manager/DummyMapsManager")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_NO_THROW(node->cycle(nav_state));
}

// ---------------------------------------------------------------------------
// 10. cycle() with two managers iterates over all of them.
// ---------------------------------------------------------------------------

TEST_F(MapsManagerNodeTestCase, cycle_with_multiple_managers_does_not_crash)
{
  auto node = std::make_shared<easynav::MapsManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "map_types", std::vector<std::string>{"map_a", "map_b"})
    .append_parameter_override(
      "map_a.plugin", std::string("easynav_maps_manager/DummyMapsManager"))
    .append_parameter_override(
      "map_b.plugin", std::string("easynav_maps_manager/DummyMapsManager")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_NO_THROW(node->cycle(nav_state));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
