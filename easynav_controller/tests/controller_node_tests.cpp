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

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_common/types/NavState.hpp"

#include "gtest/gtest.h"

class ControllerNodeTestCase : public ::testing::Test
{
protected:
  ~ControllerNodeTestCase()
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

TEST_F(ControllerNodeTestCase, node_name)
{
  auto node = std::make_shared<easynav::ControllerNode>();
  EXPECT_EQ(std::string(node->get_name()), "controller_node");
}

// ---------------------------------------------------------------------------
// 2. With an empty controller_types list configure succeeds.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, configure_no_plugins)
{
  auto node = std::make_shared<easynav::ControllerNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 3. Specifying more than one controller type must cause configure to fail.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, configure_fails_when_more_than_one_plugin_type)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"ctrl1", "ctrl2"}));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 4. Loading the built-in DummyController plugin succeeds.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, configure_succeeds_with_dummy_plugin)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"my_ctrl"})
    .append_parameter_override(
      "my_ctrl.plugin", std::string("easynav_controller/DummyController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 5. A non-existent plugin class name makes configure fail (pluginlib error).
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, configure_fails_with_nonexistent_plugin)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"my_ctrl"})
    .append_parameter_override(
      "my_ctrl.plugin", std::string("easynav_controller/NoSuchController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 6. Full lifecycle: configure → activate → deactivate → cleanup.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, complete_lifecycle_configure_activate_deactivate_cleanup)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"my_ctrl"})
    .append_parameter_override(
      "my_ctrl.plugin", std::string("easynav_controller/DummyController")));

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
// 7. Cleanup clears a pending runtime controller change.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, cleanup_clears_pending_controller_change)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"my_ctrl"})
    .append_parameter_override(
      "my_ctrl.plugin", std::string("easynav_controller/DummyController"))
    .append_parameter_override(
      "other_ctrl.plugin", std::string("easynav_controller/DummyController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto result = node->set_parameter(
    rclcpp::Parameter(
      "controller_types", std::vector<std::string>{"other_ctrl"}));
  ASSERT_TRUE(result.successful);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}
// ---------------------------------------------------------------------------
// 8. Shutdown transition from inactive state.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, controller_change_is_accepted_while_inactive)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"first_ctrl"})
    .append_parameter_override(
      "first_ctrl.plugin", std::string("easynav_controller/DummyController"))
    .append_parameter_override(
      "second_ctrl.plugin", std::string("easynav_controller/DummyController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto result = node->set_parameter(
    rclcpp::Parameter("controller_types", std::vector<std::string>{"second_ctrl"}));

  EXPECT_TRUE(result.successful);
}

// ---------------------------------------------------------------------------
// 9. Runtime controller changes are rejected while active.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, controller_change_is_rejected_while_active)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"first_ctrl"})
    .append_parameter_override(
      "first_ctrl.plugin", std::string("easynav_controller/DummyController"))
    .append_parameter_override(
      "second_ctrl.plugin", std::string("easynav_controller/DummyController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  const auto result = node->set_parameter(
    rclcpp::Parameter("controller_types", std::vector<std::string>{"second_ctrl"}));

  EXPECT_FALSE(result.successful);
  EXPECT_EQ(
    node->get_parameter("controller_types").as_string_array(),
    std::vector<std::string>{"first_ctrl"});
}

// ---------------------------------------------------------------------------
// 10. An inactive controller change is applied during activation.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, controller_change_is_applied_on_activation)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"first_ctrl"})
    .append_parameter_override(
      "first_ctrl.plugin", std::string("easynav_controller/DummyController"))
    .append_parameter_override(
      "second_ctrl.plugin", std::string("easynav_controller/DummyController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto result = node->set_parameter(
    rclcpp::Parameter("controller_types", std::vector<std::string>{"second_ctrl"}));
  ASSERT_TRUE(result.successful);

  const auto state = node->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_TRUE(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 11. A failed controller change preserves the previous controller.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, failed_controller_change_preserves_previous_controller)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"first_ctrl"})
    .append_parameter_override(
      "first_ctrl.plugin", std::string("easynav_controller/DummyController"))
    .append_parameter_override(
      "broken_ctrl.plugin", std::string("easynav_controller/NoSuchController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto result = node->set_parameter(
    rclcpp::Parameter("controller_types", std::vector<std::string>{"broken_ctrl"}));
  ASSERT_TRUE(result.successful);

  const auto state = node->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_TRUE(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 12. Shutdown transition from inactive state.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, lifecycle_shutdown_from_inactive)
{
  auto node = std::make_shared<easynav::ControllerNode>();

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
// 13. cycle_rt returns false when no plugin is loaded.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, cycle_rt_returns_false_without_plugin)
{
  auto node = std::make_shared<easynav::ControllerNode>();
  auto nav_state = std::make_shared<easynav::NavState>();

  EXPECT_FALSE(node->cycle_rt(nav_state));
  EXPECT_FALSE(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 14. cycle_rt with trigger=true executes the plugin without crashing.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, cycle_rt_with_trigger_executes)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"my_ctrl"})
    .append_parameter_override(
      "my_ctrl.plugin", std::string("easynav_controller/DummyController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_NO_THROW(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 15. cycle_rt without trigger respects timing (returns false immediately).
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, cycle_rt_without_trigger_respects_rate)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"my_ctrl"})
    .append_parameter_override(
      "my_ctrl.plugin", std::string("easynav_controller/DummyController")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  // Rate-limited: should return false immediately after initialization
  EXPECT_FALSE(node->cycle_rt(nav_state, false));
}

// ---------------------------------------------------------------------------
// 16. get_real_time_cbg returns a non-null callback group.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, get_real_time_cbg_returns_valid)
{
  auto node = std::make_shared<easynav::ControllerNode>();
  EXPECT_NE(node->get_real_time_cbg(), nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
