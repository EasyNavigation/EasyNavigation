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

#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_common/types/NavState.hpp"

#include "gtest/gtest.h"

class LocalizerNodeTestCase : public ::testing::Test
{
protected:
  ~LocalizerNodeTestCase()
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

TEST_F(LocalizerNodeTestCase, node_name)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  EXPECT_EQ(std::string(node->get_name()), "localizer_node");
}

// ---------------------------------------------------------------------------
// 2. Empty localizer_types list → configure succeeds.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, configure_no_plugins)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 3. More than one localizer type → configure fails.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, configure_fails_when_more_than_one_plugin_type)
{
  auto node = std::make_shared<easynav::LocalizerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "localizer_types", std::vector<std::string>{"loc1", "loc2"}));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 4. Loading the built-in DummyLocalizer plugin succeeds.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, configure_succeeds_with_dummy_plugin)
{
  auto node = std::make_shared<easynav::LocalizerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "localizer_types", std::vector<std::string>{"my_localizer"})
    .append_parameter_override(
      "my_localizer.plugin", std::string("easynav_localizer/DummyLocalizer")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 5. Non-existent plugin class → configure fails.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, configure_fails_with_nonexistent_plugin)
{
  auto node = std::make_shared<easynav::LocalizerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "localizer_types", std::vector<std::string>{"my_localizer"})
    .append_parameter_override(
      "my_localizer.plugin", std::string("easynav_localizer/NoSuchLocalizer")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 6. Full lifecycle: configure → activate → deactivate → cleanup.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, complete_lifecycle_configure_activate_deactivate_cleanup)
{
  auto node = std::make_shared<easynav::LocalizerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "localizer_types", std::vector<std::string>{"my_localizer"})
    .append_parameter_override(
      "my_localizer.plugin", std::string("easynav_localizer/DummyLocalizer")));

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

TEST_F(LocalizerNodeTestCase, lifecycle_shutdown_from_inactive)
{
  auto node = std::make_shared<easynav::LocalizerNode>();

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
// 8. cycle_rt returns false when no plugin is loaded.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, cycle_rt_returns_false_without_plugin)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  auto nav_state = std::make_shared<easynav::NavState>();

  EXPECT_FALSE(node->cycle_rt(nav_state));
  EXPECT_FALSE(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 9. cycle_rt with trigger=true executes the plugin without crashing.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, cycle_rt_with_trigger_executes)
{
  auto node = std::make_shared<easynav::LocalizerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "localizer_types", std::vector<std::string>{"my_localizer"})
    .append_parameter_override(
      "my_localizer.plugin", std::string("easynav_localizer/DummyLocalizer")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_NO_THROW(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 10. cycle() with no plugin loaded does not crash.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, cycle_without_plugin_does_not_crash)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  auto nav_state = std::make_shared<easynav::NavState>();

  EXPECT_NO_THROW(node->cycle(nav_state));
}

// ---------------------------------------------------------------------------
// 11. cycle() with loaded plugin runs without crashing.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, cycle_with_plugin_does_not_crash)
{
  auto node = std::make_shared<easynav::LocalizerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "localizer_types", std::vector<std::string>{"my_localizer"})
    .append_parameter_override(
      "my_localizer.plugin", std::string("easynav_localizer/DummyLocalizer")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_NO_THROW(node->cycle(nav_state));
  EXPECT_NO_THROW(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 12. get_real_time_cbg returns a non-null callback group.
// ---------------------------------------------------------------------------

TEST_F(LocalizerNodeTestCase, get_real_time_cbg_returns_valid)
{
  auto node = std::make_shared<easynav::LocalizerNode>();
  EXPECT_NE(node->get_real_time_cbg(), nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
