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


namespace
{

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

// Two aliases of the same plugin class: distinguishable through get_loaded_controller().
rclcpp::NodeOptions two_controllers_options()
{
  return rclcpp::NodeOptions()
         .append_parameter_override(
    "controller_types", std::vector<std::string>{"first_ctrl"})
         .append_parameter_override(
    "first_ctrl.plugin", std::string("easynav_controller/DummyController"))
         .append_parameter_override(
    "second_ctrl.plugin", std::string("easynav_controller/DummyController"));
}

}  // namespace

// ---------------------------------------------------------------------------
// 7. Changing controller_types has no effect until the node is configured again.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, controller_change_takes_effect_after_cleanup_and_configure)
{
  auto node = std::make_shared<easynav::ControllerNode>(two_controllers_options());

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_loaded_controller(), "first_ctrl");

  ASSERT_TRUE(
    node->set_parameter(
      rclcpp::Parameter("controller_types", std::vector<std::string>{"second_ctrl"})).successful);
  EXPECT_EQ(node->get_loaded_controller(), "first_ctrl");

  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(node->get_loaded_controller(), "");

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_loaded_controller(), "second_ctrl");

  node->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_TRUE(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 8. The controller can also be changed while unconfigured, and switched back later.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, controller_change_while_unconfigured_and_switch_back)
{
  auto node = std::make_shared<easynav::ControllerNode>(two_controllers_options());

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  ASSERT_TRUE(
    node->set_parameter(
      rclcpp::Parameter("controller_types", std::vector<std::string>{"second_ctrl"})).successful);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_loaded_controller(), "second_ctrl");

  // Back to the first one: its parameters were declared before and must not clash.
  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  ASSERT_TRUE(
    node->set_parameter(
      rclcpp::Parameter("controller_types", std::vector<std::string>{"first_ctrl"})).successful);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_loaded_controller(), "first_ctrl");
}

// ---------------------------------------------------------------------------
// 9. Every valid cycle of transitions can be repeated (parameters are declared again).
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, lifecycle_can_be_repeated)
{
  auto node = std::make_shared<easynav::ControllerNode>(two_controllers_options());
  auto nav_state = std::make_shared<easynav::NavState>();

  for (int i = 0; i < 3; ++i) {
    node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE) << "round " << i;
    node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE) << "round " << i;
    EXPECT_TRUE(node->cycle_rt(nav_state, true));
    node->trigger_transition(Transition::TRANSITION_DEACTIVATE);
    ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE) << "round " << i;
    node->trigger_transition(Transition::TRANSITION_CLEANUP);
    ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED) << "round " << i;
    EXPECT_FALSE(node->cycle_rt(nav_state, true));
  }
}

// ---------------------------------------------------------------------------
// 10. Parameters of a controller that has not been loaded yet.
//
// The parameters of a controller are declared by the plugin itself when it is
// initialized (on configure), so they do not exist while another controller is
// loaded. They can be provided in advance as parameter overrides, and they are
// applied when the plugin declares them after the switch.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, parameters_of_not_yet_loaded_controller_come_from_overrides)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    two_controllers_options().append_parameter_override("second_ctrl.cycle_time_rt", 0.5));

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);

  // Not declared yet: it cannot be set at runtime, but nothing else is affected.
  EXPECT_FALSE(node->has_parameter("second_ctrl.cycle_time_rt"));
  EXPECT_THROW(
    node->set_parameter(rclcpp::Parameter("second_ctrl.cycle_time_rt", 0.9)),
    rclcpp::exceptions::ParameterNotDeclaredException);
  EXPECT_DOUBLE_EQ(node->get_parameter("first_ctrl.cycle_time_rt").as_double(), 0.0);

  ASSERT_TRUE(
    node->set_parameter(
      rclcpp::Parameter("controller_types", std::vector<std::string>{"second_ctrl"})).successful);
  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_loaded_controller(), "second_ctrl");

  // The plugin declared its parameters now, taking the override as initial value.
  ASSERT_TRUE(node->has_parameter("second_ctrl.cycle_time_rt"));
  EXPECT_DOUBLE_EQ(node->get_parameter("second_ctrl.cycle_time_rt").as_double(), 0.5);
}

// ---------------------------------------------------------------------------
// 11. Runtime values of a controller survive a cleanup: its parameters stay declared
//     and the next load of the same controller uses them.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, controller_parameters_keep_runtime_value_after_cleanup)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    two_controllers_options().append_parameter_override("first_ctrl.cycle_time_rt", 0.5));

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(node->has_parameter("first_ctrl.cycle_time_rt"));
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("first_ctrl.cycle_time_rt", 0.9)).successful);

  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(node->has_parameter("first_ctrl.plugin"));

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(node->get_parameter("first_ctrl.cycle_time_rt").as_double(), 0.9);
}

// ---------------------------------------------------------------------------
// 12. A failed configure leaves the node unconfigured and it recovers with a valid controller.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, failed_configure_can_be_retried_with_another_controller)
{
  auto node = std::make_shared<easynav::ControllerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "controller_types", std::vector<std::string>{"broken_ctrl"})
    .append_parameter_override(
      "broken_ctrl.plugin", std::string("easynav_controller/NoSuchController"))
    .append_parameter_override(
      "first_ctrl.plugin", std::string("easynav_controller/DummyController")));

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(node->get_loaded_controller(), "");

  ASSERT_TRUE(
    node->set_parameter(
      rclcpp::Parameter("controller_types", std::vector<std::string>{"first_ctrl"})).successful);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_loaded_controller(), "first_ctrl");
}

// ---------------------------------------------------------------------------
// 13. Shutdown from active releases the controller.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, shutdown_from_active_releases_controller)
{
  auto node = std::make_shared<easynav::ControllerNode>(two_controllers_options());

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  node->trigger_transition(Transition::TRANSITION_ACTIVE_SHUTDOWN);
  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_FINALIZED);
  EXPECT_EQ(node->get_loaded_controller(), "");

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_FALSE(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 14. Shutdown transition from inactive state.
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
// 15. cycle_rt returns false when no plugin is loaded.
// ---------------------------------------------------------------------------

TEST_F(ControllerNodeTestCase, cycle_rt_returns_false_without_plugin)
{
  auto node = std::make_shared<easynav::ControllerNode>();
  auto nav_state = std::make_shared<easynav::NavState>();

  EXPECT_FALSE(node->cycle_rt(nav_state));
  EXPECT_FALSE(node->cycle_rt(nav_state, true));
}

// ---------------------------------------------------------------------------
// 16. cycle_rt with trigger=true executes the plugin without crashing.
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
// 17. cycle_rt without trigger respects timing (returns false immediately).
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
// 18. get_real_time_cbg returns a non-null callback group.
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
