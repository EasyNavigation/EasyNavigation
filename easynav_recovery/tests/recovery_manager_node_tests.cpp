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

#include <algorithm>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_recovery/RecoveryManagerNode.hpp"

class RecoveryManagerNodeTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(RecoveryManagerNodeTestCase, node_name)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  EXPECT_STREQ(node->get_name(), "recovery_node");
}

TEST_F(RecoveryManagerNodeTestCase, configure_no_evaluators)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_num_evaluators(), 0u);
}

TEST_F(RecoveryManagerNodeTestCase, configure_loads_multiple_evaluators)
{
  // Unlike ControllerNode/PlannerNode, RecoveryManagerNode does not restrict to one plugin.
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "evaluator_types", std::vector<std::string>{"eval_a", "eval_b"})
    .append_parameter_override(
      "eval_a.plugin", std::string("easynav_recovery/DummyEvaluator"))
    .append_parameter_override(
      "eval_b.plugin", std::string("easynav_recovery/DummyEvaluator")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_num_evaluators(), 2u);
}

TEST_F(RecoveryManagerNodeTestCase, configure_fails_with_nonexistent_plugin)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "evaluator_types", std::vector<std::string>{"eval_a"})
    .append_parameter_override(
      "eval_a.plugin", std::string("easynav_recovery/NoSuchEvaluator")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(RecoveryManagerNodeTestCase, cycle_runs_loaded_evaluators)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "evaluator_types", std::vector<std::string>{"eval_a"})
    .append_parameter_override(
      "eval_a.plugin", std::string("easynav_recovery/DummyEvaluator")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Evaluators default to 10 Hz (MethodBase default); give it time to be due.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  auto nav_state = std::make_shared<easynav::NavState>();
  node->cycle(nav_state);

  EXPECT_TRUE(nav_state->has("diagnostics.eval_a"));
  auto members = nav_state->get_group_keys("diagnostics");
  EXPECT_NE(
    std::find(members.begin(), members.end(), "diagnostics.eval_a"), members.end());
}

TEST_F(RecoveryManagerNodeTestCase, configure_loads_mitigations)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "mitigation_types", std::vector<std::string>{"mit_a"})
    .append_parameter_override(
      "mit_a.plugin", std::string("easynav_recovery/DummyMitigation")));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_num_mitigations(), 1u);
  EXPECT_TRUE(node->get_active_mitigation_name().empty());
}

TEST_F(RecoveryManagerNodeTestCase, cycle_selects_and_resolves_non_control_mitigation)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "mitigation_types", std::vector<std::string>{"mit_a"})
    .append_parameter_override(
      "mit_a.plugin", std::string("easynav_recovery/DummyMitigation")));
  // requires_control defaults to false: this mitigation is cycled from cycle(), not cycle_rt().

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  nav_state->set("diagnostics.fake", status);
  nav_state->set_group("diagnostics", {"diagnostics.fake"});

  // First cycle(): evaluators (none configured), then selection picks mit_a.
  node->cycle(nav_state);
  EXPECT_EQ(node->get_active_mitigation_name(), "mit_a");

  // Second cycle(): the active (non-control) mitigation is cycled; DummyMitigation always
  // reports SUCCEEDED on its first on_cycle(), so it is stopped and cleared immediately.
  node->cycle(nav_state);
  EXPECT_TRUE(node->get_active_mitigation_name().empty());
}

TEST_F(RecoveryManagerNodeTestCase, cycle_rt_drives_control_owning_mitigation_and_resets_owner)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "mitigation_types", std::vector<std::string>{"mit_a"})
    .append_parameter_override(
      "mit_a.plugin", std::string("easynav_recovery/DummyMitigation"))
    .append_parameter_override("mit_a.requires_control", true));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto nav_state = std::make_shared<easynav::NavState>();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  nav_state->set("diagnostics.fake", status);
  nav_state->set_group("diagnostics", {"diagnostics.fake"});

  // cycle() selects it and, because it requires control, sets "control_owner".
  node->cycle(nav_state);
  ASSERT_EQ(node->get_active_mitigation_name(), "mit_a");
  ASSERT_TRUE(nav_state->has("control_owner"));
  EXPECT_EQ(nav_state->get<std::string>("control_owner"), "recovery:mit_a");

  // cycle_rt() drives it (not cycle(), since it requires control); DummyMitigation succeeds
  // immediately, so control_owner is handed back to "controller".
  bool wrote_cmd_vel = node->cycle_rt(nav_state);
  EXPECT_TRUE(wrote_cmd_vel);
  EXPECT_TRUE(node->get_active_mitigation_name().empty());
  EXPECT_EQ(nav_state->get<std::string>("control_owner"), "controller");
}

TEST_F(RecoveryManagerNodeTestCase, cycle_rt_is_noop_without_a_control_owning_mitigation)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_FALSE(node->cycle_rt(nav_state));
}

TEST_F(RecoveryManagerNodeTestCase, DiagnosticStatusIsHumanReadableInDebugString)
{
  // RecoveryManagerNode's constructor registers a NavState printer for
  // diagnostic_msgs::msg::DiagnosticStatus so it shows up readable in debug_string() (and thus
  // in the "easynav_navstate" topic / EasyNav TUI), instead of the generic pointer+typehash
  // fallback.
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  (void)node;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.name = "my_eval";
  status.hardware_id = "planner";
  status.message = "planner produced an empty path";

  easynav::NavState nav_state;
  nav_state.set("diagnostics.my_eval", status);

  std::string s = nav_state.debug_string();
  EXPECT_NE(s.find("ERROR"), std::string::npos) << s;
  EXPECT_NE(s.find("my_eval"), std::string::npos) << s;
  EXPECT_NE(s.find("planner"), std::string::npos) << s;
  EXPECT_NE(s.find("planner produced an empty path"), std::string::npos) << s;
}
