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
/// \brief Tests for what RecoveryManagerNode owns besides evaluators and mitigations: the
/// level-0 safety reflexes, who has control of the robot, and the shutdown request.

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_core/VelocityCommand.hpp"
#include "easynav_recovery/RecoveryManagerNode.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class RecoveryManagerReflexTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(RecoveryManagerReflexTest, ConfigureSucceedsWithNoReflexes)
{
  // safety_reflex_types defaults to an empty list.
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_num_safety_reflexes(), 0u);
}

TEST_F(RecoveryManagerReflexTest, ConfigureLoadsSafetyReflex)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override("safety_reflex_types", std::vector<std::string>{"reflex"})
    .append_parameter_override(
      "reflex.plugin", std::string("easynav_recovery/DummySafetyReflex")));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_num_safety_reflexes(), 1u);

  // The reflex does not intervene, so recovery commands nothing.
  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_FALSE(node->cycle_rt(nav_state));
}

TEST_F(RecoveryManagerReflexTest, ConfigureFailsWithUnknownReflexPlugin)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override("safety_reflex_types", std::vector<std::string>{"bogus"})
    .append_parameter_override("bogus.plugin", std::string("no_such_pkg/NoSuchReflex")));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(RecoveryManagerReflexTest, ReflexesSurviveReconfiguration)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override("safety_reflex_types", std::vector<std::string>{"reflex"})
    .append_parameter_override(
      "reflex.plugin", std::string("easynav_recovery/DummySafetyReflex")));

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(node->get_num_safety_reflexes(), 0u);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_num_safety_reflexes(), 1u);
}

TEST_F(RecoveryManagerReflexTest, CleanupGivesControlBackAndDropsDiagnostics)
{
  // A cleanup drops the mitigation that held control and the evaluators that wrote the
  // diagnostics: the next cycle must not keep routing control to a mitigation that is gone.
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  easynav::NavState nav_state;
  nav_state.set("control_owner", std::string("recovery:retreat"));
  nav_state.set_group("diagnostics", std::vector<std::string>{"diagnostics.stale"});

  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  node->cycle(std::shared_ptr<easynav::NavState>(&nav_state, [](easynav::NavState *) {}));
  EXPECT_EQ(nav_state.get<std::string>("control_owner"), "controller");
  EXPECT_TRUE(nav_state.get_group_keys("diagnostics").empty());
}

TEST_F(RecoveryManagerReflexTest, ShutdownRequestIsLatchedOnTheNextCycle)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  auto nav_state = std::make_shared<easynav::NavState>();
  node->cycle(nav_state);
  EXPECT_FALSE(node->is_shutdown_requested());

  // What ShutdownRecovery leaves in the blackboard.
  nav_state->set("system_shutdown_reason", std::string("ros_graph: broken"));
  nav_state->set("system_shutdown_requested", true);
  node->cycle(nav_state);

  EXPECT_TRUE(node->is_shutdown_requested());
  EXPECT_EQ(node->get_shutdown_reason(), "ros_graph: broken");
}

TEST_F(RecoveryManagerReflexTest, ReflexesRunEveryRtCycleWhoeverHasControl)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override("safety_reflex_types", std::vector<std::string>{"reflex"})
    .append_parameter_override("reflex.plugin", std::string("easynav_recovery/DummySafetyReflex"))
    .append_parameter_override("reflex.trigger", true));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_num_safety_reflexes(), 1u);

  // The controller commands motion: the reflex overrides it.
  auto nav_state = std::make_shared<easynav::NavState>();
  geometry_msgs::msg::TwistStamped moving;
  moving.twist.linear.x = 0.5;

  easynav::velocity_command::propose(*nav_state, easynav::VelocitySource::CONTROLLER, moving);

  EXPECT_TRUE(node->cycle_rt(nav_state));
  const auto reflex = easynav::velocity_command::peek(*nav_state, easynav::VelocitySource::REFLEX);
  ASSERT_TRUE(reflex.has_value()) << "the reflex did not override the command";
  EXPECT_DOUBLE_EQ(reflex->twist.linear.x, 0.0);
}
