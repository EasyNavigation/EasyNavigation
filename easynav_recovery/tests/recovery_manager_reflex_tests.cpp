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
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_core/VelocityCommand.hpp"
#include "easynav_recovery/DefaultRecoveryManager.hpp"
#include "easynav_recovery/RecoveryManagerNode.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;


namespace
{

// The recovery system RecoveryManagerNode loaded (the default one unless configured otherwise).
std::shared_ptr<easynav::DefaultRecoveryManager> default_manager(
  const std::shared_ptr<easynav::RecoveryManagerNode> & node)
{
  return std::dynamic_pointer_cast<easynav::DefaultRecoveryManager>(node->get_recovery_manager());
}

// Records what the recovery system asks of the navigation system.
class RecordingSystemActions : public easynav::SystemActions
{
public:
  void abort_mission(const std::string & reason) override {aborted.push_back(reason);}
  void request_shutdown(const std::string & reason) override {shutdowns.push_back(reason);}
  void hold_mission_progress(bool hold) override {holds.push_back(hold);}
  std::vector<std::string> aborted;
  std::vector<std::string> shutdowns;
  std::vector<bool> holds;
};

}  // namespace

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
  EXPECT_EQ(default_manager(node)->get_num_safety_reflexes(), 0u);
}

TEST_F(RecoveryManagerReflexTest, ConfigureLoadsSafetyReflex)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override("recovery_manager.safety_reflex_types",
    std::vector<std::string>{"reflex"})
    .append_parameter_override(
      "recovery_manager.reflex.plugin", std::string("easynav_recovery/DummySafetyReflex")));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(default_manager(node)->get_num_safety_reflexes(), 1u);

  // The reflex does not intervene, so recovery commands nothing.
  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_FALSE(node->cycle_rt(nav_state));
}

TEST_F(RecoveryManagerReflexTest, ConfigureFailsWithUnknownReflexPlugin)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override("recovery_manager.safety_reflex_types",
    std::vector<std::string>{"bogus"})
    .append_parameter_override("recovery_manager.bogus.plugin",
    std::string("no_such_pkg/NoSuchReflex")));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  EXPECT_NE(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(RecoveryManagerReflexTest, ReflexesSurviveReconfiguration)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override("recovery_manager.safety_reflex_types",
    std::vector<std::string>{"reflex"})
    .append_parameter_override(
      "recovery_manager.reflex.plugin", std::string("easynav_recovery/DummySafetyReflex")));

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(node->get_recovery_manager(), nullptr) << "cleanup releases the recovery system";
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  EXPECT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(default_manager(node)->get_num_safety_reflexes(), 1u);
}

TEST_F(RecoveryManagerReflexTest, CleanupGivesControlBackAndDropsDiagnostics)
{
  // A cleanup drops the mitigation that held control and the evaluators that wrote the
  // diagnostics: the next cycle must not keep routing control to a mitigation that is gone.
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  auto nav_state_ptr = std::make_shared<easynav::NavState>();
  auto & nav_state = *nav_state_ptr;

  // The first recovery system runs, and leaves this state behind.
  node->cycle(nav_state_ptr);
  nav_state.set("control_owner", std::string("recovery:retreat"));
  nav_state.set_group("diagnostics", std::vector<std::string>{"diagnostics.stale"});

  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  // The new one, on its first cycle, drops it.
  node->cycle(nav_state_ptr);
  EXPECT_EQ(nav_state.get<std::string>("control_owner"), "controller");
  EXPECT_TRUE(nav_state.get_group_keys("diagnostics").empty());
}

TEST_F(RecoveryManagerReflexTest, HostLoadsTheDefaultRecoveryManagerUnlessConfigured)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(default_manager(node), nullptr);

  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(node->get_recovery_manager(), nullptr);
}

TEST_F(RecoveryManagerReflexTest, HostFailsToConfigureWithAnUnknownRecoveryManager)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions().append_parameter_override(
      "recovery_manager.plugin", std::string("no_such_pkg/NoSuchRecoveryManager")));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_NE(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(RecoveryManagerReflexTest, MitigationSignalsBecomeSystemActions)
{
  auto actions = std::make_shared<RecordingSystemActions>();
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->set_system_actions(actions);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  auto nav_state = std::make_shared<easynav::NavState>();
  node->cycle(nav_state);
  EXPECT_TRUE(actions->aborted.empty());
  EXPECT_TRUE(actions->shutdowns.empty());

  // What CancelMissionRecovery and ShutdownRecovery leave in the blackboard.
  nav_state->set("mission_cancel_requested", true);
  nav_state->set("system_shutdown_reason", std::string("ros_graph: broken"));
  nav_state->set("system_shutdown_requested", true);
  node->cycle(nav_state);

  ASSERT_EQ(actions->aborted.size(), 1u);
  ASSERT_EQ(actions->shutdowns.size(), 1u);
  EXPECT_EQ(actions->shutdowns.front(), "ros_graph: broken");
  EXPECT_FALSE(nav_state->get<bool>("mission_cancel_requested")) << "the request is consumed";

  // Requested once, not every cycle.
  node->cycle(nav_state);
  EXPECT_EQ(actions->aborted.size(), 1u);
  EXPECT_EQ(actions->shutdowns.size(), 1u);
}

TEST_F(RecoveryManagerReflexTest, HoldsMissionProgressWhileRecovering)
{
  auto actions = std::make_shared<RecordingSystemActions>();
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->set_system_actions(actions);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);

  auto nav_state = std::make_shared<easynav::NavState>();
  node->cycle(nav_state);
  ASSERT_EQ(actions->holds, std::vector<bool>({false})) << "the first cycle sets it anyway";

  // A diagnostic in ERROR (e.g. AMCL diverged) holds the mission's progress...
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  nav_state->set("diagnostics.fake", status);
  nav_state->set_group("diagnostics", {"diagnostics.fake"});
  node->cycle(nav_state);
  node->cycle(nav_state);
  ASSERT_EQ(actions->holds, std::vector<bool>({false, true})) << "only on changes";

  // ...until it is resolved.
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  nav_state->set("diagnostics.fake", status);
  node->cycle(nav_state);
  ASSERT_EQ(actions->holds, std::vector<bool>({false, true, false}));

  // Unloading the recovery system releases a hold it left.
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  nav_state->set("diagnostics.fake", status);
  node->cycle(nav_state);
  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  ASSERT_EQ(actions->holds, std::vector<bool>({false, true, false, true, false}));
}

TEST_F(RecoveryManagerReflexTest, ReflexesRunEveryRtCycleWhoeverHasControl)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions()
    .append_parameter_override("recovery_manager.safety_reflex_types",
    std::vector<std::string>{"reflex"})
    .append_parameter_override("recovery_manager.reflex.plugin",
    std::string("easynav_recovery/DummySafetyReflex"))
    .append_parameter_override("recovery_manager.reflex.trigger", true));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(default_manager(node)->get_num_safety_reflexes(), 1u);

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
