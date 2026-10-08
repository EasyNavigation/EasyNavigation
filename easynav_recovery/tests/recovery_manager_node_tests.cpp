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
/// \brief Tests for RecoveryManagerNode, the host of the recovery system (a RecoveryManagerBase
/// plugin): loading, lifecycle, forwarding of EasyNav's cycles and releasing the plugin.

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/SystemActions.hpp"
#include "easynav_recovery/DummyRecoveryManager.hpp"
#include "easynav_recovery/RecoveryManagerNode.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{

std::shared_ptr<easynav::DummyRecoveryManager> dummy_manager(
  const std::shared_ptr<easynav::RecoveryManagerNode> & node)
{
  return std::dynamic_pointer_cast<easynav::DummyRecoveryManager>(node->get_recovery_manager());
}

// Records what the recovery system asks of the navigation system.
class RecordingSystemActions : public easynav::SystemActions
{
public:
  void abort_mission(const std::string & reason) override {aborted.push_back(reason);}
  void request_shutdown(const std::string & reason) override {shutdowns.push_back(reason);}
  void hold_mission_progress(bool hold) override {holds.push_back(hold);}
  bool request_reconfigure(
    const std::vector<easynav::ParameterChange> & changes, const std::string &) override
  {
    reconfigures.push_back(changes);
    return accept;
  }
  bool request_restore_parameters(const std::string &) override
  {
    ++restores;
    return accept;
  }
  bool accept {true};
  std::vector<std::string> aborted;
  std::vector<std::string> shutdowns;
  std::vector<bool> holds;
  std::vector<std::vector<easynav::ParameterChange>> reconfigures;
  int restores {0};
};

}  // namespace

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

TEST_F(RecoveryManagerNodeTestCase, NodeName)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  EXPECT_EQ(std::string(node->get_name()), "recovery_node");
}

TEST_F(RecoveryManagerNodeTestCase, LoadsTheDummyRecoveryManagerUnlessConfigured)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  EXPECT_EQ(node->get_recovery_manager(), nullptr) << "nothing loaded before configure";

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_NE(dummy_manager(node), nullptr);
  EXPECT_EQ(node->get_recovery_manager()->get_plugin_name(), "recovery_manager");

  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(node->get_recovery_manager(), nullptr) << "released on cleanup";
}

TEST_F(RecoveryManagerNodeTestCase, FailsToConfigureWithAnUnknownRecoveryManager)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>(
    rclcpp::NodeOptions().append_parameter_override(
      "recovery_manager.plugin", std::string("no_such_pkg/NoSuchRecoveryManager")));
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_NE(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->get_recovery_manager(), nullptr);
}

TEST_F(RecoveryManagerNodeTestCase, ForwardsCyclesAndActivation)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  auto nav_state = std::make_shared<easynav::NavState>();

  // Without a recovery system, the cycles do nothing.
  node->cycle(nav_state);
  EXPECT_FALSE(node->cycle_rt(nav_state));

  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  auto manager = dummy_manager(node);
  ASSERT_NE(manager, nullptr);

  node->trigger_transition(Transition::TRANSITION_ACTIVATE);
  EXPECT_TRUE(manager->is_active());

  node->cycle(nav_state);
  node->cycle(nav_state);
  EXPECT_FALSE(node->cycle_rt(nav_state)) << "the dummy never commands the robot";
  EXPECT_EQ(manager->get_update_count(), 2u);
  EXPECT_EQ(manager->get_update_rt_count(), 1u);

  node->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  EXPECT_FALSE(manager->is_active());
}

TEST_F(RecoveryManagerNodeTestCase, ReconfigureLoadsANewInstance)
{
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  auto first = node->get_recovery_manager();

  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_NE(node->get_recovery_manager(), nullptr);
  EXPECT_NE(node->get_recovery_manager(), first);
}

TEST_F(RecoveryManagerNodeTestCase, ReleasingTheRecoveryManagerReleasesTheMissionHold)
{
  auto actions = std::make_shared<RecordingSystemActions>();
  auto node = std::make_shared<easynav::RecoveryManagerNode>();
  node->set_system_actions(actions);

  // Nothing loaded yet: nothing to release.
  node->trigger_transition(Transition::TRANSITION_CONFIGURE);
  EXPECT_TRUE(actions->holds.empty());

  // A recovery system that goes away cannot release a hold it may have left.
  node->trigger_transition(Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(actions->holds, std::vector<bool>({false}));
  EXPECT_TRUE(actions->aborted.empty());
  EXPECT_TRUE(actions->shutdowns.empty());
}
