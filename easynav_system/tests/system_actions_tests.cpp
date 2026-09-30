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
/// \brief SystemNode as SystemActions (what a recovery system can ask of EasyNav), and
/// recovery_node as one more EasyNav node.

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "easynav_system/GoalManagerClient.hpp"
#include "easynav_system/SystemNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using ClientState = easynav::GoalManagerClient::State;

class SystemActionsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      std::vector<const char *> argv{
        "system_actions_tests",
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
    }
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  }

  static void set_robot_x(easynav::NavState & nav_state, double x)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = x;
    odom.pose.pose.orientation.w = 1.0;
    nav_state.set("robot_pose", odom);
  }

  bool cycle_until(const std::function<bool()> & done, std::chrono::milliseconds timeout = 3s)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (system_node_->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
        system_node_->system_cycle();
      }
      exe_->spin_some();
      if (done()) {return true;}
      rclcpp::sleep_for(10ms);
    }
    return done();
  }

  void cycle_for(std::chrono::milliseconds duration) {cycle_until([]() {return false;}, duration);}

  bool transition_to(uint8_t transition, uint8_t expected)
  {
    return system_node_->trigger_transition(transition).id() == expected;
  }

  // An active system, a connected client and a mission to x = 5 (the robot at x = 0).
  void start_mission()
  {
    system_node_ = std::make_shared<easynav::SystemNode>();
    ASSERT_TRUE(transition_to(Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE));
    ASSERT_TRUE(transition_to(Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));
    client_node_ = rclcpp::Node::make_shared("actions_client");
    client_ = easynav::GoalManagerClient::make_shared(client_node_);
    exe_->add_node(client_node_);
    exe_->add_node(system_node_->get_node_base_interface());
    set_robot_x(*system_node_->get_nav_state(), 0.0);
    ASSERT_TRUE(
      cycle_until(
        [&]() {
          return client_node_->count_subscribers("easynav_control") >= 2 &&
                 client_node_->count_publishers("easynav_control") >= 2;
        }));

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 5.0;
    goal.pose.orientation.w = 1.0;
    client_->send_goal(goal);
    ASSERT_TRUE(cycle_until([&]() {
        return client_->get_state() == ClientState::ACCEPTED_AND_NAVIGATING;
                                                                                                       }));
  }

  bool finished() {return client_->get_state() == ClientState::NAVIGATION_FINISHED;}

  easynav::SystemNode::SharedPtr system_node_;
  rclcpp::Node::SharedPtr client_node_;
  easynav::GoalManagerClient::SharedPtr client_;
  // Created once rclcpp is initialized.
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
};

TEST_F(SystemActionsTest, RecoveryNodeIsOneMoreEasyNavNode)
{
  system_node_ = std::make_shared<easynav::SystemNode>();
  const auto nodes = system_node_->get_system_nodes();
  ASSERT_EQ(nodes.count("recovery_node"), 1u);
  auto recovery = nodes.at("recovery_node").node_ptr;

  for (const auto & [transition, expected] : std::vector<std::pair<uint8_t, uint8_t>>{
    {Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE},
    {Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE},
    {Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE},
    {Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_UNCONFIGURED}})
  {
    ASSERT_TRUE(transition_to(transition, expected));
    EXPECT_EQ(recovery->get_current_state().id(), expected);
  }
}

TEST_F(SystemActionsTest, AbortMissionEndsItWithAnError)
{
  start_mission();
  system_node_->abort_mission("localization diverged");

  ASSERT_TRUE(cycle_until([&]() {
      return client_->get_state() != ClientState::ACCEPTED_AND_NAVIGATING;
                                                                                                     }));
  EXPECT_NE(client_->get_state(), ClientState::NAVIGATION_FINISHED);
  EXPECT_EQ(
    client_->get_last_control().type, easynav_interfaces::msg::NavigationControl::ERROR);
  EXPECT_EQ(client_->get_last_control().status_message, "localization diverged");
  EXPECT_EQ(
    system_node_->get_nav_state()->get<easynav::GoalManager::State>("navigation_state"),
    easynav::GoalManager::State::IDLE);
  EXPECT_FALSE(system_node_->is_shutdown_requested()) << "aborting does not shut EasyNav down";
}

TEST_F(SystemActionsTest, AbortWithoutMissionDoesNothing)
{
  system_node_ = std::make_shared<easynav::SystemNode>();
  EXPECT_NO_THROW(system_node_->abort_mission("before configure"));
  ASSERT_TRUE(transition_to(Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE));
  ASSERT_TRUE(transition_to(Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));
  EXPECT_NO_THROW(system_node_->abort_mission("no mission"));
  system_node_->system_cycle();
  EXPECT_EQ(system_node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(
    system_node_->get_nav_state()->get<easynav::GoalManager::State>("navigation_state"),
    easynav::GoalManager::State::IDLE);
}

TEST_F(SystemActionsTest, HeldProgressDoesNotReachTheGoalUntilReleased)
{
  start_mission();
  system_node_->hold_mission_progress(true);

  set_robot_x(*system_node_->get_nav_state(), 5.0);
  cycle_for(500ms);
  EXPECT_EQ(client_->get_state(), ClientState::ACCEPTED_AND_NAVIGATING);

  system_node_->hold_mission_progress(false);
  EXPECT_TRUE(cycle_until([&]() {return finished();}));
}

TEST_F(SystemActionsTest, HoldSurvivesADeactivation)
{
  start_mission();
  system_node_->hold_mission_progress(true);
  set_robot_x(*system_node_->get_nav_state(), 5.0);

  ASSERT_TRUE(transition_to(Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE));
  ASSERT_TRUE(transition_to(Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));
  cycle_for(500ms);
  EXPECT_EQ(client_->get_state(), ClientState::ACCEPTED_AND_NAVIGATING);

  system_node_->hold_mission_progress(false);
  EXPECT_TRUE(cycle_until([&]() {return finished();}));
}

TEST_F(SystemActionsTest, ReconfigurationReleasesTheHold)
{
  // The recovery system is unloaded on cleanup: it cannot release a hold it left.
  start_mission();
  system_node_->hold_mission_progress(true);
  set_robot_x(*system_node_->get_nav_state(), 5.0);
  cycle_for(300ms);
  ASSERT_EQ(client_->get_state(), ClientState::ACCEPTED_AND_NAVIGATING);

  for (const auto & [transition, expected] : std::vector<std::pair<uint8_t, uint8_t>>{
    {Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE},
    {Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_UNCONFIGURED},
    {Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE},
    {Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE}})
  {
    ASSERT_TRUE(transition_to(transition, expected));
  }
  EXPECT_TRUE(cycle_until([&]() {return finished();}));
}

TEST_F(SystemActionsTest, ShutdownRequestKeepsTheFirstReason)
{
  system_node_ = std::make_shared<easynav::SystemNode>();
  EXPECT_FALSE(system_node_->is_shutdown_requested());
  EXPECT_EQ(system_node_->get_shutdown_reason(), "");

  system_node_->request_shutdown("first");
  system_node_->request_shutdown("second");
  EXPECT_TRUE(system_node_->is_shutdown_requested());
  EXPECT_EQ(system_node_->get_shutdown_reason(), "first");
}

TEST_F(SystemActionsTest, ShutdownRequestWithoutCyclesEndsFinalized)
{
  system_node_ = std::make_shared<easynav::SystemNode>();
  ASSERT_TRUE(transition_to(Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE));
  ASSERT_TRUE(transition_to(Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE));
  system_node_->request_shutdown("broken");

  // The supervisor deactivates: the error path takes everything to Finalized.
  EXPECT_EQ(
    system_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE).id(),
    State::PRIMARY_STATE_FINALIZED);
  for (auto & [name, info] : system_node_->get_system_nodes()) {
    EXPECT_EQ(info.node_ptr->get_current_state().id(), State::PRIMARY_STATE_FINALIZED) << name;
  }
}
