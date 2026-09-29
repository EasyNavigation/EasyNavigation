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
/// \brief A mission survives reconfiguring EasyNav (active -> inactive -> unconfigured ->
/// inactive -> active, as when switching plugins at runtime) in the middle of it.

#include <functional>
#include <string>
#include <vector>

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_system/SystemNode.hpp"
#include "easynav_system/GoalManager.hpp"
#include "easynav_system/GoalManagerClient.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class SystemLifecycleMissionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      // Every kind of plugin EasyNav reloads on configure.
      std::vector<const char *> argv{
        "system_lifecycle_mission_tests",
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
  }

  static void set_robot_x(easynav::NavState & nav_state, double x)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = x;
    odom.pose.pose.orientation.w = 1.0;
    nav_state.set("robot_pose", odom);
  }

  // Runs non-RT cycles and spins until done() or timeout.
  static bool cycle_until(
    const easynav::SystemNode::SharedPtr & system_node,
    rclcpp::executors::SingleThreadedExecutor & exe,
    const std::function<bool()> & done, std::chrono::milliseconds timeout = 3s)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (system_node->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
        system_node->system_cycle();
      }
      exe.spin_some();
      if (done()) {return true;}
      rclcpp::sleep_for(10ms);
    }
    return done();
  }
  // Full reconfiguration: deactivate, cleanup, configure, activate.
  static bool reconfigure(const easynav::SystemNode::SharedPtr & system_node)
  {
    for (const auto & [transition, expected] : std::vector<std::pair<uint8_t, uint8_t>>{
      {Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE},
      {Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_UNCONFIGURED},
      {Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE},
      {Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE}})
    {
      if (system_node->trigger_transition(transition).id() != expected) {
        return false;
      }
    }
    return true;
  }

  // An active system with a connected client.
  void start(
    easynav::SystemNode::SharedPtr & system_node, rclcpp::Node::SharedPtr & client_node,
    easynav::GoalManagerClient::SharedPtr & client, rclcpp::executors::SingleThreadedExecutor & exe)
  {
    system_node = std::make_shared<easynav::SystemNode>();
    ASSERT_EQ(
      system_node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(
      system_node->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
      State::PRIMARY_STATE_ACTIVE);
    client_node = rclcpp::Node::make_shared("mission_client");
    client = easynav::GoalManagerClient::make_shared(client_node);
    exe.add_node(client_node);
    exe.add_node(system_node->get_node_base_interface());
    set_robot_x(*system_node->get_nav_state(), 0.0);
    ASSERT_TRUE(
      cycle_until(
        system_node, exe, [&]() {
          return client_node->count_subscribers("easynav_control") >= 2 &&
                 client_node->count_publishers("easynav_control") >= 2;
        }));
  }

  static geometry_msgs::msg::PoseStamped goal_at(double x)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = x;
    goal.pose.orientation.w = 1.0;
    return goal;
  }

  using ClientState = easynav::GoalManagerClient::State;
};

TEST_F(SystemLifecycleMissionTest, MissionSurvivesReconfigurationAndReachesTheGoal)
{
  auto system_node = std::make_shared<easynav::SystemNode>();
  ASSERT_EQ(
    system_node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
    State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    system_node->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
    State::PRIMARY_STATE_ACTIVE);

  auto client_node = rclcpp::Node::make_shared("mission_client");
  auto client = easynav::GoalManagerClient::make_shared(client_node);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  auto nav_state = system_node->get_nav_state();
  set_robot_x(*nav_state, 0.0);

  // Wait until the client and GoalManager see each other before sending the goal.
  ASSERT_TRUE(
    cycle_until(
      system_node, exe, [&]() {
        return client_node->count_subscribers("easynav_control") >= 2 &&
               client_node->count_publishers("easynav_control") >= 2;
      }));

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 5.0;
  goal.pose.orientation.w = 1.0;
  client->send_goal(goal);

  ASSERT_TRUE(
    cycle_until(
      system_node, exe, [&]() {
        return client->get_state() ==
               easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING;
      }));

  // Halfway there.
  set_robot_x(*nav_state, 2.5);
  cycle_until(system_node, exe, []() {return false;}, 300ms);
  ASSERT_EQ(
    client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  // active -> inactive -> unconfigured -> inactive -> active, e.g. to switch a plugin.
  for (const auto & [transition, expected] : std::vector<std::pair<uint8_t, uint8_t>>{
    {Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE},
    {Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_UNCONFIGURED},
    {Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE},
    {Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE}})
  {
    easynav::SystemNode::CallbackReturnT cb_result;
    const auto & new_state = system_node->trigger_transition(transition, cb_result);
    ASSERT_EQ(cb_result, easynav::SystemNode::CallbackReturnT::SUCCESS) <<
      "transition " << static_cast<int>(transition);
    ASSERT_EQ(new_state.id(), expected) << "transition " << static_cast<int>(transition);

    for (auto & [name, info] : system_node->get_system_nodes()) {
      EXPECT_EQ(info.node_ptr->get_current_state().id(), expected) << name;
    }

    // Keep the client's callbacks flowing between transitions, as a real executor would.
    cycle_until(system_node, exe, []() {return false;}, 100ms);
    ASSERT_EQ(
      client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING) <<
      "mission interrupted after transition " << static_cast<int>(transition);
  }

  // Neither the goal nor the navigation were cancelled.
  cycle_until(system_node, exe, []() {return false;}, 300ms);
  EXPECT_EQ(
    client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  EXPECT_EQ(
    nav_state->get<easynav::GoalManager::State>("navigation_state"),
    easynav::GoalManager::State::ACTIVE);
  const auto goals = nav_state->get<nav_msgs::msg::Goals>("goals");
  ASSERT_EQ(goals.goals.size(), 1u);
  EXPECT_DOUBLE_EQ(goals.goals[0].pose.position.x, 5.0);

  // The robot reaches the goal: the same mission finishes.
  set_robot_x(*nav_state, 5.0);
  EXPECT_TRUE(
    cycle_until(
      system_node, exe, [&]() {
        return client->get_state() == easynav::GoalManagerClient::State::NAVIGATION_FINISHED;
      })) << "client state: " << static_cast<int>(client->get_state());
}

TEST_F(SystemLifecycleMissionTest, MissionSurvivesRepeatedReconfigurations)
{
  easynav::SystemNode::SharedPtr system_node;
  rclcpp::Node::SharedPtr client_node;
  easynav::GoalManagerClient::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor exe;
  start(system_node, client_node, client, exe);

  client->send_goal(goal_at(5.0));
  ASSERT_TRUE(
    cycle_until(
      system_node, exe, [&]() {
        return client->get_state() == ClientState::ACCEPTED_AND_NAVIGATING;
    }));

  for (int i = 1; i <= 3; ++i) {
    set_robot_x(*system_node->get_nav_state(), i);
    ASSERT_TRUE(reconfigure(system_node)) << "reconfiguration " << i;
    cycle_until(system_node, exe, []() {return false;}, 200ms);
    ASSERT_EQ(client->get_state(), ClientState::ACCEPTED_AND_NAVIGATING) << "reconfiguration " << i;
    // The last pose left in NavState is still there for the new localizer.
    EXPECT_DOUBLE_EQ(
      system_node->get_nav_state()->get<nav_msgs::msg::Odometry>("robot_pose").pose.pose.position.x,
      i);
  }

  set_robot_x(*system_node->get_nav_state(), 5.0);
  EXPECT_TRUE(
    cycle_until(
      system_node, exe, [&]() {return client->get_state() == ClientState::NAVIGATION_FINISHED;}));
}

TEST_F(SystemLifecycleMissionTest, ReconfigurationWithoutMissionThenNewMission)
{
  easynav::SystemNode::SharedPtr system_node;
  rclcpp::Node::SharedPtr client_node;
  easynav::GoalManagerClient::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor exe;
  start(system_node, client_node, client, exe);

  ASSERT_TRUE(reconfigure(system_node));
  cycle_until(system_node, exe, []() {return false;}, 200ms);
  EXPECT_EQ(
    system_node->get_nav_state()->get<easynav::GoalManager::State>("navigation_state"),
    easynav::GoalManager::State::IDLE);

  client->send_goal(goal_at(2.0));
  ASSERT_TRUE(
    cycle_until(
      system_node, exe, [&]() {
        return client->get_state() == ClientState::ACCEPTED_AND_NAVIGATING;
    }));
  set_robot_x(*system_node->get_nav_state(), 2.0);
  EXPECT_TRUE(
    cycle_until(
      system_node, exe, [&]() {return client->get_state() == ClientState::NAVIGATION_FINISHED;}));
}

TEST_F(SystemLifecycleMissionTest, MissionCanBeCancelledAfterReconfiguration)
{
  easynav::SystemNode::SharedPtr system_node;
  rclcpp::Node::SharedPtr client_node;
  easynav::GoalManagerClient::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor exe;
  start(system_node, client_node, client, exe);

  client->send_goal(goal_at(5.0));
  ASSERT_TRUE(
    cycle_until(
      system_node, exe, [&]() {
        return client->get_state() == ClientState::ACCEPTED_AND_NAVIGATING;
    }));
  ASSERT_TRUE(reconfigure(system_node));
  cycle_until(system_node, exe, []() {return false;}, 200ms);

  client->cancel();
  EXPECT_TRUE(
    cycle_until(
      system_node, exe, [&]() {return client->get_state() == ClientState::NAVIGATION_CANCELLED;}));
  EXPECT_EQ(
    system_node->get_nav_state()->get<easynav::GoalManager::State>("navigation_state"),
    easynav::GoalManager::State::IDLE);
}

TEST_F(SystemLifecycleMissionTest, ParametersAndPluginsChangedWhileUnconfiguredApplyOnActivation)
{
  easynav::SystemNode::SharedPtr system_node;
  rclcpp::Node::SharedPtr client_node;
  easynav::GoalManagerClient::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor exe;
  start(system_node, client_node, client, exe);

  client->send_goal(goal_at(5.0));
  ASSERT_TRUE(
    cycle_until(
      system_node, exe, [&]() {
        return client->get_state() == ClientState::ACCEPTED_AND_NAVIGATING;
    }));

  // Close to the goal, but outside the default position tolerance: not finished.
  set_robot_x(*system_node->get_nav_state(), 4.2);
  cycle_until(system_node, exe, []() {return false;}, 200ms);
  ASSERT_EQ(client->get_state(), ClientState::ACCEPTED_AND_NAVIGATING);

  auto controller_node = std::dynamic_pointer_cast<easynav::ControllerNode>(
    system_node->get_system_nodes().at("controller_node").node_ptr);
  ASSERT_NE(controller_node, nullptr);
  ASSERT_EQ(controller_node->get_loaded_controller(), "dummy_controller");

  ASSERT_EQ(
    system_node->trigger_transition(Transition::TRANSITION_DEACTIVATE).id(),
    State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    system_node->trigger_transition(Transition::TRANSITION_CLEANUP).id(),
    State::PRIMARY_STATE_UNCONFIGURED);

  // Unconfigured: change a parameter and a plugin.
  system_node->set_parameter(rclcpp::Parameter("position_tolerance", 1.0));
  controller_node->set_parameter(
    rclcpp::Parameter("controller_types", std::vector<std::string>{"other_controller"}));
  controller_node->declare_parameter(
    "other_controller.plugin", std::string("easynav_controller/DummyController"));

  ASSERT_EQ(
    system_node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
    State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    system_node->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
    State::PRIMARY_STATE_ACTIVE);

  // Both are in effect, and the same mission goes on: now within tolerance, it finishes.
  EXPECT_EQ(controller_node->get_loaded_controller(), "other_controller");
  EXPECT_DOUBLE_EQ(
    system_node->get_nav_state()->get<double>("goal_tolerance.position"), 1.0);
  EXPECT_TRUE(
    cycle_until(
      system_node, exe, [&]() {return client->get_state() == ClientState::NAVIGATION_FINISHED;}));
}
