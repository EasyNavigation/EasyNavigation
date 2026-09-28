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
/// \brief A mission survives SystemNode going active -> inactive -> unconfigured -> inactive ->
/// active in the middle of it (the lifecycle sequence used to switch plugins at runtime): the
/// goal is neither cancelled nor lost, and the mission finishes once the robot reaches it.

#include <functional>
#include <vector>

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
      // Every kind of plugin EasyNav reloads on configure, recovery and safety reflexes included.
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
        "-p", "recovery_manager.evaluator_types:=['dummy_evaluator']",
        "-p", "recovery_manager.dummy_evaluator.plugin:=easynav_recovery/DummyEvaluator",
        "-p", "recovery_manager.mitigation_types:=['dummy_mitigation']",
        "-p", "recovery_manager.dummy_mitigation.plugin:=easynav_recovery/DummyMitigation",
        "-p", "recovery_manager.safety_reflex_types:=['collision']",
        "-p", "recovery_manager.collision.plugin:=easynav_recovery/DummySafetyReflex",
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
