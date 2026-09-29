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


#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "easynav_system/GoalManager.hpp"
#include "easynav_system/GoalManagerClient.hpp"
#include "easynav_common/types/NavState.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"


class GoalManagerTestCase : public ::testing::Test
{
protected:
  ~GoalManagerTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    if (!initialized) {
      rclcpp::init(0, nullptr);
      initialized = true;
    }
  }

  void TearDown()
  {
  }

  bool initialized {false};
};

using namespace std::chrono_literals;


TEST_F(GoalManagerTestCase, initpose_topic)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  // client_node->get_logger().set_level(rclcpp::Logger::Level::Debug);
  // system_node->get_logger().set_level(rclcpp::Logger::Level::Debug);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  easynav_interfaces::msg::NavigationControl last_control;

  auto pose_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 100);
  auto control_sub = client_node->create_subscription<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100,
    [&last_control](easynav_interfaces::msg::NavigationControl::UniquePtr msg) {
      last_control = *msg;
    });

  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  ASSERT_TRUE(nav_state->has("navigation_state"));
  auto state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);

  // Navigation 1 succesfull
  RCLCPP_INFO(client_node->get_logger(), "Navigation 1 succesfull");

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  pose_pub->publish(goal);

  rclcpp::Rate rate(20);
  auto start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);

  nav_msgs::msg::Goals req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");
  ASSERT_EQ(req_goals.header.stamp, goal.header.stamp);

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals.size(), 1u);
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
}

TEST_F(GoalManagerTestCase, initpose_topic_with_preempt)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());
  auto client_node = rclcpp::Node::make_shared("client_node");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  // client_node->get_logger().set_level(rclcpp::Logger::Level::Debug);
  // system_node->get_logger().set_level(rclcpp::Logger::Level::Debug);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  easynav_interfaces::msg::NavigationControl last_control;

  auto pose_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 100);
  auto control_sub = client_node->create_subscription<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100,
    [&last_control](easynav_interfaces::msg::NavigationControl::UniquePtr msg) {
      last_control = *msg;
    });

  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);
  auto gm_client = easynav::GoalManagerClient::make_shared(client_node);

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  ASSERT_TRUE(nav_state->has("navigation_state"));
  auto state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);

  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

 // Navigation 1 succesfull
  RCLCPP_INFO(client_node->get_logger(), "Navigation 1 succesfull");

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  rclcpp::Rate rate(20);
  auto start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  nav_msgs::msg::Goals req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);

  last_control = gm_client->get_last_control();
  auto last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control, last_feedback);
  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals.size(), 1u);
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  goal.pose.position.x = 6.0;

  // Navigation 2 preempt
  RCLCPP_INFO(client_node->get_logger(), "Navigation 2 preempt");

  pose_pub->publish(goal);

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);

  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_CANCELLED);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals.size(), 1u);
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);

  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);
}

TEST_F(GoalManagerTestCase, simple_nav_node)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());
  auto client_node = rclcpp::Node::make_shared("client_node");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  // client_node->get_logger().set_level(rclcpp::Logger::Level::Debug);
  // system_node->get_logger().set_level(rclcpp::Logger::Level::Debug);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  auto gm_client = easynav::GoalManagerClient::make_shared(client_node);
  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  ASSERT_TRUE(nav_state->has("navigation_state"));
  auto state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);


  // Navigation 1 succesfull
  RCLCPP_INFO(client_node->get_logger(), "Navigation 1 succesfull");

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  rclcpp::Rate rate(20);
  auto start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  nav_msgs::msg::Goals req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  auto last_control = gm_client->get_last_control();
  auto last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control, last_feedback);
  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals.size(), 1u);
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 2 succesfull
  RCLCPP_INFO(client_node->get_logger(), "Navigation 2 succesfull");

  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 3 : Refresh goal position
  RCLCPP_INFO(client_node->get_logger(), "Navigation 3 : Refresh goal position");

  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 800ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();
  auto last_result = gm_client->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 3 : Cancel Navigation from Client
  RCLCPP_INFO(client_node->get_logger(), "Navigation 3 : Cancel Navigation from Client");

  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_client->cancel();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_CANCELLED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  // NavState's "goals" must also reflect the cancellation, not just the server's
  // internal goals_ member -- CANCEL clears goals_ synchronously in the callback,
  // before update() ever sees the transition, so a guard keyed off goals_.goals
  // alone would skip republishing and leave NavState stuck on the last non-empty
  // value.
  const auto nav_state_goals = nav_state->get<nav_msgs::msg::Goals>("goals");
  ASSERT_TRUE(nav_state_goals.goals.empty());

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();
  last_result = gm_client->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::CANCELLED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::CANCELLED);
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 5 succesfull

  RCLCPP_INFO(client_node->get_logger(), "Navigation 5 succesfull");

  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();
  last_result = gm_client->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 6 : Navigation failed

  RCLCPP_INFO(client_node->get_logger(), "Navigation 6 : Navigation failed");

  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_server->set_failed("Reason 1");

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FAILED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();
  last_result = gm_client->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FAILED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::FAILED);
  ASSERT_EQ(last_result.status_message, "Reason 1");
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 7 succesfull

  RCLCPP_INFO(client_node->get_logger(), "Navigation 7 succesfull");

  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();
  last_result = gm_client->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 8 : Navigation error

  RCLCPP_INFO(client_node->get_logger(), "Navigation 8 : Navigation error");

  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_server->set_error("Reason 2");

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ERROR);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());
  ASSERT_EQ(req_goals.header.frame_id, "");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();
  last_result = gm_client->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::ERROR);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::ERROR);
  ASSERT_EQ(last_result.status_message, "Reason 2");
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 9 succesfull

  RCLCPP_INFO(client_node->get_logger(), "Navigation 9 succesfull");

  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);
  ASSERT_EQ(last_control, last_feedback);

  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());
  ASSERT_EQ(req_goals.header.frame_id, "");

  last_control = gm_client->get_last_control();
  last_feedback = gm_client->get_feedback();
  last_result = gm_client->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::IDLE);
}

TEST_F(GoalManagerTestCase, two_clients)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());
  auto client_node1 = rclcpp::Node::make_shared("client_node1");
  auto client_node2 = rclcpp::Node::make_shared("client_node2");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  // client_node1->get_logger().set_level(rclcpp::Logger::Level::Debug);
  // client_node2->get_logger().set_level(rclcpp::Logger::Level::Debug);
  // system_node->get_logger().set_level(rclcpp::Logger::Level::Debug);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node1);
  exe.add_node(client_node2);
  exe.add_node(system_node->get_node_base_interface());

  auto gm_client1 = easynav::GoalManagerClient::make_shared(client_node1);
  auto gm_client2 = easynav::GoalManagerClient::make_shared(client_node2);
  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  ASSERT_TRUE(nav_state->has("navigation_state"));
  auto state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);

  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);


  // Navigation 1 succesfull client 1
  RCLCPP_INFO(system_node->get_logger(), "Navigation 1 succesfull client 1");

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = system_node->now();
  goal.pose.position.x = 5.0;

  gm_client1->send_goal(goal);

  rclcpp::Rate rate(20);
  auto start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);

  nav_msgs::msg::Goals req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  auto last_control = gm_client1->get_last_control();
  auto last_feedback = gm_client1->get_feedback();

  ASSERT_EQ(last_control, last_feedback);
  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals.size(), 1u);
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);

  start = system_node->now();
  while (system_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  gm_server->set_finished();

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client1->get_last_control();
  last_feedback = gm_client1->get_feedback();
  auto last_result = gm_client1->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client1->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 2 succesfull client 2

  RCLCPP_INFO(system_node->get_logger(), "Navigation 2 succesfull client 2");

  gm_client2->send_goal(goal);

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client2->get_last_control();
  last_feedback = gm_client2->get_feedback();

  ASSERT_EQ(last_control, last_feedback);
  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals.size(), 1u);
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);

  start = system_node->now();
  while (system_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  gm_server->set_finished();

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client2->get_last_control();
  last_feedback = gm_client2->get_feedback();
  last_result = gm_client2->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client2->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 3 succesfull client 2 and client 1 cancelling

  RCLCPP_INFO(system_node->get_logger(), "Navigation 2 succesfull client 2 with client 1 cancel");

  gm_client2->send_goal(goal);

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client2->get_last_control();
  last_feedback = gm_client2->get_feedback();

  ASSERT_EQ(last_control, last_feedback);
  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals.size(), 1u);
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);

  start = system_node->now();
  while (system_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  gm_client1->cancel();

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);

  gm_server->set_finished();

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client2->get_last_control();
  last_feedback = gm_client2->get_feedback();
  last_result = gm_client2->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_result.user_id, std::string("easynav_system"));

  gm_client2->reset();
  gm_client1->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);

  // Navigation 4 Preempt from different clients

  RCLCPP_INFO(system_node->get_logger(), "Navigation 2 succesfull client 2 with client 1 cancel");

  gm_client2->send_goal(goal);

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);

  req_goals = gm_server->get_goals();
  ASSERT_EQ(req_goals.header, goal.header);
  ASSERT_EQ(req_goals.goals.size(), 1);
  ASSERT_EQ(req_goals.goals[0], goal);
  ASSERT_EQ(req_goals.header.frame_id, "map");

  last_control = gm_client2->get_last_control();
  last_feedback = gm_client2->get_feedback();

  ASSERT_EQ(last_control, last_feedback);
  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FEEDBACK);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_control.goals.goals.size(), 1u);
  ASSERT_EQ(last_control.goals.goals, req_goals.goals);

  start = system_node->now();
  while (system_node->now() - start < 200ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  gm_client1->send_goal(goal);

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::NAVIGATION_CANCELLED);

  gm_server->set_finished();

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::NAVIGATION_CANCELLED);

  req_goals = gm_server->get_goals();
  ASSERT_TRUE(req_goals.goals.empty());

  last_control = gm_client1->get_last_control();
  last_feedback = gm_client1->get_feedback();
  auto last_result2 = gm_client2->get_result();
  auto last_result1 = gm_client1->get_result();

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_control.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result1.type, easynav_interfaces::msg::NavigationControl::FINISHED);
  ASSERT_EQ(last_result1.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result2.type, easynav_interfaces::msg::NavigationControl::CANCELLED);
  ASSERT_EQ(last_result2.user_id, std::string("easynav_system"));
  ASSERT_EQ(last_result2.status_message, std::string("Navigation preempted by others"));

  gm_client2->reset();
  gm_client1->reset();

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  state = nav_state->get<easynav::GoalManager::State>("navigation_state");
  ASSERT_EQ(state, easynav::GoalManager::State::IDLE);
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::IDLE);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);
}

TEST_F(GoalManagerTestCase, default_update_frequency)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  double freq = 0.0;
  ASSERT_TRUE(system_node->get_parameter("update_frequency", freq));
  ASSERT_DOUBLE_EQ(freq, 20.0);
}

TEST_F(GoalManagerTestCase, update_respects_frequency_limit)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());

  auto client_node = rclcpp::Node::make_shared("client_node");

  // Use a non-default frequency to prove the parameter actually drives the gating.
  const double test_frequency = 10.0;
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared(
    "system_node",
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("update_frequency", test_frequency)}));

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  int feedback_count = 0;
  auto control_sub = client_node->create_subscription<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100,
    [&feedback_count](easynav_interfaces::msg::NavigationControl::UniquePtr msg) {
      if (msg->type == easynav_interfaces::msg::NavigationControl::FEEDBACK) {
        feedback_count++;
      }
    });

  auto pose_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 100);

  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = system_node->now();
  goal.pose.position.x = 1000.0;  // Far enough to never be reached during the test.

  pose_pub->publish(goal);

  // Warm-up: get GoalManager into ACTIVE state.
  rclcpp::Rate warmup_rate(test_frequency);
  auto start = system_node->now();
  while (system_node->now() - start < 500ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    warmup_rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);

  // Hammer update() as fast as possible; only test_frequency executions per second
  // should actually go through and publish feedback.
  feedback_count = 0;
  const auto window = 600ms;
  start = system_node->now();
  while (system_node->now() - start < window) {
    gm_server->update(*nav_state);
    exe.spin_some();
  }

  const int expected = static_cast<int>(
    test_frequency * std::chrono::duration<double>(window).count());
  ASSERT_GE(feedback_count, expected - 2);
  ASSERT_LE(feedback_count, expected + 2);
}

TEST_F(GoalManagerTestCase, PauseAndResumeCycle)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  auto gm_client = easynav::GoalManagerClient::make_shared(client_node);
  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  ASSERT_FALSE(gm_server->is_paused());
  ASSERT_TRUE(nav_state->has("navigation_paused"));
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));
  ASSERT_FALSE(gm_client->is_paused());

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  rclcpp::Rate rate(20);
  auto start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  gm_client->pause();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_TRUE(gm_server->is_paused());
  ASSERT_TRUE(nav_state->get_safe<bool>("navigation_paused"));
  ASSERT_TRUE(gm_client->is_paused());
  // Pausing must not touch the navigation/goal state itself.
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  gm_client->resume();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_FALSE(gm_server->is_paused());
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));
  ASSERT_FALSE(gm_client->is_paused());
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
}

TEST_F(GoalManagerTestCase, PauseRejectedWhenIdle)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  easynav_interfaces::msg::NavigationControl last_control;
  auto control_sub = client_node->create_subscription<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100,
    [&last_control](easynav_interfaces::msg::NavigationControl::UniquePtr msg) {
      last_control = *msg;
    });
  auto control_pub = client_node->create_publisher<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100);

  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);

  easynav_interfaces::msg::NavigationControl pause_msg;
  pause_msg.type = easynav_interfaces::msg::NavigationControl::PAUSE;
  pause_msg.user_id = "some_client";
  control_pub->publish(pause_msg);

  rclcpp::Rate rate(20);
  auto start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(last_control.type, easynav_interfaces::msg::NavigationControl::REJECT);
  ASSERT_FALSE(gm_server->is_paused());
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));
}

TEST_F(GoalManagerTestCase, PauseAcceptedFromThirdPartyClient)
{
  // Unlike CANCEL, PAUSE/RESUME are not restricted to the goal's owner: an
  // operator tool or a fleet-level conflict monitor -- represented here by
  // gm_client2, which never sent any goal of its own -- must be able to
  // pause/resume whatever navigation client_node1 currently has active.
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());

  auto client_node1 = rclcpp::Node::make_shared("client_node1");
  auto client_node2 = rclcpp::Node::make_shared("client_node2");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node1);
  exe.add_node(client_node2);
  exe.add_node(system_node->get_node_base_interface());

  auto gm_client1 = easynav::GoalManagerClient::make_shared(client_node1);
  auto gm_client2 = easynav::GoalManagerClient::make_shared(client_node2);
  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = system_node->now();
  goal.pose.position.x = 5.0;

  gm_client1->send_goal(goal);

  rclcpp::Rate rate(20);
  auto start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);

  gm_client2->pause();

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_TRUE(gm_server->is_paused());
  ASSERT_TRUE(nav_state->get_safe<bool>("navigation_paused"));
  ASSERT_TRUE(gm_client2->is_paused());
  // The requester's own goal-ownership state must stay untouched.
  ASSERT_EQ(gm_client2->get_state(), easynav::GoalManagerClient::State::IDLE);
  // The owning client is not the one who paused, but the effect is still global.
  ASSERT_EQ(gm_client1->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  gm_client2->resume();

  start = system_node->now();
  while (system_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_FALSE(gm_server->is_paused());
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));
  ASSERT_FALSE(gm_client2->is_paused());
}

TEST_F(GoalManagerTestCase, PauseFlagResetOnCancelAndFinish)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  auto gm_client = easynav::GoalManagerClient::make_shared(client_node);
  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  rclcpp::Rate rate(20);
  auto start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  gm_client->pause();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_TRUE(gm_server->is_paused());

  // Cancelling a paused navigation must clear the pause flag too.
  gm_client->cancel();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  ASSERT_FALSE(gm_server->is_paused());
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));

  gm_client->reset();

  // A brand new goal must start unpaused.
  goal.header.stamp = client_node->now();
  gm_client->send_goal(goal);

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  ASSERT_FALSE(gm_server->is_paused());
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));

  gm_client->pause();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_TRUE(gm_server->is_paused());

  // Finishing a paused navigation must also clear the pause flag.
  gm_server->set_finished();

  start = client_node->now();
  while (client_node->now() - start < 400ms) {
    gm_server->update(*nav_state);
    exe.spin_some();
    rate.sleep();
  }

  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  ASSERT_FALSE(gm_server->is_paused());
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));
}

TEST_F(GoalManagerTestCase, FinalInfoPublishedWhenMissionEnds)
{
  // Info is throttled while navigating, but the end of the mission is always published.
  auto nav_state = std::make_shared<easynav::NavState>();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());

  auto client_node = rclcpp::Node::make_shared("client_node");
  auto system_node = rclcpp_lifecycle::LifecycleNode::make_shared("system_node");

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(client_node);
  exe.add_node(system_node->get_node_base_interface());

  std::vector<easynav_interfaces::msg::GoalManagerInfo> infos;
  auto info_sub = client_node->create_subscription<easynav_interfaces::msg::GoalManagerInfo>(
    "easynav_manager_info", 100,
    [&infos](easynav_interfaces::msg::GoalManagerInfo::UniquePtr msg) {
      infos.push_back(*msg);
    });

  auto pose_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 100);

  auto gm_server = easynav::GoalManager::make_shared(*nav_state, system_node);

  auto spin_for = [&](std::chrono::milliseconds duration) {
      rclcpp::Rate rate(20);
      auto start = client_node->now();
      while (client_node->now() - start < duration) {
        gm_server->update(*nav_state);
        exe.spin_some();
        rate.sleep();
      }
    };

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = system_node->now();
  goal.pose.position.x = 1.0;
  goal.pose.orientation.w = 1.0;
  pose_pub->publish(goal);

  spin_for(500ms);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::ACTIVE);
  ASSERT_FALSE(infos.empty());
  ASSERT_EQ(infos.back().status, easynav_interfaces::msg::GoalManagerInfo::ACTIVE);

  // Arrival: last info is IDLE, no goals, last distance.
  nav_msgs::msg::Odometry at_goal;
  at_goal.pose.pose.position.x = 0.99;
  at_goal.pose.pose.orientation.w = 1.0;
  nav_state->set("robot_pose", at_goal);

  spin_for(300ms);
  ASSERT_EQ(gm_server->get_state(), easynav::GoalManager::State::IDLE);
  ASSERT_EQ(infos.back().status, easynav_interfaces::msg::GoalManagerInfo::IDLE);
  ASSERT_TRUE(infos.back().goals.goals.empty());
  ASSERT_NEAR(infos.back().position_distance, 0.01, 1e-6);

  // Same when the mission ends outside update().
  goal.header.stamp = system_node->now();
  goal.pose.position.x = 1000.0;
  pose_pub->publish(goal);
  spin_for(500ms);
  ASSERT_EQ(infos.back().status, easynav_interfaces::msg::GoalManagerInfo::ACTIVE);

  gm_server->set_error("aborted");
  spin_for(200ms);
  ASSERT_EQ(infos.back().status, easynav_interfaces::msg::GoalManagerInfo::IDLE);
}

// GoalManagerInfo across mission ends, sequences and multi-goal missions.
class GoalManagerInfoTest : public GoalManagerTestCase
{
protected:
  void SetUp() override
  {
    GoalManagerTestCase::SetUp();
    nav_state_ = std::make_shared<easynav::NavState>();
    set_robot(0.0, 0.0);

    client_node_ = rclcpp::Node::make_shared("info_client_node");
    system_node_ = rclcpp_lifecycle::LifecycleNode::make_shared("info_system_node");
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_node(client_node_);
    exe_->add_node(system_node_->get_node_base_interface());

    info_sub_ = client_node_->create_subscription<easynav_interfaces::msg::GoalManagerInfo>(
      "easynav_manager_info", 100,
      [this](easynav_interfaces::msg::GoalManagerInfo::UniquePtr msg) {infos_.push_back(*msg);});

    gm_server_ = easynav::GoalManager::make_shared(*nav_state_, system_node_);
    gm_client_ = easynav::GoalManagerClient::make_shared(client_node_);
  }

  void set_robot(double x, double y)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.w = 1.0;
    nav_state_->set("robot_pose", odom);
  }

  static geometry_msgs::msg::PoseStamped goal_at(double x, double y)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    return goal;
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    rclcpp::Rate rate(20);
    const auto start = client_node_->now();
    while (client_node_->now() - start < duration) {
      gm_server_->update(*nav_state_);
      exe_->spin_some();
      rate.sleep();
    }
  }

  int count(uint8_t status) const
  {
    return static_cast<int>(std::count_if(
             infos_.begin(), infos_.end(),
             [status](const auto & i) {return i.status == status;}));
  }

  static constexpr uint8_t kActive = easynav_interfaces::msg::GoalManagerInfo::ACTIVE;
  static constexpr uint8_t kIdle = easynav_interfaces::msg::GoalManagerInfo::IDLE;

  std::shared_ptr<easynav::NavState> nav_state_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr system_node_;
  // Created in SetUp(): needs rclcpp initialized.
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  rclcpp::Subscription<easynav_interfaces::msg::GoalManagerInfo>::SharedPtr info_sub_;
  std::vector<easynav_interfaces::msg::GoalManagerInfo> infos_;
  easynav::GoalManager::SharedPtr gm_server_;
  easynav::GoalManagerClient::SharedPtr gm_client_;
};

TEST_F(GoalManagerInfoTest, NothingPublishedWhileIdle)
{
  spin_for(400ms);
  EXPECT_TRUE(infos_.empty());
}

TEST_F(GoalManagerInfoTest, FinalInfoOnClientCancel)
{
  gm_client_->send_goal(goal_at(100.0, 0.0));
  spin_for(400ms);
  ASSERT_EQ(gm_server_->get_state(), easynav::GoalManager::State::ACTIVE);
  ASSERT_FALSE(infos_.empty());
  ASSERT_EQ(infos_.back().status, kActive);

  gm_client_->cancel();
  spin_for(300ms);
  ASSERT_EQ(gm_server_->get_state(), easynav::GoalManager::State::IDLE);
  EXPECT_EQ(infos_.back().status, kIdle);
  EXPECT_TRUE(infos_.back().goals.goals.empty());
}

TEST_F(GoalManagerInfoTest, FinalInfoOnFailure)
{
  gm_client_->send_goal(goal_at(100.0, 0.0));
  spin_for(400ms);
  ASSERT_EQ(infos_.back().status, kActive);

  gm_server_->set_failed("no way");
  spin_for(200ms);
  EXPECT_EQ(infos_.back().status, kIdle);
}

TEST_F(GoalManagerInfoTest, MultiGoalMissionStaysActiveUntilTheLastGoal)
{
  nav_msgs::msg::Goals goals;
  goals.header.frame_id = "map";
  goals.goals = {goal_at(1.0, 0.0), goal_at(2.0, 0.0)};
  gm_client_->send_goals(goals);
  spin_for(400ms);
  ASSERT_EQ(infos_.back().goals.goals.size(), 2u);

  // First goal reached: still active, one goal left.
  set_robot(1.0, 0.0);
  spin_for(300ms);
  ASSERT_EQ(gm_server_->get_state(), easynav::GoalManager::State::ACTIVE);
  EXPECT_EQ(infos_.back().status, kActive);
  EXPECT_EQ(infos_.back().goals.goals.size(), 1u);
  EXPECT_EQ(count(kIdle), 0);

  // Last goal reached.
  set_robot(2.0, 0.0);
  spin_for(300ms);
  ASSERT_EQ(gm_server_->get_state(), easynav::GoalManager::State::IDLE);
  EXPECT_EQ(infos_.back().status, kIdle);
  EXPECT_TRUE(infos_.back().goals.goals.empty());
  EXPECT_NEAR(infos_.back().position_distance, 0.0, 1e-6);
}

TEST_F(GoalManagerInfoTest, FinalInfoPublishedOnlyOnce)
{
  gm_client_->send_goal(goal_at(1.0, 0.0));
  spin_for(400ms);
  set_robot(1.0, 0.0);
  spin_for(1000ms);  // Many idle cycles after the end.

  EXPECT_EQ(count(kIdle), 1);
  EXPECT_EQ(infos_.back().status, kIdle);
}

TEST_F(GoalManagerInfoTest, ConsecutiveMissionsEachEndIdle)
{
  for (int mission = 1; mission <= 3; ++mission) {
    const double x = static_cast<double>(mission);
    gm_client_->reset();  // The client must be reset after a finished mission.
    gm_client_->send_goal(goal_at(x, 0.0));
    spin_for(400ms);
    ASSERT_EQ(infos_.back().status, kActive) << "mission " << mission;

    set_robot(x, 0.0);
    spin_for(300ms);
    ASSERT_EQ(infos_.back().status, kIdle) << "mission " << mission;
    EXPECT_EQ(count(kIdle), mission);
  }
}

TEST_F(GoalManagerInfoTest, MissionFinishedOnItsFirstCycleEndsIdle)
{
  // Goal where the robot already is: finished before any ACTIVE info.
  gm_client_->send_goal(goal_at(0.0, 0.0));
  spin_for(400ms);

  ASSERT_EQ(gm_server_->get_state(), easynav::GoalManager::State::IDLE);
  ASSERT_FALSE(infos_.empty());
  EXPECT_EQ(infos_.back().status, kIdle);
  EXPECT_EQ(count(kIdle), 1);
  EXPECT_NEAR(infos_.back().position_distance, 0.0, 1e-6);
}

TEST_F(GoalManagerInfoTest, MissionCancelledBeforeAnyCycleEndsIdle)
{
  // Accept and cancel without running update() in between.
  auto spin_until = [this](easynav::GoalManager::State state) {
      const auto start = client_node_->now();
      while (client_node_->now() - start < 1s && gm_server_->get_state() != state) {
        exe_->spin_some();
        rclcpp::sleep_for(10ms);
      }
      return gm_server_->get_state() == state;
    };
  gm_client_->send_goal(goal_at(100.0, 0.0));
  ASSERT_TRUE(spin_until(easynav::GoalManager::State::ACTIVE));
  // The client can only cancel once it got the acceptance.
  const auto start = client_node_->now();
  while (client_node_->now() - start < 1s &&
    gm_client_->get_state() != easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING)
  {
    exe_->spin_some();
    rclcpp::sleep_for(10ms);
  }
  ASSERT_EQ(
    gm_client_->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);
  gm_client_->cancel();
  ASSERT_TRUE(spin_until(easynav::GoalManager::State::IDLE));
  ASSERT_TRUE(infos_.empty());

  spin_for(200ms);
  ASSERT_FALSE(infos_.empty());
  EXPECT_EQ(infos_.back().status, kIdle);
  EXPECT_EQ(count(kActive), 0);
  EXPECT_EQ(count(kIdle), 1);
}

// Height tolerance: readable default that ignores height, and a configured one that does not.
class GoalManagerHeightTest : public GoalManagerTestCase
{
protected:
  void make_server(std::vector<rclcpp::Parameter> overrides = {})
  {
    nav_state_ = std::make_shared<easynav::NavState>();
    system_node_ = rclcpp_lifecycle::LifecycleNode::make_shared(
      "height_system_node", rclcpp::NodeOptions().parameter_overrides(overrides));
    gm_server_ = easynav::GoalManager::make_shared(*nav_state_, system_node_);
    set_robot_z(0.0);
  }

  void set_robot_z(double z)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation.w = 1.0;
    nav_state_->set("robot_pose", odom);
  }

  // Sends a goal above the robot (same x/y) and runs a few cycles.
  bool reaches_goal_at_height(double goal_z)
  {
    auto client_node = rclcpp::Node::make_shared("height_client_node");
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(client_node);
    exe.add_node(system_node_->get_node_base_interface());
    auto pose_pub = client_node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 100);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = system_node_->now();
    goal.pose.position.z = goal_z;
    goal.pose.orientation.w = 1.0;
    pose_pub->publish(goal);

    // Wait for the goal, then let the manager check it.
    const auto start = client_node->now();
    while (client_node->now() - start < 500ms &&
      gm_server_->get_state() == easynav::GoalManager::State::IDLE)
    {
      exe.spin_some();
      rclcpp::sleep_for(10ms);
    }
    if (gm_server_->get_state() != easynav::GoalManager::State::ACTIVE) {
      ADD_FAILURE() << "goal at z = " << goal_z << " was not accepted";
      return false;
    }
    for (int i = 0; i < 5; ++i) {
      gm_server_->update(*nav_state_);
      exe.spin_some();
    }
    return gm_server_->get_state() == easynav::GoalManager::State::IDLE;
  }

  std::shared_ptr<easynav::NavState> nav_state_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr system_node_;
  easynav::GoalManager::SharedPtr gm_server_;
};

TEST_F(GoalManagerHeightTest, DefaultIsReadableInNavState)
{
  make_server();
  ASSERT_DOUBLE_EQ(nav_state_->get<double>("goal_tolerance.height"), 10000.0);

  // Its line in the NavState dump stays short (DBL_MAX printed 300+ digits).
  std::istringstream lines(nav_state_->debug_string());
  std::string line;
  bool found = false;
  while (std::getline(lines, line)) {
    if (line.rfind("goal_tolerance.height", 0) == 0) {
      found = true;
      EXPECT_LT(line.size(), 80u) << line;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(GoalManagerHeightTest, DefaultIgnoresHeight)
{
  make_server();
  EXPECT_TRUE(reaches_goal_at_height(0.0));
  EXPECT_TRUE(reaches_goal_at_height(3.0));
  EXPECT_TRUE(reaches_goal_at_height(-50.0));
}

TEST_F(GoalManagerHeightTest, ConfiguredToleranceLimitsHeight)
{
  make_server({rclcpp::Parameter("height_tolerance", 0.5)});
  ASSERT_DOUBLE_EQ(nav_state_->get<double>("goal_tolerance.height"), 0.5);

  EXPECT_FALSE(reaches_goal_at_height(3.0)) << "3 m above, tolerance 0.5 m";
}

TEST_F(GoalManagerHeightTest, ConfiguredToleranceAcceptsCloseHeights)
{
  make_server({rclcpp::Parameter("height_tolerance", 0.5)});
  EXPECT_TRUE(reaches_goal_at_height(0.3));
}
