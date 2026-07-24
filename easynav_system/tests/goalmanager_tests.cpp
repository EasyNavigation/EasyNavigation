// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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
