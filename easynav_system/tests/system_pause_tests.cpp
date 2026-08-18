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

#include <vector>

#include "easynav_system/SystemNode.hpp"
#include "easynav_system/GoalManagerClient.hpp"
#include "easynav_common/types/NavState.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

// This test binary is its own process (one gtest executable per
// ament_add_gtest() entry), so it is safe to seed global parameter
// overrides at rclcpp::init() time: they apply to any node in this
// process that later declares a parameter with a matching name. This is
// what lets ControllerNode (a private member of SystemNode, not otherwise
// reachable from test code before on_configure() runs) load the
// already-registered "easynav_controller/DummyController" plugin, which is
// required to get SystemNode::system_cycle_rt()'s trigger_controller to
// ever be true -- without a loaded plugin, ControllerNode::cycle_rt()
// always returns false and SystemNode never publishes anything.
class SystemPauseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!initialized_) {
      std::vector<const char *> argv{
        "system_pause_tests",
        "--ros-args",
        "-p", "controller_types:=['dummy']",
        "-p", "dummy.plugin:=easynav_controller/DummyController",
      };
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
      initialized_ = true;
    }
  }

  void TearDown() override
  {
  }

  static bool initialized_;
};

bool SystemPauseTest::initialized_ = false;

using namespace std::chrono_literals;

TEST_F(SystemPauseTest, PublishesZeroVelocityWhenPaused)
{
  auto system_node = std::make_shared<easynav::SystemNode>();

  auto state = rclcpp_lifecycle::State();
  ASSERT_EQ(system_node->on_configure(state), easynav::SystemNode::CallbackReturnT::SUCCESS);
  ASSERT_EQ(system_node->on_activate(state), easynav::SystemNode::CallbackReturnT::SUCCESS);

  auto nav_state = system_node->get_nav_state();
  ASSERT_TRUE(nav_state->has("navigation_paused"));
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));

  auto listener_node = rclcpp::Node::make_shared("cmd_vel_listener");
  std::vector<geometry_msgs::msg::Twist> received;
  auto sub = listener_node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [&received](geometry_msgs::msg::Twist::UniquePtr msg) {
      received.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(listener_node);

  // system_cycle_rt() only publishes when vel_pub_ has a matched subscriber
  // (get_subscription_count() > 0); pub/sub discovery is asynchronous, so
  // the listener must be matched *before* the first system_cycle_rt() call
  // or that cycle's publish is silently skipped.
  {
    auto start = listener_node->now();
    while (listener_node->now() - start < 2s && sub->get_publisher_count() == 0) {
      exe.spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(sub->get_publisher_count(), 0u);
  }

  geometry_msgs::msg::TwistStamped nonzero_cmd;
  nonzero_cmd.twist.linear.x = 1.5;
  nonzero_cmd.twist.angular.z = 0.5;

  // Keep invoking system_cycle_rt() (as the RT thread would, at rt_freq)
  // until a message arrives: ControllerMethodBase's own internal RT-cycle
  // timing (isTime2RunRT(), independent of pause/resume) may make the very
  // first call(s) a no-op, so a single call is not guaranteed to publish.
  auto cycle_until_message = [&]() {
      auto start = listener_node->now();
      while (listener_node->now() - start < 1s && received.empty()) {
        system_node->system_cycle_rt();
        exe.spin_some();
        rclcpp::sleep_for(10ms);
      }
    };

  // Baseline: unpaused, the nonzero twist must be published unchanged.
  nav_state->set("cmd_vel", nonzero_cmd);
  cycle_until_message();
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(received.back().linear.x, 1.5);
  EXPECT_DOUBLE_EQ(received.back().angular.z, 0.5);

  // Paused: same nonzero cmd_vel in NavState, but the publish must be zero.
  received.clear();
  nav_state->set("navigation_paused", true);
  nav_state->set("cmd_vel", nonzero_cmd);
  cycle_until_message();
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(received.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(received.back().linear.y, 0.0);
  EXPECT_DOUBLE_EQ(received.back().linear.z, 0.0);
  EXPECT_DOUBLE_EQ(received.back().angular.x, 0.0);
  EXPECT_DOUBLE_EQ(received.back().angular.y, 0.0);
  EXPECT_DOUBLE_EQ(received.back().angular.z, 0.0);

  // Resumed: the nonzero twist must flow through again -- no regression
  // after unpausing.
  received.clear();
  nav_state->set("navigation_paused", false);
  nav_state->set("cmd_vel", nonzero_cmd);
  cycle_until_message();
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(received.back().linear.x, 1.5);
  EXPECT_DOUBLE_EQ(received.back().angular.z, 0.5);
}

TEST_F(SystemPauseTest, PauseEndToEndThroughGoalManagerClient)
{
  // The real regression test: GoalManagerClient::pause()/resume() ->
  // GoalManager::control_callback() -> GoalManager::update() -> NavState ->
  // SystemNode::system_cycle_rt(), with the rest of the cycle
  // (sensors/localizer/controller) left completely untouched.
  auto system_node = std::make_shared<easynav::SystemNode>();

  auto state = rclcpp_lifecycle::State();
  ASSERT_EQ(system_node->on_configure(state), easynav::SystemNode::CallbackReturnT::SUCCESS);
  ASSERT_EQ(system_node->on_activate(state), easynav::SystemNode::CallbackReturnT::SUCCESS);

  auto nav_state = system_node->get_nav_state();
  nav_state->set("robot_pose", nav_msgs::msg::Odometry());

  auto client_node = rclcpp::Node::make_shared("pause_client_node");
  auto gm_client = easynav::GoalManagerClient::make_shared(client_node);

  auto listener_node = rclcpp::Node::make_shared("cmd_vel_listener2");
  std::vector<geometry_msgs::msg::Twist> received;
  auto sub = listener_node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [&received](geometry_msgs::msg::Twist::UniquePtr msg) {
      received.push_back(*msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(system_node->get_node_base_interface());
  exe.add_node(client_node);
  exe.add_node(listener_node);

  // See PublishesZeroVelocityWhenPaused: wait for pub/sub discovery before
  // relying on any system_cycle_rt() publish.
  {
    auto wstart = listener_node->now();
    while (listener_node->now() - wstart < 2s && sub->get_publisher_count() == 0) {
      exe.spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(sub->get_publisher_count(), 0u);
  }

  rclcpp::Rate rate(50);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = client_node->now();
  goal.pose.position.x = 5.0;

  gm_client->send_goal(goal);

  auto start = client_node->now();
  while (client_node->now() - start < 1s &&
    gm_client->get_state() != easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING)
  {
    system_node->system_cycle();
    exe.spin_some();
    rate.sleep();
  }
  ASSERT_EQ(gm_client->get_state(), easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING);

  geometry_msgs::msg::TwistStamped nonzero_cmd;
  nonzero_cmd.twist.linear.x = 1.5;

  // See PublishesZeroVelocityWhenPaused: keep invoking system_cycle_rt()
  // until a message arrives, since ControllerMethodBase's own internal
  // RT-cycle timing may make a single call a no-op.
  auto cycle_until_message = [&]() {
      auto wstart = listener_node->now();
      while (listener_node->now() - wstart < 1s && received.empty()) {
        system_node->system_cycle_rt();
        exe.spin_some();
        rclcpp::sleep_for(10ms);
      }
    };

  // Before pausing: nonzero cmd_vel goes through.
  received.clear();
  nav_state->set("cmd_vel", nonzero_cmd);
  cycle_until_message();
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(received.back().linear.x, 1.5);

  // Pause via the real client -> server protocol.
  gm_client->pause();
  start = client_node->now();
  while (client_node->now() - start < 1s && !gm_client->is_paused()) {
    exe.spin_some();
    rate.sleep();
  }
  ASSERT_TRUE(gm_client->is_paused());

  // GoalManager::update() (the non-RT cycle) is what actually propagates
  // paused_ into NavState -- receiving the message alone does not.
  system_node->system_cycle();
  ASSERT_TRUE(nav_state->get_safe<bool>("navigation_paused"));

  received.clear();
  nav_state->set("cmd_vel", nonzero_cmd);
  cycle_until_message();
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(received.back().linear.x, 0.0);

  // Resume via the real client -> server protocol.
  gm_client->resume();
  start = client_node->now();
  while (client_node->now() - start < 1s && gm_client->is_paused()) {
    exe.spin_some();
    rate.sleep();
  }
  ASSERT_FALSE(gm_client->is_paused());

  system_node->system_cycle();
  ASSERT_FALSE(nav_state->get_safe<bool>("navigation_paused"));

  received.clear();
  nav_state->set("cmd_vel", nonzero_cmd);
  cycle_until_message();
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(received.back().linear.x, 1.5);
}
