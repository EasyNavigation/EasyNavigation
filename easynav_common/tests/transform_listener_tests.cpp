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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/qos.hpp"

#include "easynav_common/TransformListener.hpp"

using namespace std::chrono_literals;

class TransformListenerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    publisher_node_ = rclcpp::Node::make_shared("static_tf_publisher");
    // Same QoS as tf2_ros' static broadcaster (portable across distros)
    static_pub_ = publisher_node_->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf2_ros::StaticBroadcasterQoS());
  }

  void publish_static_tf(const std::string & parent, const std::string & child)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = publisher_node_->now();
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = 1.5;
    tf.transform.rotation.w = 1.0;
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(tf);
    static_pub_->publish(msg);
  }

  // Waits for the transform, spinning `node` meanwhile if given
  static bool wait_for_transform(
    tf2_ros::Buffer & buffer, const std::string & parent, const std::string & child,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node = nullptr,
    std::chrono::milliseconds timeout = 5s)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    if (node) {
      exec.add_node(node);
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (node) {
        exec.spin_some(10ms);
      } else {
        std::this_thread::sleep_for(10ms);
      }
      if (buffer.canTransform(parent, child, tf2::TimePointZero)) {
        return true;
      }
    }
    return false;
  }

  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr static_pub_;
};

TEST_F(TransformListenerTest, ReceivesTransformsThroughANode)
{
  auto node = rclcpp::Node::make_shared("listener_node");
  tf2_ros::Buffer buffer(node->get_clock());
  auto listener = easynav::make_transform_listener(buffer, node, true);
  ASSERT_NE(listener, nullptr);

  publish_static_tf("map", "odom");

  ASSERT_TRUE(wait_for_transform(buffer, "map", "odom"));
  const auto tf = buffer.lookupTransform("map", "odom", tf2::TimePointZero);
  EXPECT_DOUBLE_EQ(tf.transform.translation.x, 1.5);
}

TEST_F(TransformListenerTest, ReceivesTransformsThroughALifecycleNode)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("listener_lifecycle_node");
  tf2_ros::Buffer buffer(node->get_clock());
  auto listener = easynav::make_transform_listener(buffer, node, true);
  ASSERT_NE(listener, nullptr);

  publish_static_tf("odom", "base_link");

  EXPECT_TRUE(wait_for_transform(buffer, "odom", "base_link"));
}

TEST_F(TransformListenerTest, WithoutSpinThreadTheNodeMustBeSpun)
{
  auto node = rclcpp::Node::make_shared("listener_no_thread_node");
  tf2_ros::Buffer buffer(node->get_clock());
  auto listener = easynav::make_transform_listener(buffer, node, false);

  publish_static_tf("map", "base_footprint");

  // Nobody spins the node: the transform cannot arrive
  EXPECT_FALSE(wait_for_transform(buffer, "map", "base_footprint", nullptr, 500ms));
  // Once the node is spun, it does
  EXPECT_TRUE(
    wait_for_transform(
      buffer, "map", "base_footprint", node->get_node_base_interface()));
}

TEST_F(TransformListenerTest, ListenersOnDifferentNodesAreIndependent)
{
  auto node_a = rclcpp::Node::make_shared("listener_a");
  auto node_b = rclcpp::Node::make_shared("listener_b");
  tf2_ros::Buffer buffer_a(node_a->get_clock());
  tf2_ros::Buffer buffer_b(node_b->get_clock());
  auto listener_a = easynav::make_transform_listener(buffer_a, node_a, true);
  auto listener_b = easynav::make_transform_listener(buffer_b, node_b, true);

  publish_static_tf("world", "map");

  EXPECT_TRUE(wait_for_transform(buffer_a, "world", "map"));
  EXPECT_TRUE(wait_for_transform(buffer_b, "world", "map"));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
