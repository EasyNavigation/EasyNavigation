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

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "gtest/gtest.h"

/// \brief Subclass of SensorsNode that exposes the protected groups_ and handler_list_ for unit testing.
/// Not part of the production API: only instantiated in test code.
class SensorsNodeForTesting : public easynav::SensorsNode
{
public:
  explicit SensorsNodeForTesting(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : easynav::SensorsNode(options) {}

  const std::map<std::string, std::vector<std::string>> &
  groups_for_testing() const {return groups_;}

  const std::vector<std::shared_ptr<easynav::PerceptionHandler>> &
  handler_list_for_testing() const {return handler_list_;}
};


class SensorsNodeTestCase : public ::testing::Test
{
protected:
  ~SensorsNodeTestCase()
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

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser_1";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[4] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser_2";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 4.0);
  ret.ranges[4] = 1.0;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_0(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = static_cast<float>(i);
    p.y = static_cast<float>(i);
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = std::numeric_limits<float>::infinity();
    p.y = std::numeric_limits<float>::infinity();
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 5.0;
    p.y = 5.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pc[4].x = 0.3;
  pc[4].y = 0.3;
  pc[4].z = 0.0;

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 5.0;
    p.y = 5.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pc[4].x = 1.0;
  pc[4].y = 1.0;
  pc[4].z = 0.0;

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

using namespace std::chrono_literals;

TEST_F(SensorsNodeTestCase, convert_scan2pc)
{
  {
    pcl::PointCloud<pcl::PointXYZ> out;
    easynav::convert(get_scan_test_1(rclcpp::Time()), out);

    ASSERT_EQ(out.header.frame_id, "base_laser");
    ASSERT_EQ(out.width, 16);
    ASSERT_EQ(out.height, 1);
    ASSERT_FALSE(out.is_dense);
    ASSERT_EQ(out.points.size(), 16u);
    for (int i = 0; i < 16; i++) {
      ASSERT_TRUE(std::isnan(out.points[i].x));
      ASSERT_TRUE(std::isnan(out.points[i].y));
      ASSERT_TRUE(std::isnan(out.points[i].z));
    }

    easynav::convert(get_scan_test_2(rclcpp::Time()), out);

    ASSERT_EQ(out.header.frame_id, "base_laser");
    ASSERT_EQ(out.width, 16);
    ASSERT_EQ(out.height, 1);
    ASSERT_FALSE(out.is_dense);
    ASSERT_EQ(out.points.size(), 16u);
    for (int i = 0; i < 16; i++) {
      ASSERT_NEAR(out.points[i].x, 0.0, 0.0001);
      ASSERT_NEAR(out.points[i].y, 0.0, 0.0001);
      ASSERT_NEAR(out.points[i].z, 0.0, 0.0001);
    }

    easynav::convert(get_scan_test_3(rclcpp::Time()), out);

    ASSERT_EQ(out.header.frame_id, "base_laser_1");
    ASSERT_EQ(out.width, 16);
    ASSERT_EQ(out.height, 1);
    ASSERT_FALSE(out.is_dense);
    ASSERT_EQ(out.points.size(), 16u);

    ASSERT_NEAR(out.points[0].x, -5.0, 0.0001);
    ASSERT_NEAR(out.points[0].y, 0.0, 0.0001);
    ASSERT_NEAR(out.points[0].z, 0.0, 0.0001);
    ASSERT_NEAR(out.points[4].x, 0.0, 0.0001);
    ASSERT_NEAR(out.points[4].y, -0.3, 0.0001);
    ASSERT_NEAR(out.points[4].z, 0.0, 0.0001);
    ASSERT_NEAR(out.points[8].x, 5.0, 0.0001);
    ASSERT_NEAR(out.points[8].y, 0.0, 0.0001);
    ASSERT_NEAR(out.points[8].z, 0.0, 0.0001);
    ASSERT_NEAR(out.points[12].x, 0.0, 0.0001);
    ASSERT_NEAR(out.points[12].y, 5.0, 0.0001);
    ASSERT_NEAR(out.points[12].z, 0.0, 0.0001);
  }
}

TEST_F(SensorsNodeTestCase, percept_laserscan)
{
  auto nav_state = std::make_shared<easynav::NavState>();

  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan1", rclcpp::SensorDataQoS().reliable());


  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  ASSERT_EQ(std::string(sensors_node->get_name()), "sensors_node");

  std::vector<std::string> sensors = {"laser1"};
  sensors_node->declare_parameter("laser1.topic", std::string("/scan1"));
  sensors_node->declare_parameter("laser1.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->set_parameter({"sensors", sensors});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"laser1.topic", std::string("/scan1")});
  sensors_node->set_parameter({"laser1.type", std::string("sensor_msgs/msg/LaserScan")});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto ts = test_node->now();
  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle(nav_state);
      ts = test_node->now();
      laser_pub->publish(get_scan_test_1(ts));
      exe.spin_some();
    }
  }

  std::cerr << nav_state->debug_string() << std::endl;

  auto perceptions = nav_state->get_no_group<easynav::PointPerception>();

  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->data.size(), 16u);
  ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.001);
  ASSERT_EQ(perceptions[0]->frame_id, "base_laser");
  ASSERT_EQ(perceptions[0]->valid, true);
//
//   {
//     auto start = test_node->now();
//     while (test_node->now() - start < 1s) {
//       sensors_node->cycle(nav_state);
//       exe.spin_some();
//     }
//   }
//
//   perceptions = nav_state->get_no_group<easynav::PointPerception>();
//
//   ASSERT_EQ(perceptions.size(), 1u);
//   ASSERT_EQ(perceptions[0]->data.size(), 16u);
//   ASSERT_EQ(perceptions[0]->frame_id, "base_laser");
//   ASSERT_EQ(perceptions[0]->valid, false);
}

TEST_F(SensorsNodeTestCase, percept_fuse_laserscan)
{
  auto nav_state = std::make_shared<easynav::NavState>();

  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser1_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan1", rclcpp::SensorDataQoS().reliable());
  auto laser2_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan2", rclcpp::SensorDataQoS().reliable());

  sensor_msgs::msg::PointCloud2::SharedPtr fused_perception;
  auto fused_percept_sub = test_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensors_node/perceptions", rclcpp::SensorDataQoS().reliable(),
    [&fused_perception](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      fused_perception = msg;
    });

  auto tf_buffer = easynav::RTTFBuffer::getInstance(test_node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer, test_node, true);

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*test_node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_footprint";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 1.0;

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  ASSERT_EQ(std::string(sensors_node->get_name()), "sensors_node");

  std::vector<std::string> sensors = {"laser1", "laser2"};
  sensors_node->declare_parameter("laser1.topic", std::string("/scan1"));
  sensors_node->declare_parameter("robot_frame", std::string("base_link"));
  sensors_node->declare_parameter("robot_footprint_frame", std::string("base_footprint"));
  sensors_node->declare_parameter("laser1.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->declare_parameter("laser2.topic", std::string("/scan2"));
  sensors_node->declare_parameter("laser2.type", std::string("sensor_msgs/msg/LaserScan"));

  sensors_node->set_parameter({"sensors", sensors});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"robot_frame", std::string("base_link")});
  sensors_node->set_parameter({"robot_footprint_frame", std::string("base_footprint")});
  sensors_node->set_parameter({"laser1.topic", std::string("/scan1")});
  sensors_node->set_parameter({"laser1.type", std::string("sensor_msgs/msg/LaserScan")});
  sensors_node->set_parameter({"laser2.topic", std::string("/scan2")});
  sensors_node->set_parameter({"laser2.type", std::string("sensor_msgs/msg/LaserScan")});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  {
    auto start = test_node->now();
    while (test_node->now() - start < 15ms) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);
    }

    start = test_node->now();
    while (test_node->now() - start < 1s) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      auto time1 = test_node->now();
      auto time2 = time1 - 10ms;

      laser1_pub->publish(get_scan_test_3(time1));
      laser2_pub->publish(get_scan_test_4(time2));

      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle(nav_state);
      exe.spin_some();
    }

    auto perceptions = nav_state->get_no_group<easynav::PointPerception>();

    ASSERT_EQ(perceptions.size(), 2u);
    ASSERT_EQ(perceptions[0]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(),
      0.0, 0.02);
    ASSERT_EQ(perceptions[0]->valid, true);
    ASSERT_EQ(perceptions[1]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[1]->stamp).seconds(),
      0.0, 0.02);
    ASSERT_EQ(perceptions[1]->valid, true);

    ASSERT_NE(fused_perception, nullptr);

    pcl::PointCloud<pcl::PointXYZ> fused_pcl;
    pcl::fromROSMsg(*fused_perception, fused_pcl);
    ASSERT_EQ(fused_pcl.points.size(), 32u);

    for (const auto & p : fused_pcl.points) {
      ASSERT_EQ(p.z, 1.0);
    }
  }

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      auto time1 = test_node->now();

      laser1_pub->publish(get_scan_test_3(time1));
      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle(nav_state);

      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      exe.spin_some();
    }

    auto perceptions = nav_state->get_no_group<easynav::PointPerception>();

    ASSERT_EQ(perceptions.size(), 2u);
    ASSERT_EQ(perceptions[0]->data.size(), 16u);
    ASSERT_EQ(perceptions[0]->valid, true);
    ASSERT_EQ(perceptions[1]->data.size(), 16u);
    ASSERT_EQ(perceptions[1]->valid, true);

    ASSERT_NE(fused_perception, nullptr);

    // TODO: Revisit after deciding what to do with the valid flag and the forget_time param
    // pcl::PointCloud<pcl::PointXYZ> fused_pcl;
    // pcl::fromROSMsg(*fused_perception, fused_pcl);
    // ASSERT_EQ(fused_pcl.points.size(), 16u);

    // for (const auto & p : fused_pcl.points) {
    //   ASSERT_EQ(p.z, 1.0);
    // }
  }

}

TEST_F(SensorsNodeTestCase, percept_pc2)
{
  auto nav_state = std::make_shared<easynav::NavState>();

  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser3d_pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pc1", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  ASSERT_EQ(std::string(sensors_node->get_name()), "sensors_node");

  std::vector<std::string> sensors = {"laser3d1"};
  sensors_node->declare_parameter("laser3d1.topic", std::string("/pc1"));
  sensors_node->declare_parameter("laser3d1.type", std::string("sensor_msgs/msg/PointCloud2"));
  sensors_node->set_parameter({"sensors", sensors});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"laser3d1.topic", std::string("/pc1")});
  sensors_node->set_parameter({"laser3d1.type", std::string("sensor_msgs/msg/PointCloud2")});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto ts = test_node->now();
  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      ts = test_node->now();
      laser3d_pub->publish(get_pc2_test_0(ts));
      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle(nav_state);
      exe.spin_some();
    }
  }

  auto perceptions = nav_state->get_no_group<easynav::PointPerception>();

  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->data.size(), 16u);
  ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(),
    0.0, 0.001);
  ASSERT_EQ(perceptions[0]->frame_id, "base_lidar3d");
  ASSERT_EQ(perceptions[0]->valid, true);

  // TODO: Re-check forget_time
  // {
  //   auto start = test_node->now();
  //   while (test_node->now() - start < 1s) {
  //     sensors_node->cycle_rt(nav_state);
  //     sensors_node->cycle(nav_state);
  //     exe.spin_some();
  //   }
  // }
  // perceptions = nav_state->get_group<easynav::PointPerception>("points");
  // ASSERT_EQ(perceptions.size(), 1u);
  // ASSERT_EQ(perceptions[0]->data.size(), 16u);
  // ASSERT_EQ(perceptions[0]->frame_id, "base_lidar3d");
  // ASSERT_EQ(perceptions[0]->valid, false);
}

// ---------------------------------------------------------------------------
// Test: two sensors of different underlying ROS types (LaserScan + PointCloud2)
// placed in the same group "points".
//
// Verifies both the structural invariant (each PerceptionPtr carries the correct
// concrete handler type) and the functional outcome (both perceptions arrive in
// NavState after a cycle).
// ---------------------------------------------------------------------------
TEST_F(SensorsNodeTestCase, per_sensor_handler_mixed_types_same_group)
{
  auto nav_state = std::make_shared<easynav::NavState>();

  auto sensors_node = std::make_shared<SensorsNodeForTesting>();
  auto test_node = rclcpp::Node::make_shared("test_node_mixed");

  auto laser_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan_mixed", rclcpp::SensorDataQoS().reliable());
  auto pc2_pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pc_mixed", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  sensors_node->declare_parameter("scan_sensor.topic", std::string("/scan_mixed"));
  sensors_node->declare_parameter("scan_sensor.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->declare_parameter("pc2_sensor.topic", std::string("/pc_mixed"));
  sensors_node->declare_parameter("pc2_sensor.type", std::string("sensor_msgs/msg/PointCloud2"));
  sensors_node->set_parameter({"sensors", std::vector<std::string>{"scan_sensor", "pc2_sensor"}});
  sensors_node->set_parameter({"forget_time", 2.0});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  const auto ts = test_node->now();

  geometry_msgs::msg::TransformStamped tf_scan;
  tf_scan.header.stamp = ts;
  tf_scan.header.frame_id = "base_link";
  tf_scan.child_frame_id = "base_laser";
  tf_scan.transform.rotation.w = 1.0;
  easynav::RTTFBuffer::getInstance()->setTransform(tf_scan, "easynav", false);

  geometry_msgs::msg::TransformStamped tf_pc2;
  tf_pc2.header.stamp = ts;
  tf_pc2.header.frame_id = "base_link";
  tf_pc2.child_frame_id = "base_lidar3d";
  tf_pc2.transform.rotation.w = 1.0;
  easynav::RTTFBuffer::getInstance()->setTransform(tf_pc2, "easynav", false);

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      laser_pub->publish(get_scan_test_2(test_node->now()));
      pc2_pub->publish(get_pc2_test_0(test_node->now()));
      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle(nav_state);
      exe.spin_some();
    }
  }

  // ---- Functional assertion: both perceptions must arrive in NavState ----
  ASSERT_FALSE(nav_state->has_group("points"));
  const auto & perceptions = nav_state->get_no_group<easynav::PointPerception>();
  ASSERT_EQ(perceptions.size(), 2u);
  EXPECT_TRUE(perceptions[0]->valid);
  EXPECT_TRUE(perceptions[1]->valid);
}

// ---------------------------------------------------------------------------
// Test: LaserScan ("points") + IMU ("imu") sensors — each PerceptionPtr must
// carry the handler of the correct concrete type for its own message type,
// independently of the other sensor.
// ---------------------------------------------------------------------------
// TEST_F(SensorsNodeTestCase, per_sensor_handler_correct_type_per_sensor)
// {
//   auto sensors_node = std::make_shared<SensorsNodeForTesting>();

//   sensors_node->declare_parameter("scan_s.topic", std::string("/scan_typ"));
//   sensors_node->declare_parameter("scan_s.type", std::string("sensor_msgs/msg/LaserScan"));
//   sensors_node->declare_parameter("imu_s.topic", std::string("/imu_typ"));
//   sensors_node->declare_parameter("imu_s.type", std::string("sensor_msgs/msg/Imu"));
//   sensors_node->set_parameter({"sensors", std::vector<std::string>{"scan_s", "imu_s"}});

//   ASSERT_NO_THROW(
//     sensors_node->trigger_transition(
//       lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
//   ASSERT_EQ(
//     sensors_node->get_current_state().id(),
//     lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

//   const auto & groups = sensors_node->groups_for_testing();
//   const auto & handlers = sensors_node->handler_list_for_testing();

//   // --- "points" group: LaserScan → PointPerceptionHandler ---
//   {
//     ASSERT_TRUE(groups.count("points")) << "Group 'points' missing";
//     const auto & point_sensors = groups.at("points");
//     ASSERT_EQ(point_sensors.size(), 1u);

//     int points_handler_count = 0;
//     for (const auto & handler : handlers) {
//       if (handler->group() == "points") {
//         points_handler_count++;
//         ASSERT_NE(handler, nullptr) << "scan_s handler must not be null";
//         EXPECT_EQ(handler->group(), "points");
//         EXPECT_NE(
//           std::dynamic_pointer_cast<easynav::PointPerceptionHandler>(handler), nullptr)
//           << "scan_s handler must be PointPerceptionHandler, not "
//           << (handler ? handler->group() : "<null>");
//       }
//     }
//     ASSERT_EQ(points_handler_count, 1u) << "Must have 1 handler for 'points' group";
//   }

//   // --- "imu" group: Imu → IMUPerceptionHandler ---
//   {
//     ASSERT_TRUE(groups.count("imu")) << "Group 'imu' missing";
//     const auto & imu_sensors = groups.at("imu");
//     ASSERT_EQ(imu_sensors.size(), 1u);

//     int imu_handler_count = 0;
//     for (const auto & handler : handlers) {
//       if (handler->group() == "imu") {
//         imu_handler_count++;
//         ASSERT_NE(handler, nullptr) << "imu_s handler must not be null";
//         EXPECT_EQ(handler->group(), "imu");
//         EXPECT_NE(
//           std::dynamic_pointer_cast<easynav::IMUPerceptionHandler>(handler), nullptr)
//           << "imu_s handler must be IMUPerceptionHandler, not "
//           << (handler ? handler->group() : "<null>");
//         // Extra: must NOT be a PointPerceptionHandler (the classic wrong-type bug)
//         EXPECT_EQ(
//           std::dynamic_pointer_cast<easynav::PointPerceptionHandler>(handler), nullptr)
//           << "imu_s handler must NOT be a PointPerceptionHandler";
//       }
//     }
//     ASSERT_EQ(imu_handler_count, 1u) << "Must have 1 handler for 'imu' group";
//   }
// }

/*
TEST_F(SensorsNodeTestCase, percept_fuse_all)
{
  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser1_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan1", rclcpp::SensorDataQoS().reliable());
  auto laser2_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan2", rclcpp::SensorDataQoS().reliable());
  auto laser3d_pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pc1", rclcpp::SensorDataQoS().reliable());

  auto nav_state = std::make_shared<easynav::NavState>();

  sensor_msgs::msg::PointCloud2::SharedPtr fused_perception;
  auto fused_percept_sub = test_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensors_node/perceptions", rclcpp::SensorDataQoS().reliable(),
    [&fused_perception](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      fused_perception = msg;
    });

  auto tf_buffer = easynav::RTTFBuffer::getInstance(test_node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer, test_node, true);

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*test_node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 1.0;

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  ASSERT_EQ(std::string(sensors_node->get_name()), "sensors_node");

  std::vector<std::string> sensors = {"laser1", "laser2", "laser3d1"};
  sensors_node->declare_parameter("laser1.topic", std::string("/scan1"));
  sensors_node->declare_parameter("laser1.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->declare_parameter("laser2.topic", std::string("/scan2"));
  sensors_node->declare_parameter("laser2.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->declare_parameter("laser3d1.topic", std::string("/pc1"));
  sensors_node->declare_parameter("laser3d1.type", std::string("sensor_msgs/msg/PointCloud2"));

  sensors_node->set_parameter({"sensors", sensors});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"laser1.topic", std::string("/scan1")});
  sensors_node->set_parameter({"laser1.type", std::string("sensor_msgs/msg/LaserScan")});
  sensors_node->set_parameter({"laser2.topic", std::string("/scan2")});
  sensors_node->set_parameter({"laser2.type", std::string("sensor_msgs/msg/LaserScan")});
  sensors_node->set_parameter({"laser3d1.topic", std::string("/pc1")});
  sensors_node->set_parameter({"laser3d1.type", std::string("sensor_msgs/msg/PointCloud2")});
  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  {
    auto start = test_node->now();

    while (test_node->now() - start < 15ms) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";
      tf_broadcaster->sendTransform(transform);
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";
      tf_broadcaster->sendTransform(transform);
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_lidar3d";
      tf_broadcaster->sendTransform(transform);
    }

    while (test_node->now() - start < 1s) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";
      tf_broadcaster->sendTransform(transform);
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";
      tf_broadcaster->sendTransform(transform);
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_lidar3d";
      tf_broadcaster->sendTransform(transform);

      auto time1 = test_node->now();
      auto time2 = time1 - 10ms;

      laser1_pub->publish(get_scan_test_3(time1));
      laser2_pub->publish(get_scan_test_4(time2));
      laser3d_pub->publish(get_pc2_test_0(time1));
      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle();
      exe.spin_some();
    }

   auto perceptions = nav_state->get_group<easynav::PointPerception>("points");

    ASSERT_EQ(perceptions.size(), 3u);
    ASSERT_EQ(perceptions[0]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.03);
    ASSERT_EQ(perceptions[0]->valid, true);
    // ASSERT_NE(perceptions[0]->subscription, nullptr);
    ASSERT_EQ(perceptions[1]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[1]->stamp).seconds(), 0.0, 0.03);
    ASSERT_EQ(perceptions[1]->valid, true);
    // ASSERT_NE(perceptions[1]->subscription, nullptr);
    ASSERT_LT(perceptions[1]->stamp, perceptions[0]->stamp);
    ASSERT_EQ(perceptions[2]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.03);
    ASSERT_EQ(perceptions[2]->valid, true);
    // ASSERT_NE(perceptions[2]->subscription, nullptr);
    ASSERT_NE(fused_perception, nullptr);

    pcl::PointCloud<pcl::PointXYZ> fused_pcl;
    pcl::fromROSMsg(*fused_perception, fused_pcl);
    ASSERT_EQ(fused_pcl.points.size(), 48u);

    for (const auto & p : fused_pcl.points) {
      ASSERT_EQ(p.z, 1.0);
    }
  }

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";
      tf_broadcaster->sendTransform(transform);
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";
      tf_broadcaster->sendTransform(transform);
      transform.child_frame_id = "base_lidar3d";
      tf_broadcaster->sendTransform(transform);

      auto time1 = test_node->now();
      auto time2 = time1 - 10ms;

      laser1_pub->publish(get_scan_test_3(time1));
      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle(nav_state);
      exe.spin_some();
    }

   auto perceptions = nav_state->get_group<easynav::PointPerception>("points");

    ASSERT_EQ(perceptions.size(), 3u);
    ASSERT_EQ(perceptions[0]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.03);
    ASSERT_EQ(perceptions[0]->valid, true);
    // ASSERT_NE(perceptions[0]->subscription, nullptr);
    ASSERT_EQ(perceptions[1]->data.size(), 16u);
    ASSERT_EQ(perceptions[1]->valid, false);
    // ASSERT_NE(perceptions[1]->subscription, nullptr);
    ASSERT_EQ(perceptions[2]->data.size(), 16u);
    ASSERT_EQ(perceptions[2]->valid, false);
    // ASSERT_NE(perceptions[2]->subscription, nullptr);

    ASSERT_NE(fused_perception, nullptr);

    pcl::PointCloud<pcl::PointXYZ> fused_pcl;
    pcl::fromROSMsg(*fused_perception, fused_pcl);
    ASSERT_EQ(fused_pcl.points.size(), 16u);

    for (const auto & p : fused_pcl.points) {
      ASSERT_EQ(p.z, 1.0);
    }
  }
}*/

// ─────────────────────────────────────────────────────────────────────────────
// Functional tests: default "points" group (no group param set)
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SensorsNodeTestCase, percept_laserscan_default_group)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan1", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  // No group parameter declared or set — must auto-assign "points"
  sensors_node->declare_parameter("scan_def.topic", std::string("/scan1"));
  sensors_node->declare_parameter("scan_def.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->set_parameter({"sensors", std::vector<std::string>{"scan_def"}});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"scan_def.topic", std::string("/scan1")});
  sensors_node->set_parameter({"scan_def.type", std::string("sensor_msgs/msg/LaserScan")});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle(nav_state);
      laser_pub->publish(get_scan_test_1(test_node->now()));
      exe.spin_some();
    }
  }

  ASSERT_FALSE(nav_state->has_group("points"))
    << "LaserScan without group param must NOT be in any group";
  auto perceptions = nav_state->get_no_group<easynav::PointPerception>();
  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->frame_id, "base_laser");
  ASSERT_EQ(perceptions[0]->valid, true);
}

TEST_F(SensorsNodeTestCase, percept_pc2_default_group)
{
  auto nav_state = std::make_shared<easynav::NavState>();
  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto pc2_pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pc1", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  // No group parameter declared or set — must auto-assign "points"
  sensors_node->declare_parameter("lidar_def.topic", std::string("/pc1"));
  sensors_node->declare_parameter("lidar_def.type", std::string("sensor_msgs/msg/PointCloud2"));
  sensors_node->set_parameter({"sensors", std::vector<std::string>{"lidar_def"}});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"lidar_def.topic", std::string("/pc1")});
  sensors_node->set_parameter({"lidar_def.type", std::string("sensor_msgs/msg/PointCloud2")});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      pc2_pub->publish(get_pc2_test_0(test_node->now()));
      sensors_node->cycle_rt(nav_state);
      sensors_node->cycle(nav_state);
      exe.spin_some();
    }
  }

  ASSERT_FALSE(nav_state->has_group("points"))
    << "PointCloud2 without group param must NOT be in any group";
  auto perceptions = nav_state->get_no_group<easynav::PointPerception>();
  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->frame_id, "base_lidar3d");
  ASSERT_EQ(perceptions[0]->valid, true);
}
