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
/// \brief The perception handlers other than points (IMU, GNSS, odometry, detections, image):
/// they reject wrong types, store each message in NavState, trigger once per message, and their
/// perceptions copy, print and report their latest stamp.

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/types/DetectionsPerception.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/ImagePerception.hpp"
#include "easynav_sensors/types/OdometryPerception.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

using namespace std::chrono_literals;

class PerceptionHandlersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("perception_handlers_test");
    cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    publisher_node_ = rclcpp::Node::make_shared("perception_publisher");
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_callback_group(cbg_, node_->get_node_base_interface());
    exe_->add_node(publisher_node_);
  }

  void TearDown() override
  {
    exe_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  // A HandlerT for "sensor" on "/<sensor>" with \p type.
  template<typename HandlerT>
  std::shared_ptr<HandlerT> make_handler(
    const std::string & sensor, const std::string & type,
    std::vector<rclcpp::Parameter> extra = {})
  {
    node_->declare_parameter(sensor + ".topic", "/" + sensor);
    node_->declare_parameter(sensor + ".type", type);
    for (const auto & param : extra) {
      node_->declare_parameter(param.get_name(), param.get_parameter_value());
    }
    auto handler = std::make_shared<HandlerT>();
    handler->initialize(node_, cbg_, sensor);
    return handler;
  }

  // Publishes \p msg on "/<sensor>" and lets the handler receive it.
  template<typename MsgT>
  void send(const std::string & sensor, MsgT msg)
  {
    auto pub = publisher_node_->create_publisher<MsgT>("/" + sensor, rclcpp::QoS(1));
    const auto start = std::chrono::steady_clock::now();
    while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() - start < 2s) {
      exe_->spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(pub->get_subscription_count(), 0u);
    msg.header.frame_id = "sensor_frame";
    msg.header.stamp = rclcpp::Time(5, 0);
    pub->publish(msg);
    const auto end = std::chrono::steady_clock::now() + 100ms;
    while (std::chrono::steady_clock::now() < end) {
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::Node::SharedPtr publisher_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::shared_ptr<easynav::NavState> nav_state_ = std::make_shared<easynav::NavState>();
};

// ─── Wrong types ────────────────────────────────────────────────────────────────────────────

TEST_F(PerceptionHandlersTest, EachHandlerRejectsOtherMessageTypes)
{
  EXPECT_THROW(
    make_handler<easynav::IMUPerceptionHandler>("a", "sensor_msgs/msg/LaserScan"),
    std::runtime_error);
  EXPECT_THROW(
    make_handler<easynav::GNSSPerceptionHandler>("b", "sensor_msgs/msg/Imu"),
    std::runtime_error);
  EXPECT_THROW(
    make_handler<easynav::OdometryPerceptionHandler>("c", "sensor_msgs/msg/Imu"),
    std::runtime_error);
  EXPECT_THROW(
    make_handler<easynav::DetectionsPerceptionsHandler>("d", "sensor_msgs/msg/Imu"),
    std::runtime_error);
  EXPECT_THROW(
    make_handler<easynav::ImagePerceptionHandler>("e", "sensor_msgs/msg/Imu"),
    std::runtime_error);
}

// ─── Each handler, with data ────────────────────────────────────────────────────────────────

TEST_F(PerceptionHandlersTest, ImuMessagesReachNavStateAndTriggerOnce)
{
  auto handler = make_handler<easynav::IMUPerceptionHandler>("imu", "sensor_msgs/msg/Imu");
  EXPECT_FALSE(handler->cycle_rt(nav_state_)) << "no data yet";
  EXPECT_FALSE(nav_state_->get<easynav::IMUPerception>("imu").valid);

  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 9.8;
  send("imu", msg);
  EXPECT_TRUE(handler->cycle_rt(nav_state_));
  EXPECT_FALSE(handler->cycle_rt(nav_state_)) << "triggers once per message";

  const auto & perception = nav_state_->get<easynav::IMUPerception>("imu");
  EXPECT_TRUE(perception.valid);
  EXPECT_DOUBLE_EQ(perception.data.linear_acceleration.x, 9.8);
  EXPECT_EQ(perception.frame_id, "sensor_frame");
  EXPECT_EQ(perception.stamp.seconds(), 5.0);
  EXPECT_EQ(handler->get_perception()->frame_id, "sensor_frame");
  EXPECT_NE(nav_state_->debug_string().find("IMUPerception"), std::string::npos);
}

TEST_F(PerceptionHandlersTest, GnssMessagesReachNavStateAndTriggerOnce)
{
  auto handler = make_handler<easynav::GNSSPerceptionHandler>(
    "gnss", "sensor_msgs/msg/NavSatFix");
  sensor_msgs::msg::NavSatFix msg;
  msg.latitude = 40.3;
  msg.longitude = -3.8;
  send("gnss", msg);
  EXPECT_TRUE(handler->cycle_rt(nav_state_));
  EXPECT_FALSE(handler->cycle_rt(nav_state_));

  const auto & perception = nav_state_->get<easynav::GNSSPerception>("gnss");
  EXPECT_TRUE(perception.valid);
  EXPECT_DOUBLE_EQ(perception.data.latitude, 40.3);
  EXPECT_DOUBLE_EQ(perception.data.longitude, -3.8);
  EXPECT_NE(nav_state_->debug_string().find("GNSSPerception"), std::string::npos);
}

TEST_F(PerceptionHandlersTest, OdometryIsStoredAsAnOdometryMessage)
{
  auto handler = make_handler<easynav::OdometryPerceptionHandler>(
    "odom", "nav_msgs/msg/Odometry");
  nav_msgs::msg::Odometry msg;
  msg.pose.pose.position.x = 1.5;
  send("odom", msg);
  EXPECT_TRUE(handler->cycle_rt(nav_state_));
  EXPECT_FALSE(handler->cycle_rt(nav_state_));

  // The message itself, so this handler can stand in for a localizer.
  EXPECT_DOUBLE_EQ(nav_state_->get<nav_msgs::msg::Odometry>("odom").pose.pose.position.x, 1.5);
  EXPECT_TRUE(handler->get_perception()->valid);
}

TEST_F(PerceptionHandlersTest, OdometryCanBeStoredUnderAnotherKey)
{
  auto handler = make_handler<easynav::OdometryPerceptionHandler>(
    "wheels", "nav_msgs/msg/Odometry", {{"wheels.nav_state_key", std::string("robot_pose")}});
  nav_msgs::msg::Odometry msg;
  msg.pose.pose.position.y = 2.5;
  send("wheels", msg);
  handler->cycle_rt(nav_state_);
  EXPECT_FALSE(nav_state_->has("wheels"));
  EXPECT_DOUBLE_EQ(
    nav_state_->get<nav_msgs::msg::Odometry>("robot_pose").pose.pose.position.y, 2.5);
}

TEST_F(PerceptionHandlersTest, DetectionsReachNavStateAndTriggerOnce)
{
  auto handler = make_handler<easynav::DetectionsPerceptionsHandler>(
    "detections", "vision_msgs/msg/Detection3DArray");
  vision_msgs::msg::Detection3DArray msg;
  msg.detections.resize(2);
  send("detections", msg);
  EXPECT_TRUE(handler->cycle_rt(nav_state_));
  EXPECT_FALSE(handler->cycle_rt(nav_state_));

  const auto & perception = nav_state_->get<easynav::DetectionsPerception>("detections");
  EXPECT_TRUE(perception.valid);
  EXPECT_EQ(perception.data.detections.size(), 2u);
  EXPECT_NE(nav_state_->debug_string().find("DetectionsPerception"), std::string::npos);
}

TEST_F(PerceptionHandlersTest, ImagesReachNavStateAndTriggerOnce)
{
  auto handler = make_handler<easynav::ImagePerceptionHandler>(
    "camera", "sensor_msgs/msg/Image");
  sensor_msgs::msg::Image msg;
  msg.encoding = "mono8";
  msg.width = 4;
  msg.height = 3;
  msg.step = 4;
  msg.data.assign(12, 7);
  send("camera", msg);
  EXPECT_TRUE(handler->cycle_rt(nav_state_));
  EXPECT_FALSE(handler->cycle_rt(nav_state_));

  const auto & perception = nav_state_->get<easynav::ImagePerception>("camera");
  EXPECT_TRUE(perception.valid);
  EXPECT_EQ(perception.data.cols, 4);
  EXPECT_EQ(perception.data.rows, 3);
}

TEST_F(PerceptionHandlersTest, AnUndecodableImageIsMarkedInvalid)
{
  auto handler = make_handler<easynav::ImagePerceptionHandler>(
    "camera", "sensor_msgs/msg/Image");
  sensor_msgs::msg::Image msg;
  msg.encoding = "not_an_encoding";
  msg.width = 2;
  msg.height = 2;
  msg.step = 2;
  msg.data.assign(4, 0);
  send("camera", msg);
  handler->cycle_rt(nav_state_);
  EXPECT_FALSE(nav_state_->get<easynav::ImagePerception>("camera").valid);
}

// ─── Perceptions ────────────────────────────────────────────────────────────────────────────

TEST_F(PerceptionHandlersTest, PerceptionsCopyEverything)
{
  easynav::IMUPerception imu;
  imu.set_data(sensor_msgs::msg::Imu(), rclcpp::Time(3, 0), "imu_frame");
  easynav::IMUPerception imu_copy(imu);
  EXPECT_EQ(imu_copy.frame_id, "imu_frame");
  EXPECT_TRUE(imu_copy.valid);
  easynav::IMUPerception imu_assigned;
  imu_assigned = imu;
  EXPECT_EQ(imu_assigned.stamp.seconds(), 3.0);
  imu_assigned = imu_assigned;  // Self-assignment is harmless.
  EXPECT_EQ(imu_assigned.frame_id, "imu_frame");

  easynav::GNSSPerception gnss;
  gnss.set_data(sensor_msgs::msg::NavSatFix(), rclcpp::Time(4, 0), "gps");
  easynav::GNSSPerception gnss_assigned;
  gnss_assigned = easynav::GNSSPerception(gnss);
  EXPECT_EQ(gnss_assigned.frame_id, "gps");

  easynav::DetectionsPerception detections;
  detections.set_data(vision_msgs::msg::Detection3DArray(), rclcpp::Time(6, 0), "cam");
  easynav::DetectionsPerception detections_assigned;
  detections_assigned = easynav::DetectionsPerception(detections);
  EXPECT_EQ(detections_assigned.stamp.seconds(), 6.0);

  easynav::ImagePerception image;
  image.set_data(cv::Mat::zeros(2, 2, CV_8UC1), rclcpp::Time(7, 0), "cam");
  easynav::ImagePerception image_assigned;
  image_assigned = easynav::ImagePerception(image);
  EXPECT_EQ(image_assigned.data.rows, 2);
  EXPECT_TRUE(image_assigned.consume_new_data());
  EXPECT_FALSE(image_assigned.consume_new_data());
}

TEST_F(PerceptionHandlersTest, TheLatestStampIsTheNewestOne)
{
  auto imu = [](int sec) {
      auto p = std::make_shared<easynav::IMUPerception>();
      p->stamp = rclcpp::Time(sec, 0, RCL_ROS_TIME);
      return p;
    };
  EXPECT_EQ(easynav::get_latest_imu_perceptions_stamp({imu(3), imu(9), imu(5)}).seconds(), 9.0);
  EXPECT_EQ(easynav::get_latest_imu_perceptions_stamp({}).nanoseconds(), 0);

  auto gnss = std::make_shared<easynav::GNSSPerception>();
  gnss->stamp = rclcpp::Time(4, 0);
  EXPECT_EQ(easynav::get_latest_gnss_perceptions_stamp({gnss}).seconds(), 4.0);

  auto odom = std::make_shared<easynav::OdometryPerception>();
  odom->stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);
  auto odom_steady = std::make_shared<easynav::OdometryPerception>();
  odom_steady->stamp = rclcpp::Time(8, 0, RCL_STEADY_TIME);
  EXPECT_EQ(
    easynav::get_latest_odometry_perceptions_stamp({odom, odom_steady}).nanoseconds(),
    8000000000) << "different clocks: compared by their nanoseconds";

  auto detections = std::make_shared<easynav::DetectionsPerception>();
  detections->stamp = rclcpp::Time(6, 0);
  EXPECT_EQ(easynav::get_latest_detections_perceptions_stamp({detections}).seconds(), 6.0);

  auto image = std::make_shared<easynav::ImagePerception>();
  image->stamp = rclcpp::Time(7, 0);
  EXPECT_EQ(easynav::get_latest_image_perceptions_stamp({image}).seconds(), 7.0);
}

// ─── Point perceptions ──────────────────────────────────────────────────────────────────────

TEST_F(PerceptionHandlersTest, ThePointHandlerRejectsOtherMessageTypes)
{
  EXPECT_THROW(
    make_handler<easynav::PointPerceptionHandler>("p", "sensor_msgs/msg/Imu"),
    std::runtime_error);
}

TEST_F(PerceptionHandlersTest, APointPerceptionConvertsToAPointCloud2)
{
  easynav::PointPerception perception;
  perception.data.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  perception.data.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
  perception.frame_id = "lidar";
  perception.stamp = rclcpp::Time(9, 0);
  const auto msg = easynav::perception_to_rosmsg(perception);
  EXPECT_EQ(msg.header.frame_id, "lidar");
  EXPECT_EQ(rclcpp::Time(msg.header.stamp).seconds(), 9.0);
  EXPECT_EQ(msg.width * msg.height, 2u);
}

TEST_F(PerceptionHandlersTest, AnOwningViewSkipsInvalidAndEmptyPerceptions)
{
  auto valid = std::make_shared<easynav::PointPerception>();
  valid->data.push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
  valid->frame_id = "base_link";
  valid->stamp = rclcpp::Time(3, 0);
  valid->valid = true;
  auto invalid = std::make_shared<easynav::PointPerception>(*valid);
  invalid->valid = false;
  invalid->stamp = rclcpp::Time(8, 0);
  auto empty = std::make_shared<easynav::PointPerception>();
  empty->valid = true;

  easynav::PointPerceptionsOpsView view(easynav::PointPerceptions{valid, invalid, empty});
  EXPECT_EQ(view.as_points().size(), 1u) << "only the valid one with data";
  EXPECT_EQ(view.get_latest_stamp().seconds(), 8.0) << "the newest stamp, valid or not";
}

TEST_F(PerceptionHandlersTest, TheLatestPointPerceptionStampIsTheNewest)
{
  auto at = [](int sec, rcl_clock_type_t clock) {
      auto p = std::make_shared<easynav::PointPerception>();
      p->stamp = rclcpp::Time(sec, 0, clock);
      return p;
    };
  EXPECT_EQ(
    easynav::get_latest_point_perceptions_stamp(
      {at(2, RCL_ROS_TIME), at(7, RCL_ROS_TIME), at(4, RCL_ROS_TIME)}).seconds(), 7.0);
  EXPECT_EQ(
    easynav::get_latest_point_perceptions_stamp(
      {at(2, RCL_ROS_TIME), at(5, RCL_STEADY_TIME)}).nanoseconds(), 5000000000);
  EXPECT_EQ(easynav::get_latest_point_perceptions_stamp({}).nanoseconds(), 0);
}

TEST_F(PerceptionHandlersTest, CollapsingFixesTheGivenAxes)
{
  auto p = std::make_shared<easynav::PointPerception>();
  p->data.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  p->frame_id = "base_link";
  p->valid = true;
  easynav::PointPerceptionsOpsView view(easynav::PointPerceptions{p});
  view.collapse({std::nan(""), std::nan(""), 0.0});
  const auto points = view.as_points();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_FLOAT_EQ(points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(points[0].z, 0.0f);

  easynav::PointPerceptionsOpsView flat(easynav::PointPerceptions{p});
  flat.collapse({5.0, 6.0, std::nan("")}, false);  // Applied now, to the view's own copy.
  const auto flat_points = flat.as_points();
  ASSERT_EQ(flat_points.size(), 1u);
  EXPECT_FLOAT_EQ(flat_points[0].x, 5.0f);
  EXPECT_FLOAT_EQ(flat_points[0].y, 6.0f);
  EXPECT_FLOAT_EQ(flat_points[0].z, 3.0f);
}
