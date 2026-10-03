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
/// \brief SensorsNode: perceptions older than "forget_time" are invalidated and reported.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/PointPerception.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class SensorsDataAgeTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}

  void TearDown() override
  {
    exe_.reset();
    node_.reset();
    publisher_node_.reset();
    rclcpp::shutdown();
  }

  // A sensors_node with \p sensors (name -> type) on topics "/<name>", configured and active.
  bool start(
    const std::vector<std::pair<std::string, std::string>> & sensors, double forget_time = 0.3)
  {
    std::vector<std::string> names;
    std::vector<rclcpp::Parameter> params {{"forget_time", forget_time}};
    for (const auto & [name, type] : sensors) {
      names.push_back(name);
      params.emplace_back(name + ".topic", "/" + name);
      params.emplace_back(name + ".type", type);
    }
    params.emplace_back("sensors", names);
    node_ = std::make_shared<easynav::SensorsNode>(
      rclcpp::NodeOptions().parameter_overrides(params));
    if (node_->trigger_transition(Transition::TRANSITION_CONFIGURE).id() !=
      State::PRIMARY_STATE_INACTIVE)
    {
      return false;
    }
    node_->trigger_transition(Transition::TRANSITION_ACTIVATE);

    publisher_node_ = rclcpp::Node::make_shared("sensor_publisher");
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_callback_group(node_->get_real_time_cbg(), node_->get_node_base_interface());
    exe_->add_node(publisher_node_);
    return true;
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher(const std::string & name)
  {
    auto pub = publisher_node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/" + name, rclcpp::SensorDataQoS().reliable());
    wait_for_subscriber(pub);
    return pub;
  }

  template<typename PubT>
  void wait_for_subscriber(const PubT & pub)
  {
    const auto start = std::chrono::steady_clock::now();
    while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() - start < 2s) {
      exe_->spin_some();
      rclcpp::sleep_for(10ms);
    }
    ASSERT_GT(pub->get_subscription_count(), 0u);
  }

  // A scan stamped \p age ago, received.
  void send_scan(
    const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr & pub,
    std::chrono::milliseconds age = 0ms)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "base_link";
    scan.header.stamp = node_->now() - rclcpp::Duration(age);
    scan.angle_min = -0.1;
    scan.angle_max = 0.1;
    scan.angle_increment = 0.1;
    scan.range_min = 0.1;
    scan.range_max = 10.0;
    scan.ranges = {1.0, 1.0, 1.0};
    pub->publish(scan);
    spin_for(50ms);
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      exe_->spin_some();
      rclcpp::sleep_for(5ms);
    }
  }

  void cycle() {node_->cycle_rt(nav_state_);}

  std::optional<DiagnosticStatus> diagnostic() const
  {
    if (!nav_state_->has("diagnostics.sensors")) {return std::nullopt;}
    return nav_state_->get<DiagnosticStatus>("diagnostics.sensors");
  }

  std::string value(const std::string & key) const
  {
    const auto status = diagnostic().value();  // A copy: the loop must not use a temporary.
    for (const auto & kv : status.values) {
      if (kv.key == key) {return kv.value;}
    }
    return "<missing>";
  }

  bool valid(const std::string & sensor) const
  {
    return nav_state_->get<easynav::PointPerception>(sensor).valid;
  }

  easynav::SensorsNode::SharedPtr node_;
  rclcpp::Node::SharedPtr publisher_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::shared_ptr<easynav::NavState> nav_state_ = std::make_shared<easynav::NavState>();
};

TEST_F(SensorsDataAgeTest, InvalidForgetTimesFailToConfigure)
{
  for (const double forget_time : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_FALSE(start({}, forget_time)) << forget_time;
    exe_.reset();
    node_.reset();
  }
}

TEST_F(SensorsDataAgeTest, ASensorWithoutDataYetIsReported)
{
  ASSERT_TRUE(start({{"laser1", "sensor_msgs/msg/LaserScan"}}));
  cycle();
  ASSERT_TRUE(diagnostic());
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::WARN);
  EXPECT_EQ(diagnostic()->hardware_id, "sensors_node");
  EXPECT_EQ(value("no_data"), "laser1");
  EXPECT_EQ(value("stale"), "");
  EXPECT_FALSE(valid("laser1"));
}

TEST_F(SensorsDataAgeTest, FreshDataIsValidAndReportedOk)
{
  ASSERT_TRUE(start({{"laser1", "sensor_msgs/msg/LaserScan"}}));
  auto pub = scan_publisher("laser1");
  send_scan(pub);
  cycle();
  EXPECT_TRUE(valid("laser1"));
  ASSERT_TRUE(diagnostic());
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::OK);
}

TEST_F(SensorsDataAgeTest, DataOlderThanForgetTimeIsInvalidatedAndValidAgainWithNewData)
{
  ASSERT_TRUE(start({{"laser1", "sensor_msgs/msg/LaserScan"}}, 0.3));
  auto pub = scan_publisher("laser1");
  send_scan(pub);
  cycle();
  ASSERT_TRUE(valid("laser1"));

  rclcpp::sleep_for(200ms);
  cycle();
  EXPECT_TRUE(valid("laser1")) << "0.25 s old: still within forget_time";

  rclcpp::sleep_for(150ms);  // The sensor goes silent for longer than forget_time.
  cycle();
  EXPECT_FALSE(valid("laser1"));
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::WARN);
  EXPECT_EQ(value("stale"), "laser1");
  EXPECT_NE(diagnostic()->message.find("laser1"), std::string::npos);

  send_scan(pub);  // Back.
  cycle();
  EXPECT_TRUE(valid("laser1"));
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::OK);
}

TEST_F(SensorsDataAgeTest, DataTooOldOnArrivalIsNeverUsed)
{
  ASSERT_TRUE(start({{"laser1", "sensor_msgs/msg/LaserScan"}}, 0.3));
  auto pub = scan_publisher("laser1");
  send_scan(pub, 1000ms);  // A delayed sensor: stamped 1 s ago.
  cycle();
  EXPECT_FALSE(valid("laser1"));
  EXPECT_EQ(value("stale"), "laser1");
}

TEST_F(SensorsDataAgeTest, OnlyTheSilentSensorIsInvalidated)
{
  ASSERT_TRUE(start(
      {{"front", "sensor_msgs/msg/LaserScan"}, {"rear", "sensor_msgs/msg/LaserScan"},
        {"imu", "sensor_msgs/msg/Imu"}}, 0.3));
  auto front = scan_publisher("front");
  auto rear = scan_publisher("rear");
  send_scan(front);
  send_scan(rear);
  cycle();
  EXPECT_EQ(value("no_data"), "imu") << "an IMU is checked too";

  for (int i = 0; i < 8; ++i) {  // Only the front keeps publishing.
    send_scan(front);
  }
  cycle();
  EXPECT_TRUE(valid("front"));
  EXPECT_FALSE(valid("rear"));
  EXPECT_EQ(value("stale"), "rear");
  EXPECT_EQ(value("no_data"), "imu");
}

TEST_F(SensorsDataAgeTest, TheDiagnosticIsOnlyWrittenOnChanges)
{
  ASSERT_TRUE(start({{"laser1", "sensor_msgs/msg/LaserScan"}}));
  cycle();
  auto marked = diagnostic().value();
  marked.message = "marker";
  nav_state_->set("diagnostics.sensors", marked);

  cycle();
  cycle();
  EXPECT_EQ(diagnostic()->message, "marker") << "unchanged: not written again";
}

TEST_F(SensorsDataAgeTest, NoSensorsIsOk)
{
  ASSERT_TRUE(start({}));
  cycle();
  ASSERT_TRUE(diagnostic());
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::OK);
}

TEST_F(SensorsDataAgeTest, ReconfiguringStartsOver)
{
  ASSERT_TRUE(start({{"laser1", "sensor_msgs/msg/LaserScan"}}));
  cycle();
  ASSERT_EQ(diagnostic()->level, DiagnosticStatus::WARN);

  node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  node_->set_parameter(rclcpp::Parameter("sensors", std::vector<std::string>{}));
  ASSERT_EQ(node_->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
    State::PRIMARY_STATE_INACTIVE);
  node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
  cycle();
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::OK) << "reported again after reconfiguring";
}
