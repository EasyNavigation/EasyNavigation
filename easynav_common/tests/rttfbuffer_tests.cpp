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

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/TransformListener.hpp"
#include "tf2_ros/qos.hpp"

using namespace std::chrono_literals;

namespace
{

tf2::TimePoint tp(double sec)
{
  return tf2::TimePoint(std::chrono::nanoseconds(static_cast<int64_t>(std::llround(sec * 1e9))));
}

tf2::TimePoint tp_ns(int64_t ns)
{
  return tf2::TimePoint(std::chrono::nanoseconds(ns));
}

double yaw_of(const geometry_msgs::msg::TransformStamped & msg)
{
  const auto & q = msg.transform.rotation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double sec_of(const geometry_msgs::msg::TransformStamped & msg)
{
  return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
}

// Pose after t seconds of constant body twist: linear (vx, 0, vz), angular w about z.
tf2::Transform twist_pose(double vx, double vz, double w, double t)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, w * t);
  if (std::abs(w) < 1e-12) {
    return tf2::Transform(q, tf2::Vector3(vx * t, 0, vz * t));
  }
  return tf2::Transform(
    q, tf2::Vector3(vx / w * std::sin(w * t), vx / w * (1 - std::cos(w * t)), vz * t));
}

// Same motion, but rotating about x (pitch-like loop in the y-z plane, forward along x).
tf2::Transform twist_pose_x(double vx, double vy, double w, double t)
{
  tf2::Quaternion q(tf2::Vector3(1, 0, 0), w * t);
  return tf2::Transform(
    q, tf2::Vector3(vx * t, vy / w * std::sin(w * t), vy / w * (1 - std::cos(w * t))));
}

void expect_near(
  const geometry_msgs::msg::TransformStamped & msg, const tf2::Transform & expected,
  double tol = 1e-6)
{
  EXPECT_NEAR(msg.transform.translation.x, expected.getOrigin().x(), tol);
  EXPECT_NEAR(msg.transform.translation.y, expected.getOrigin().y(), tol);
  EXPECT_NEAR(msg.transform.translation.z, expected.getOrigin().z(), tol);
  const tf2::Quaternion q(
    msg.transform.rotation.x, msg.transform.rotation.y,
    msg.transform.rotation.z, msg.transform.rotation.w);
  EXPECT_NEAR(std::abs(q.dot(expected.getRotation())), 1.0, tol);
}

}  // namespace

class RTTFBufferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    buffer_ = std::make_shared<easynav::RTTFBuffer>(
      std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
    // Window aligned to the 20 ms samples: tf2 interpolates linearly between samples, so
    // an aligned window makes the twist estimate (and the extrapolation) exact.
    buffer_->set_extrapolation_window(tf2::durationFromSec(0.04));
  }

  void set(
    const std::string & parent, const std::string & child, double t,
    const tf2::Transform & tf, bool is_static = false)
  {
    set_ns(parent, child, std::llround(t * 1e9), tf, is_static);
  }

  static geometry_msgs::msg::TransformStamped make_msg(
    const std::string & parent, const std::string & child, int64_t ns, const tf2::Transform & tf)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = parent;
    msg.child_frame_id = child;
    msg.header.stamp.sec = static_cast<int32_t>(ns / 1'000'000'000);
    msg.header.stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000);
    msg.transform.translation.x = tf.getOrigin().x();
    msg.transform.translation.y = tf.getOrigin().y();
    msg.transform.translation.z = tf.getOrigin().z();
    const auto q = tf.getRotation();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    return msg;
  }

  void set_ns(
    const std::string & parent, const std::string & child, int64_t ns,
    const tf2::Transform & tf, bool is_static = false)
  {
    buffer_->setTransform(make_msg(parent, child, ns, tf), "test", is_static);
  }

  // odom->base_link samples every `period` s in [t0, t1] with a constant body twist.
  void feed_twist(
    double vx, double vz, double w, double t0 = 10.0, double t1 = 11.0, double period = 0.02)
  {
    for (double t = t0; t <= t1 + 1e-9; t += period) {
      set("odom", "base_link", t, twist_pose(vx, vz, w, t - t0));
    }
  }

  std::shared_ptr<easynav::RTTFBuffer> buffer_;
};

TEST_F(RTTFBufferTest, EnabledByDefault)
{
  feed_twist(1.0, 0.0, 0.0);
  EXPECT_EQ(buffer_->get_max_future_extrapolation().count(), 50'000'000);
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.03)), twist_pose(1.0, 0.0, 0.0, 1.03));
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(11.06)), tf2::ExtrapolationException);
}

TEST_F(RTTFBufferTest, ZeroDisables)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::Duration(0));
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(11.01)), tf2::ExtrapolationException);
  EXPECT_FALSE(buffer_->canTransform("odom", "base_link", tp(11.01)));
}

TEST_F(RTTFBufferTest, LinearMotion)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));

  const auto tf = buffer_->lookupTransform("odom", "base_link", tp(11.03));
  EXPECT_NEAR(sec_of(tf), 11.03, 1e-9);
  EXPECT_EQ(tf.header.frame_id, "odom");
  EXPECT_EQ(tf.child_frame_id, "base_link");
  expect_near(tf, twist_pose(1.0, 0.0, 0.0, 1.03));
  EXPECT_TRUE(buffer_->canTransform("odom", "base_link", tp(11.03)));
}

TEST_F(RTTFBufferTest, PureRotation)
{
  feed_twist(0.0, 0.0, 0.8);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.05)), twist_pose(0.0, 0.0, 0.8, 1.05));
}

TEST_F(RTTFBufferTest, ArcIsExactNotChord)
{
  // Fast turn so a straight-line extrapolation would visibly miss.
  feed_twist(1.0, 0.0, 2.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.2));
  for (double ahead : {0.001, 0.01, 0.05, 0.1, 0.2}) {
    SCOPED_TRACE(ahead);
    expect_near(
      buffer_->lookupTransform("odom", "base_link", tp(11.0 + ahead)),
      twist_pose(1.0, 0.0, 2.0, 1.0 + ahead));
  }
}

TEST_F(RTTFBufferTest, UnalignedWindowErrorIsSmall)
{
  // Default 50 ms window over 20 ms samples: the older pose is tf2's chord, not the arc.
  feed_twist(1.0, 0.0, 2.0);
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.05));
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  for (double ahead : {0.001, 0.01, 0.05, 0.1}) {
    SCOPED_TRACE(ahead);
    expect_near(
      buffer_->lookupTransform("odom", "base_link", tp(11.0 + ahead)),
      twist_pose(1.0, 0.0, 2.0, 1.0 + ahead), 5e-4);
  }
}

TEST_F(RTTFBufferTest, NegativeTwist)
{
  feed_twist(-0.5, 0.0, -1.5);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.08)),
    twist_pose(-0.5, 0.0, -1.5, 1.08));
}

TEST_F(RTTFBufferTest, HelixIn3D)
{
  feed_twist(0.7, 0.3, 1.2);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.07)), twist_pose(0.7, 0.3, 1.2, 1.07));
}

TEST_F(RTTFBufferTest, RotationAboutOtherAxis)
{
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    set("odom", "base_link", t, twist_pose_x(0.4, 0.6, 1.1, t - 10.0));
  }
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.06)), twist_pose_x(0.4, 0.6, 1.1, 1.06));
}

TEST_F(RTTFBufferTest, InverseDirection)
{
  feed_twist(1.0, 0.0, 2.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  expect_near(
    buffer_->lookupTransform("base_link", "odom", tp(11.05)),
    twist_pose(1.0, 0.0, 2.0, 1.05).inverse());
}

TEST_F(RTTFBufferTest, ChainUsesLatestCommonTime)
{
  // map->odom slower (10 Hz, ends at 10.9) than odom->base_link (50 Hz, ends at 11.0).
  const tf2::Transform map_odom(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0.3), {2.0, -1.0, 0});
  for (double t = 10.0; t <= 10.9 + 1e-9; t += 0.1) {
    set("map", "odom", t, map_odom);
  }
  feed_twist(1.0, 0.0, 0.5);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.15));

  // Latest common time is 10.9: 11.0 is reachable (0.1 ahead), 11.1 is not (0.2 ahead).
  expect_near(
    buffer_->lookupTransform("map", "base_link", tp(11.0)),
    map_odom * twist_pose(1.0, 0.0, 0.5, 1.0));
  EXPECT_THROW(
    buffer_->lookupTransform("map", "base_link", tp(11.1)), tf2::ExtrapolationException);
}

TEST_F(RTTFBufferTest, StaticParentInChain)
{
  set("base_link", "laser", 0.0, tf2::Transform(tf2::Quaternion::getIdentity(), {0.2, 0, 0.3}),
    true);
  feed_twist(1.0, 0.0, 1.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  const tf2::Transform laser(tf2::Quaternion::getIdentity(), {0.2, 0, 0.3});
  expect_near(
    buffer_->lookupTransform("odom", "laser", tp(11.04)),
    twist_pose(1.0, 0.0, 1.0, 1.04) * laser);
}

TEST_F(RTTFBufferTest, StaticOnlyChainUnchanged)
{
  const tf2::Transform laser(tf2::Quaternion::getIdentity(), {0.2, 0, 0.3});
  set("base_link", "laser", 0.0, laser, true);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  const auto tf = buffer_->lookupTransform("base_link", "laser", tp(1000.0));
  expect_near(tf, laser);
}

TEST_F(RTTFBufferTest, BeyondMaxThrows)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.05));
  EXPECT_NO_THROW(buffer_->lookupTransform("odom", "base_link", tp(11.05)));  // Boundary.
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(11.051)), tf2::ExtrapolationException);
  EXPECT_FALSE(buffer_->canTransform("odom", "base_link", tp(11.051)));
}

TEST_F(RTTFBufferTest, PastAndInsideHistoryUnaffected)
{
  feed_twist(1.0, 0.0, 2.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  // Interpolation between samples keeps tf2's own result.
  easynav::RTTFBuffer plain(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    geometry_msgs::msg::TransformStamped m =
      buffer_->tf2_ros::Buffer::lookupTransform("odom", "base_link", tp(t));
    plain.setTransform(m, "test");
  }
  const auto a = buffer_->lookupTransform("odom", "base_link", tp(10.513));
  const auto b = plain.lookupTransform("odom", "base_link", tp(10.513));
  EXPECT_DOUBLE_EQ(a.transform.translation.x, b.transform.translation.x);
  EXPECT_DOUBLE_EQ(a.transform.translation.y, b.transform.translation.y);
  EXPECT_NEAR(sec_of(a), 10.513, 1e-9);

  // Before the oldest sample is still a past-extrapolation error.
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(9.9)), tf2::ExtrapolationException);
}

TEST_F(RTTFBufferTest, LatestQueryUnaffected)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  const auto tf = buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  EXPECT_NEAR(sec_of(tf), 11.0, 1e-9);
  expect_near(tf, twist_pose(1.0, 0.0, 0.0, 1.0));
}

TEST_F(RTTFBufferTest, NoHistoryThrows)
{
  set("odom", "base_link", 11.0, twist_pose(1.0, 0.0, 0.0, 0.0));
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(11.01)), tf2::ExtrapolationException);
  EXPECT_FALSE(buffer_->canTransform("odom", "base_link", tp(11.01)));
}

TEST_F(RTTFBufferTest, HistoryShorterThanWindowThrows)
{
  feed_twist(1.0, 0.0, 0.0, 10.0, 10.04);  // 40 ms of history, 50 ms window.
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.05));
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(10.05)), tf2::ExtrapolationException);
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.02));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(10.05)), twist_pose(1.0, 0.0, 0.0, 0.05));
}

TEST_F(RTTFBufferTest, WideWindowSameForConstantTwist)
{
  feed_twist(1.0, 0.0, 2.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.5));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.05)), twist_pose(1.0, 0.0, 2.0, 1.05));
}

TEST_F(RTTFBufferTest, SlowSamplesAndShortWindow)
{
  // Window smaller than the sample period: slope of the last segment.
  feed_twist(1.0, 0.0, 0.0, 10.0, 11.0, 0.1);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.09)), twist_pose(1.0, 0.0, 0.0, 1.09));
}

TEST_F(RTTFBufferTest, StoppedRobotHoldsPose)
{
  feed_twist(0.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.1)), tf2::Transform::getIdentity());
}

TEST_F(RTTFBufferTest, UnknownFrameNotSwallowed)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  EXPECT_THROW(buffer_->lookupTransform("odom", "nowhere", tp(11.01)), tf2::LookupException);
  EXPECT_FALSE(buffer_->canTransform("odom", "nowhere", tp(11.01)));
}

TEST_F(RTTFBufferTest, DisconnectedTreeNotSwallowed)
{
  feed_twist(1.0, 0.0, 0.0);
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    set("other", "thing", t, tf2::Transform::getIdentity());
  }
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  // tf2 reports the extrapolation first; it must be rethrown untouched.
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "thing", tp(11.01)), tf2::ExtrapolationException);
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "thing", tp(10.5)), tf2::ConnectivityException);
  EXPECT_FALSE(buffer_->canTransform("odom", "thing", tp(11.01)));
}

TEST_F(RTTFBufferTest, CanTransformClearsError)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  std::string err = "previous";
  EXPECT_TRUE(buffer_->canTransform("odom", "base_link", tp(11.01), &err));
  EXPECT_TRUE(err.empty());
  EXPECT_FALSE(buffer_->canTransform("odom", "base_link", tp(11.5), &err));
  EXPECT_FALSE(err.empty());
}

TEST_F(RTTFBufferTest, TimeoutOverloadsAnswerImmediately)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  const auto start = std::chrono::steady_clock::now();
  const auto tf = buffer_->lookupTransform(
    "odom", "base_link", tp(11.02), tf2::durationFromSec(1.0));
  EXPECT_TRUE(buffer_->canTransform("odom", "base_link", tp(11.02), tf2::durationFromSec(1.0)));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 200ms);
  expect_near(tf, twist_pose(1.0, 0.0, 0.0, 1.02));
}

TEST_F(RTTFBufferTest, RclcppTimeOverloadExtrapolates)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  const rclcpp::Time t(11, 30'000'000, RCL_SYSTEM_TIME);
  expect_near(
    buffer_->lookupTransform("odom", "base_link", t), twist_pose(1.0, 0.0, 0.0, 1.03));
  EXPECT_TRUE(buffer_->canTransform("odom", "base_link", t));
}

TEST_F(RTTFBufferTest, ThroughBaseReference)
{
  // Callers holding a tf2_ros::Buffer & get it only on the virtual (timeout) overloads.
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  tf2_ros::Buffer & base = *buffer_;
  expect_near(
    base.lookupTransform("odom", "base_link", tp(11.03), tf2::durationFromSec(0.0)),
    twist_pose(1.0, 0.0, 0.0, 1.03));
}

TEST_F(RTTFBufferTest, Setters)
{
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(-1.0));
  EXPECT_EQ(buffer_->get_max_future_extrapolation().count(), 0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.02));
  EXPECT_EQ(buffer_->get_max_future_extrapolation().count(), 20'000'000);

  easynav::RTTFBuffer fresh(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  EXPECT_EQ(fresh.get_extrapolation_window().count(), 50'000'000);
  const auto window = buffer_->get_extrapolation_window();
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.0));
  EXPECT_EQ(buffer_->get_extrapolation_window(), window);
  buffer_->set_extrapolation_window(tf2::durationFromSec(-0.1));
  EXPECT_EQ(buffer_->get_extrapolation_window(), window);
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.2));
  EXPECT_EQ(buffer_->get_extrapolation_window().count(), 200'000'000);
}

TEST_F(RTTFBufferTest, DisableAgain)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  EXPECT_NO_THROW(buffer_->lookupTransform("odom", "base_link", tp(11.01)));
  buffer_->set_max_future_extrapolation(tf2::Duration(0));
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(11.01)), tf2::ExtrapolationException);
}

TEST_F(RTTFBufferTest, RealEpochStampsAndSecondRollover)
{
  // Wall-clock stamps (~1.79e9 s); the extrapolated stamp crosses a second boundary.
  const int64_t t0 = 1'793'385'000'990'000'000;
  for (int i = 0; i <= 50; ++i) {
    set_ns("odom", "base_link", t0 + i * 20'000'000LL, twist_pose(1.0, 0.0, 1.0, i * 0.02));
  }
  const int64_t t_last = t0 + 1'000'000'000LL;  // ...001.990000000
  const int64_t query = t_last + 15'000'001LL;  // ...002.005000001
  const auto tf = buffer_->lookupTransform("odom", "base_link", tp_ns(query));
  EXPECT_EQ(tf.header.stamp.sec, 1'793'385'002);
  EXPECT_EQ(tf.header.stamp.nanosec, 5'000'001u);
  expect_near(tf, twist_pose(1.0, 0.0, 1.0, 1.015000001));
}

TEST_F(RTTFBufferTest, SameFrameIsIdentity)
{
  feed_twist(1.0, 0.0, 0.0);
  expect_near(
    buffer_->lookupTransform("odom", "odom", tp(11.01)), tf2::Transform::getIdentity());
  expect_near(
    buffer_->lookupTransform("odom", "odom", tp(500.0)), tf2::Transform::getIdentity());
}

TEST_F(RTTFBufferTest, FixedFrameInsideHistoryUnchanged)
{
  feed_twist(1.0, 0.0, 1.0);
  const auto tf = buffer_->lookupTransform("base_link", tp(10.8), "base_link", tp(10.3), "odom");
  EXPECT_NEAR(sec_of(tf), 10.8, 1e-9);  // tf2 stamps the target time.
  expect_near(
    tf, twist_pose(1.0, 0.0, 1.0, 0.8).inverse() * twist_pose(1.0, 0.0, 1.0, 0.3));
}

TEST_F(RTTFBufferTest, FixedFrameSourceAhead)
{
  // NavMap AMCL: a cloud slightly newer than the odometry, in the footprint as it is now.
  const tf2::Transform laser(tf2::Quaternion::getIdentity(), {0.2, 0, 0.3});
  set("base_link", "laser", 0.0, laser, true);
  feed_twist(1.0, 0.0, 1.0);
  const auto tf = buffer_->lookupTransform(
    "base_link", tf2::TimePointZero, "laser", tp(11.03), "odom");
  EXPECT_NEAR(sec_of(tf), 11.0, 1e-9);
  EXPECT_EQ(tf.header.frame_id, "base_link");
  EXPECT_EQ(tf.child_frame_id, "laser");
  expect_near(
    tf, twist_pose(1.0, 0.0, 1.0, 1.0).inverse() * twist_pose(1.0, 0.0, 1.0, 1.03) * laser);
  EXPECT_TRUE(
    buffer_->canTransform("base_link", tf2::TimePointZero, "laser", tp(11.03), "odom"));
}

TEST_F(RTTFBufferTest, FixedFrameTargetAhead)
{
  feed_twist(1.0, 0.0, 1.0);
  const auto tf = buffer_->lookupTransform("base_link", tp(11.02), "base_link", tp(10.5), "odom");
  EXPECT_NEAR(sec_of(tf), 11.02, 1e-9);
  expect_near(
    tf, twist_pose(1.0, 0.0, 1.0, 1.02).inverse() * twist_pose(1.0, 0.0, 1.0, 0.5));
}

TEST_F(RTTFBufferTest, FixedFrameBothAhead)
{
  feed_twist(1.0, 0.0, 1.0);
  expect_near(
    buffer_->lookupTransform("base_link", tp(11.04), "base_link", tp(11.01), "odom"),
    twist_pose(1.0, 0.0, 1.0, 1.04).inverse() * twist_pose(1.0, 0.0, 1.0, 1.01));
}

TEST_F(RTTFBufferTest, FixedFrameBeyondMaxThrows)
{
  feed_twist(1.0, 0.0, 0.0);
  EXPECT_THROW(
    buffer_->lookupTransform("base_link", tp(11.2), "base_link", tp(10.5), "odom"),
    tf2::ExtrapolationException);
  EXPECT_FALSE(buffer_->canTransform("base_link", tp(11.2), "base_link", tp(10.5), "odom"));
  EXPECT_THROW(
    buffer_->lookupTransform("base_link", tp(10.5), "base_link", tp(11.2), "odom"),
    tf2::ExtrapolationException);
}

TEST_F(RTTFBufferTest, FixedFrameDisabledThrows)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::Duration(0));
  EXPECT_THROW(
    buffer_->lookupTransform("base_link", tp(11.01), "base_link", tp(10.5), "odom"),
    tf2::ExtrapolationException);
}

TEST_F(RTTFBufferTest, FixedFrameTimeoutOverloads)
{
  feed_twist(1.0, 0.0, 0.0);
  const auto start = std::chrono::steady_clock::now();
  expect_near(
    buffer_->lookupTransform(
      "base_link", tp(11.02), "base_link", tp(10.5), "odom", tf2::durationFromSec(1.0)),
    twist_pose(1.0, 0.0, 0.0, 1.02).inverse() * twist_pose(1.0, 0.0, 0.0, 0.5));
  EXPECT_TRUE(
    buffer_->canTransform(
      "base_link", tp(11.02), "base_link", tp(10.5), "odom", tf2::durationFromSec(1.0)));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 200ms);
  // Through the rclcpp::Time overload too.
  expect_near(
    buffer_->lookupTransform(
      "base_link", rclcpp::Time(11, 20'000'000, RCL_SYSTEM_TIME), "base_link",
      rclcpp::Time(10, 500'000'000, RCL_SYSTEM_TIME), "odom"),
    twist_pose(1.0, 0.0, 0.0, 1.02).inverse() * twist_pose(1.0, 0.0, 0.0, 0.5));
}

TEST_F(RTTFBufferTest, ConstantAcceleration)
{
  // x = a t^2 / 2: the twist is v(t_last - w/2), so the error is a*h^2/2 + a*h*w/2.
  const double a = 2.0;
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    const double r = t - 10.0;
    set("odom", "base_link", t, tf2::Transform(
        tf2::Quaternion::getIdentity(), {0.5 * a * r * r, 0, 0}));
  }
  const double w = 0.04;
  for (double h : {0.005, 0.02, 0.05}) {
    SCOPED_TRACE(h);
    const double truth = 0.5 * a * (1.0 + h) * (1.0 + h);
    const double x = buffer_->lookupTransform("odom", "base_link", tp(11.0 + h))
      .transform.translation.x;
    EXPECT_NEAR(truth - x, a * h * h / 2.0 + a * h * w / 2.0, 1e-6);
    EXPECT_LT(std::abs(truth - x), std::abs(truth - 0.5 * a));  // Better than the latest TF.
  }
}

TEST_F(RTTFBufferTest, SuddenStop)
{
  // 1 m/s until 10.96, then stopped.
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    set("odom", "base_link", t, tf2::Transform(
        tf2::Quaternion::getIdentity(), {std::min(t, 10.96) - 10.0, 0, 0}));
  }
  // Aligned 40 ms window sees no motion: exact.
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.05)),
    tf2::Transform(tf2::Quaternion::getIdentity(), {0.96, 0, 0}));
  // 50 ms window still sees 10 ms of motion: overshoot of (0.01 / 0.05) * h.
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.05));
  const double x = buffer_->lookupTransform("odom", "base_link", tp(11.05))
    .transform.translation.x;
  EXPECT_NEAR(x - 0.96, 0.2 * 0.05, 1e-6);
}

TEST_F(RTTFBufferTest, StartFromRest)
{
  // Stopped until 10.98, then 1 m/s.
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    set("odom", "base_link", t, tf2::Transform(
        tf2::Quaternion::getIdentity(), {std::max(t - 10.98, 0.0), 0, 0}));
  }
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.05));
  const double h = 0.05;
  const double x = buffer_->lookupTransform("odom", "base_link", tp(11.0 + h))
    .transform.translation.x;
  // Twist estimated as 0.02 / 0.05 = 0.4 m/s: undershoots, still better than the latest TF.
  EXPECT_NEAR(x, 0.02 + 0.4 * h, 1e-6);
  EXPECT_LT(std::abs((0.02 + h) - x), std::abs((0.02 + h) - 0.02));
}

TEST_F(RTTFBufferTest, GapInsideWindow)
{
  // 10.96 and 10.98 lost: tf2 interpolates between 10.94 and 11.0.
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    if (std::abs(t - 10.96) < 1e-6 || std::abs(t - 10.98) < 1e-6) {continue;}
    set("odom", "base_link", t, twist_pose(1.0, 0.0, 0.0, t - 10.0));
    set("odom", "arc_link", t, twist_pose(1.0, 0.0, 2.0, t - 10.0));
  }
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.03)), twist_pose(1.0, 0.0, 0.0, 1.03));
  // On an arc, the interpolated pose is the chord of the 60 ms gap: ~0.5 mm error.
  expect_near(
    buffer_->lookupTransform("odom", "arc_link", tp(11.03)),
    twist_pose(1.0, 0.0, 2.0, 1.03), 1e-3);
}

TEST_F(RTTFBufferTest, NoiseAmplification)
{
  // Linear motion with Gaussian noise (sigma) on x: with an aligned window of k samples
  // the error is (1 + a) e_n - a e_{n-k}, a = h / w, so RMS = sigma * sqrt((1+a)^2 + a^2).
  const double sigma = 0.001;
  const double h = 0.05;
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, sigma);

  for (double w : {0.02, 0.04, 0.1}) {
    SCOPED_TRACE(w);
    double sum_sq = 0.0;
    constexpr int trials = 400;
    for (int k = 0; k < trials; ++k) {
      easynav::RTTFBuffer buf(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
      buf.set_extrapolation_window(tf2::durationFromSec(w));
      buf.set_max_future_extrapolation(tf2::durationFromSec(0.1));
      for (double t = 10.8; t <= 11.0 + 1e-9; t += 0.02) {
        buf.setTransform(
          make_msg("odom", "base_link", std::llround(t * 1e9), tf2::Transform(
            tf2::Quaternion::getIdentity(), {t - 10.0 + noise(rng), 0, 0})), "test");
      }
      const double err =
        buf.lookupTransform("odom", "base_link", tp(11.0 + h)).transform.translation.x -
        (1.0 + h);
      sum_sq += err * err;
    }
    const double rms = std::sqrt(sum_sq / trials);
    const double a = h / w;
    const double expected = sigma * std::sqrt((1 + a) * (1 + a) + a * a);
    std::printf("  window %.2f s: noise RMS %.3f mm (expected %.3f mm)\n",
      w, rms * 1e3, expected * 1e3);
    EXPECT_NEAR(rms, expected, 0.12 * expected);
    EXPECT_LT(rms, h * 1.0 / 5.0);  // Far below the 50 mm of the latest TF at 1 m/s.
  }
}

TEST_F(RTTFBufferTest, TinyWindowIsNumericallyStable)
{
  // 1 us window: the twist is scaled by 50000; small-angle math must stay accurate.
  feed_twist(1.0, 0.0, 2.0);
  buffer_->set_extrapolation_window(tf2::durationFromSec(1e-6));
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.05));
  // Last segment slope: tf2 interpolates the chord, so compare with a 20 ms window.
  easynav::RTTFBuffer ref(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    ref.setTransform(
      make_msg("odom", "base_link", std::llround(t * 1e9), twist_pose(1.0, 0.0, 2.0, t - 10.0)),
      "test");
  }
  ref.set_extrapolation_window(tf2::durationFromSec(0.02));
  const auto tiny = buffer_->lookupTransform("odom", "base_link", tp(11.05));
  const auto aligned = ref.lookupTransform("odom", "base_link", tp(11.05));
  // Same twist up to the chord-vs-arc of a 20 ms segment.
  EXPECT_NEAR(tiny.transform.translation.x, aligned.transform.translation.x, 1e-3);
  EXPECT_NEAR(tiny.transform.translation.y, aligned.transform.translation.y, 1e-3);
  EXPECT_NEAR(yaw_of(tiny), yaw_of(aligned), 1e-6);  // tf2 slerp: rotation is exact.
}

TEST_F(RTTFBufferTest, TinyRotationIsAccurate)
{
  // 1e-4 rad/s: angles in the window ~4e-6 rad; acos-based angles would lose them.
  feed_twist(0.0, 0.0, 1e-4);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.1));
  const auto tf = buffer_->lookupTransform("odom", "base_link", tp(11.1));
  EXPECT_NEAR(yaw_of(tf), 1e-4 * 1.1, 1e-10);
}

TEST_F(RTTFBufferTest, NearHalfTurnInWindow)
{
  // 75 rad/s with a 40 ms window: 3.0 rad between the two poses, still exact.
  feed_twist(1.0, 0.0, 75.0);
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.01)),
    twist_pose(1.0, 0.0, 75.0, 1.01));
}

TEST_F(RTTFBufferTest, OverHalfTurnInWindowAliases)
{
  // 100 rad/s: 4 rad in the 40 ms window is read as 4 - 2*pi (shortest arc).
  feed_twist(0.0, 0.0, 100.0);
  const double h = 0.01;
  const double aliased = (4.0 - 2.0 * M_PI) / 0.04;
  const auto tf = buffer_->lookupTransform("odom", "base_link", tp(11.0 + h));
  EXPECT_NEAR(
    std::remainder(yaw_of(tf) - (100.0 * 1.0 + aliased * h), 2.0 * M_PI), 0.0, 1e-6);
  // Shorter window resolves it: 2 rad in 20 ms.
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.02));
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.0 + h)),
    twist_pose(0.0, 0.0, 100.0, 1.0 + h));
}

TEST_F(RTTFBufferTest, NegatedQuaternionsAreSameRotation)
{
  // q and -q are the same rotation; publishers may send either.
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.02) {
    auto msg = make_msg("odom", "base_link", std::llround(t * 1e9),
      twist_pose(1.0, 0.0, 2.0, t - 10.0));
    if (std::llround(t * 50) % 2 == 0) {
      msg.transform.rotation.x *= -1; msg.transform.rotation.y *= -1;
      msg.transform.rotation.z *= -1; msg.transform.rotation.w *= -1;
    }
    buffer_->setTransform(msg, "test");
  }
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(11.03)), twist_pose(1.0, 0.0, 2.0, 1.03));
  expect_near(
    buffer_->lookupTransform("base_link", "odom", tp(11.03)),
    twist_pose(1.0, 0.0, 2.0, 1.03).inverse());
}

TEST_F(RTTFBufferTest, ClearAndTimeJumpBack)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->clear();  // What tf2_ros does on a time jump back (e.g. simulation reset).
  set("odom", "base_link", 5.0, tf2::Transform::getIdentity());
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(5.01)), tf2::ExtrapolationException);
  feed_twist(0.5, 0.0, 0.0, 5.0, 6.0);
  expect_near(
    buffer_->lookupTransform("odom", "base_link", tp(6.03)), twist_pose(0.5, 0.0, 0.0, 1.03));
  // Old future times are gone.
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(11.01)), tf2::ExtrapolationException);
}

TEST_F(RTTFBufferTest, ChainWithSingleSampleFrameThrows)
{
  // map->odom has one sample: no history to estimate its twist.
  set("map", "odom", 11.0, tf2::Transform::getIdentity());
  feed_twist(1.0, 0.0, 0.0);
  EXPECT_THROW(
    buffer_->lookupTransform("map", "base_link", tp(11.01)), tf2::ExtrapolationException);
  EXPECT_NO_THROW(buffer_->lookupTransform("map", "base_link", tp(11.0)));
}

TEST_F(RTTFBufferTest, TimeoutBeyondMaxStillWaits)
{
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.05));
  buffer_->setUsingDedicatedThread(true);
  auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(
    buffer_->canTransform("odom", "base_link", tp(11.5), tf2::durationFromSec(0.1)));
  EXPECT_GE(std::chrono::steady_clock::now() - start, 90ms);

  start = std::chrono::steady_clock::now();
  EXPECT_THROW(
    buffer_->lookupTransform("odom", "base_link", tp(11.5), tf2::durationFromSec(0.1)),
    tf2::TransformException);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 90ms);
}

TEST_F(RTTFBufferTest, TimeoutWakesWhenDataArrives)
{
  // Beyond max: the timeout overload waits and returns real data once it arrives.
  feed_twist(1.0, 0.0, 0.0);
  buffer_->set_max_future_extrapolation(tf2::durationFromSec(0.01));
  buffer_->setUsingDedicatedThread(true);
  std::thread feeder([this] {
      std::this_thread::sleep_for(50ms);
      set("odom", "base_link", 11.02, twist_pose(1.0, 0.0, 0.0, 1.02));
      set("odom", "base_link", 11.04, twist_pose(1.0, 0.0, 0.0, 1.04));
    });
  const auto tf = buffer_->lookupTransform(
    "odom", "base_link", tp(11.04), tf2::durationFromSec(2.0));
  feeder.join();
  expect_near(tf, twist_pose(1.0, 0.0, 0.0, 1.04));
}

TEST_F(RTTFBufferTest, ConcurrentSettersAndLookups)
{
  // RT thread looks up while another thread reconfigures and inserts TFs.
  feed_twist(1.0, 0.0, 0.0);
  std::atomic<bool> stop {false};
  std::atomic<int> ok {0}, refused {0}, wrong {0};

  std::thread reader([&] {
      while (!stop) {
        try {
          const auto tf = buffer_->lookupTransform("odom", "base_link", tp(11.01));
          if (std::abs(tf.transform.translation.x - 1.01) < 1e-6) {++ok;} else {++wrong;}
        } catch (const tf2::ExtrapolationException &) {
          ++refused;
        }
      }
    });
  std::thread config([&] {
      for (int i = 0; !stop; ++i) {
        buffer_->set_max_future_extrapolation(tf2::durationFromSec(i % 2 ? 0.1 : 0.0));
        buffer_->set_extrapolation_window(tf2::durationFromSec(i % 3 ? 0.02 : 0.04));
        std::this_thread::yield();
      }
    });
  std::thread writer([&] {
      for (int i = 0; !stop; ++i) {
        set("base_link", "laser" + std::to_string(i % 10), 11.0, tf2::Transform::getIdentity(),
          true);
      }
    });

  std::this_thread::sleep_for(300ms);
  stop = true;
  reader.join();
  config.join();
  writer.join();
  EXPECT_GT(ok.load(), 0);
  EXPECT_GT(refused.load(), 0);
  EXPECT_EQ(wrong.load(), 0);
}

TEST_F(RTTFBufferTest, ThroughTransformListener)
{
  // End to end: /tf messages received by a listener, then extrapolated.
  auto pub_node = rclcpp::Node::make_shared("rttfbuffer_test_pub");
  auto sub_node = rclcpp::Node::make_shared("rttfbuffer_test_sub");
  auto pub = pub_node->create_publisher<tf2_msgs::msg::TFMessage>("/tf",
    tf2_ros::DynamicBroadcasterQoS());
  auto listener = easynav::make_transform_listener(*buffer_, sub_node, false);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(sub_node);
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(10ms);
  }
  ASSERT_GT(pub->get_subscription_count(), 0u);

  for (int i = 0; i <= 50; ++i) {
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(make_msg("rtl_odom", "rtl_base", 10'000'000'000LL + i * 20'000'000LL,
      twist_pose(1.0, 0.0, 1.0, i * 0.02)));
    pub->publish(msg);
  }
  while (!buffer_->tf2_ros::Buffer::canTransform("rtl_odom", "rtl_base", tp(11.0)) &&
    std::chrono::steady_clock::now() < deadline)
  {
    exec.spin_some(10ms);
  }
  // Let late messages arrive so the whole window is there.
  for (int i = 0; i < 10; ++i) {
    exec.spin_some(10ms);
                                                     }

  expect_near(
    buffer_->lookupTransform("rtl_odom", "rtl_base", tp(11.03)), twist_pose(1.0, 0.0, 1.0, 1.03));
}

// Cost per call and accuracy, with and without extrapolation. Prints a report; only
// accuracy is asserted (timings depend on the machine).
TEST_F(RTTFBufferTest, Measurements)
{
  // map->odom at 10 Hz, odom->base_link at 50 Hz (1 m/s, 1 rad/s), static base_link->laser.
  const tf2::Transform map_odom(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0.3), {2.0, -1.0, 0});
  const tf2::Transform laser(tf2::Quaternion::getIdentity(), {0.2, 0, 0.3});
  for (double t = 10.0; t <= 11.0 + 1e-9; t += 0.1) {
    set("map", "odom", t, map_odom);
  }
  feed_twist(1.0, 0.0, 1.0);
  set("base_link", "laser", 0.0, laser, true);
  buffer_->set_extrapolation_window(tf2::durationFromSec(0.05));  // Default, unaligned.

  constexpr int N = 20000;
  auto ns_per_call = [](const std::function<void()> & f) {
      for (int i = 0; i < N / 10; ++i) {
        f();
      }                                        // Warm up.
      const auto start = std::chrono::steady_clock::now();
      for (int i = 0; i < N; ++i) {
        f();
      }
      return std::chrono::duration<double, std::nano>(
        std::chrono::steady_clock::now() - start).count() / N;
    };
  auto set_enabled = [this](bool on) {
      buffer_->set_max_future_extrapolation(tf2::durationFromSec(on ? 0.05 : 0.0));
    };
  // What callers do today without extrapolation (PointPerception): catch, use the latest.
  auto lookup_or_latest = [this](const tf2::TimePoint & t) {
      try {
        return buffer_->lookupTransform("map", "laser", t);
      } catch (const tf2::ExtrapolationException &) {
        return buffer_->lookupTransform("map", "laser", tf2::TimePointZero);
      }
    };

  struct Row {const char * what; double off; double on;};
  std::vector<Row> rows;
  auto measure = [&](const char * what, const std::function<void()> & f) {
      set_enabled(false);
      const double off = ns_per_call(f);
      set_enabled(true);
      const double on = ns_per_call(f);
      rows.push_back({what, off, on});
    };

  measure("lookup inside history", [&] {buffer_->lookupTransform("map", "laser", tp(10.513));});
  measure("lookup latest (time 0)", [&] {
      buffer_->lookupTransform("map", "laser", tf2::TimePointZero);
    });
  measure("canTransform inside history", [&] {buffer_->canTransform("map", "laser", tp(10.5));});
  measure("lookup +10 ms (off: catch + latest)", [&] {lookup_or_latest(tp(11.01));});
  measure("canTransform +10 ms", [&] {buffer_->canTransform("map", "laser", tp(11.01));});
  measure("lookup +100 ms, beyond max (throws)", [&] {
      try {
        buffer_->lookupTransform("map", "laser", tp(11.1));
      } catch (const tf2::ExtrapolationException &) {
      }
    });

  std::printf("\n  Cost per call (ns), extrapolation off / on (max 50 ms)\n");
  std::printf("  %-40s %10s %10s\n", "case", "off", "on");
  for (const auto & r : rows) {
    std::printf("  %-40s %10.0f %10.0f\n", r.what, r.off, r.on);
  }

  std::printf("\n  Error vs ground truth, map->laser, 1 m/s + 1 rad/s\n");
  std::printf(
    "  %-10s %14s %14s %14s %14s\n", "ahead(ms)", "latest(mm)", "extrap(mm)",
    "latest(mrad)", "extrap(mrad)");
  for (double ahead : {0.001, 0.005, 0.01, 0.02, 0.05}) {
    const tf2::TimePoint t = tp(11.0 + ahead);
    const tf2::Transform truth = map_odom * twist_pose(1.0, 0.0, 1.0, 1.0 + ahead) * laser;
    auto err = [&](const geometry_msgs::msg::TransformStamped & m, double & mm, double & mrad) {
        const tf2::Vector3 p(
          m.transform.translation.x, m.transform.translation.y, m.transform.translation.z);
        const tf2::Quaternion q(
          m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
          m.transform.rotation.w);
        mm = (p - truth.getOrigin()).length() * 1e3;
        mrad = q.angleShortestPath(truth.getRotation()) * 1e3;
      };
    double l_mm, l_mrad, e_mm, e_mrad;
    set_enabled(false);
    err(lookup_or_latest(t), l_mm, l_mrad);
    set_enabled(true);
    err(buffer_->lookupTransform("map", "laser", t), e_mm, e_mrad);
    std::printf(
      "  %-10.0f %14.3f %14.4f %14.3f %14.4f\n", ahead * 1e3, l_mm, e_mm, l_mrad, e_mrad);
    EXPECT_LT(e_mm, l_mm);
    EXPECT_LT(e_mm, 0.5);
    EXPECT_LT(e_mrad, 0.5);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
