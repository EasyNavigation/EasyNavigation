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

// Tests for the mutex-guarded copy/assignment/set_data/consume_new_data (and, for
// ImagePerception, mark_invalid) added to the "Simple" perception types
// (IMU/GNSS/Detections/Image). These types are shared into NavState via the
// shared_ptr overload of NavState::set(), so the *same* live object a subscription
// callback mutates can be read cross-thread (e.g. via NavState::get_safe(), which
// copy-constructs it). Without their own locking, that copy could observe a
// partially-written object. See bugs.md finding #5.

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "easynav_sensors/types/DetectionsPerception.hpp"
#include "easynav_sensors/types/ImagePerception.hpp"

#include "gtest/gtest.h"

class PerceptionLockingTestCase : public ::testing::Test
{
protected:
  ~PerceptionLockingTestCase()
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

  bool initialized {false};
};

namespace
{
// Builds a frame_id long enough that std::string reallocates on every write
// (well past libstdc++'s ~15-byte SSO buffer), so a torn read is actually
// observable rather than accidentally safe due to small-string optimization.
std::string long_frame_id(int i)
{
  return "frame_" + std::string(64, static_cast<char>('a' + (i % 26))) + "_" + std::to_string(i);
}
}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// IMUPerception
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(PerceptionLockingTestCase, ImuSetDataUpdatesAllFields)
{
  easynav::IMUPerception p;
  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 1.0;
  msg.linear_acceleration.y = 2.0;
  msg.linear_acceleration.z = 3.0;

  const rclcpp::Time stamp(123, 456, RCL_ROS_TIME);
  p.set_data(msg, stamp, "imu_link");

  EXPECT_EQ(p.frame_id, "imu_link");
  EXPECT_EQ(p.stamp, stamp);
  EXPECT_TRUE(p.valid);
  EXPECT_TRUE(p.new_data);
  EXPECT_DOUBLE_EQ(p.data.linear_acceleration.x, 1.0);
  EXPECT_DOUBLE_EQ(p.data.linear_acceleration.y, 2.0);
  EXPECT_DOUBLE_EQ(p.data.linear_acceleration.z, 3.0);
}

TEST_F(PerceptionLockingTestCase, ImuConsumeNewDataReturnsThenClears)
{
  easynav::IMUPerception p;
  sensor_msgs::msg::Imu msg;
  p.set_data(msg, rclcpp::Time(0, 0, RCL_ROS_TIME), "imu_link");

  EXPECT_TRUE(p.consume_new_data());
  EXPECT_FALSE(p.consume_new_data())
    << "consume_new_data() must clear new_data; a second call must return false";
  EXPECT_FALSE(p.new_data);
}

TEST_F(PerceptionLockingTestCase, ImuCopyConstructorIsIndependentSnapshot)
{
  easynav::IMUPerception p;
  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 1.0;
  p.set_data(msg, rclcpp::Time(1, 0, RCL_ROS_TIME), "imu_link_1");

  easynav::IMUPerception snapshot(p);

  sensor_msgs::msg::Imu msg2;
  msg2.linear_acceleration.x = 99.0;
  p.set_data(msg2, rclcpp::Time(2, 0, RCL_ROS_TIME), "imu_link_2");

  EXPECT_EQ(snapshot.frame_id, "imu_link_1");
  EXPECT_DOUBLE_EQ(snapshot.data.linear_acceleration.x, 1.0);
  EXPECT_EQ(p.frame_id, "imu_link_2");
  EXPECT_DOUBLE_EQ(p.data.linear_acceleration.x, 99.0);
}

TEST_F(PerceptionLockingTestCase, ImuCopyAssignmentIsIndependentSnapshot)
{
  easynav::IMUPerception p;
  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 1.0;
  p.set_data(msg, rclcpp::Time(1, 0, RCL_ROS_TIME), "imu_link_1");

  easynav::IMUPerception snapshot;
  snapshot = p;

  sensor_msgs::msg::Imu msg2;
  msg2.linear_acceleration.x = 99.0;
  p.set_data(msg2, rclcpp::Time(2, 0, RCL_ROS_TIME), "imu_link_2");

  EXPECT_EQ(snapshot.frame_id, "imu_link_1");
  EXPECT_DOUBLE_EQ(snapshot.data.linear_acceleration.x, 1.0);
  EXPECT_EQ(p.frame_id, "imu_link_2");
}

TEST_F(PerceptionLockingTestCase, ImuSelfCopyAssignmentIsNoop)
{
  easynav::IMUPerception p;
  sensor_msgs::msg::Imu msg;
  msg.linear_acceleration.x = 1.0;
  p.set_data(msg, rclcpp::Time(1, 0, RCL_ROS_TIME), "imu_link");

  p = p;  // NOLINT(clang-diagnostic-self-assign-overloaded)

  EXPECT_EQ(p.frame_id, "imu_link");
  EXPECT_DOUBLE_EQ(p.data.linear_acceleration.x, 1.0);
}

// Reproduces the hazard set_data()/the copy constructor are meant to close: a
// writer thread repeatedly overwriting frame_id (forcing std::string
// reallocation) while a reader thread concurrently copy-constructs (as
// NavState::get_safe() would). Every observed snapshot must be internally
// consistent — the frame_id string must never be seen half-written/corrupted,
// and its numeric suffix must always parse back to a value within range.
TEST_F(PerceptionLockingTestCase, ImuConcurrentSetDataDuringCopyConstruction)
{
  easynav::IMUPerception p;
  std::atomic<bool> start_flag{false};
  std::atomic<bool> stop_flag{false};
  std::atomic<int> reads{0};

  auto writer = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      for (int i = 0; i < 3000; ++i) {
        sensor_msgs::msg::Imu msg;
        msg.linear_acceleration.x = static_cast<double>(i);
        p.set_data(msg, rclcpp::Time(i, 0, RCL_ROS_TIME), long_frame_id(i));
      }
      stop_flag.store(true);
    };

  auto reader = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      while (!stop_flag.load()) {
        easynav::IMUPerception snapshot(p);
        std::this_thread::yield();
        // A torn/corrupted frame_id would very likely not match this prefix.
        EXPECT_TRUE(snapshot.frame_id.empty() || snapshot.frame_id.substr(0, 6) == "frame_")
          << "corrupted frame_id: " << snapshot.frame_id;
        reads.fetch_add(1);
      }
    };

  std::vector<std::thread> threads;
  threads.emplace_back(writer);
  threads.emplace_back(reader);
  start_flag.store(true);
  for (auto & t : threads) {
    t.join();
  }

  EXPECT_GT(reads.load(), 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// GNSSPerception
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(PerceptionLockingTestCase, GnssSetDataUpdatesAllFields)
{
  easynav::GNSSPerception p;
  sensor_msgs::msg::NavSatFix msg;
  msg.latitude = 40.4;
  msg.longitude = -3.7;
  msg.altitude = 650.0;

  const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
  p.set_data(msg, stamp, "gnss_link");

  EXPECT_EQ(p.frame_id, "gnss_link");
  EXPECT_EQ(p.stamp, stamp);
  EXPECT_TRUE(p.valid);
  EXPECT_TRUE(p.new_data);
  EXPECT_DOUBLE_EQ(p.data.latitude, 40.4);
  EXPECT_DOUBLE_EQ(p.data.longitude, -3.7);
  EXPECT_DOUBLE_EQ(p.data.altitude, 650.0);
}

TEST_F(PerceptionLockingTestCase, GnssConsumeNewDataReturnsThenClears)
{
  easynav::GNSSPerception p;
  sensor_msgs::msg::NavSatFix msg;
  p.set_data(msg, rclcpp::Time(0, 0, RCL_ROS_TIME), "gnss_link");

  EXPECT_TRUE(p.consume_new_data());
  EXPECT_FALSE(p.consume_new_data());
}

TEST_F(PerceptionLockingTestCase, GnssCopyConstructorIsIndependentSnapshot)
{
  easynav::GNSSPerception p;
  sensor_msgs::msg::NavSatFix msg;
  msg.latitude = 1.0;
  p.set_data(msg, rclcpp::Time(1, 0, RCL_ROS_TIME), "gnss_link_1");

  easynav::GNSSPerception snapshot(p);

  sensor_msgs::msg::NavSatFix msg2;
  msg2.latitude = 99.0;
  p.set_data(msg2, rclcpp::Time(2, 0, RCL_ROS_TIME), "gnss_link_2");

  EXPECT_EQ(snapshot.frame_id, "gnss_link_1");
  EXPECT_DOUBLE_EQ(snapshot.data.latitude, 1.0);
  EXPECT_DOUBLE_EQ(p.data.latitude, 99.0);
}

TEST_F(PerceptionLockingTestCase, GnssCopyAssignmentIsIndependentSnapshot)
{
  easynav::GNSSPerception p;
  sensor_msgs::msg::NavSatFix msg;
  msg.latitude = 1.0;
  p.set_data(msg, rclcpp::Time(1, 0, RCL_ROS_TIME), "gnss_link_1");

  easynav::GNSSPerception snapshot;
  snapshot = p;

  sensor_msgs::msg::NavSatFix msg2;
  msg2.latitude = 99.0;
  p.set_data(msg2, rclcpp::Time(2, 0, RCL_ROS_TIME), "gnss_link_2");

  EXPECT_DOUBLE_EQ(snapshot.data.latitude, 1.0);
  EXPECT_DOUBLE_EQ(p.data.latitude, 99.0);
}

TEST_F(PerceptionLockingTestCase, GnssConcurrentSetDataDuringCopyConstruction)
{
  easynav::GNSSPerception p;
  std::atomic<bool> start_flag{false};
  std::atomic<bool> stop_flag{false};
  std::atomic<int> reads{0};

  auto writer = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      for (int i = 0; i < 3000; ++i) {
        sensor_msgs::msg::NavSatFix msg;
        msg.latitude = static_cast<double>(i);
        p.set_data(msg, rclcpp::Time(i, 0, RCL_ROS_TIME), long_frame_id(i));
      }
      stop_flag.store(true);
    };

  auto reader = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      while (!stop_flag.load()) {
        easynav::GNSSPerception snapshot(p);
        std::this_thread::yield();
        EXPECT_TRUE(snapshot.frame_id.empty() || snapshot.frame_id.substr(0, 6) == "frame_")
          << "corrupted frame_id: " << snapshot.frame_id;
        reads.fetch_add(1);
      }
    };

  std::vector<std::thread> threads;
  threads.emplace_back(writer);
  threads.emplace_back(reader);
  start_flag.store(true);
  for (auto & t : threads) {
    t.join();
  }

  EXPECT_GT(reads.load(), 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// DetectionsPerception
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(PerceptionLockingTestCase, DetectionsSetDataUpdatesAllFields)
{
  easynav::DetectionsPerception p;
  vision_msgs::msg::Detection3DArray msg;
  msg.detections.resize(2);

  const rclcpp::Time stamp(5, 0, RCL_ROS_TIME);
  p.set_data(msg, stamp, "camera_link");

  EXPECT_EQ(p.frame_id, "camera_link");
  EXPECT_EQ(p.stamp, stamp);
  EXPECT_TRUE(p.valid);
  EXPECT_TRUE(p.new_data);
  EXPECT_EQ(p.data.detections.size(), 2u);
}

TEST_F(PerceptionLockingTestCase, DetectionsConsumeNewDataReturnsThenClears)
{
  easynav::DetectionsPerception p;
  vision_msgs::msg::Detection3DArray msg;
  p.set_data(msg, rclcpp::Time(0, 0, RCL_ROS_TIME), "camera_link");

  EXPECT_TRUE(p.consume_new_data());
  EXPECT_FALSE(p.consume_new_data());
}

TEST_F(PerceptionLockingTestCase, DetectionsCopyConstructorIsIndependentSnapshot)
{
  easynav::DetectionsPerception p;
  vision_msgs::msg::Detection3DArray msg;
  msg.detections.resize(1);
  p.set_data(msg, rclcpp::Time(1, 0, RCL_ROS_TIME), "camera_link_1");

  easynav::DetectionsPerception snapshot(p);

  vision_msgs::msg::Detection3DArray msg2;
  msg2.detections.resize(5);
  p.set_data(msg2, rclcpp::Time(2, 0, RCL_ROS_TIME), "camera_link_2");

  EXPECT_EQ(snapshot.frame_id, "camera_link_1");
  EXPECT_EQ(snapshot.data.detections.size(), 1u);
  EXPECT_EQ(p.data.detections.size(), 5u);
}

TEST_F(PerceptionLockingTestCase, DetectionsConcurrentSetDataDuringCopyConstruction)
{
  easynav::DetectionsPerception p;
  std::atomic<bool> start_flag{false};
  std::atomic<bool> stop_flag{false};
  std::atomic<int> reads{0};

  auto writer = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      for (int i = 0; i < 2000; ++i) {
        vision_msgs::msg::Detection3DArray msg;
        // Varying size forces the internal vector to reallocate on (almost)
        // every write, same hazard class as NavState's "path"/"goals" bug.
        msg.detections.resize(1 + (i % 23));
        p.set_data(msg, rclcpp::Time(i, 0, RCL_ROS_TIME), long_frame_id(i));
      }
      stop_flag.store(true);
    };

  auto reader = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      while (!stop_flag.load()) {
        easynav::DetectionsPerception snapshot(p);
        std::this_thread::yield();
        // size() must always be sane; a torn read on the vector's internal
        // pointers/size could otherwise report a garbage size or crash.
        EXPECT_LE(snapshot.data.detections.size(), 23u);
        reads.fetch_add(1);
      }
    };

  std::vector<std::thread> threads;
  threads.emplace_back(writer);
  threads.emplace_back(reader);
  start_flag.store(true);
  for (auto & t : threads) {
    t.join();
  }

  EXPECT_GT(reads.load(), 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// ImagePerception
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(PerceptionLockingTestCase, ImageSetDataUpdatesAllFieldsAndIsValid)
{
  easynav::ImagePerception p;
  cv::Mat img(2, 3, CV_8UC1, cv::Scalar(42));

  const rclcpp::Time stamp(7, 0, RCL_ROS_TIME);
  p.set_data(img.clone(), stamp, "camera_link");

  EXPECT_EQ(p.frame_id, "camera_link");
  EXPECT_EQ(p.stamp, stamp);
  EXPECT_TRUE(p.valid);
  EXPECT_TRUE(p.new_data);
  EXPECT_EQ(p.data.rows, 2);
  EXPECT_EQ(p.data.cols, 3);
}

TEST_F(PerceptionLockingTestCase, ImageMarkInvalidClearsValidButKeepsPreviousData)
{
  easynav::ImagePerception p;
  cv::Mat img(2, 3, CV_8UC1, cv::Scalar(42));
  p.set_data(img.clone(), rclcpp::Time(1, 0, RCL_ROS_TIME), "camera_link_1");
  ASSERT_TRUE(p.valid);

  p.mark_invalid(rclcpp::Time(2, 0, RCL_ROS_TIME), "camera_link_2");

  EXPECT_FALSE(p.valid);
  EXPECT_TRUE(p.new_data);
  EXPECT_EQ(p.frame_id, "camera_link_2");
  // Matches the pre-existing cv_bridge-exception behavior: data is left
  // untouched on a failed decode, not cleared.
  EXPECT_EQ(p.data.rows, 2);
  EXPECT_EQ(p.data.cols, 3);
}

TEST_F(PerceptionLockingTestCase, ImageConsumeNewDataReturnsThenClears)
{
  easynav::ImagePerception p;
  cv::Mat img(1, 1, CV_8UC1);
  p.set_data(img.clone(), rclcpp::Time(0, 0, RCL_ROS_TIME), "camera_link");

  EXPECT_TRUE(p.consume_new_data());
  EXPECT_FALSE(p.consume_new_data());
}

TEST_F(PerceptionLockingTestCase, ImageCopyConstructorIsIndependentSnapshot)
{
  easynav::ImagePerception p;
  cv::Mat img1(2, 2, CV_8UC1, cv::Scalar(1));
  p.set_data(img1.clone(), rclcpp::Time(1, 0, RCL_ROS_TIME), "camera_link_1");

  easynav::ImagePerception snapshot(p);

  cv::Mat img2(4, 4, CV_8UC1, cv::Scalar(2));
  p.set_data(img2.clone(), rclcpp::Time(2, 0, RCL_ROS_TIME), "camera_link_2");

  EXPECT_EQ(snapshot.data.rows, 2);
  EXPECT_EQ(snapshot.data.cols, 2);
  EXPECT_EQ(p.data.rows, 4);
  EXPECT_EQ(p.data.cols, 4);
}

// cv::Mat is itself reference-counted/shared-buffer; the point of set_data()
// taking the lock is to make the (header, buffer) pair change atomically as
// observed by a concurrent copy. Varying image size on every write forces a
// new buffer allocation, so a torn read would show a rows/cols mismatch
// against the buffer's actual size.
TEST_F(PerceptionLockingTestCase, ImageConcurrentSetDataDuringCopyConstruction)
{
  easynav::ImagePerception p;
  std::atomic<bool> start_flag{false};
  std::atomic<bool> stop_flag{false};
  std::atomic<int> reads{0};

  auto writer = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      for (int i = 0; i < 1000; ++i) {
        const int size = 1 + (i % 17);
        cv::Mat img(size, size, CV_8UC1, cv::Scalar(static_cast<double>(i % 256)));
        p.set_data(std::move(img), rclcpp::Time(i, 0, RCL_ROS_TIME), long_frame_id(i));
      }
      stop_flag.store(true);
    };

  auto reader = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      while (!stop_flag.load()) {
        easynav::ImagePerception snapshot(p);
        std::this_thread::yield();
        if (!snapshot.data.empty()) {
          EXPECT_EQ(snapshot.data.rows, snapshot.data.cols)
            << "torn read: mismatched image dimensions";
          EXPECT_EQ(
            static_cast<size_t>(snapshot.data.total() * snapshot.data.elemSize()),
            snapshot.data.dataend - snapshot.data.datastart)
            << "torn read: buffer size does not match header dimensions";
        }
        reads.fetch_add(1);
      }
    };

  std::vector<std::thread> threads;
  threads.emplace_back(writer);
  threads.emplace_back(reader);
  start_flag.store(true);
  for (auto & t : threads) {
    t.join();
  }

  EXPECT_GT(reads.load(), 0);
}
