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
/// \brief FaultyLocalizer: each fault does what it says.

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_localizer/fault_injection/FaultyLocalizer.hpp"

using namespace std::chrono_literals;

class FaultyLocalizerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::shared_ptr<easynav::FaultyLocalizer> make(
    const std::string & fault, int fault_after = 0, std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> params {
      {"loc.fault", fault}, {"loc.fault_after", fault_after}, {"loc.x", 1.0}, {"loc.y", 2.0},
      {"loc.yaw", 0.5}, {"loc.hang_time", 0.05}};
    params.insert(params.end(), extra.begin(), extra.end());
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "faulty_localizer_test", rclcpp::NodeOptions().parameter_overrides(params));
    auto localizer = std::make_shared<easynav::FaultyLocalizer>();
    localizer->initialize(node_, "loc");
    return localizer;
  }

  nav_msgs::msg::Odometry pose() const
  {
    return nav_state_.get<nav_msgs::msg::Odometry>("robot_pose");
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
};

TEST_F(FaultyLocalizerTest, UnknownFaultsFailToInitialize)
{
  EXPECT_THROW(make("explode"), std::invalid_argument);
}

TEST_F(FaultyLocalizerTest, WithoutFaultItLocalizesAtTheConfiguredPoseStampedNow)
{
  auto localizer = make("none");
  const auto before = node_->now();
  localizer->update_rt(nav_state_);
  EXPECT_DOUBLE_EQ(pose().pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose().pose.pose.position.y, 2.0);
  EXPECT_NEAR(
    2.0 * std::atan2(pose().pose.pose.orientation.z, pose().pose.pose.orientation.w),
    0.5, 1e-9);
  EXPECT_GE(rclcpp::Time(pose().header.stamp), before);
  EXPECT_EQ(pose().header.frame_id, "map");
}

TEST_F(FaultyLocalizerTest, TheFaultStartsAfterFaultAfterUpdates)
{
  auto localizer = make("throw", 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NO_THROW(localizer->update_rt(nav_state_)) << i;
  }
  EXPECT_THROW(localizer->update_rt(nav_state_), std::runtime_error);
  EXPECT_THROW(localizer->update_rt(nav_state_), std::runtime_error) << "every update after";
}

TEST_F(FaultyLocalizerTest, HangBlocksAndThenLocalizes)
{
  auto localizer = make("hang");
  const auto start = std::chrono::steady_clock::now();
  localizer->update_rt(nav_state_);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 50ms);
  EXPECT_DOUBLE_EQ(pose().pose.pose.position.x, 1.0);
}

TEST_F(FaultyLocalizerTest, FreezeKeepsTheOldStamp)
{
  auto localizer = make("freeze", 1);
  localizer->update_rt(nav_state_);
  const auto stamp = rclcpp::Time(pose().header.stamp);
  std::this_thread::sleep_for(20ms);
  localizer->update_rt(nav_state_);
  localizer->update_rt(nav_state_);
  EXPECT_EQ(rclcpp::Time(pose().header.stamp), stamp);
}

TEST_F(FaultyLocalizerTest, StopPublishingWritesNothing)
{
  auto localizer = make("stop_publishing");
  localizer->update_rt(nav_state_);
  EXPECT_FALSE(nav_state_.has("robot_pose"));
}

TEST_F(FaultyLocalizerTest, NanWritesANanPose)
{
  auto localizer = make("nan");
  localizer->update_rt(nav_state_);
  EXPECT_TRUE(std::isnan(pose().pose.pose.position.x));
  EXPECT_TRUE(std::isnan(pose().pose.pose.position.y));
}

TEST_F(FaultyLocalizerTest, JumpMovesThePoseOnceAndStaysThere)
{
  auto localizer = make("jump", 1, {{"loc.jump_distance", 3.0}});
  localizer->update_rt(nav_state_);
  EXPECT_DOUBLE_EQ(pose().pose.pose.position.x, 1.0);
  localizer->update_rt(nav_state_);
  EXPECT_DOUBLE_EQ(pose().pose.pose.position.x, 4.0);
  localizer->update_rt(nav_state_);
  EXPECT_DOUBLE_EQ(pose().pose.pose.position.x, 4.0);
}
