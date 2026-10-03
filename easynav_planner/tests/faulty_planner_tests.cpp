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
/// \brief FaultyPlanner: each fault does what it says.

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_planner/fault_injection/FaultyPlanner.hpp"

using namespace std::chrono_literals;

class FaultyPlannerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::shared_ptr<easynav::FaultyPlanner> make(const std::string & fault, int fault_after = 0)
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "faulty_planner_test", rclcpp::NodeOptions().parameter_overrides(
        {{"plan.fault", fault}, {"plan.fault_after", fault_after}, {"plan.hang_time", 0.05}}));
    auto planner = std::make_shared<easynav::FaultyPlanner>();
    planner->initialize(node_, "plan");
    return planner;
  }

  // Robot at (0, 0), goal at (\p x, 0).
  void navigate_to(double x)
  {
    nav_msgs::msg::Odometry pose;
    nav_state_.set("robot_pose", pose);
    nav_msgs::msg::Goals goals;
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = x;
    goal.pose.orientation.w = 1.0;
    goals.goals.push_back(goal);
    nav_state_.set("goals", goals);
  }

  nav_msgs::msg::Path path() const {return nav_state_.get<nav_msgs::msg::Path>("path");}

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
};

TEST_F(FaultyPlannerTest, UnknownFaultsFailToInitialize)
{
  EXPECT_THROW(make("explode"), std::invalid_argument);
}

TEST_F(FaultyPlannerTest, WithoutFaultItPlansAStraightLineToTheGoal)
{
  auto planner = make("none");
  navigate_to(2.0);
  planner->update(nav_state_);
  ASSERT_GT(path().poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path().poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(path().poses.back().pose.position.x, 2.0);
  EXPECT_EQ(path().header.frame_id, "map");
}

TEST_F(FaultyPlannerTest, WithoutGoalThePathIsEmpty)
{
  auto planner = make("none");
  planner->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(FaultyPlannerTest, TheFaultStartsAfterFaultAfterUpdates)
{
  auto planner = make("throw", 2);
  navigate_to(2.0);
  EXPECT_NO_THROW(planner->update(nav_state_));
  EXPECT_NO_THROW(planner->update(nav_state_));
  EXPECT_THROW(planner->update(nav_state_), std::runtime_error);
}

TEST_F(FaultyPlannerTest, HangBlocksAndThenPlans)
{
  auto planner = make("hang");
  navigate_to(2.0);
  const auto start = std::chrono::steady_clock::now();
  planner->update(nav_state_);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 50ms);
  EXPECT_FALSE(path().poses.empty());
}

TEST_F(FaultyPlannerTest, EmptyPathHasNoPoses)
{
  auto planner = make("empty_path");
  navigate_to(2.0);
  planner->update(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(FaultyPlannerTest, FreezeKeepsTheOldPathWhenTheGoalChanges)
{
  auto planner = make("freeze", 1);
  navigate_to(2.0);
  planner->update(nav_state_);
  const auto stamp = rclcpp::Time(path().header.stamp);
  navigate_to(5.0);
  std::this_thread::sleep_for(20ms);
  planner->update(nav_state_);
  EXPECT_DOUBLE_EQ(path().poses.back().pose.position.x, 2.0);
  EXPECT_EQ(rclcpp::Time(path().header.stamp), stamp);
}

TEST_F(FaultyPlannerTest, NanWritesNanPoses)
{
  auto planner = make("nan");
  navigate_to(2.0);
  planner->update(nav_state_);
  ASSERT_FALSE(path().poses.empty());
  for (const auto & pose : path().poses) {
    EXPECT_TRUE(std::isnan(pose.pose.position.x));
  }
}
