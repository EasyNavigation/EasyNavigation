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
/// \brief PlannerNode discards non-finite paths and reports them.

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
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_planner/PlannerNode.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class PlannerPathCheckTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    // The Dummy planner does not write "path": the tests write it, as a planner would.
    node_ = std::make_shared<easynav::PlannerNode>(rclcpp::NodeOptions().parameter_overrides({
      {"planner_types", std::vector<std::string>{"dummy"}},
      {"dummy.plugin", "easynav_planner/DummyPlanner"}}));
    ASSERT_EQ(node_->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
      State::PRIMARY_STATE_ACTIVE);
  }

  void set_path(std::vector<double> xs)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (const double x : xs) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    nav_state_->set("path", path);
  }

  nav_msgs::msg::Path path() const {return nav_state_->get<nav_msgs::msg::Path>("path");}

  std::optional<DiagnosticStatus> diagnostic() const
  {
    if (!nav_state_->has("diagnostics.path")) {return std::nullopt;}
    return nav_state_->get<DiagnosticStatus>("diagnostics.path");
  }

  easynav::PlannerNode::SharedPtr node_;
  std::shared_ptr<easynav::NavState> nav_state_ = std::make_shared<easynav::NavState>();
};

TEST_F(PlannerPathCheckTest, AFinitePathIsKeptAndNotReported)
{
  set_path({0.0, 1.0, 2.0});
  node_->cycle(nav_state_);
  EXPECT_EQ(path().poses.size(), 3u);
  EXPECT_FALSE(diagnostic()) << "nothing to report until something goes wrong";
}

TEST_F(PlannerPathCheckTest, APathWithANanPoseIsDiscardedAndReported)
{
  set_path({0.0, std::nan(""), 2.0});
  node_->cycle(nav_state_);
  EXPECT_TRUE(path().poses.empty()) << "nothing to follow: controllers stop";
  EXPECT_EQ(path().header.frame_id, "map");
  ASSERT_TRUE(diagnostic());
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostic()->hardware_id, "planner");
}

TEST_F(PlannerPathCheckTest, InfiniteOrientationsAreNotFiniteEither)
{
  set_path({0.0});
  auto bad = path();
  bad.poses[0].pose.orientation.z = std::numeric_limits<double>::infinity();
  nav_state_->set("path", bad);
  node_->cycle(nav_state_);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(PlannerPathCheckTest, AFinitePathAgainIsReportedOkOnce)
{
  set_path({std::nan("")});
  node_->cycle(nav_state_);
  ASSERT_EQ(diagnostic()->level, DiagnosticStatus::ERROR);

  set_path({0.0, 1.0});
  node_->cycle(nav_state_);
  EXPECT_EQ(path().poses.size(), 2u);
  EXPECT_EQ(diagnostic()->level, DiagnosticStatus::OK);

  auto marked = diagnostic().value();
  marked.message = "marker";
  nav_state_->set("diagnostics.path", marked);
  node_->cycle(nav_state_);
  EXPECT_EQ(diagnostic()->message, "marker") << "only changes are reported";
}

TEST_F(PlannerPathCheckTest, AnEmptyPathIsNotThePlannersFault)
{
  set_path({});
  node_->cycle(nav_state_);
  EXPECT_FALSE(diagnostic()) << "an empty path is NoPathEvaluator's business";
}
