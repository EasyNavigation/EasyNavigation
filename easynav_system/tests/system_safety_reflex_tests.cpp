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

/// \file
/// \brief Tests for SystemNode's loading of level-0 SafetyReflexBase plugins.
/// See docs/recoveries_easynav.md, level 0.

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_system/SystemNode.hpp"

class SystemSafetyReflexTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(SystemSafetyReflexTest, ConfigureSucceedsWithNoReflexes)
{
  // Backward compatibility: safety_reflex_types defaults to an empty list, so on_configure()
  // must succeed exactly as it did before level-0 reflexes existed.
  auto system_node = std::make_shared<easynav::SystemNode>();

  auto state = rclcpp_lifecycle::State();
  auto ret = system_node->on_configure(state);

  EXPECT_EQ(ret, easynav::SystemNode::CallbackReturnT::SUCCESS);
}

TEST_F(SystemSafetyReflexTest, ConfigureLoadsCollisionSafetyReflex)
{
  auto system_node = std::make_shared<easynav::SystemNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "safety_reflex_types", std::vector<std::string>{"collision"})
    .append_parameter_override(
      "collision.plugin", std::string("easynav_controller/CollisionSafetyReflex")));

  auto state = rclcpp_lifecycle::State();
  auto ret = system_node->on_configure(state);

  EXPECT_EQ(ret, easynav::SystemNode::CallbackReturnT::SUCCESS);
}

TEST_F(SystemSafetyReflexTest, ConfigureFailsWithUnknownReflexPlugin)
{
  auto system_node = std::make_shared<easynav::SystemNode>(
    rclcpp::NodeOptions()
    .append_parameter_override(
      "safety_reflex_types", std::vector<std::string>{"bogus"})
    .append_parameter_override(
      "bogus.plugin", std::string("easynav_controller/NoSuchReflex")));

  auto state = rclcpp_lifecycle::State();
  auto ret = system_node->on_configure(state);

  EXPECT_EQ(ret, easynav::SystemNode::CallbackReturnT::FAILURE);
}
