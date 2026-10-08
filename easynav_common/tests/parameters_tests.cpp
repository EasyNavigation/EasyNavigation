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
/// \brief Tests for declare_parameter_if_absent().

#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/Parameters.hpp"

class ParametersTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(ParametersTest, DeclaresWhenAbsent)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_absent_node");
  easynav::declare_parameter_if_absent<double>(*node, "p.speed", 1);
  easynav::declare_parameter_if_absent(*node, "p.name", std::string("a"));
  easynav::declare_parameter_if_absent(*node, "p.flag", true);

  EXPECT_DOUBLE_EQ(node->get_parameter("p.speed").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("p.name").as_string(), "a");
  EXPECT_TRUE(node->get_parameter("p.flag").as_bool());
}

TEST_F(ParametersTest, KeepsAnExistingValue)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "params_existing_node", rclcpp::NodeOptions().append_parameter_override("p.speed", 3.0));

  easynav::declare_parameter_if_absent<double>(*node, "p.speed", 1.0);  // Override wins.
  EXPECT_DOUBLE_EQ(node->get_parameter("p.speed").as_double(), 3.0);

  node->set_parameter(rclcpp::Parameter("p.speed", 5.0));
  EXPECT_NO_THROW(easynav::declare_parameter_if_absent<double>(*node, "p.speed", 1.0));
  EXPECT_DOUBLE_EQ(node->get_parameter("p.speed").as_double(), 5.0);
}

TEST_F(ParametersTest, SwitchingPluginsSharingSomeParameters)
{
  // Plugin A, then plugin B under the same name: they share "max_speed" only.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("params_switch_node");
  easynav::declare_parameter_if_absent(*node, "ctrl.max_speed", 0.5);
  easynav::declare_parameter_if_absent(*node, "ctrl.a_gain", 2.0);

  ASSERT_NO_THROW(easynav::declare_parameter_if_absent(*node, "ctrl.max_speed", 0.8));
  ASSERT_NO_THROW(easynav::declare_parameter_if_absent(*node, "ctrl.b_horizon", 10));

  EXPECT_DOUBLE_EQ(node->get_parameter("ctrl.max_speed").as_double(), 0.5);
  EXPECT_EQ(node->get_parameter("ctrl.b_horizon").as_int(), 10);
  EXPECT_TRUE(node->has_parameter("ctrl.a_gain"));
}
