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
/// \brief SystemNode shares "robot_geometry.*" with every component before they configure.

#include <memory>
#include <string>
#include <vector>

#include "easynav_common/RobotGeometry.hpp"
#include "easynav_system/SystemNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class SystemRobotGeometryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      std::vector<const char *> argv{
        "system_robot_geometry_tests",
        "--ros-args",
        "-p", "controller_types:=['dummy_controller']",
        "-p", "dummy_controller.plugin:=easynav_controller/DummyController",
        "-p", "localizer_types:=['dummy_localizer']",
        "-p", "dummy_localizer.plugin:=easynav_localizer/DummyLocalizer",
        "-p", "planner_types:=['dummy_planner']",
        "-p", "dummy_planner.plugin:=easynav_planner/DummyPlanner",
        "-p", "map_types:=['dummy_map']",
        "-p", "dummy_map.plugin:=easynav_maps_manager/DummyMapsManager",
      };
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }
    // As left by another EasyNav: must be replaced on configure.
    registry()->set_geometry({9.0, 9.0, 9.0}, {"radius", "inscribed_radius", "height"});
  }

  static easynav::RobotGeometryRegistry * registry()
  {
    return easynav::RobotGeometryRegistry::getInstance();
  }

  easynav::SystemNode::SharedPtr configure(const std::vector<rclcpp::Parameter> & geometry = {})
  {
    auto system_node = std::make_shared<easynav::SystemNode>(
      rclcpp::NodeOptions().parameter_overrides(geometry));
    EXPECT_EQ(
      system_node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      State::PRIMARY_STATE_INACTIVE);
    return system_node;
  }

  static bool configured(const std::string & field)
  {
    return registry()->is_configured(field);
  }
};

TEST_F(SystemRobotGeometryTest, DefaultsWhenNotConfigured)
{
  auto system_node = configure();
  const auto geometry = registry()->get_geometry();
  const easynav::RobotGeometry defaults;
  EXPECT_DOUBLE_EQ(geometry.radius, defaults.radius);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, defaults.radius);
  EXPECT_DOUBLE_EQ(geometry.height, defaults.height);
  EXPECT_FALSE(configured("radius"));
  EXPECT_FALSE(configured("inscribed_radius"));
  EXPECT_FALSE(configured("height"));
}

TEST_F(SystemRobotGeometryTest, ConfiguredGeometryIsShared)
{
  auto system_node = configure({
    {"robot_geometry.radius", 0.7},
    {"robot_geometry.inscribed_radius", 0.5},
    {"robot_geometry.height", 1.2}});
  const auto geometry = registry()->get_geometry();
  EXPECT_DOUBLE_EQ(geometry.radius, 0.7);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.5);
  EXPECT_DOUBLE_EQ(geometry.height, 1.2);
  EXPECT_TRUE(configured("radius"));
  EXPECT_TRUE(configured("inscribed_radius"));
  EXPECT_TRUE(configured("height"));
}

TEST_F(SystemRobotGeometryTest, InscribedRadiusDefaultsToTheRadius)
{
  auto system_node = configure({{"robot_geometry.radius", 0.45}});
  const auto geometry = registry()->get_geometry();
  EXPECT_DOUBLE_EQ(geometry.radius, 0.45);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.45) << "a round robot";
  EXPECT_TRUE(configured("radius"));
  EXPECT_FALSE(configured("inscribed_radius")) << "deprecated inscribed radii still apply";
  EXPECT_FALSE(configured("height"));
}

TEST_F(SystemRobotGeometryTest, ConfiguredToTheDefaultValueCountsAsConfigured)
{
  const easynav::RobotGeometry defaults;
  auto system_node = configure({{"robot_geometry.radius", defaults.radius}});
  EXPECT_TRUE(configured("radius"));
}

TEST_F(SystemRobotGeometryTest, AReconfigurationUpdatesIt)
{
  auto system_node = configure({{"robot_geometry.radius", 0.4}});
  ASSERT_EQ(
    system_node->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
    State::PRIMARY_STATE_ACTIVE);

  system_node->request_reconfigure(
    {{"system_node", rclcpp::Parameter("robot_geometry.radius", 0.6)},
      {"system_node", rclcpp::Parameter("robot_geometry.height", 0.9)}}, "bigger");
  ASSERT_TRUE(system_node->apply_pending_reconfigure());
  auto geometry = registry()->get_geometry();
  EXPECT_DOUBLE_EQ(geometry.radius, 0.6);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.6);
  EXPECT_DOUBLE_EQ(geometry.height, 0.9);
  EXPECT_TRUE(configured("height")) << "changed at runtime";

  system_node->request_restore_parameters("back");
  ASSERT_TRUE(system_node->apply_pending_reconfigure());
  geometry = registry()->get_geometry();
  EXPECT_DOUBLE_EQ(geometry.radius, 0.4);
  EXPECT_DOUBLE_EQ(geometry.height, easynav::RobotGeometry{}.height);
  EXPECT_FALSE(configured("height"));
}
