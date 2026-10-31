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

#include <cmath>
#include <limits>
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
  auto system_node = configure(
  {
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

// Footprint.

TEST_F(SystemRobotGeometryTest, AFootprintIsSharedAndGivesTheRadii)
{
  // 0.8 x 0.4 m, its center 0.1 m ahead of the robot frame.
  auto system_node = configure(
    {{"robot_geometry.footprint",
      std::vector<double>{0.5, 0.2, -0.3, 0.2, -0.3, -0.2, 0.5, -0.2}}});
  const auto geometry = registry()->get_geometry();
  ASSERT_EQ(geometry.footprint.size(), 4u);
  EXPECT_DOUBLE_EQ(geometry.footprint[0].x, 0.5);
  EXPECT_DOUBLE_EQ(geometry.footprint[3].y, -0.2);
  EXPECT_DOUBLE_EQ(geometry.radius, std::hypot(0.5, 0.2)) << "the farthest vertex";
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.2) << "the nearest side";
  EXPECT_TRUE(configured("footprint"));
  EXPECT_FALSE(configured("radius"));
}

TEST_F(SystemRobotGeometryTest, ConfiguredRadiiAreKeptWithAFootprint)
{
  // Planning margins: larger, or even smaller than the footprint (with a warning).
  for (double radius : {0.9, 0.4}) {
    auto system_node = configure(
    {
      {"robot_geometry.footprint",
        std::vector<double>{0.4, 0.25, -0.4, 0.25, -0.4, -0.25, 0.4, -0.25}},
      {"robot_geometry.radius", radius},
      {"robot_geometry.inscribed_radius", 0.3}});
    const auto geometry = registry()->get_geometry();
    EXPECT_DOUBLE_EQ(geometry.radius, radius);
    EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.3);
    EXPECT_EQ(geometry.footprint.size(), 4u);
  }
}

TEST_F(SystemRobotGeometryTest, WithoutAFootprintTheRobotIsRound)
{
  auto system_node = configure({{"robot_geometry.radius", 0.45}});
  EXPECT_TRUE(registry()->get_geometry().footprint.empty());
  EXPECT_FALSE(configured("footprint"));
}

TEST_F(SystemRobotGeometryTest, AnInvalidFootprintFailsToConfigure)
{
  for (const auto & footprint : std::vector<std::vector<double>>{
    {0.4, 0.2, -0.4, 0.2, -0.4},                        // odd
    {0.4, 0.2, -0.4, 0.2},                              // 2 vertices
    {0.4, 0.2, -0.4, std::nan(""), -0.4, -0.2},         // not finite
    {0.4, 0.2, -0.4, 0.2, -0.4, std::numeric_limits<double>::infinity()}})
  {
    auto system_node = std::make_shared<easynav::SystemNode>(
      rclcpp::NodeOptions().parameter_overrides({{"robot_geometry.footprint", footprint}}));
    EXPECT_EQ(
      system_node->trigger_transition(Transition::TRANSITION_CONFIGURE).id(),
      State::PRIMARY_STATE_UNCONFIGURED) << footprint.size() << " values";
  }
}
