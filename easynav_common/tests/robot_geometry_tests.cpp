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
/// \brief Tests for get_robot_geometry(): the system's geometry and deprecated parameters.

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/RobotGeometry.hpp"
#include "easynav_common/testing/LogCapture.hpp"

using easynav::RobotGeometry;

class RobotGeometryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    easynav::RobotGeometryRegistry::getInstance()->set_geometry(RobotGeometry{});
  }

  static rclcpp_lifecycle::LifecycleNode::SharedPtr node(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "robot_geometry_test_node", rclcpp::NodeOptions().parameter_overrides(overrides));
  }

  static void system_geometry(
    double radius, double inscribed, double height, const std::set<std::string> & configured)
  {
    easynav::RobotGeometryRegistry::getInstance()->set_geometry(
      RobotGeometry{radius, inscribed, height}, configured);
  }

  const easynav::LegacyRobotGeometryNames legacy_ {
    "plugin.robot_radius", "plugin.inscribed_radius", "plugin.robot_height"};
};

TEST_F(RobotGeometryTest, DefaultsWithoutAnyConfiguration)
{
  auto n = node();
  const auto geometry = easynav::get_robot_geometry(*n, legacy_);
  const RobotGeometry defaults;
  EXPECT_DOUBLE_EQ(geometry.radius, defaults.radius);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, defaults.inscribed_radius);
  EXPECT_DOUBLE_EQ(geometry.height, defaults.height);
  EXPECT_FALSE(n->has_parameter("plugin.robot_radius")) << "not configured: not declared";
}

TEST_F(RobotGeometryTest, UsesTheSystemGeometry)
{
  system_geometry(0.7, 0.5, 1.2, {"radius", "inscribed_radius", "height"});
  auto n = node();
  const auto geometry = easynav::get_robot_geometry(*n);
  EXPECT_DOUBLE_EQ(geometry.radius, 0.7);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.5);
  EXPECT_DOUBLE_EQ(geometry.height, 1.2);
}

TEST_F(RobotGeometryTest, DeprecatedParametersApplyWhenNotConfigured)
{
  auto n = node({
    {"plugin.robot_radius", 0.25}, {"plugin.inscribed_radius", 0.2},
    {"plugin.robot_height", 0.4}});
  const auto geometry = easynav::get_robot_geometry(*n, legacy_);
  EXPECT_DOUBLE_EQ(geometry.radius, 0.25);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.2);
  EXPECT_DOUBLE_EQ(geometry.height, 0.4);
}

TEST_F(RobotGeometryTest, ConfiguredGeometryTakesPrecedence)
{
  system_geometry(0.7, 0.5, 1.2, {"radius", "inscribed_radius", "height"});
  auto n = node({
    {"plugin.robot_radius", 0.25}, {"plugin.inscribed_radius", 0.2},
    {"plugin.robot_height", 0.4}});
  const auto geometry = easynav::get_robot_geometry(*n, legacy_);
  EXPECT_DOUBLE_EQ(geometry.radius, 0.7);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.5);
  EXPECT_DOUBLE_EQ(geometry.height, 1.2);
}

TEST_F(RobotGeometryTest, PrecedenceIsPerField)
{
  system_geometry(0.7, 0.7, 0.5, {"radius"});
  auto n = node({{"plugin.robot_radius", 0.25}, {"plugin.robot_height", 0.4}});
  const auto geometry = easynav::get_robot_geometry(*n, legacy_);
  EXPECT_DOUBLE_EQ(geometry.radius, 0.7) << "configured";
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.7) << "neither configured nor deprecated";
  EXPECT_DOUBLE_EQ(geometry.height, 0.4) << "deprecated, not configured";
}

TEST_F(RobotGeometryTest, OnlyTheListedNamesCount)
{
  auto n = node({{"plugin.robot_radius", 0.25}, {"other.robot_radius", 0.1}});
  EXPECT_DOUBLE_EQ(easynav::get_robot_geometry(*n).radius, 0.3) << "no legacy names";
  EXPECT_DOUBLE_EQ(
    easynav::get_robot_geometry(*n, {"other.robot_radius", "", ""}).radius, 0.1);
  EXPECT_DOUBLE_EQ(easynav::get_robot_geometry(*n, legacy_).radius, 0.25);
}

TEST_F(RobotGeometryTest, DeprecatedParameterDeclaredByAPreviousInstance)
{
  // Across a reconfiguration, the override is gone from the new plugin's view, but declared.
  auto n = node();
  n->declare_parameter("plugin.robot_radius", 0.25);
  EXPECT_DOUBLE_EQ(easynav::get_robot_geometry(*n, legacy_).radius, 0.25);

  n->set_parameter(rclcpp::Parameter("plugin.robot_radius", 0.35));
  EXPECT_DOUBLE_EQ(easynav::get_robot_geometry(*n, legacy_).radius, 0.35);
}

TEST_F(RobotGeometryTest, RepeatedCallsAreStable)
{
  auto n = node({{"plugin.robot_radius", 0.25}});
  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(easynav::get_robot_geometry(*n, legacy_).radius, 0.25) << i;
  }
}

TEST_F(RobotGeometryTest, FollowsChangesOfTheSystemGeometry)
{
  auto n = node({{"plugin.robot_radius", 0.25}});
  EXPECT_DOUBLE_EQ(easynav::get_robot_geometry(*n, legacy_).radius, 0.25);
  system_geometry(0.4, 0.4, 0.5, {"radius"});
  EXPECT_DOUBLE_EQ(easynav::get_robot_geometry(*n, legacy_).radius, 0.4);
}

TEST_F(RobotGeometryTest, WorksWithAPlainNode)
{
  auto n = std::make_shared<rclcpp::Node>(
    "robot_geometry_plain_node",
    rclcpp::NodeOptions().parameter_overrides({{"plugin.robot_height", 0.8}}));
  EXPECT_DOUBLE_EQ(easynav::get_robot_geometry(*n, legacy_).height, 0.8);
}

// ─── Deprecation warnings ────────────────────────────────────────────────────────────────────

using easynav::testing::LogCapture;

TEST_F(RobotGeometryTest, WarnsWhenADeprecatedParameterApplies)
{
  LogCapture log;
  auto n = node({{"plugin.robot_radius", 0.25}, {"plugin.robot_height", 0.4}});
  easynav::get_robot_geometry(*n, legacy_);

  EXPECT_EQ(
    log.count(
      {"'plugin.robot_radius' is deprecated", "configure 'system_node.robot_geometry.radius'",
        "stop working soon"}), 1u);
  EXPECT_EQ(
    log.count(
      {"'plugin.robot_height' is deprecated", "configure 'system_node.robot_geometry.height'"}),
    1u);
  EXPECT_EQ(log.count({"inscribed_radius"}), 0u) << "not configured: no warning";
}

TEST_F(RobotGeometryTest, WarnsWhenADeprecatedParameterIsIgnored)
{
  system_geometry(0.7, 0.7, 0.5, {"radius"});
  LogCapture log;
  auto n = node({{"plugin.robot_radius", 0.25}, {"plugin.robot_height", 0.4}});
  easynav::get_robot_geometry(*n, legacy_);

  EXPECT_EQ(
    log.count(
      {"'plugin.robot_radius' is deprecated and ignored",
        "'system_node.robot_geometry.radius' takes precedence"}), 1u);
  EXPECT_EQ(log.count({"'plugin.robot_radius' is deprecated: configure"}), 0u);
  EXPECT_EQ(
    log.count({"'plugin.robot_height' is deprecated: configure"}), 1u) << "height applies";
}

TEST_F(RobotGeometryTest, NoWarningWithoutDeprecatedParameters)
{
  LogCapture log;
  auto n = node();
  easynav::get_robot_geometry(*n, legacy_);
  system_geometry(0.7, 0.5, 1.2, {"radius", "inscribed_radius", "height"});
  easynav::get_robot_geometry(*n, legacy_);
  EXPECT_EQ(log.count({"deprecated"}), 0u);
}

TEST_F(RobotGeometryTest, WarnsOnEveryRead)
{
  // Each component that reads it (e.g. after a reconfiguration) reminds it.
  LogCapture log;
  auto n = node({{"plugin.robot_radius", 0.25}});
  easynav::get_robot_geometry(*n, legacy_);
  easynav::get_robot_geometry(*n, legacy_);
  EXPECT_EQ(log.count({"'plugin.robot_radius' is deprecated"}), 2u);
}

TEST_F(RobotGeometryTest, WarningsAreWarnings)
{
  LogCapture log;
  auto n = node({{"plugin.robot_radius", 0.25}});
  easynav::get_robot_geometry(*n, legacy_);
  EXPECT_EQ(log.count({"is deprecated"}, RCUTILS_LOG_SEVERITY_WARN), 1u);
  EXPECT_EQ(log.count({"is deprecated"}, -1), 1u) << "only as a warning";
}
