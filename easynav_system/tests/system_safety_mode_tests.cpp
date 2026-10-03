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
/// \brief Configuration validation, safety mode, frozen configuration and configuration hash.

#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easynav_system/RealTime.hpp"
#include "easynav_system/SystemNode.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{

// A valid safety mode configuration, plus \p extra.
std::vector<std::string> safe(std::vector<std::string> extra = {})
{
  std::vector<std::string> args {
    "safety.mode:=true",
    "safety.plc_limits.max_linear_vel:=1.0",
    "safety.plc_limits.max_angular_vel:=2.0",
    "cmd_vel_keepalive_period:=0.1",
  };
  args.insert(args.end(), extra.begin(), extra.end());
  return args;
}

}  // namespace

class SystemSafetyModeTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    system_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // The subnodes take their parameters from the global arguments: a new context each time.
  void start(const std::vector<std::string> & params)
  {
    system_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::vector<std::string> args {"system_safety_mode_tests", "--ros-args"};
    for (const auto & param : params) {
      args.push_back("-p");
      args.push_back(param);
    }
    std::vector<const char *> argv;
    for (const auto & arg : args) {
      argv.push_back(arg.c_str());
    }
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    system_node_ = std::make_shared<easynav::SystemNode>();
  }

  bool configure()
  {
    return system_node_->trigger_transition(Transition::TRANSITION_CONFIGURE).id() ==
           State::PRIMARY_STATE_INACTIVE;
  }

  // Every subnode in \p state.
  void expect_subnodes_in(uint8_t state)
  {
    for (const auto & [name, info] : system_node_->get_system_nodes()) {
      EXPECT_EQ(info.node_ptr->get_current_state().id(), state) << name;
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr subnode(const std::string & name)
  {
    return system_node_->get_system_nodes().at(name).node_ptr;
  }

  easynav::SystemNode::SharedPtr system_node_;
};

TEST_F(SystemSafetyModeTest, DefaultsConfigureOutsideSafetyMode)
{
  start({});
  ASSERT_TRUE(configure());
  EXPECT_FALSE(system_node_->get_safety().is_safety_mode());
  expect_subnodes_in(State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SystemSafetyModeTest, InvalidSystemParametersFailToConfigureInAnyMode)
{
  for (const std::string param : {
    "rt_freq:=0.0", "rt_freq:=-10.0", "rt_freq:=.nan", "freq:=0.0", "freq:=.inf",
    "spin_time_rt:=-0.001", "spin_time_nort:=-1.0", "robot_geometry.radius:=-0.3",
    "robot_geometry.inscribed_radius:=-0.1", "robot_geometry.height:=.nan",
    "safety.plc_limits.max_linear_vel:=-1.0", "safety.plc_limits.max_angular_vel:=.nan"})
  {
    start({param});
    EXPECT_FALSE(configure()) << param;
    expect_subnodes_in(State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_F(SystemSafetyModeTest, InvalidRobotLimitsFailToConfigureAndLeaveNothingConfigured)
{
  start({"robot_limits.max_linear_acc:=0.0"});
  EXPECT_FALSE(configure());
  expect_subnodes_in(State::PRIMARY_STATE_UNCONFIGURED);

  // Fixed, it configures.
  subnode("controller_node")->set_parameter(rclcpp::Parameter("robot_limits.max_linear_acc", 1.0));
  EXPECT_TRUE(configure());
  expect_subnodes_in(State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SystemSafetyModeTest, RobotLimitsMayNotExceedTheSafetyLimitsWhenGiven)
{
  // Defaults: max_linear_vel 0.5, min_linear_vel -0.2, max_angular_vel 1.0.
  struct Case
  {
    std::vector<std::string> params;
    bool valid;
  };
  const std::vector<Case> cases {
    {{"safety.plc_limits.max_linear_vel:=0.4"}, false},
    {{"safety.plc_limits.max_linear_vel:=0.5"}, true},  // Equal: fine
    {{"safety.plc_limits.max_linear_vel:=0.6", "robot_limits.min_linear_vel:=-0.7"}, false},
    {{"safety.plc_limits.max_linear_vel:=0.6", "robot_limits.min_linear_vel:=-0.6"}, true},
    {{"safety.plc_limits.max_angular_vel:=0.9"}, false},
    {{"safety.plc_limits.max_angular_vel:=1.0"}, true},
    {{"safety.plc_limits.max_linear_vel:=0.0", "safety.plc_limits.max_angular_vel:=0.0"}, true},
  };
  for (const auto & c : cases) {
    start(c.params);
    EXPECT_EQ(configure(), c.valid) << c.params.front();
    expect_subnodes_in(c.valid ? State::PRIMARY_STATE_INACTIVE : State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_F(SystemSafetyModeTest, SafetyModeConfiguresWithEverythingItRequires)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }
  start(safe());
  ASSERT_TRUE(configure());
  EXPECT_TRUE(system_node_->get_safety().is_safety_mode());
}

TEST_F(SystemSafetyModeTest, SafetyModeFailsToConfigureWithoutWhatItRequires)
{
  for (const std::string missing : {
    "safety.plc_limits.max_linear_vel:=0.0", "safety.plc_limits.max_angular_vel:=0.0",
    "cmd_vel_keepalive_period:=0.0", "cmd_timeout:=0.0", "use_real_time:=false"})
  {
    start(safe({missing}));
    EXPECT_FALSE(configure()) << missing;
    expect_subnodes_in(State::PRIMARY_STATE_UNCONFIGURED);
  }

  // The same values are fine outside the safety mode.
  for (const std::string param : {
    "cmd_vel_keepalive_period:=0.0", "cmd_timeout:=0.0", "use_real_time:=false"})
  {
    start({param});
    EXPECT_TRUE(configure()) << param;
  }
}

TEST_F(SystemSafetyModeTest, SafetyModeRejectsReconfigurationRequests)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }
  const std::vector<easynav::ParameterChange> slow_down {
    {"controller_node", rclcpp::Parameter("robot_limits.max_linear_vel", 0.1)}};

  start(safe());
  ASSERT_TRUE(configure());
  EXPECT_FALSE(system_node_->request_reconfigure(slow_down, "slow down"));
  EXPECT_FALSE(system_node_->request_restore_parameters("restore"));
  EXPECT_FALSE(system_node_->is_reconfigure_pending());

  start({});
  ASSERT_TRUE(configure());
  EXPECT_TRUE(system_node_->request_reconfigure(slow_down, "slow down"));
  EXPECT_TRUE(system_node_->is_reconfigure_pending());
  EXPECT_TRUE(system_node_->request_restore_parameters("restore"));
}

TEST_F(SystemSafetyModeTest, SafetyModeFreezesTheConfiguration)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }
  start(safe());

  // Before configuring, parameters can still be changed.
  auto controller = subnode("controller_node");
  EXPECT_TRUE(
    controller->set_parameter(rclcpp::Parameter("robot_limits.max_linear_vel", 0.4)).successful);
  ASSERT_TRUE(configure());

  // A change, on any EasyNav node, is rejected...
  auto result = controller->set_parameter(rclcpp::Parameter("robot_limits.max_linear_vel", 0.3));
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("frozen"), std::string::npos) << result.reason;
  EXPECT_DOUBLE_EQ(controller->get_parameter("robot_limits.max_linear_vel").as_double(), 0.4);
  EXPECT_FALSE(
    subnode("planner_node")->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  EXPECT_FALSE(system_node_->set_parameter(rclcpp::Parameter("safety.mode", false)).successful);
  EXPECT_FALSE(
    system_node_->set_parameter(rclcpp::Parameter("safety.plc_limits.max_linear_vel",
    5.0)).successful);

  // ...and so is a new parameter; setting the same value is not.
  EXPECT_TRUE(
    controller->set_parameter(rclcpp::Parameter("robot_limits.max_linear_vel", 0.4)).successful);
  EXPECT_THROW(
    controller->declare_parameter("a_new_parameter", 1.0),
    rclcpp::exceptions::InvalidParameterValueException);

  // Reconfiguring through the lifecycle keeps working, and keeps it frozen.
  ASSERT_EQ(
    system_node_->trigger_transition(Transition::TRANSITION_CLEANUP).id(),
    State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(
    controller->set_parameter(rclcpp::Parameter("robot_limits.max_linear_vel", 0.3)).successful);
  ASSERT_TRUE(configure());
  ASSERT_EQ(
    system_node_->trigger_transition(Transition::TRANSITION_ACTIVATE).id(),
    State::PRIMARY_STATE_ACTIVE);
  EXPECT_FALSE(
    controller->set_parameter(rclcpp::Parameter("robot_limits.max_linear_vel", 0.3)).successful);
}

TEST_F(SystemSafetyModeTest, OutsideSafetyModeTheConfigurationIsNotFrozen)
{
  start({});
  ASSERT_TRUE(configure());
  EXPECT_TRUE(
    subnode("controller_node")->set_parameter(
      rclcpp::Parameter("robot_limits.max_linear_vel", 0.3)).successful);
  EXPECT_TRUE(system_node_->set_parameter(rclcpp::Parameter("safety.mode", true)).successful);
}

TEST_F(SystemSafetyModeTest, ConfigurationHashIsAStableSha256OfEveryParameter)
{
  start({});
  ASSERT_TRUE(configure());
  const auto hash = system_node_->get_safety().get_configuration_hash();
  EXPECT_TRUE(std::regex_match(hash, std::regex("[0-9a-f]{64}"))) << hash;
  EXPECT_EQ(system_node_->get_nav_state()->get_safe<std::string>("configuration_hash"), hash);

  const auto dump = system_node_->get_configuration_dump();
  EXPECT_NE(dump.find("controller_node/robot_limits.max_linear_vel (double) = "),
    std::string::npos);
  EXPECT_NE(dump.find("system_node/safety.mode (bool) = false"), std::string::npos);

  // The same configuration in another process run: the same hash.
  start({});
  ASSERT_TRUE(configure());
  EXPECT_EQ(system_node_->get_safety().get_configuration_hash(), hash);

  // Any parameter of any node changes it.
  for (const std::string param : {
    "robot_limits.max_linear_vel:=0.4", "rt_freq:=100.0", "cmd_timeout:=0.6"})
  {
    start({param});
    ASSERT_TRUE(configure()) << param;
    EXPECT_NE(system_node_->get_safety().get_configuration_hash(), hash) << param;
  }
}

TEST_F(SystemSafetyModeTest, ConfigurationHashFollowsReconfigurations)
{
  start({});
  ASSERT_TRUE(configure());
  const auto first = system_node_->get_safety().get_configuration_hash();

  system_node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  subnode("controller_node")->set_parameter(rclcpp::Parameter("robot_limits.max_linear_vel", 0.3));
  ASSERT_TRUE(configure());
  const auto second = system_node_->get_safety().get_configuration_hash();
  EXPECT_NE(second, first);
  EXPECT_EQ(system_node_->get_nav_state()->get_safe<std::string>("configuration_hash"), second);

  // Back to the first values: the first hash.
  system_node_->trigger_transition(Transition::TRANSITION_CLEANUP);
  subnode("controller_node")->set_parameter(rclcpp::Parameter("robot_limits.max_linear_vel", 0.5));
  ASSERT_TRUE(configure());
  EXPECT_EQ(system_node_->get_safety().get_configuration_hash(), first);
}
