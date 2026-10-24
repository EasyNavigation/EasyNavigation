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
/// \brief SystemNode tells which EasyNav nodes have use_sim_time and no /clock (their
/// components never run), and warns about them while Active.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "easynav_system/SystemNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "rosgraph_msgs/msg/clock.hpp"

#include "gtest/gtest.h"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace
{

std::atomic<int> g_clock_warnings{0};

void count_clock_warnings(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  char msg[512];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(msg, sizeof(msg), format, copy);
  va_end(copy);
  if (severity == RCUTILS_LOG_SEVERITY_WARN &&
    std::string(msg).find("its ROS clock is still at 0") != std::string::npos)
  {
    ++g_clock_warnings;
  }
}

}  // namespace

class SystemClockWatchdogTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      std::vector<const char *> argv{
        "system_clock_watchdog_tests",
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
    handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(count_clock_warnings);
    g_clock_warnings = 0;
  }

  void TearDown() override {rcutils_logging_set_output_handler(handler_);}

  // An active system; with \p sim_time, use_sim_time on every node but \p except.
  easynav::SystemNode::SharedPtr make_active_system(
    bool sim_time, const std::string & except = "")
  {
    auto system_node = std::make_shared<easynav::SystemNode>();
    std::vector<rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes{system_node};
    for (const auto & [name, info] : system_node->get_system_nodes()) {
      nodes.push_back(info.node_ptr);
    }
    for (auto & node : nodes) {
      if (sim_time && node->get_name() != except) {
        node->set_parameter(rclcpp::Parameter("use_sim_time", true));
      }
    }
    system_node->trigger_transition(Transition::TRANSITION_CONFIGURE);
    system_node->trigger_transition(Transition::TRANSITION_ACTIVATE);
    EXPECT_EQ(system_node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
    return system_node;
  }

  // Runs system_cycle() for \p duration.
  static void cycle_for(easynav::SystemNode & system_node, std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      system_node.system_cycle();
      std::this_thread::sleep_for(20ms);
    }
  }

  rcutils_logging_output_handler_t handler_;
};

TEST_F(SystemClockWatchdogTest, WithoutSimTimeEveryClockRuns)
{
  auto system_node = make_active_system(false);
  EXPECT_TRUE(system_node->get_nodes_without_clock().empty());
}

TEST_F(SystemClockWatchdogTest, WithSimTimeAndNoClockEveryNodeIsListed)
{
  auto system_node = make_active_system(true);
  auto names = system_node->get_nodes_without_clock();
  EXPECT_EQ(names.size(), system_node->get_system_nodes().size() + 1);
  EXPECT_NE(std::find(names.begin(), names.end(), "system_node"), names.end());
}

TEST_F(SystemClockWatchdogTest, OnlyTheNodesWithSimTimeAreListed)
{
  auto system_node = make_active_system(true, "planner_node");
  auto names = system_node->get_nodes_without_clock();
  EXPECT_EQ(names.size(), system_node->get_system_nodes().size());
  EXPECT_EQ(std::find(names.begin(), names.end(), "planner_node"), names.end());
}

TEST_F(SystemClockWatchdogTest, OnceTheClockArrivesNoneIsListed)
{
  auto system_node = make_active_system(true);
  auto clock_node = std::make_shared<rclcpp::Node>("clock_watchdog_publisher");
  auto pub = clock_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  rosgraph_msgs::msg::Clock clock;
  clock.clock.sec = 100;
  const auto end = std::chrono::steady_clock::now() + 10s;
  while (!system_node->get_nodes_without_clock().empty() &&
    std::chrono::steady_clock::now() < end)
  {
    pub->publish(clock);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(system_node->get_nodes_without_clock().empty());
}

TEST_F(SystemClockWatchdogTest, WhileActiveItWarnsEveryPeriod)
{
  auto system_node = make_active_system(true, "planner_node");
  const int n = static_cast<int>(system_node->get_system_nodes().size());  // all but planner
  cycle_for(*system_node, easynav::SystemNode::kClockCheckPeriod - 1s);
  EXPECT_EQ(g_clock_warnings, 0);
  cycle_for(*system_node, 2s);
  EXPECT_EQ(g_clock_warnings, n);  // once per node
  cycle_for(*system_node, easynav::SystemNode::kClockCheckPeriod - 2s);
  EXPECT_EQ(g_clock_warnings, n);
  cycle_for(*system_node, 2s);
  EXPECT_EQ(g_clock_warnings, 2 * n);
}

TEST_F(SystemClockWatchdogTest, WithoutSimTimeItNeverWarns)
{
  auto system_node = make_active_system(false);
  cycle_for(*system_node, easynav::SystemNode::kClockCheckPeriod + 1s);
  EXPECT_EQ(g_clock_warnings, 0);
}

TEST_F(SystemClockWatchdogTest, AReactivationRestartsThePeriod)
{
  auto system_node = make_active_system(true);
  cycle_for(*system_node, easynav::SystemNode::kClockCheckPeriod - 1s);
  system_node->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  system_node->trigger_transition(Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(system_node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
  cycle_for(*system_node, 2s);
  EXPECT_EQ(g_clock_warnings, 0);
}
