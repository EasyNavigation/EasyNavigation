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

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <regex>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "easynav_interfaces/msg/heartbeat.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
    "safety.heartbeat.period:=0.1",
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
    exe_.reset();
    cmd_vel_sub_.reset();
    listener_.reset();
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

  bool activate()
  {
    return system_node_->trigger_transition(Transition::TRANSITION_ACTIVATE).id() ==
           State::PRIMARY_STATE_ACTIVE;
  }

  // RT cycles of the whole system, \p gap apart (the default rt_freq is 200 Hz: 5 ms).
  void run_rt_cycles(int cycles, std::chrono::milliseconds gap)
  {
    for (int i = 0; i < cycles; ++i) {
      system_node_->system_cycle_rt();
      rclcpp::sleep_for(gap);
    }
  }

  // The RT cycle as system_main runs it: every 5 ms (200 Hz); after an overrun, the next one
  // starts right away (as rclcpp::WallRate does), for \p duration. With \p real_time, in a
  // thread with EasyNav's SCHED_FIFO priority, so the machine's load cannot delay it.
  void run_rt_at_rate(std::chrono::milliseconds duration, bool real_time = false)
  {
    auto loop = [this, duration, real_time]() {
        if (real_time) {
          ASSERT_EQ(easynav::set_real_time_priority(easynav::kRealTimePriority), "");
        }
        using Clock = std::chrono::steady_clock;
        const auto end = Clock::now() + duration;
        auto next = Clock::now();
        while (Clock::now() < end) {
          system_node_->system_cycle_rt();
          if (exe_ && !real_time) {
            exe_->spin_some();
          }
          next += std::chrono::milliseconds(5);
          if (Clock::now() < next) {
            std::this_thread::sleep_until(next);
          } else {
            next = Clock::now();
          }
        }
      };
    if (real_time) {
      std::thread(loop).join();
    } else {
      loop();
    }
  }

  // A controller that commands 0.5 m/s and, after \p fault_after updates, blocks for
  // \p hang_time s on every update (one every 1 / \p rt_freq s, the controller's own rate).
  static std::vector<std::string> hanging_controller(
    double hang_time, double rt_freq = 200.0, std::vector<std::string> extra = {},
    int fault_after = 20)
  {
    std::vector<std::string> params {
      "controller_types:=['ctrl']",
      "ctrl.plugin:=easynav_controller/FaultyController",
      "ctrl.fault:='hang'",
      "ctrl.fault_after:=" + std::to_string(fault_after),
      "ctrl.hang_time:=" + std::to_string(hang_time),
      "ctrl.rt_freq:=" + std::to_string(rt_freq),
    };
    params.insert(params.end(), extra.begin(), extra.end());
    return params;
  }

  // Records the velocity commands published on "cmd_vel".
  void listen_cmd_vel()
  {
    listener_ = rclcpp::Node::make_shared("cmd_vel_listener");
    cmd_vel_sub_ = listener_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 100,
      [this](geometry_msgs::msg::Twist::UniquePtr msg) {cmd_vels_.push_back(msg->linear.x);});
    exe_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exe_->add_node(listener_);
    const auto start = std::chrono::steady_clock::now();
    while (cmd_vel_sub_->get_publisher_count() == 0 &&
      std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
    {
      exe_->spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      exe_->spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
  }

  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exe_;
  std::vector<double> cmd_vels_;

  std::optional<diagnostic_msgs::msg::DiagnosticStatus> rt_diagnostic()
  {
    auto nav_state = system_node_->get_nav_state();
    if (!nav_state->has("diagnostics.rt_cycle")) {return std::nullopt;}
    return nav_state->get_safe<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.rt_cycle");
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
    "safety.heartbeat.period:=0.0",
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

TEST_F(SystemSafetyModeTest, LateRtCyclesStopEasyNavInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }

  start(safe({"safety.rt_monitor.max_late_cycles:=3"}));
  ASSERT_TRUE(configure());
  ASSERT_TRUE(activate());

  run_rt_cycles(20, std::chrono::milliseconds(5));
  EXPECT_FALSE(system_node_->is_shutdown_requested()) << "on time";

  // E.g. a plugin that blocks the cycle: more than 2 periods (10 ms) between cycles.
  run_rt_cycles(4, std::chrono::milliseconds(30));
  EXPECT_TRUE(system_node_->is_shutdown_requested());
  EXPECT_NE(system_node_->get_shutdown_reason().find("late"), std::string::npos) <<
    system_node_->get_shutdown_reason();
}

TEST_F(SystemSafetyModeTest, LateRtCyclesAreOnlyReportedOutsideSafetyMode)
{
  start({"safety.rt_monitor.max_late_cycles:=3"});
  ASSERT_TRUE(configure());
  ASSERT_TRUE(activate());

  run_rt_cycles(4, std::chrono::milliseconds(30));
  EXPECT_FALSE(system_node_->is_shutdown_requested());
  ASSERT_TRUE(rt_diagnostic());
  EXPECT_EQ(rt_diagnostic()->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  run_rt_cycles(3, std::chrono::milliseconds(5));
  EXPECT_EQ(rt_diagnostic()->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(SystemSafetyModeTest, AnInactivePeriodIsNotALateCycle)
{
  start({"safety.rt_monitor.max_late_cycles:=1"});
  ASSERT_TRUE(configure());
  ASSERT_TRUE(activate());
  run_rt_cycles(5, std::chrono::milliseconds(5));

  // No RT cycles while inactive: the first one after activating again is not late.
  system_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(activate());
  run_rt_cycles(5, std::chrono::milliseconds(5));
  EXPECT_FALSE(rt_diagnostic()) << "nothing late";
}

TEST_F(SystemSafetyModeTest, TheHeartbeatStopsWhenTheRtCycleStops)
{
  start({"safety.heartbeat.period:=0.02"});
  ASSERT_TRUE(configure());
  ASSERT_TRUE(activate());

  auto listener = rclcpp::Node::make_shared("heartbeat_watchdog");
  std::vector<uint64_t> sequences;
  std::atomic<int> missed {0};
  rclcpp::SubscriptionOptions options;
  options.event_callbacks.deadline_callback =
    [&missed](rclcpp::QOSDeadlineRequestedInfo &) {++missed;};
  auto sub = listener->create_subscription<easynav_interfaces::msg::Heartbeat>(
    "/easynav_heartbeat", rclcpp::QoS(100).reliable().deadline(std::chrono::milliseconds(80)),
    [&sequences](easynav_interfaces::msg::Heartbeat::UniquePtr msg) {
      sequences.push_back(msg->sequence);
    }, options);
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(listener);
  const auto start_time = std::chrono::steady_clock::now();
  while (sub->get_publisher_count() == 0 &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))
  {
    exe.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  for (int i = 0; i < 60; ++i) {  // ~0.3 s of RT cycles
    system_node_->system_cycle_rt();
    exe.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GE(sequences.size(), 8u);
  for (size_t i = 1; i < sequences.size(); ++i) {
    EXPECT_EQ(sequences[i], sequences[i - 1] + 1);
  }
  EXPECT_EQ(missed.load(), 0) << "in time while the RT cycle runs";

  // The RT cycle stops (e.g. blocked): heartbeats stop, the watchdog notices.
  const auto stop = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - stop < std::chrono::milliseconds(300)) {
    exe.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GT(missed.load(), 0);
}

// ─── A component that does not meet the RT frequency ────────────────────────────────────────

TEST_F(SystemSafetyModeTest, AControllerOverrunningTheRtCycleStopsEasyNavInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }

  start(safe(hanging_controller(0.03, 200.0, {"safety.rt_monitor.max_late_cycles:=3"})));
  ASSERT_TRUE(configure());
  ASSERT_TRUE(activate());
  listen_cmd_vel();

  run_rt_at_rate(std::chrono::milliseconds(80));  // 16 cycles on time
  EXPECT_FALSE(system_node_->is_shutdown_requested());
  ASSERT_FALSE(cmd_vels_.empty());
  EXPECT_GT(cmd_vels_.back(), 0.0) << "moving";

  // Each cycle now takes 30 ms instead of 5: more than 2 periods late.
  run_rt_at_rate(std::chrono::milliseconds(300));
  EXPECT_TRUE(system_node_->is_shutdown_requested());
  EXPECT_NE(system_node_->get_shutdown_reason().find("late"), std::string::npos) <<
    system_node_->get_shutdown_reason();
  ASSERT_TRUE(rt_diagnostic());
  EXPECT_EQ(rt_diagnostic()->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  // What system_main does then: deactivate, which ends in Finalized with the robot stopped.
  system_node_->trigger_transition(Transition::TRANSITION_DEACTIVATE);
  EXPECT_EQ(system_node_->get_current_state().id(), State::PRIMARY_STATE_FINALIZED);
  spin_for(std::chrono::milliseconds(200));
  ASSERT_FALSE(cmd_vels_.empty());
  EXPECT_EQ(cmd_vels_.back(), 0.0) << "the last command is an exact zero";
}

TEST_F(SystemSafetyModeTest, AControllerOverrunningTheRtCycleIsOnlyReportedOutsideSafetyMode)
{
  start(hanging_controller(0.03, 200.0, {"safety.rt_monitor.max_late_cycles:=3"}));
  ASSERT_TRUE(configure());
  ASSERT_TRUE(activate());
  listen_cmd_vel();

  run_rt_at_rate(std::chrono::milliseconds(400));
  EXPECT_FALSE(system_node_->is_shutdown_requested());
  ASSERT_TRUE(rt_diagnostic());
  EXPECT_EQ(rt_diagnostic()->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(
    system_node_->get_safety().get_rt_monitor().status(),
    easynav::safety::RtMonitor::Status::ERROR);

  // The navigation goes on: the controller keeps commanding the robot.
  spin_for(std::chrono::milliseconds(50));
  ASSERT_FALSE(cmd_vels_.empty());
  EXPECT_GT(cmd_vels_.back(), 0.0);
  EXPECT_EQ(system_node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(SystemSafetyModeTest, ASlowControllerWithinTheToleranceIsNotReported)
{
  // That nothing is late can only be asserted if the load of the machine cannot delay the cycle.
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "needs real-time scheduling, not allowed here";
  }
  // 7 ms per cycle instead of 5: slower than the period, but well within 3 periods (15 ms).
  start(hanging_controller(0.007, 200.0, {"safety.rt_monitor.max_period_factor:=3.0"}));
  ASSERT_TRUE(configure());
  ASSERT_TRUE(activate());

  run_rt_at_rate(std::chrono::milliseconds(400), true);
  EXPECT_FALSE(system_node_->is_shutdown_requested());
  EXPECT_EQ(system_node_->get_safety().get_rt_monitor().late_cycles(), 0u);
  EXPECT_FALSE(rt_diagnostic()) << "nothing to report";
  EXPECT_GT(system_node_->get_safety().get_rt_monitor().last_period(), 0.0065) <<
    "the cycles were really slower than 5 ms";
}

TEST_F(SystemSafetyModeTest, IsolatedOverrunsDoNotStopEasyNavInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }

  // The controller runs (and blocks 30 ms) only every 0.5 s: isolated late cycles.
  start(
    safe(
      hanging_controller(
        0.03, 2.0, {"safety.rt_monitor.max_late_cycles:=3", "cmd_timeout:=0.6"}, 0)));
  ASSERT_TRUE(configure());
  ASSERT_TRUE(activate());

  run_rt_at_rate(std::chrono::milliseconds(1300), true);
  const auto & monitor = system_node_->get_safety().get_rt_monitor();
  EXPECT_GE(monitor.late_cycles(), 1u) << "the overruns were seen";
  EXPECT_EQ(monitor.status(), easynav::safety::RtMonitor::Status::OK);
  EXPECT_FALSE(system_node_->is_shutdown_requested());
  ASSERT_TRUE(rt_diagnostic());
  EXPECT_EQ(rt_diagnostic()->level, diagnostic_msgs::msg::DiagnosticStatus::OK) <<
    "late, then on time again";
}
