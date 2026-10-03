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
/// \brief The safety module of easynav_system, on its own: configuration fingerprint, parameter
/// freezer and safety supervisor.

#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "easynav_interfaces/msg/heartbeat.hpp"
#include "easynav_interfaces/msg/safety_status.hpp"

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_core/SafetyChannel.hpp"
#include "easynav_core/VelocityCommand.hpp"
#include "easynav_system/RealTime.hpp"
#include "easynav_system/safety/ConfigurationFingerprint.hpp"
#include "easynav_system/safety/ParameterFreezer.hpp"
#include "easynav_system/safety/SafetySupervisor.hpp"

using easynav::safety::Nodes;

namespace
{

// Sets an environment variable for the scope, then restores it.
class ScopedEnv
{
public:
  ScopedEnv(const std::string & name, const std::string & value)
  : name_(name)
  {
    if (const char * old = std::getenv(name.c_str())) {
      old_ = old;
    }
    setenv(name.c_str(), value.c_str(), 1);
  }
  ~ScopedEnv()
  {
    if (old_) {
      setenv(name_.c_str(), old_->c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::optional<std::string> old_;
};

// A new, empty directory for this test.
std::filesystem::path scratch_dir(const std::string & name)
{
  const auto dir = std::filesystem::temp_directory_path() /
    ("easynav_safety_tests_" + std::to_string(getpid())) / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

std::string read(const std::string & path)
{
  std::ifstream file(path);
  return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

}  // namespace

class SafetyModuleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static rclcpp_lifecycle::LifecycleNode::SharedPtr node(
    const std::string & name, std::vector<rclcpp::Parameter> overrides = {})
  {
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      name, rclcpp::NodeOptions().parameter_overrides(overrides));
  }

  // A SystemNode-like node: the safety parameters and "use_real_time".
  static rclcpp_lifecycle::LifecycleNode::SharedPtr system(
    std::vector<rclcpp::Parameter> overrides, easynav::safety::SafetySupervisor & supervisor)
  {
    auto n = node("system_node", overrides);
    supervisor.declare_parameters(*n);
    n->declare_parameter("use_real_time", true);
    n->declare_parameter("rt_freq", 200.0);
    return n;
  }
};

// ─── ConfigurationFingerprint ───────────────────────────────────────────────────────────────

TEST_F(SafetyModuleTest, Sha256MatchesKnownVectors)
{
  EXPECT_EQ(
    easynav::safety::sha256_hex(""),
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(
    easynav::safety::sha256_hex("abc"),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST_F(SafetyModuleTest, DumpListsEveryParameterSortedByNodeAndName)
{
  auto b = node("b_node");
  b->declare_parameter("zeta", 1);
  b->declare_parameter("alpha.x", std::string("text"));
  auto a = node("a_node");
  a->declare_parameter("speed", 0.5);

  const auto dump = easynav::safety::configuration_dump({{"b_node", b}, {"a_node", a}});
  const auto pos_a = dump.find("a_node/speed (double) = 0.5\n");
  const auto pos_alpha = dump.find("b_node/alpha.x (string) = \"text\"\n");
  const auto pos_zeta = dump.find("b_node/zeta (integer) = 1\n");
  ASSERT_NE(pos_a, std::string::npos) << dump;
  ASSERT_NE(pos_alpha, std::string::npos) << dump;
  ASSERT_NE(pos_zeta, std::string::npos) << dump;
  EXPECT_LT(pos_a, pos_alpha);
  EXPECT_LT(pos_alpha, pos_zeta);
}

TEST_F(SafetyModuleTest, DumpDoesNotDependOnDeclarationOrder)
{
  auto first = node("n1");
  first->declare_parameter("x", 1.0);
  first->declare_parameter("y", 2.0);
  auto second = node("n2");
  second->declare_parameter("y", 2.0);
  second->declare_parameter("x", 1.0);
  EXPECT_EQ(
    easynav::safety::configuration_dump({{"n", first}}),
    easynav::safety::configuration_dump({{"n", second}}));
}

TEST_F(SafetyModuleTest, DifferentConfigurationsNeverGiveTheSameDump)
{
  // Pairs that an untyped "name=value" text could not tell apart.
  struct Case
  {
    const char * what;
    std::vector<rclcpp::Parameter> first;
    std::vector<rclcpp::Parameter> second;
  };
  const std::vector<Case> cases {
    {"type", {{"p", std::string("1")}}, {{"p", 1}}},
    {"integer and double", {{"p", 1}}, {{"p", 1.0}}},
    {"close doubles", {{"p", 0.1234567}}, {{"p", 0.1234568}}},
    {"a newline faking a parameter",
      {{"a", std::string("x\"\nn/b (string) = \"y")}},
      {{"a", std::string("x")}, {"b", std::string("y")}}},
    {"a comma in a string array",
      {{"p", std::vector<std::string>{"a, b"}}}, {{"p", std::vector<std::string>{"a", "b"}}}},
    {"quotes in a string array",
      {{"p", std::vector<std::string>{"a\", \"b"}}},
      {{"p", std::vector<std::string>{"a", "b"}}}},
    {"empty string and empty list",
      {{"p", std::string("")}}, {{"p", std::vector<std::string>{}}}},
  };
  for (const auto & c : cases) {
    auto first = node("n", c.first);
    for (const auto & p : c.first) {
      first->declare_parameter(p.get_name(), p.get_parameter_value());
    }
    auto second = node("n", c.second);
    for (const auto & p : c.second) {
      second->declare_parameter(p.get_name(), p.get_parameter_value());
    }
    const auto dump1 = easynav::safety::configuration_dump({{"n", first}});
    const auto dump2 = easynav::safety::configuration_dump({{"n", second}});
    EXPECT_NE(dump1, dump2) << c.what;
    EXPECT_NE(easynav::safety::sha256_hex(dump1), easynav::safety::sha256_hex(dump2)) << c.what;
  }
}

TEST_F(SafetyModuleTest, DumpIsReadable)
{
  auto n = node("n");
  n->declare_parameter("d", 0.1);
  n->declare_parameter("s", std::string("say \"hi\"\n"));
  n->declare_parameter("l", std::vector<double>{1.5, -2.0});
  n->declare_parameter("b", true);
  const auto dump = easynav::safety::configuration_dump({{"n", n}});
  EXPECT_NE(dump.find("n/d (double) = 0.1\n"), std::string::npos) << dump;
  EXPECT_NE(dump.find("n/s (string) = \"say \\\"hi\\\"\\n\"\n"), std::string::npos) << dump;
  EXPECT_NE(dump.find("n/l (double_array) = [1.5, -2]\n"), std::string::npos) << dump;
  EXPECT_NE(dump.find("n/b (bool) = true\n"), std::string::npos) << dump;
}

TEST_F(SafetyModuleTest, LoadedPluginsComeFromTheTypesParameters)
{
  auto n = node("controller_node");
  n->declare_parameter("controller_types", std::vector<std::string>{"ctrl", "other"});
  n->declare_parameter("ctrl.plugin", std::string("pkg/Controller"));
  n->declare_parameter("other.plugin", std::string("pkg/Other"));
  n->declare_parameter("unused.plugin", std::string("pkg/Unused"));  // Not listed: not loaded
  n->declare_parameter("not_types", 3);

  const auto plugins = easynav::safety::loaded_plugins({{"controller_node", n}});
  EXPECT_NE(plugins.find("controller_node: ctrl [pkg/Controller]"), std::string::npos) << plugins;
  EXPECT_NE(plugins.find("controller_node: other [pkg/Other]"), std::string::npos) << plugins;
  EXPECT_EQ(plugins.find("Unused"), std::string::npos) << plugins;
}

TEST_F(SafetyModuleTest, DumpFileNameIncludesTheNamespace)
{
  EXPECT_EQ(easynav::safety::dump_file_name("", "abc"), "easynav_configuration_abc.txt");
  EXPECT_EQ(easynav::safety::dump_file_name("/", "abc"), "easynav_configuration_abc.txt");
  EXPECT_EQ(
    easynav::safety::dump_file_name("/robot1", "abc"), "easynav_configuration_robot1_abc.txt");
  EXPECT_EQ(
    easynav::safety::dump_file_name("/fleet/robot1", "abc"),
    "easynav_configuration_fleet_robot1_abc.txt");
}

TEST_F(SafetyModuleTest, SaveDumpWritesOnceAndCreatesTheDirectories)
{
  namespace fs = std::filesystem;
  const auto dir = scratch_dir("save_dump");
  const auto path = (dir / "nested" / "dump.txt").string();

  EXPECT_EQ(easynav::safety::save_dump("a=1\n", path), "");
  EXPECT_EQ(read(path), "a=1\n");
  EXPECT_FALSE(fs::exists(path + ".tmp")) << "no temporary file left";

  // Already there (same hash, same contents): not written again.
  std::ofstream(path) << "edited";
  EXPECT_EQ(easynav::safety::save_dump("a=1\n", path), "");
  EXPECT_EQ(read(path), "edited");

  // A path that cannot be a file: the reason is returned.
  std::ofstream((dir / "a_file").string()) << "x";
  EXPECT_NE(easynav::safety::save_dump("a=1\n", (dir / "a_file" / "dump.txt").string()), "");
}

TEST_F(SafetyModuleTest, LogDirectoryFollowsRosLogDir)
{
  const auto dir = scratch_dir("log_dir").string();
  ScopedEnv env("ROS_LOG_DIR", dir);
  EXPECT_EQ(easynav::safety::log_directory(), dir);
}

// ─── ParameterFreezer ───────────────────────────────────────────────────────────────────────

TEST_F(SafetyModuleTest, FrozenParametersCannotChange)
{
  auto a = node("a");
  a->declare_parameter("speed", 0.5);
  auto b = node("b");
  b->declare_parameter("name", std::string("robot"));

  easynav::safety::ParameterFreezer freezer;
  EXPECT_FALSE(freezer.is_frozen());
  EXPECT_TRUE(a->set_parameter(rclcpp::Parameter("speed", 0.6)).successful) << "not frozen yet";
  freezer.freeze({{"a", a}, {"b", b}});
  EXPECT_TRUE(freezer.is_frozen());

  const auto result = a->set_parameter(rclcpp::Parameter("speed", 0.7));
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("a/speed"), std::string::npos) << result.reason;
  EXPECT_DOUBLE_EQ(a->get_parameter("speed").as_double(), 0.6);
  EXPECT_FALSE(b->set_parameter(rclcpp::Parameter("name", std::string("other"))).successful);

  // The same value is accepted.
  EXPECT_TRUE(a->set_parameter(rclcpp::Parameter("speed", 0.6)).successful);

  // A new parameter, only while new ones are accepted (EasyNav configuring)...
  freezer.accept_new_parameters(false);
  EXPECT_THROW(
    a->declare_parameter("new_one", 1), rclcpp::exceptions::InvalidParameterValueException);
  EXPECT_FALSE(a->has_parameter("new_one"));
  freezer.accept_new_parameters(true);
  EXPECT_NO_THROW(a->declare_parameter("new_one", 1));
  EXPECT_TRUE(a->set_parameter(rclcpp::Parameter("new_one", 2)).successful) << "not frozen yet";

  // ...and refreezing takes it, without adding callbacks.
  freezer.freeze({{"a", a}, {"b", b}});
  freezer.accept_new_parameters(false);
  EXPECT_FALSE(a->set_parameter(rclcpp::Parameter("new_one", 3)).successful);
  EXPECT_TRUE(a->set_parameter(rclcpp::Parameter("new_one", 2)).successful);
}

TEST_F(SafetyModuleTest, AnAtomicSetWithOneFrozenChangeIsRejectedWhole)
{
  auto a = node("a");
  a->declare_parameter("speed", 0.5);
  a->declare_parameter("other", 1);
  easynav::safety::ParameterFreezer freezer;
  freezer.freeze({{"a", a}});

  const auto result = a->set_parameters_atomically(
    {rclcpp::Parameter("other", 1), rclcpp::Parameter("speed", 0.9)});
  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(a->get_parameter("speed").as_double(), 0.5);
}

// ─── SafetySupervisor ───────────────────────────────────────────────────────────────────────

TEST_F(SafetyModuleTest, SupervisorChecksTheSafetyLimits)
{
  struct Case
  {
    std::vector<rclcpp::Parameter> params;
    bool valid;
  };
  const std::vector<Case> cases {
    {{}, true},  // Not given, outside the safety mode: fine
    {{{"safety.plc_limits.max_linear_vel", -1.0}}, false},
    {{{"safety.plc_limits.max_angular_vel", std::nan("")}}, false},
    {{{"safety.mode", true}}, false},  // Required in safety.mode
    {{{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0}}, false},
    {{{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1},
      {"safety.status.timeout", 0.5}}, true},
    {{{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1},
      {"safety.status.timeout", 0.5},
      {"use_real_time", false}}, false},
  };
  // The safety mode also needs real-time scheduling, checked on configure.
  const bool real_time = easynav::check_real_time_priority(easynav::kRealTimePriority).empty();
  for (size_t i = 0; i < cases.size(); ++i) {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system(cases[i].params, supervisor);
    const bool mode = n->get_parameter("safety.mode").as_bool();
    EXPECT_EQ(supervisor.check_system(*n), cases[i].valid && (!mode || real_time)) <<
      "case " << i;
  }
}

TEST_F(SafetyModuleTest, SafetyModeFailsToConfigureWithoutRealTimeScheduling)
{
  rlimit original {};
  ASSERT_EQ(getrlimit(RLIMIT_RTPRIO, &original), 0);
  rlimit none = original;
  none.rlim_cur = 0;  // Lowering the soft limit is always allowed.
  ASSERT_EQ(setrlimit(RLIMIT_RTPRIO, &none), 0);
  const bool privileged = easynav::check_real_time_priority(easynav::kRealTimePriority).empty();

  bool valid = true;
  if (!privileged) {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system(
      {{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
        {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1},
        {"safety.status.timeout", 0.5}}, supervisor);
    valid = supervisor.check_system(*n);
  }
  ASSERT_EQ(setrlimit(RLIMIT_RTPRIO, &original), 0);
  if (privileged) {
    GTEST_SKIP() << "this process may use SCHED_FIFO whatever its RLIMIT_RTPRIO";
  }
  EXPECT_FALSE(valid);
}

TEST_F(SafetyModuleTest, LockMemoryFailsToConfigureWithAFiniteMemlockLimit)
{
  rlimit original {};
  ASSERT_EQ(getrlimit(RLIMIT_MEMLOCK, &original), 0);
  rlimit finite = original;
  finite.rlim_cur = 64 * 1024;
  ASSERT_EQ(setrlimit(RLIMIT_MEMLOCK, &finite), 0);

  easynav::safety::SafetySupervisor supervisor;
  auto n = system({{"safety.lock_memory", true}}, supervisor);
  const bool valid = supervisor.check_system(*n);
  ASSERT_EQ(setrlimit(RLIMIT_MEMLOCK, &original), 0);
  EXPECT_FALSE(valid) << "in any mode";
}

TEST_F(SafetyModuleTest, MemoryLockIsRequestedIndependentlyOfTheSafetyMode)
{
  const bool real_time = easynav::check_real_time_priority(easynav::kRealTimePriority).empty();
  const bool memlock = easynav::check_memory_lock().empty();
  struct Case
  {
    bool mode;
    std::optional<bool> lock_memory;
  };
  for (const auto & c : std::vector<Case>{
    {false, std::nullopt}, {false, true}, {true, std::nullopt}, {true, false}, {true, true}})
  {
    std::vector<rclcpp::Parameter> params {
      {"safety.mode", c.mode}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1},
      {"safety.status.timeout", 0.5}};
    if (c.lock_memory) {
      params.emplace_back("safety.lock_memory", *c.lock_memory);
    }
    easynav::safety::SafetySupervisor supervisor;
    auto n = system(params, supervisor);
    // Only what each one requires is checked.
    const bool valid = (!c.mode || real_time) && (!c.lock_memory.value_or(false) || memlock);
    EXPECT_EQ(supervisor.check_system(*n), valid);
    EXPECT_EQ(supervisor.is_safety_mode(), c.mode);
    EXPECT_EQ(supervisor.is_memory_lock_requested(), c.lock_memory.value_or(false)) <<
      "off by default, even in the safety mode";
  }
}

TEST_F(SafetyModuleTest, SupervisorChecksTheControllerAgainstTheSafetyLimits)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.plc_limits.max_linear_vel", 0.5}, {"safety.plc_limits.max_angular_vel", 1.0},
      {"safety.heartbeat.period", 0.1}, {"safety.status.timeout", 0.5}},
    supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));

  auto controller = [](std::vector<rclcpp::Parameter> params) {
      return std::make_shared<easynav::ControllerNode>(
        rclcpp::NodeOptions().parameter_overrides(params));
    };
  EXPECT_TRUE(supervisor.check_controller(*controller({{"robot_limits.max_linear_vel", 0.5}})));
  EXPECT_FALSE(supervisor.check_controller(*controller({{"robot_limits.max_linear_vel", 0.6}})));
  EXPECT_FALSE(supervisor.check_controller(*controller({{"robot_limits.min_linear_vel", -0.6}})));
  EXPECT_FALSE(supervisor.check_controller(*controller({{"robot_limits.max_angular_vel", 1.1}})));
  EXPECT_TRUE(
    supervisor.check_controller(*controller({{"cmd_vel_keepalive_period", 0.0}}))) <<
    "not in safety mode";
}

TEST_F(SafetyModuleTest, SupervisorRequiresTheCommandGuardInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }
  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 2.0}, {"safety.heartbeat.period", 0.1},
      {"safety.status.timeout", 0.5}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));

  auto controller = [](std::vector<rclcpp::Parameter> params) {
      return std::make_shared<easynav::ControllerNode>(
        rclcpp::NodeOptions().parameter_overrides(params));
    };
  EXPECT_FALSE(supervisor.check_controller(*controller({})));  // No keepalive by default
  EXPECT_TRUE(supervisor.check_controller(*controller({{"cmd_vel_keepalive_period", 0.1}})));
  EXPECT_FALSE(
    supervisor.check_controller(
      *controller({{"cmd_vel_keepalive_period", 0.1}, {"cmd_timeout", 0.0}})));
}

TEST_F(SafetyModuleTest, SupervisorFingerprintsAndFreezesOnlyInSafetyMode)
{
  const bool real_time = easynav::check_real_time_priority(easynav::kRealTimePriority).empty();
  for (const bool safety_mode : {false, true}) {
    if (safety_mode && !real_time) {
      continue;  // The safety mode needs real-time scheduling, not allowed here.
    }
    easynav::safety::SafetySupervisor supervisor;
    auto n = system(
      {{"safety.mode", safety_mode}, {"safety.plc_limits.max_linear_vel", 1.0},
        {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1},
        {"safety.status.timeout", 0.5}}, supervisor);
    ASSERT_TRUE(supervisor.check_system(*n));
    EXPECT_EQ(supervisor.is_safety_mode(), safety_mode);
    EXPECT_EQ(supervisor.get_configuration_hash(), "") << "before the first configure";

    easynav::NavState nav_state;
    const Nodes nodes {{"system_node", n}};
    supervisor.on_configured(nodes, nav_state);
    const auto hash = supervisor.get_configuration_hash();
    EXPECT_EQ(hash, easynav::safety::sha256_hex(easynav::safety::configuration_dump(nodes)));
    EXPECT_EQ(nav_state.get_safe<std::string>("configuration_hash"), hash);

    EXPECT_EQ(
      n->set_parameter(rclcpp::Parameter("safety.plc_limits.max_linear_vel", 2.0)).successful,
      !safety_mode) << safety_mode;
    EXPECT_EQ(supervisor.allows_reconfiguration("test"), !safety_mode) << safety_mode;
  }
}

TEST_F(SafetyModuleTest, SupervisorSavesAndPublishesTheConfiguration)
{
  const auto dir = scratch_dir("supervisor");
  ScopedEnv env("ROS_LOG_DIR", dir.string());

  easynav::safety::SafetySupervisor supervisor;
  auto n = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "system_node", "/robot7", rclcpp::NodeOptions());
  supervisor.declare_parameters(*n);
  n->declare_parameter("use_real_time", true);
  n->declare_parameter("rt_freq", 200.0);
  ASSERT_TRUE(supervisor.check_system(*n));

  easynav::NavState nav_state;
  const Nodes nodes {{"system_node", n}};
  supervisor.on_configured(nodes, nav_state);
  const auto hash = supervisor.get_configuration_hash();
  const auto dump = easynav::safety::configuration_dump(nodes);

  // Saved in the ROS log directory, named after the namespace and the hash.
  EXPECT_EQ(read((dir / ("easynav_configuration_robot7_" + hash + ".txt")).string()), dump);

  // Published latched: a subscriber that comes later still gets it.
  auto listener = std::make_shared<rclcpp::Node>("configuration_listener", "/robot7");
  std::optional<std::string> received;
  auto sub = listener->create_subscription<std_msgs::msg::String>(
    "easynav_configuration", rclcpp::QoS(1).reliable().transient_local(),
    [&received](std_msgs::msg::String::UniquePtr msg) {received = msg->data;});
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(listener);
  const auto start = std::chrono::steady_clock::now();
  while (!received && std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    exe.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(*received, "# SHA-256: " + hash + "\n" + dump);
}

TEST_F(SafetyModuleTest, NewParametersOnlyWhileConfiguringInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }

  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1},
      {"safety.status.timeout", 0.5}}, supervisor);
  easynav::NavState nav_state;
  const Nodes nodes {{"system_node", n}};

  // Configuring: a plugin may declare its parameters.
  ASSERT_TRUE(supervisor.check_system(*n));
  EXPECT_NO_THROW(n->declare_parameter("plugin.gain", 1.0));
  supervisor.on_configured(nodes, nav_state);

  // Configured: frozen, new parameters included.
  EXPECT_THROW(
    n->declare_parameter("plugin.other", 1.0), rclcpp::exceptions::InvalidParameterValueException);
  EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("plugin.gain", 2.0)).successful);

  // Configuring again (cleanup and configure): declarations allowed until configured.
  ASSERT_TRUE(supervisor.check_system(*n));
  EXPECT_NO_THROW(n->declare_parameter("plugin.other", 1.0));
  EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("plugin.gain", 2.0)).successful) <<
    "still frozen while configuring";
  supervisor.on_configured(nodes, nav_state);
  EXPECT_FALSE(n->set_parameter(rclcpp::Parameter("plugin.other", 2.0)).successful);
}

// ─── RT cycle: monitor and heartbeat ────────────────────────────────────────────────────────

namespace
{

using RtClock = easynav::safety::RtMonitor::Clock;
using namespace std::chrono_literals;

std::optional<diagnostic_msgs::msg::DiagnosticStatus> rt_diagnostic(
  const easynav::NavState & nav_state)
{
  if (!nav_state.has("diagnostics.rt_cycle")) {return std::nullopt;}
  return nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.rt_cycle");
}

}  // namespace

TEST_F(SafetyModuleTest, HeartbeatAndRtMonitorParametersAreChecked)
{
  struct Case
  {
    std::vector<rclcpp::Parameter> params;
    bool valid;
  };
  const std::vector<Case> cases {
    {{{"safety.heartbeat.period", -0.1}}, false},
    {{{"safety.heartbeat.period", std::nan("")}}, false},
    {{{"safety.heartbeat.period", 0.0}}, true},  // Off, outside the safety mode
    {{{"safety.rt_monitor.max_period_factor", 1.0}}, false},
    {{{"safety.rt_monitor.max_period_factor", 0.5}}, false},
    {{{"safety.rt_monitor.max_period_factor", std::nan("")}}, false},
    {{{"safety.rt_monitor.max_period_factor", 1.01}}, true},
    {{{"safety.rt_monitor.max_late_cycles", 0}}, false},
    {{{"safety.rt_monitor.max_late_cycles", 1}}, true},
  };
  for (size_t i = 0; i < cases.size(); ++i) {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system(cases[i].params, supervisor);
    EXPECT_EQ(supervisor.check_system(*n), cases[i].valid) << "case " << i;
  }
}

TEST_F(SafetyModuleTest, TheHeartbeatIsRequiredInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }

  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}}, supervisor);
  EXPECT_FALSE(supervisor.check_system(*n));
}

TEST_F(SafetyModuleTest, HeartbeatIsPublishedAtItsPeriodFromTheRtCycle)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system({{"safety.heartbeat.period", 0.05}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  easynav::NavState nav_state;
  supervisor.on_configured({{"system_node", n}}, nav_state);
  supervisor.on_activate();

  // The publisher promises its period.
  const auto info = n->get_publishers_info_by_topic("/easynav_heartbeat");
  ASSERT_EQ(info.size(), 1u);
  EXPECT_EQ(info[0].qos_profile().deadline(), rclcpp::Duration(100ms));
  EXPECT_EQ(info[0].qos_profile().liveliness_lease_duration(), rclcpp::Duration(100ms));

  auto listener = std::make_shared<rclcpp::Node>("heartbeat_listener");
  std::vector<easynav_interfaces::msg::Heartbeat> received;
  auto sub = listener->create_subscription<easynav_interfaces::msg::Heartbeat>(
    "/easynav_heartbeat", rclcpp::QoS(100).reliable(),
    [&received](easynav_interfaces::msg::Heartbeat::UniquePtr msg) {received.push_back(*msg);});
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(listener);
  const auto start = std::chrono::steady_clock::now();
  while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() - start < 2s) {
    exe.spin_some();
    rclcpp::sleep_for(10ms);
  }

  // 1 s of simulated 200 Hz cycles: a heartbeat every 50 ms, the first one right away.
  auto t = RtClock::now();
  for (int i = 0; i < 200; ++i, t += 5ms) {
    ASSERT_TRUE(supervisor.cycle_rt(nav_state, t));
    exe.spin_some();
    rclcpp::sleep_for(1ms);
  }
  const auto end = std::chrono::steady_clock::now();
  while (received.size() < 20 && std::chrono::steady_clock::now() - end < 1s) {
    exe.spin_some();
    rclcpp::sleep_for(5ms);
  }
  ASSERT_EQ(received.size(), 20u);
  for (size_t i = 0; i < received.size(); ++i) {
    EXPECT_EQ(received[i].sequence, i + 1) << "consecutive, no gaps";
    EXPECT_EQ(received[i].rt_status, easynav_interfaces::msg::Heartbeat::RT_OK);
    EXPECT_EQ(received[i].late_cycles, 0u);
    EXPECT_FALSE(received[i].safety_mode);
    EXPECT_EQ(received[i].configuration_hash, supervisor.get_configuration_hash());
  }
}

TEST_F(SafetyModuleTest, NoHeartbeatIfItsPeriodIsZero)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system({}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  EXPECT_TRUE(n->get_publishers_info_by_topic("/easynav_heartbeat").empty());
  easynav::NavState nav_state;
  supervisor.on_activate();
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
}

TEST_F(SafetyModuleTest, RtDiagnosticFollowsTheMonitorOutsideSafetyMode)
{
  // 200 Hz, late if more than 10 ms after the previous cycle; 3 in a row are an error.
  easynav::safety::SafetySupervisor supervisor;
  auto n = system({{"safety.rt_monitor.max_late_cycles", 3}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  easynav::NavState nav_state;
  supervisor.on_activate();

  auto t = RtClock::now();
  for (int i = 0; i < 10; ++i, t += 5ms) {
    supervisor.cycle_rt(nav_state, t);
  }
  EXPECT_FALSE(rt_diagnostic(nav_state)) << "nothing to report while all is fine";

  using diagnostic_msgs::msg::DiagnosticStatus;
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, t += 30ms));
  ASSERT_TRUE(rt_diagnostic(nav_state));
  EXPECT_EQ(rt_diagnostic(nav_state)->level, DiagnosticStatus::WARN);
  EXPECT_EQ(rt_diagnostic(nav_state)->hardware_id, "system_node");

  supervisor.cycle_rt(nav_state, t += 30ms);
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, t += 30ms)) << "only reported outside safety mode";
  EXPECT_EQ(rt_diagnostic(nav_state)->level, DiagnosticStatus::ERROR);
  const auto keys = nav_state.get_group_keys("diagnostics");
  EXPECT_NE(std::find(keys.begin(), keys.end(), "diagnostics.rt_cycle"), keys.end());

  supervisor.cycle_rt(nav_state, t += 5ms);
  EXPECT_EQ(rt_diagnostic(nav_state)->level, DiagnosticStatus::OK);
  EXPECT_EQ(supervisor.get_rt_monitor().late_cycles(), 3u);
}

TEST_F(SafetyModuleTest, TooManyLateRtCyclesStopEasyNavInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }

  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1},
      {"safety.status.timeout", 0.5},
      {"safety.rt_monitor.max_late_cycles", 2}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  easynav::NavState nav_state;
  supervisor.on_activate();

  auto t = RtClock::now();
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, t));
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, t += 30ms)) << "one late cycle is tolerated";
  EXPECT_FALSE(supervisor.cycle_rt(nav_state, t += 30ms));
  EXPECT_NE(supervisor.failure().find("late"), std::string::npos) << supervisor.failure();
  EXPECT_FALSE(supervisor.cycle_rt(nav_state, t += 5ms)) << "an on-time cycle no longer matters";

  // Activated again (e.g. after a restart), it starts over.
  supervisor.on_activate();
  EXPECT_TRUE(supervisor.failure().empty());
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, t += 10s));
}

TEST_F(SafetyModuleTest, TheHeartbeatCarriesTheRtStatus)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.heartbeat.period", 0.001}, {"safety.rt_monitor.max_late_cycles", 2}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  easynav::NavState nav_state;
  supervisor.on_activate();

  auto listener = std::make_shared<rclcpp::Node>("heartbeat_status_listener");
  std::vector<easynav_interfaces::msg::Heartbeat> received;
  auto sub = listener->create_subscription<easynav_interfaces::msg::Heartbeat>(
    "/easynav_heartbeat", rclcpp::QoS(100).reliable(),
    [&received](easynav_interfaces::msg::Heartbeat::UniquePtr msg) {received.push_back(*msg);});
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(listener);
  const auto start = std::chrono::steady_clock::now();
  while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() - start < 2s) {
    exe.spin_some();
    rclcpp::sleep_for(10ms);
  }

  auto t = RtClock::now();
  for (const auto step : {0ms, 30ms, 30ms, 5ms}) {  // OK, LATE, ERROR, OK
    supervisor.cycle_rt(nav_state, t += step);
    const auto wait = std::chrono::steady_clock::now();
    const auto expected = received.size() + 1;
    while (received.size() < expected && std::chrono::steady_clock::now() - wait < 1s) {
      exe.spin_some();
      rclcpp::sleep_for(5ms);
    }
  }
  using easynav_interfaces::msg::Heartbeat;
  ASSERT_EQ(received.size(), 4u);
  EXPECT_EQ(received[0].rt_status, Heartbeat::RT_OK);
  EXPECT_EQ(received[1].rt_status, Heartbeat::RT_LATE);
  EXPECT_EQ(received[1].late_cycles, 1u);
  EXPECT_EQ(received[2].rt_status, Heartbeat::RT_ERROR);
  EXPECT_EQ(received[3].rt_status, Heartbeat::RT_OK);
  EXPECT_EQ(received[3].late_cycles, 2u);
}

// ─── Safety channel ─────────────────────────────────────────────────────────────────────────

namespace
{

std::optional<diagnostic_msgs::msg::DiagnosticStatus> safety_diagnostic(
  const easynav::NavState & nav_state)
{
  if (!nav_state.has("diagnostics.safety_status")) {return std::nullopt;}
  return nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.safety_status");
}

easynav::SafetyChannelState channel_state(const easynav::NavState & nav_state)
{
  return nav_state.get_safe<easynav::SafetyChannelState>(easynav::kSafetyStatusKey);
}

}  // namespace

TEST_F(SafetyModuleTest, SafetyStatusTimeoutIsChecked)
{
  const std::vector<std::pair<double, bool>> cases {
    {-0.1, false}, {std::nan(""), false}, {std::numeric_limits<double>::infinity(), false},
    {0.0, true},  // Off, outside the safety mode
    {0.5, true}};
  for (const auto & [timeout, valid] : cases) {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system({{"safety.status.timeout", timeout}}, supervisor);
    EXPECT_EQ(supervisor.check_system(*n), valid) << timeout;
    EXPECT_EQ(supervisor.is_safety_status_enabled(), valid && timeout > 0.0) << timeout;
  }
}

TEST_F(SafetyModuleTest, TheSafetyStatusIsRequiredInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }

  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1}}, supervisor);
  EXPECT_FALSE(supervisor.check_system(*n));
}

TEST_F(SafetyModuleTest, WithoutSafetyStatusThereIsNoRestriction)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system({}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  EXPECT_TRUE(n->get_subscriptions_info_by_topic("/easynav_safety_status").empty());

  easynav::NavState nav_state;
  supervisor.on_configured({{"system_node", n}}, nav_state);
  supervisor.on_activate();
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_EQ(channel_state(nav_state), easynav::SafetyChannelState());
  EXPECT_FALSE(safety_diagnostic(nav_state)) << "nothing to report";
}

TEST_F(SafetyModuleTest, TheSafetyChannelStateIsWrittenEveryRtCycleAndReportedOnChanges)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system({{"safety.status.timeout", 0.2}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  easynav::NavState nav_state;
  supervisor.on_configured({{"system_node", n}}, nav_state);
  EXPECT_TRUE(channel_state(nav_state).protective_stop) << "a stop until the first status";
  supervisor.on_activate();

  auto publisher_node = std::make_shared<rclcpp::Node>("safety_channel");
  auto pub = publisher_node->create_publisher<easynav_interfaces::msg::SafetyStatus>(
    "/easynav_safety_status", rclcpp::QoS(1).reliable());
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(n->get_node_base_interface());  // No RT group given: the default one.
  exe.add_node(publisher_node);
  const auto start = std::chrono::steady_clock::now();
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() - start < 2s) {
    exe.spin_some();
    rclcpp::sleep_for(10ms);
  }
  ASSERT_GT(pub->get_subscription_count(), 0u);

  // No status yet: stopped, an ERROR.
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_TRUE(channel_state(nav_state).status_lost);
  ASSERT_TRUE(safety_diagnostic(nav_state));
  EXPECT_EQ(safety_diagnostic(nav_state)->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(safety_diagnostic(nav_state)->hardware_id, "system_node");

  // A speed limit, in the field "slow".
  easynav_interfaces::msg::SafetyStatus msg;
  msg.speed_limited = true;
  msg.max_linear_vel = 0.3;
  msg.max_angular_vel = 0.6;
  msg.active_field = "slow";
  auto send = [&](const easynav_interfaces::msg::SafetyStatus & status) {
      pub->publish(status);
      const auto sent = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - sent < 50ms) {
        exe.spin_some();
        rclcpp::sleep_for(5ms);
      }
    };
  send(msg);
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  auto state = channel_state(nav_state);
  EXPECT_FALSE(state.protective_stop);
  EXPECT_DOUBLE_EQ(state.max_linear_vel, 0.3);
  EXPECT_DOUBLE_EQ(state.max_angular_vel, 0.6);
  auto diagnostic = safety_diagnostic(nav_state).value();
  EXPECT_EQ(diagnostic.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_NE(diagnostic.message.find("speed limited to 0.3"), std::string::npos);
  ASSERT_EQ(diagnostic.values.size(), 2u);
  EXPECT_EQ(diagnostic.values[0].key, "active_field");
  EXPECT_EQ(diagnostic.values[0].value, "slow");
  EXPECT_EQ(diagnostic.values[1].value, "false");

  // Unchanged: not reported again.
  diagnostic.message = "marker";
  nav_state.set("diagnostics.safety_status", diagnostic);
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_EQ(safety_diagnostic(nav_state)->message, "marker");

  // A protective stop: a WARN, and still running (EasyNav does not stop for it).
  msg.protective_stop = true;
  msg.muting = true;
  send(msg);
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_TRUE(channel_state(nav_state).protective_stop);
  EXPECT_FALSE(channel_state(nav_state).status_lost);
  diagnostic = safety_diagnostic(nav_state).value();
  EXPECT_EQ(diagnostic.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(diagnostic.values[1].value, "true");

  // Silent for longer than the timeout: lost.
  rclcpp::sleep_for(250ms);
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_TRUE(channel_state(nav_state).status_lost);
  diagnostic = safety_diagnostic(nav_state).value();
  EXPECT_EQ(diagnostic.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(diagnostic.message.find("No safety status for more than"), std::string::npos);

  // Invalid: still stopped, with the reason.
  msg.protective_stop = false;
  msg.max_linear_vel = -1.0;
  send(msg);
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_TRUE(channel_state(nav_state).protective_stop);
  diagnostic = safety_diagnostic(nav_state).value();
  EXPECT_NE(diagnostic.message.find("max_linear_vel"), std::string::npos) << diagnostic.message;
}

TEST_F(SafetyModuleTest, DisablingTheSafetyStatusOnReconfigureLiftsItsRestrictions)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system({{"safety.status.timeout", 0.2}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  easynav::NavState nav_state;
  supervisor.on_configured({{"system_node", n}}, nav_state);
  ASSERT_TRUE(channel_state(nav_state).protective_stop);

  n->set_parameter(rclcpp::Parameter("safety.status.timeout", 0.0));
  ASSERT_TRUE(supervisor.check_system(*n));
  EXPECT_FALSE(supervisor.is_safety_status_enabled());
  supervisor.on_configured({{"system_node", n}}, nav_state);
  EXPECT_EQ(channel_state(nav_state), easynav::SafetyChannelState());
}

// ─── Robot pose age ─────────────────────────────────────────────────────────────────────────

namespace
{

void set_pose(
  easynav::NavState & nav_state, const rclcpp::Node::SharedPtr & clock_node,
  std::chrono::milliseconds age)
{
  nav_msgs::msg::Odometry pose;
  pose.header.frame_id = "map";
  pose.header.stamp = clock_node->now() - rclcpp::Duration(age);
  nav_state.set("robot_pose", pose);
}

std::optional<diagnostic_msgs::msg::DiagnosticStatus> pose_diagnostic(
  const easynav::NavState & nav_state)
{
  if (!nav_state.has("diagnostics.robot_pose")) {return std::nullopt;}
  return nav_state.get_safe<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.robot_pose");
}

}  // namespace

TEST_F(SafetyModuleTest, MaxPoseAgeIsChecked)
{
  const std::vector<std::pair<double, bool>> cases {
    {-0.1, false}, {std::nan(""), false}, {std::numeric_limits<double>::infinity(), false},
    {0.0, true}, {0.5, true}};
  for (const auto & [age, valid] : cases) {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system({{"safety.max_pose_age", age}}, supervisor);
    EXPECT_EQ(supervisor.check_system(*n), valid) << age;
  }
}

TEST_F(SafetyModuleTest, AnOldRobotPoseIsAnErrorOutsideSafetyModeButDoesNotStopTheRobot)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system({{"safety.max_pose_age", 0.5}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  supervisor.on_activate();
  auto clock_node = std::make_shared<rclcpp::Node>("pose_clock");
  easynav::NavState nav_state;

  set_pose(nav_state, clock_node, 100ms);
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_FALSE(pose_diagnostic(nav_state)) << "nothing to report until something goes wrong";

  set_pose(nav_state, clock_node, 600ms);
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  ASSERT_TRUE(pose_diagnostic(nav_state));
  EXPECT_EQ(pose_diagnostic(nav_state)->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(pose_diagnostic(nav_state)->hardware_id, "system_node");
  EXPECT_NE(pose_diagnostic(nav_state)->message.find("robot_pose is"), std::string::npos);
  EXPECT_FALSE(nav_state.has(easynav::kInhibitMotionKey)) << "only in safety mode";

  set_pose(nav_state, clock_node, 0ms);
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_EQ(pose_diagnostic(nav_state)->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(SafetyModuleTest, TheRobotPoseAgeIsReportedOnlyOnChanges)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system({{"safety.max_pose_age", 0.5}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  supervisor.on_activate();
  auto clock_node = std::make_shared<rclcpp::Node>("pose_clock");
  easynav::NavState nav_state;

  set_pose(nav_state, clock_node, 600ms);
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  auto marked = pose_diagnostic(nav_state).value();
  marked.message = "marker";
  nav_state.set("diagnostics.robot_pose", marked);
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_EQ(pose_diagnostic(nav_state)->message, "marker");

  supervisor.on_activate();  // Starts over: reported again.
  ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_NE(pose_diagnostic(nav_state)->message, "marker");
}

TEST_F(SafetyModuleTest, NoRobotPoseOrNotLocalizedYetOrOffIsNotChecked)
{
  auto clock_node = std::make_shared<rclcpp::Node>("pose_clock");
  {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system({{"safety.max_pose_age", 0.5}}, supervisor);
    ASSERT_TRUE(supervisor.check_system(*n));
    supervisor.on_activate();
    easynav::NavState nav_state;
    ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));  // No robot_pose
    nav_state.set("robot_pose", nav_msgs::msg::Odometry());  // Stamp 0: not localized yet
    ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
    EXPECT_FALSE(pose_diagnostic(nav_state));
  }
  {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system({{"safety.max_pose_age", 0.0}}, supervisor);
    ASSERT_TRUE(supervisor.check_system(*n));
    supervisor.on_activate();
    easynav::NavState nav_state;
    set_pose(nav_state, clock_node, 10000ms);
    ASSERT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
    EXPECT_FALSE(pose_diagnostic(nav_state)) << "off";
  }
}

TEST_F(SafetyModuleTest, AnOldRobotPoseInhibitsMotionInSafetyMode)
{
  if (!easynav::check_real_time_priority(easynav::kRealTimePriority).empty()) {
    GTEST_SKIP() << "the safety mode needs real-time scheduling, not allowed here";
  }
  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}, {"safety.heartbeat.period", 0.1},
      {"safety.status.timeout", 0.5}, {"safety.max_pose_age", 0.5}}, supervisor);
  ASSERT_TRUE(supervisor.check_system(*n));
  supervisor.on_activate();
  auto clock_node = std::make_shared<rclcpp::Node>("pose_clock");
  easynav::NavState nav_state;

  set_pose(nav_state, clock_node, 100ms);
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_FALSE(nav_state.get<bool>(easynav::kInhibitMotionKey));

  set_pose(nav_state, clock_node, 600ms);
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now())) << "EasyNav keeps running";
  EXPECT_TRUE(nav_state.get<bool>(easynav::kInhibitMotionKey));
  EXPECT_NE(pose_diagnostic(nav_state)->message.find("motion inhibited"), std::string::npos);

  set_pose(nav_state, clock_node, 0ms);
  EXPECT_TRUE(supervisor.cycle_rt(nav_state, RtClock::now()));
  EXPECT_FALSE(nav_state.get<bool>(easynav::kInhibitMotionKey));
}
