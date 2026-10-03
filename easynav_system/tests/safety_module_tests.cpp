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

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "easynav_controller/ControllerNode.hpp"
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
  const auto pos_a = dump.find("a_node/speed=0.5");
  const auto pos_alpha = dump.find("b_node/alpha.x=text\n");
  const auto pos_zeta = dump.find("b_node/zeta=1\n");
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

  // The same value, or a new parameter, is accepted.
  EXPECT_TRUE(a->set_parameter(rclcpp::Parameter("speed", 0.6)).successful);
  EXPECT_NO_THROW(a->declare_parameter("new_one", 1));
  EXPECT_TRUE(a->set_parameter(rclcpp::Parameter("new_one", 2)).successful) << "not frozen yet";

  // Refreezing takes what was declared since, without adding callbacks.
  freezer.freeze({{"a", a}, {"b", b}});
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
      {"safety.plc_limits.max_angular_vel", 1.0}}, true},
    {{{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 1.0}, {"use_real_time", false}}, false},
  };
  for (size_t i = 0; i < cases.size(); ++i) {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system(cases[i].params, supervisor);
    EXPECT_EQ(supervisor.check_system(*n), cases[i].valid) << "case " << i;
  }
}

TEST_F(SafetyModuleTest, MemoryLockIsRequestedIndependentlyOfTheSafetyMode)
{
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
      {"safety.plc_limits.max_angular_vel", 1.0}};
    if (c.lock_memory) {
      params.emplace_back("safety.lock_memory", *c.lock_memory);
    }
    easynav::safety::SafetySupervisor supervisor;
    auto n = system(params, supervisor);
    EXPECT_TRUE(supervisor.check_system(*n)) << "it does not affect the checks";
    EXPECT_EQ(supervisor.is_safety_mode(), c.mode);
    EXPECT_EQ(supervisor.is_memory_lock_requested(), c.lock_memory.value_or(false)) <<
      "off by default, even in the safety mode";
  }
}

TEST_F(SafetyModuleTest, SupervisorChecksTheControllerAgainstTheSafetyLimits)
{
  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.plc_limits.max_linear_vel", 0.5}, {"safety.plc_limits.max_angular_vel", 1.0}},
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
  easynav::safety::SafetySupervisor supervisor;
  auto n = system(
    {{"safety.mode", true}, {"safety.plc_limits.max_linear_vel", 1.0},
      {"safety.plc_limits.max_angular_vel", 2.0}}, supervisor);
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
  for (const bool safety_mode : {false, true}) {
    easynav::safety::SafetySupervisor supervisor;
    auto n = system(
      {{"safety.mode", safety_mode}, {"safety.plc_limits.max_linear_vel", 1.0},
        {"safety.plc_limits.max_angular_vel", 1.0}}, supervisor);
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
