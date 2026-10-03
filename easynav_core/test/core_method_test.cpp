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

#include <atomic>
#include <chrono>
#include <string>
#include <vector>
#include <memory>
#include <limits>
#include <stdexcept>
#include <thread>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/RobotGeometry.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_core/MethodBase.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_core/ControllerMethodBase.hpp"

class CoreMethodTestCase : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

// Mock class to test the behaviour of MethodBase initialization
class MockMethod : public easynav::MethodBase
{
public:
  MockMethod() = default;
  ~MockMethod() = default;

  void on_initialize() override
  {
    on_initialize_called_ = true;
  }

public:
  bool on_initialize_called_ {false};
};

/** Dummy method class to test behvaiour of derived methods */
class TestLocalizer : public easynav::LocalizerMethodBase
{
public:
  TestLocalizer() = default;
  ~TestLocalizer() = default;

  void on_initialize() override
  {
    odom_.header.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().robot_footprint_frame;
    odom_.pose.pose.position.x = 5;
  }

  virtual void update_rt(easynav::NavState & nav_state) override
  {
    (void) nav_state;
    odom_.pose.pose.position.x = 10;
  }

  virtual void update(easynav::NavState & nav_state) override
  {
    (void) nav_state;
    odom_.pose.pose.position.x = 10;
  }

private:
  nav_msgs::msg::Odometry odom_ {};
};


TEST(MethodBaseTest, DefaultConstructor)
{
  easynav::MethodBase method;
  EXPECT_EQ(
    method.get_node(),
    nullptr
  ) << "Default constructor should initialize parent_node_ to nullptr.";
}

TEST_F(CoreMethodTestCase, InitializeSetsParentNode)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  easynav::MethodBase method;
  method.initialize(node, "test");
}

TEST_F(CoreMethodTestCase, OnInitializeCalled)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  MockMethod method;
  method.initialize(node, "test");
  EXPECT_TRUE(method.on_initialize_called_) <<
    "on_initialize() should be called during initialization.";
}

TEST_F(CoreMethodTestCase, InvalidFrequenciesAreRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const std::string name : {"test.rt_freq", "test.freq"}) {
    for (const double value : {0.0, -1.0, nan, inf, -inf}) {
      auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "test_node", rclcpp::NodeOptions().parameter_overrides({{name, value}}));
      MockMethod method;
      EXPECT_THROW(method.initialize(node, "test"), std::runtime_error) << name << " = " << value;
      EXPECT_FALSE(method.on_initialize_called_) << name << " = " << value;
    }
  }

  // Positive and finite, however small or large: valid.
  for (const double value : {1e-3, 1e6}) {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "test_node", rclcpp::NodeOptions().parameter_overrides({
      {"test.rt_freq", value}, {"test.freq", value}}));
    MockMethod method;
    EXPECT_NO_THROW(method.initialize(node, "test")) << value;
  }
}

TEST_F(CoreMethodTestCase, TFInfoPropagatesToDerived)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_tfinfo_node");

  class TFInfoProbeMethod : public easynav::MethodBase
  {
public:
    void on_initialize() override
    {
      seen_tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
    }

    easynav::TFInfo seen_tf_info;
  };

  TFInfoProbeMethod method;
  easynav::TFInfo tf_info;
  tf_info.tf_prefix = "robot_1";
  tf_info.map_frame = "my_map";
  tf_info.odom_frame = "my_odom";
  tf_info.robot_frame = "my_base";
  tf_info.robot_footprint_frame = "my_base_footprint";
  tf_info.world_frame = "my_world";

  easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  method.initialize(node, "test_plugin");

  EXPECT_EQ(method.seen_tf_info.tf_prefix, "robot_1");
  EXPECT_EQ(method.seen_tf_info.map_frame, "robot_1/my_map");
  EXPECT_EQ(method.seen_tf_info.odom_frame, "robot_1/my_odom");
  EXPECT_EQ(method.seen_tf_info.robot_frame, "robot_1/my_base");
  EXPECT_EQ(method.seen_tf_info.robot_footprint_frame, "robot_1/my_base_footprint");
  EXPECT_EQ(method.seen_tf_info.world_frame, "robot_1/my_world");
}

// ─────────────────────────────────────────────────────────────────────────────
// Concrete sub-classes used to exercise the derived-class method bases
// ─────────────────────────────────────────────────────────────────────────────

/** Minimal LocalizerMethodBase implementation that counts calls. */
class TrackingLocalizer : public easynav::LocalizerMethodBase
{
public:
  int rt_call_count {0};
  int non_rt_call_count {0};

  void on_initialize() override {}

  void update_rt(easynav::NavState &) override {rt_call_count++;}
  void update(easynav::NavState &) override {non_rt_call_count++;}
};

/** Minimal PlannerMethodBase implementation that counts calls. */
class TrackingPlanner : public easynav::PlannerMethodBase
{
public:
  int call_count {0};

  void on_initialize() override {}
  void update(easynav::NavState &) override {call_count++;}
};

/** Minimal MapsManagerBase implementation that counts calls. */
class TrackingMapsManager : public easynav::MapsManagerBase
{
public:
  int call_count {0};

  void on_initialize() override {}
  void update(easynav::NavState &) override {call_count++;}
};

/** Minimal ControllerMethodBase implementation that counts calls. */
class TrackingController : public easynav::ControllerMethodBase
{
public:
  int rt_call_count {0};

  void on_initialize() override {}
  void update_rt(easynav::NavState &) override {rt_call_count++;}
};

// Plugins that throw on every update.
class ThrowingLocalizer : public easynav::LocalizerMethodBase
{
public:
  int rt_call_count {0};
  int non_rt_call_count {0};

  void on_initialize() override {}
  void update_rt(easynav::NavState &) override
  {
    rt_call_count++;
    throw std::runtime_error("boom in localizer update_rt");
  }
  void update(easynav::NavState &) override
  {
    non_rt_call_count++;
    throw std::runtime_error("boom in localizer update");
  }
};

class ThrowingPlanner : public easynav::PlannerMethodBase
{
public:
  int call_count {0};

  void on_initialize() override {}
  void update(easynav::NavState &) override
  {
    call_count++;
    throw std::runtime_error("boom in planner update");
  }
};

class ThrowingMapsManager : public easynav::MapsManagerBase
{
public:
  int call_count {0};

  void on_initialize() override {}
  void update(easynav::NavState &) override
  {
    call_count++;
    throw std::runtime_error("boom in maps manager update");
  }
};

class ThrowingController : public easynav::ControllerMethodBase
{
public:
  int rt_call_count {0};

  void on_initialize() override {}
  void update_rt(easynav::NavState &) override
  {
    rt_call_count++;
    throw std::runtime_error("boom in controller update_rt");
  }
};

// Throws something that is not a std::exception.
class NonStdThrowingLocalizer : public easynav::LocalizerMethodBase
{
public:
  int calls {0};
  void on_initialize() override {}
  void update_rt(easynav::NavState &) override {calls++; throw 42;}
  void update(easynav::NavState &) override {calls++; throw 42;}
};

class NonStdThrowingPlanner : public easynav::PlannerMethodBase
{
public:
  int calls {0};
  void on_initialize() override {}
  void update(easynav::NavState &) override {calls++; throw 42;}
};

class NonStdThrowingMapsManager : public easynav::MapsManagerBase
{
public:
  int calls {0};
  void on_initialize() override {}
  void update(easynav::NavState &) override {calls++; throw 42;}
};

class NonStdThrowingController : public easynav::ControllerMethodBase
{
public:
  int calls {0};
  void on_initialize() override {}
  void update_rt(easynav::NavState &) override {calls++; throw 42;}
};

// Records the last known poses it is handed; optionally throws from the hook.
class RecordingLocalizer : public easynav::LocalizerMethodBase
{
public:
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> last_known;
  std::atomic<int> updates {0};
  bool throw_in_hook {false};
  std::chrono::milliseconds hook_delay {0};
  std::atomic<bool> hook_done {false};
  std::atomic<bool> updated_before_hook_done {false};
  void on_initialize() override {}
  void update_rt(easynav::NavState &) override {record_update();}
  void update(easynav::NavState &) override {record_update();}

protected:
  void on_last_known_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) override
  {
    last_known.push_back(pose);
    std::this_thread::sleep_for(hook_delay);
    hook_done = true;
    if (throw_in_hook) {throw std::runtime_error("bad pose");}
  }

private:
  void record_update()
  {
    if (!last_known.empty() && !hook_done) {updated_before_hook_done = true;}
    updates++;
  }
};

// Writes a command, then throws on odd calls.
class FlakyController : public easynav::ControllerMethodBase
{
public:
  int calls {0};
  void on_initialize() override {}
  void update_rt(easynav::NavState & nav_state) override
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = 0.4;
    nav_state.set("cmd_vel", cmd);
    if (++calls % 2 == 1) {throw std::runtime_error("flaky");}
  }
};

// Throws on odd calls, counts successful ones in NavState.
class FlakyPlanner : public easynav::PlannerMethodBase
{
public:
  int calls {0};
  void on_initialize() override {}
  void update(easynav::NavState & nav_state) override
  {
    if (++calls % 2 == 1) {throw std::runtime_error("flaky");}
    nav_state.set("ok", nav_state.has("ok") ? nav_state.get<int>("ok") + 1 : 1);
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// MethodBase: get_plugin_name, timestamps, and timing helpers
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(CoreMethodTestCase, GetPluginNameAfterInit)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_plugin_name_node");
  easynav::MethodBase method;
  method.initialize(node, "my_plugin");
  EXPECT_EQ(method.get_plugin_name(), "my_plugin");
}

TEST_F(CoreMethodTestCase, GetLastRtTimestampAfterInit)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_rt_ts_node");
  easynav::MethodBase method;
  method.initialize(node, "test_ts");
  // Timestamps should be valid (non-zero) after initialization
  EXPECT_GT(method.get_last_rt_execution_ts().nanoseconds(), 0);
}

TEST_F(CoreMethodTestCase, GetLastTimestampAfterInit)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ts_node");
  easynav::MethodBase method;
  method.initialize(node, "test_ts2");
  EXPECT_GT(method.get_last_execution_ts().nanoseconds(), 0);
}

TEST_F(CoreMethodTestCase, SetRunRT_UpdatesTimestamp)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_setrunrt_node");
  easynav::MethodBase method;
  method.initialize(node, "test_setrunrt");
  auto before = method.get_last_rt_execution_ts();
  // Small sleep to guarantee a different clock tick
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  method.setRunRT();
  EXPECT_GE(method.get_last_rt_execution_ts(), before);
}

TEST_F(CoreMethodTestCase, SetRun_UpdatesTimestamp)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_setrun_node");
  easynav::MethodBase method;
  method.initialize(node, "test_setrun");
  auto before = method.get_last_execution_ts();
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  method.setRun();
  EXPECT_GE(method.get_last_execution_ts(), before);
}

TEST_F(CoreMethodTestCase, IsTime2RunRT_FalseImmediately)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_not_time_rt_node");
  // Default 10 Hz → period = 100 ms → should not be time right after init
  easynav::MethodBase method;
  method.initialize(node, "test_p");
  EXPECT_FALSE(method.isTime2RunRT());
}

TEST_F(CoreMethodTestCase, IsTime2Run_FalseImmediately)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_not_time_node");
  // Default 10 Hz → period = 100 ms → should not be time right after init
  easynav::MethodBase method;
  method.initialize(node, "test_p2");
  EXPECT_FALSE(method.isTime2Run());
}

TEST_F(CoreMethodTestCase, IsTime2RunRT_TrueAfterSleep)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_time_rt_node");
  // Default 10 Hz → period = 100 ms. Sleep 120 ms → should be time to run.
  easynav::MethodBase method;
  method.initialize(node, "fast_p");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_TRUE(method.isTime2RunRT());
}

TEST_F(CoreMethodTestCase, IsTime2Run_TrueAfterSleep)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_time_node");
  // Default 10 Hz → period = 100 ms. Sleep 120 ms → should be time to run.
  easynav::MethodBase method;
  method.initialize(node, "fast_p2");
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_TRUE(method.isTime2Run());
}

// ─────────────────────────────────────────────────────────────────────────────
// LocalizerMethodBase: internal_update_rt and internal_update
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(CoreMethodTestCase, LocalizerUpdateRtWithTriggerAlwaysRuns)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loc_rt_trig_node");
  // Default 10 Hz → won't fire on its own without sleep
  TrackingLocalizer localizer;
  localizer.initialize(node, "loc_p");

  easynav::NavState nav_state;
  bool result = localizer.internal_update_rt(nav_state, true);  // trigger=true

  EXPECT_TRUE(result);
  EXPECT_EQ(localizer.rt_call_count, 1);
}

TEST_F(CoreMethodTestCase, LocalizerUpdateRtWithoutTriggerDoesNotRunImmediately)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loc_rt_no_trig_node");
  // Default 10 Hz → not time to run immediately after init
  TrackingLocalizer localizer;
  localizer.initialize(node, "loc_p2");

  easynav::NavState nav_state;
  bool result = localizer.internal_update_rt(nav_state, false);  // trigger=false

  EXPECT_FALSE(result);
  EXPECT_EQ(localizer.rt_call_count, 0);
}

TEST_F(CoreMethodTestCase, LocalizerUpdateRtRunsWhenTimeElapsed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loc_rt_time_node");
  // Default 10 Hz → period = 100 ms. Sleep 120 ms → time to run.
  TrackingLocalizer localizer;
  localizer.initialize(node, "loc_p3");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  bool result = localizer.internal_update_rt(nav_state, false);

  EXPECT_TRUE(result);
  EXPECT_EQ(localizer.rt_call_count, 1);
}

TEST_F(CoreMethodTestCase, LocalizerInternalUpdateRunsWhenTimeElapsed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loc_upd_node");
  // Default 10 Hz → period = 100 ms. Sleep 120 ms → time to run.
  TrackingLocalizer localizer;
  localizer.initialize(node, "loc_p4");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  localizer.internal_update(nav_state);

  EXPECT_EQ(localizer.non_rt_call_count, 1);
}

TEST_F(CoreMethodTestCase, LocalizerInternalUpdateDoesNotRunTooSoon)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loc_upd2_node");
  // Default 10 Hz → not time to run immediately after init
  TrackingLocalizer localizer;
  localizer.initialize(node, "loc_p5");

  easynav::NavState nav_state;
  localizer.internal_update(nav_state);

  EXPECT_EQ(localizer.non_rt_call_count, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// PlannerMethodBase: internal_update and force_update
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(CoreMethodTestCase, PlannerForceUpdateAlwaysRuns)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_plan_force_node");
  TrackingPlanner planner;
  planner.initialize(node, "plan_p");

  easynav::NavState nav_state;
  planner.force_update(nav_state);

  EXPECT_EQ(planner.call_count, 1);
}

TEST_F(CoreMethodTestCase, PlannerInternalUpdateDoesNotRunTooSoon)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_plan_upd_node");
  // Default 10 Hz → not time to run immediately after init
  TrackingPlanner planner;
  planner.initialize(node, "plan_p2");

  easynav::NavState nav_state;
  planner.internal_update(nav_state);

  EXPECT_EQ(planner.call_count, 0);
}

TEST_F(CoreMethodTestCase, PlannerInternalUpdateRunsWhenTimeElapsed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_plan_upd2_node");
  // Default 10 Hz → period = 100 ms. Sleep 120 ms → time to run.
  TrackingPlanner planner;
  planner.initialize(node, "plan_p3");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  planner.internal_update(nav_state);

  EXPECT_EQ(planner.call_count, 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// MapsManagerBase: internal_update
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(CoreMethodTestCase, MapsManagerInternalUpdateDoesNotRunTooSoon)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_maps_upd_node");
  // Default 10 Hz → not time to run immediately after init
  TrackingMapsManager maps;
  maps.initialize(node, "maps_p");

  easynav::NavState nav_state;
  maps.internal_update(nav_state);

  EXPECT_EQ(maps.call_count, 0);
}

TEST_F(CoreMethodTestCase, MapsManagerInternalUpdateRunsWhenTimeElapsed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_maps_upd2_node");
  // Default 10 Hz → period = 100 ms. Sleep 120 ms → time to run.
  TrackingMapsManager maps;
  maps.initialize(node, "maps_p2");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  maps.internal_update(nav_state);

  EXPECT_EQ(maps.call_count, 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// ControllerMethodBase: initialize and internal_update_rt
// ─────────────────────────────────────────────────────────────────────────────

// The collision checker moved to the recovery system.
class CollisionCheckerRemovedTest : public CoreMethodTestCase {};

// A controller that always commands forward.
class ForwardController : public easynav::ControllerMethodBase
{
public:
  void on_initialize() override {}
  void update_rt(easynav::NavState & nav_state) override
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = 0.5;
    nav_state.set("cmd_vel", cmd);
  }
};

TEST_F(CollisionCheckerRemovedTest, NoCollisionCheckerParameters)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_init_node");
  TrackingController ctrl;
  ctrl.initialize(node, "ctrl_p");
  for (const auto & name : {"active", "debug_markers", "robot_radius", "robot_height",
      "brake_acc", "safety_margin", "z_min_filter", "downsample_leaf_size"})
  {
    EXPECT_FALSE(node->has_parameter(std::string("colision_checker.") + name)) << name;
  }
}

TEST_F(CollisionCheckerRemovedTest, TheControllerCommandIsNotAltered)
{
  // Even configured as before and with an obstacle right ahead: braking is the recovery
  // system's job now.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_ctrl_checker_node", rclcpp::NodeOptions().parameter_overrides(
      {{"colision_checker.active", true}, {"colision_checker.robot_radius", 0.3}}));
  ForwardController ctrl;
  ctrl.initialize(node, "ctrl");

  easynav::NavState nav_state;
  easynav::PointPerception perception;
  perception.data.push_back(pcl::PointXYZ(0.2, 0.0, 0.2));
  perception.frame_id = "base_link";
  perception.stamp = node->now();
  perception.valid = true;
  nav_state.set("scan", perception);
  ASSERT_TRUE(ctrl.internal_update_rt(nav_state, true));
  EXPECT_DOUBLE_EQ(
    nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist.linear.x, 0.5);
}

// ─── Robot geometry ──────────────────────────────────────────────────────────────────────────

class RobotGeometryCoreTest : public CoreMethodTestCase
{
protected:
  void SetUp() override
  {
    CoreMethodTestCase::SetUp();
    easynav::RobotGeometryRegistry::getInstance()->set_geometry(easynav::RobotGeometry{});
  }

  void TearDown() override
  {
    easynav::RobotGeometryRegistry::getInstance()->set_geometry(easynav::RobotGeometry{});
    CoreMethodTestCase::TearDown();
  }

  static rclcpp_lifecycle::LifecycleNode::SharedPtr node(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "test_geometry_node", rclcpp::NodeOptions().parameter_overrides(overrides));
  }
};

TEST_F(RobotGeometryCoreTest, PluginDeprecatedNamesAreRelativeToThePlugin)
{
  TrackingController ctrl;
  auto n = node({{"ctrl.robot_radius", 0.2}, {"robot_radius", 0.9}, {"ctrl.inscribed", 0.15}});
  ctrl.initialize(n, "ctrl");
  const auto geometry = ctrl.get_robot_geometry({"robot_radius", "inscribed", ""});
  EXPECT_DOUBLE_EQ(geometry.radius, 0.2);
  EXPECT_DOUBLE_EQ(geometry.inscribed_radius, 0.15);
  EXPECT_DOUBLE_EQ(geometry.height, easynav::RobotGeometry{}.height);
  EXPECT_DOUBLE_EQ(ctrl.get_robot_geometry().radius, easynav::RobotGeometry{}.radius)
    << "no deprecated names";
}

TEST_F(CoreMethodTestCase, ControllerInternalUpdateRtWithTriggerAlwaysRuns)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_rt_node");
  TrackingController ctrl;
  ctrl.initialize(node, "ctrl_p2");

  easynav::NavState nav_state;
  bool result = ctrl.internal_update_rt(nav_state, true);

  EXPECT_TRUE(result);
  EXPECT_EQ(ctrl.rt_call_count, 1);
}

TEST_F(CoreMethodTestCase, ControllerInternalUpdateRtWithoutTriggerDoesNotRunImmediately)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_rt2_node");
  // Default 10 Hz → not time to run immediately after init
  TrackingController ctrl;
  ctrl.initialize(node, "ctrl_p3");

  easynav::NavState nav_state;
  bool result = ctrl.internal_update_rt(nav_state, false);

  EXPECT_FALSE(result);
  EXPECT_EQ(ctrl.rt_call_count, 0);
}

TEST_F(CoreMethodTestCase, ControllerInternalUpdateRtRunsWhenTimeElapsed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_rt3_node");
  // Default 10 Hz → period = 100 ms. Sleep 120 ms → time to run.
  TrackingController ctrl;
  ctrl.initialize(node, "ctrl_p4");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  bool result = ctrl.internal_update_rt(nav_state, false);

  EXPECT_TRUE(result);
  EXPECT_EQ(ctrl.rt_call_count, 1);
}

// A throwing plugin must not propagate the exception.

TEST_F(CoreMethodTestCase, LocalizerUpdateRtExceptionDoesNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loc_throw_rt_node");
  ThrowingLocalizer localizer;
  localizer.initialize(node, "loc_throw_rt");

  easynav::NavState nav_state;
  bool result = false;
  EXPECT_NO_THROW(result = localizer.internal_update_rt(nav_state, true));

  EXPECT_TRUE(result);
  EXPECT_EQ(localizer.rt_call_count, 1);
}

TEST_F(CoreMethodTestCase, LocalizerUpdateExceptionDoesNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loc_throw_node");
  ThrowingLocalizer localizer;
  localizer.initialize(node, "loc_throw");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  EXPECT_NO_THROW(localizer.internal_update(nav_state));

  EXPECT_EQ(localizer.non_rt_call_count, 1);
}

TEST_F(CoreMethodTestCase, PlannerInternalUpdateExceptionDoesNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_plan_throw_node");
  ThrowingPlanner planner;
  planner.initialize(node, "plan_throw");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  EXPECT_NO_THROW(planner.internal_update(nav_state));

  EXPECT_EQ(planner.call_count, 1);
}

TEST_F(CoreMethodTestCase, PlannerForceUpdateExceptionDoesNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_plan_throw_force_node");
  ThrowingPlanner planner;
  planner.initialize(node, "plan_throw_force");

  easynav::NavState nav_state;
  EXPECT_NO_THROW(planner.force_update(nav_state));

  EXPECT_EQ(planner.call_count, 1);
}

TEST_F(CoreMethodTestCase, MapsManagerInternalUpdateExceptionDoesNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_maps_throw_node");
  ThrowingMapsManager maps;
  maps.initialize(node, "maps_throw");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  EXPECT_NO_THROW(maps.internal_update(nav_state));

  EXPECT_EQ(maps.call_count, 1);
}

TEST_F(CoreMethodTestCase, ControllerInternalUpdateRtExceptionDoesNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_throw_node");
  ThrowingController ctrl;
  ctrl.initialize(node, "ctrl_throw");

  easynav::NavState nav_state;
  bool result = false;
  EXPECT_NO_THROW(result = ctrl.internal_update_rt(nav_state, true));

  EXPECT_TRUE(result);
  EXPECT_EQ(ctrl.rt_call_count, 1);
}

TEST_F(CoreMethodTestCase, ThrowingPluginsKeepBeingCalled)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_keep_called_node");
  ThrowingLocalizer localizer;
  localizer.initialize(node, "loc_keep");
  ThrowingController ctrl;
  ctrl.initialize(node, "ctrl_keep");
  ThrowingPlanner planner;
  planner.initialize(node, "plan_keep");

  easynav::NavState nav_state;
  for (int i = 1; i <= 3; ++i) {
    EXPECT_TRUE(localizer.internal_update_rt(nav_state, true));
    EXPECT_TRUE(ctrl.internal_update_rt(nav_state, true));
    EXPECT_NO_THROW(planner.force_update(nav_state));
    EXPECT_EQ(localizer.rt_call_count, i);
    EXPECT_EQ(ctrl.rt_call_count, i);
    EXPECT_EQ(planner.call_count, i);
  }
}

TEST_F(CoreMethodTestCase, IntermittentFailuresDoNotAffectGoodCycles)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_flaky_node");
  FlakyPlanner planner;
  planner.initialize(node, "flaky");

  easynav::NavState nav_state;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NO_THROW(planner.force_update(nav_state));
  }
  EXPECT_EQ(planner.calls, 6);
  ASSERT_TRUE(nav_state.has("ok"));
  EXPECT_EQ(nav_state.get<int>("ok"), 3);
}

TEST_F(CoreMethodTestCase, AFailedCycleStillCountsForTheRate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_fail_rate_node");
  ThrowingPlanner planner;
  planner.initialize(node, "plan_rate");
  easynav::NavState nav_state;

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  planner.internal_update(nav_state);
  EXPECT_EQ(planner.call_count, 1);

  // Too soon after the failed run: not retried every cycle.
  planner.internal_update(nav_state);
  EXPECT_EQ(planner.call_count, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  planner.internal_update(nav_state);
  EXPECT_EQ(planner.call_count, 2);
}

TEST_F(CoreMethodTestCase, NonStdExceptionsDoNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_non_std_node");
  NonStdThrowingLocalizer localizer;
  localizer.initialize(node, "loc_non_std");
  NonStdThrowingPlanner planner;
  planner.initialize(node, "plan_non_std");
  NonStdThrowingMapsManager maps;
  maps.initialize(node, "maps_non_std");
  NonStdThrowingController ctrl;
  ctrl.initialize(node, "ctrl_non_std");

  easynav::NavState nav_state;
  EXPECT_NO_THROW(localizer.internal_update_rt(nav_state, true));
  EXPECT_NO_THROW(ctrl.internal_update_rt(nav_state, true));
  EXPECT_NO_THROW(planner.force_update(nav_state));

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_NO_THROW(localizer.internal_update(nav_state));
  EXPECT_NO_THROW(planner.internal_update(nav_state));
  EXPECT_NO_THROW(maps.internal_update(nav_state));

  EXPECT_EQ(localizer.calls, 2);
  EXPECT_EQ(planner.calls, 2);
  EXPECT_EQ(maps.calls, 1);
  EXPECT_EQ(ctrl.calls, 1);
}

namespace
{
geometry_msgs::msg::TwistStamped moving_cmd()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = 0.5;
  cmd.twist.angular.z = 0.3;
  return cmd;
}

bool is_stop(const easynav::NavState & nav_state)
{
  const auto & t = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist;
  return t.linear.x == 0.0 && t.linear.y == 0.0 && t.angular.z == 0.0;
}
}  // namespace

TEST_F(CoreMethodTestCase, ControllerExceptionStopsTheRobot)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_stop_node");
  ThrowingController ctrl;
  ctrl.initialize(node, "ctrl_stop");

  easynav::NavState nav_state;
  nav_state.set("cmd_vel", moving_cmd());  // Last command, from a previous cycle.
  EXPECT_TRUE(ctrl.internal_update_rt(nav_state, true)) << "the stop must be published";
  EXPECT_TRUE(is_stop(nav_state));
}

TEST_F(CoreMethodTestCase, ControllerNonStdExceptionStopsTheRobot)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_stop_non_std_node");
  NonStdThrowingController ctrl;
  ctrl.initialize(node, "ctrl_stop_non_std");

  easynav::NavState nav_state;
  nav_state.set("cmd_vel", moving_cmd());
  EXPECT_TRUE(ctrl.internal_update_rt(nav_state, true));
  EXPECT_TRUE(is_stop(nav_state));
}

TEST_F(CoreMethodTestCase, ControllerCommandWrittenBeforeThrowingIsDiscarded)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_partial_node");
  FlakyController ctrl;  // First call writes 0.4 and throws.
  ctrl.initialize(node, "ctrl_partial");

  easynav::NavState nav_state;
  EXPECT_TRUE(ctrl.internal_update_rt(nav_state, true));
  EXPECT_TRUE(is_stop(nav_state));
}

TEST_F(CoreMethodTestCase, ControllerRecoversAfterAFailedCycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_recover_node");
  FlakyController ctrl;  // Fails on odd calls.
  ctrl.initialize(node, "ctrl_recover");

  easynav::NavState nav_state;
  for (int i = 1; i <= 4; ++i) {
    EXPECT_TRUE(ctrl.internal_update_rt(nav_state, true));
    const double x = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist.linear.x;
    EXPECT_DOUBLE_EQ(x, i % 2 == 1 ? 0.0 : 0.4) << "cycle " << i;
  }
}

namespace
{
nav_msgs::msg::Odometry robot_pose(double x, double y, const std::string & frame = "")
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id =
    frame.empty() ? easynav::RTTFBuffer::getInstance()->get_tf_info().map_frame : frame;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.covariance[0] = 0.25;
  return odom;
}

std::shared_ptr<RecordingLocalizer> make_localizer(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  auto localizer = std::make_shared<RecordingLocalizer>();
  localizer->initialize(node, name);
  return localizer;
}
}  // namespace

TEST_F(CoreMethodTestCase, LastKnownPoseIsHandedOnTheFirstCycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_last_pose_node");
  auto localizer = make_localizer(node, "loc");

  easynav::NavState nav_state;
  nav_state.set("robot_pose", robot_pose(1.5, -2.0));
  localizer->internal_update_rt(nav_state, true);

  ASSERT_EQ(localizer->last_known.size(), 1u);
  const auto & pose = localizer->last_known.front();
  EXPECT_DOUBLE_EQ(pose.pose.pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(pose.pose.pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(pose.pose.covariance[0], 0.25) << "covariance kept";
  EXPECT_EQ(localizer->updates, 1);
}

TEST_F(CoreMethodTestCase, LastKnownPoseAlsoOnAFirstNonRtCycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_last_pose_nort_node");
  auto localizer = make_localizer(node, "loc");

  easynav::NavState nav_state;
  nav_state.set("robot_pose", robot_pose(1.0, 1.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  localizer->internal_update(nav_state);
  EXPECT_EQ(localizer->last_known.size(), 1u);
}

TEST_F(CoreMethodTestCase, LastKnownPoseOnlyOnce)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_last_pose_once_node");
  auto localizer = make_localizer(node, "loc");

  easynav::NavState nav_state;
  nav_state.set("robot_pose", robot_pose(1.0, 1.0));
  for (int i = 0; i < 5; ++i) {
    nav_state.set("robot_pose", robot_pose(1.0 + i, 1.0));  // Its own estimate from now on.
    localizer->internal_update_rt(nav_state, true);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  localizer->internal_update(nav_state);

  ASSERT_EQ(localizer->last_known.size(), 1u);
  EXPECT_DOUBLE_EQ(localizer->last_known.front().pose.pose.position.x, 1.0);
}

TEST_F(CoreMethodTestCase, NoLastKnownPoseOnAFreshStart)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_last_pose_fresh_node");
  auto localizer = make_localizer(node, "loc");

  easynav::NavState nav_state;
  localizer->internal_update_rt(nav_state, true);
  // A pose written afterwards is this localizer's own: not handed back.
  nav_state.set("robot_pose", robot_pose(1.0, 1.0));
  localizer->internal_update_rt(nav_state, true);

  EXPECT_TRUE(localizer->last_known.empty());
  EXPECT_EQ(localizer->updates, 2);
}

TEST_F(CoreMethodTestCase, InvalidLastKnownPosesAreIgnored)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_last_pose_invalid_node");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<nav_msgs::msg::Odometry> invalid;
  invalid.push_back(robot_pose(1.0, 1.0, "odom"));  // Not the map frame.
  invalid.push_back(robot_pose(nan, 1.0));
  invalid.push_back(robot_pose(1.0, std::numeric_limits<double>::infinity()));
  invalid.push_back(robot_pose(1.0, 1.0));
  invalid.back().pose.pose.orientation.w = 0.0;  // Zero quaternion.
  invalid.push_back(robot_pose(1.0, 1.0));
  invalid.back().pose.pose.position.z = nan;
  invalid.push_back(robot_pose(1.0, 1.0));
  invalid.back().pose.pose.orientation.x = nan;

  for (std::size_t i = 0; i < invalid.size(); ++i) {
    auto localizer = make_localizer(node, "loc_invalid_" + std::to_string(i));
    easynav::NavState nav_state;
    nav_state.set("robot_pose", invalid[i]);
    localizer->internal_update_rt(nav_state, true);
    EXPECT_TRUE(localizer->last_known.empty()) << "case " << i;
    EXPECT_EQ(localizer->updates, 1) << "case " << i;
  }
}

TEST_F(CoreMethodTestCase, ThrowingLastKnownPoseHookDoesNotStopTheLocalizer)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_last_pose_throw_node");
  auto localizer = std::make_shared<RecordingLocalizer>();
  localizer->throw_in_hook = true;
  localizer->initialize(node, "loc");

  easynav::NavState nav_state;
  nav_state.set("robot_pose", robot_pose(1.0, 1.0));
  EXPECT_NO_THROW(localizer->internal_update_rt(nav_state, true));
  EXPECT_NO_THROW(localizer->internal_update_rt(nav_state, true));
  EXPECT_EQ(localizer->last_known.size(), 1u);
  EXPECT_EQ(localizer->updates, 2);
}

TEST_F(CoreMethodTestCase, EachNewLocalizerInstanceGetsTheLastKnownPose)
{
  // Reconfiguration: every configure creates a new instance, even of another localizer type.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_last_pose_reconf_node");
  easynav::NavState nav_state;
  nav_state.set("robot_pose", robot_pose(0.0, 0.0));

  for (int i = 1; i <= 3; ++i) {
    auto localizer = make_localizer(node, "loc_" + std::to_string(i));
    localizer->internal_update_rt(nav_state, true);
    ASSERT_EQ(localizer->last_known.size(), 1u) << "instance " << i;
    EXPECT_DOUBLE_EQ(localizer->last_known.front().pose.pose.position.x, i - 1.0);
    nav_state.set("robot_pose", robot_pose(i, 0.0));  // Where it leaves the robot.
  }
}

TEST_F(CoreMethodTestCase, NonZeroQuaternionWithZeroWIsValid)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_last_pose_quat_node");
  auto localizer = make_localizer(node, "loc");

  easynav::NavState nav_state;
  auto odom = robot_pose(1.0, 1.0);
  odom.pose.pose.orientation.w = 0.0;
  odom.pose.pose.orientation.x = 1.0;  // 180 deg about x: unusual, but a valid rotation.
  nav_state.set("robot_pose", odom);
  localizer->internal_update_rt(nav_state, true);

  EXPECT_EQ(localizer->last_known.size(), 1u);
}

TEST_F(CoreMethodTestCase, NoUpdateRunsBeforeTheLastKnownPoseHookFinishes)
{
  // The RT and non-RT loops run concurrently: whichever comes second waits for the hook.
  for (int round = 0; round < 5; ++round) {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "test_last_pose_race_node_" + std::to_string(round));
    auto localizer = std::make_shared<RecordingLocalizer>();
    localizer->hook_delay = std::chrono::milliseconds(50);
    localizer->initialize(node, "loc");

    easynav::NavState nav_state;
    nav_state.set("robot_pose", robot_pose(1.0, 1.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(120));  // Non-RT is due.

    std::thread rt([&]() {localizer->internal_update_rt(nav_state, true);});
    std::thread nort([&]() {localizer->internal_update(nav_state);});
    rt.join();
    nort.join();

    EXPECT_EQ(localizer->last_known.size(), 1u) << "round " << round;
    EXPECT_EQ(localizer->updates, 2) << "round " << round;
    EXPECT_FALSE(localizer->updated_before_hook_done) << "round " << round;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
