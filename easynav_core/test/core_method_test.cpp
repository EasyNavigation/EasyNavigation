// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

#include "gtest/gtest.h"

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/RTTFBuffer.hpp"
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

TEST_F(CoreMethodTestCase, ControllerInitializeDeclaresCollisionParams)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctrl_init_node");
  TrackingController ctrl;
  ctrl.initialize(node, "ctrl_p");

  // All collision checker parameters should have been declared
  EXPECT_TRUE(node->has_parameter("colision_checker.active"));
  EXPECT_TRUE(node->has_parameter("colision_checker.robot_radius"));
  EXPECT_TRUE(node->has_parameter("colision_checker.brake_acc"));
  EXPECT_TRUE(node->has_parameter("colision_checker.safety_margin"));
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
