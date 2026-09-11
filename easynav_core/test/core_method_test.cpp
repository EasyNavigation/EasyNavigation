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

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_core/MethodBase.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_core/ControllerMethodBase.hpp"
#include "easynav_core/SafetyReflexBase.hpp"
#include "easynav_core/RecoveryEvaluatorBase.hpp"
#include "easynav_core/RecoveryMitigationBase.hpp"

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
// Plugins that throw: internal_update*/force_update must not propagate the
// exception (see docs/recoveries_easynav.md, "Fase 0" of the recovery roadmap).
// ─────────────────────────────────────────────────────────────────────────────

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

// ─────────────────────────────────────────────────────────────────────────────
// SafetyReflexBase mocks (level 0, see docs/recoveries_easynav.md).
// ─────────────────────────────────────────────────────────────────────────────

class TrackingReflex : public easynav::SafetyReflexBase
{
public:
  bool should_trigger {false};
  bool throw_on_mitigate {false};
  int check_calls {0};
  int mitigate_calls {0};

  void on_initialize() override {}

  bool check(easynav::NavState &) override
  {
    check_calls++;
    return should_trigger;
  }

  void mitigate(easynav::NavState & nav_state) override
  {
    mitigate_calls++;
    if (throw_on_mitigate) {
      throw std::runtime_error("boom in mitigate (toggled)");
    }
    geometry_msgs::msg::TwistStamped applied;
    applied.twist.linear.x = 42.0;  // sentinel value, distinguishable from a fail-safe stop
    nav_state.set("cmd_vel", applied);
  }
};

class ThrowingCheckReflex : public easynav::SafetyReflexBase
{
public:
  void on_initialize() override {}
  bool check(easynav::NavState &) override {throw std::runtime_error("boom in check");}
  void mitigate(easynav::NavState &) override {}
};

class ThrowingMitigateReflex : public easynav::SafetyReflexBase
{
public:
  void on_initialize() override {}
  bool check(easynav::NavState &) override {return true;}
  void mitigate(easynav::NavState &) override {throw std::runtime_error("boom in mitigate");}
};

// ─────────────────────────────────────────────────────────────────────────────
// RecoveryEvaluatorBase mocks (level 1, see docs/recoveries_easynav.md).
// ─────────────────────────────────────────────────────────────────────────────

class TrackingEvaluator : public easynav::RecoveryEvaluatorBase
{
public:
  int call_count {0};

  void on_initialize() override {}
  void update(easynav::NavState &) override {call_count++;}
};

class ThrowingEvaluator : public easynav::RecoveryEvaluatorBase
{
public:
  void on_initialize() override {}
  void update(easynav::NavState &) override {throw std::runtime_error("boom in evaluator");}
};

class PublishingEvaluator : public easynav::RecoveryEvaluatorBase
{
public:
  int8_t level_to_publish {diagnostic_msgs::msg::DiagnosticStatus::OK};

  void on_initialize() override {}

  void update(easynav::NavState & nav_state) override
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level_to_publish;
    status.name = get_plugin_name();
    publish_diagnostic(nav_state, status);
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// RecoveryMitigationBase mocks (level 1, see docs/recoveries_easynav.md).
// ─────────────────────────────────────────────────────────────────────────────

class TrackingMitigation : public easynav::RecoveryMitigationBase
{
public:
  int start_calls {0};
  int cycle_calls {0};
  int stop_calls {0};
  easynav::RecoveryStatus status_to_return {easynav::RecoveryStatus::RUNNING};

  void on_initialize() override {}
  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus &) const override {return true;}
  bool requires_control() const override {return true;}

  void on_start(easynav::NavState &) override {start_calls++;}
  easynav::RecoveryStatus on_cycle(easynav::NavState &) override
  {
    cycle_calls++;
    return status_to_return;
  }
  void on_stop(easynav::NavState &) override {stop_calls++;}
};

class ThrowingCycleMitigation : public easynav::RecoveryMitigationBase
{
public:
  void on_initialize() override {}
  bool can_handle(const diagnostic_msgs::msg::DiagnosticStatus &) const override {return true;}
  easynav::RecoveryStatus on_cycle(easynav::NavState &) override
  {
    throw std::runtime_error("boom in mitigation on_cycle");
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

// ─────────────────────────────────────────────────────────────────────────────
// A throwing plugin must not crash the RT thread / the process (Fase 0, see
// docs/recoveries_easynav.md).
// ─────────────────────────────────────────────────────────────────────────────

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

// ─────────────────────────────────────────────────────────────────────────────
// SafetyReflexBase: internal_check_and_mitigate (level 0, see
// docs/recoveries_easynav.md).
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(CoreMethodTestCase, ReflexDoesNotMitigateWhenCheckReturnsFalse)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_no_trigger_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_p");
  reflex.should_trigger = false;

  easynav::NavState nav_state;
  bool result = reflex.internal_check_and_mitigate(nav_state);

  EXPECT_FALSE(result);
  EXPECT_EQ(reflex.check_calls, 1);
  EXPECT_EQ(reflex.mitigate_calls, 0);
}

TEST_F(CoreMethodTestCase, ReflexMitigatesWhenCheckReturnsTrue)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_trigger_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_p2");
  reflex.should_trigger = true;

  easynav::NavState nav_state;
  bool result = reflex.internal_check_and_mitigate(nav_state);

  EXPECT_TRUE(result);
  EXPECT_EQ(reflex.mitigate_calls, 1);
  EXPECT_DOUBLE_EQ(
    nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel").twist.linear.x, 42.0);
}

TEST_F(CoreMethodTestCase, ReflexFailsSafeWhenCheckThrows)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_check_throw_node");
  ThrowingCheckReflex reflex;
  reflex.initialize(node, "reflex_p3");

  easynav::NavState nav_state;
  bool result = false;
  EXPECT_NO_THROW(result = reflex.internal_check_and_mitigate(nav_state));

  EXPECT_TRUE(result);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & applied = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(applied.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(applied.twist.angular.z, 0.0);
}

TEST_F(CoreMethodTestCase, ReflexFailsSafeWhenMitigateThrows)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_reflex_mitigate_throw_node");
  ThrowingMitigateReflex reflex;
  reflex.initialize(node, "reflex_p4");

  easynav::NavState nav_state;
  bool result = false;
  EXPECT_NO_THROW(result = reflex.internal_check_and_mitigate(nav_state));

  EXPECT_TRUE(result);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & applied = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(applied.twist.linear.x, 0.0);
}

// Reflexes also report into NavState's shared "diagnostics" group, per §5.2 of the design, so
// a future level-1 evaluator could notice one triggering repeatedly without depending on the
// non-RT cycle for its own reaction. See docs/recoveries_easynav.md.

TEST_F(CoreMethodTestCase, ReflexNotTriggeredPublishesOkDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_diag_ok_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_diag_ok");
  reflex.should_trigger = false;

  easynav::NavState nav_state;
  reflex.internal_check_and_mitigate(nav_state);

  ASSERT_TRUE(nav_state.has("diagnostics.reflex_diag_ok"));
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.reflex_diag_ok").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_NE(
    std::find(members.begin(), members.end(), "diagnostics.reflex_diag_ok"), members.end());
}

TEST_F(CoreMethodTestCase, ReflexTriggeredPublishesWarnDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_reflex_diag_warn_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_diag_warn");
  reflex.should_trigger = true;

  easynav::NavState nav_state;
  reflex.internal_check_and_mitigate(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.reflex_diag_warn").level,
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST_F(CoreMethodTestCase, ReflexCheckThrowPublishesErrorDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_reflex_diag_check_throw_node");
  ThrowingCheckReflex reflex;
  reflex.initialize(node, "reflex_diag_check_throw");

  easynav::NavState nav_state;
  reflex.internal_check_and_mitigate(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(
      "diagnostics.reflex_diag_check_throw").level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(CoreMethodTestCase, ReflexMitigateThrowPublishesErrorDiagnostic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_reflex_diag_mitigate_throw_node");
  ThrowingMitigateReflex reflex;
  reflex.initialize(node, "reflex_diag_mitigate_throw");

  easynav::NavState nav_state;
  reflex.internal_check_and_mitigate(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(
      "diagnostics.reflex_diag_mitigate_throw").level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(CoreMethodTestCase, ReflexDiagnosticTracksLevelAcrossCycles)
{
  // Reproduces the exact bug edge-triggering-on-a-plain-bool would have: going from
  // "triggered, mitigate() succeeded" (WARN) to "triggered, mitigate() throws" (ERROR) must
  // still be reported even though "triggered" itself does not change between those two calls.
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_reflex_diag_transitions_node");
  TrackingReflex reflex;
  reflex.initialize(node, "reflex_diag_transitions");
  easynav::NavState nav_state;
  const std::string key = "diagnostics.reflex_diag_transitions";

  reflex.should_trigger = false;
  reflex.internal_check_and_mitigate(nav_state);
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key).level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  reflex.should_trigger = true;
  reflex.internal_check_and_mitigate(nav_state);
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key).level,
    diagnostic_msgs::msg::DiagnosticStatus::WARN);

  reflex.throw_on_mitigate = true;
  reflex.internal_check_and_mitigate(nav_state);
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key).level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  reflex.throw_on_mitigate = false;
  reflex.should_trigger = false;
  reflex.internal_check_and_mitigate(nav_state);
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>(key).level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  // No duplicate membership, regardless of how many cycles were reported above.
  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_EQ(std::count(members.begin(), members.end(), key), 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// RecoveryEvaluatorBase: internal_update and publish_diagnostic (level 1, see
// docs/recoveries_easynav.md).
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(CoreMethodTestCase, EvaluatorInternalUpdateDoesNotRunTooSoon)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_node");
  TrackingEvaluator eval;
  eval.initialize(node, "eval_p");

  easynav::NavState nav_state;
  eval.internal_update(nav_state);

  EXPECT_EQ(eval.call_count, 0);
}

TEST_F(CoreMethodTestCase, EvaluatorInternalUpdateRunsWhenTimeElapsed)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval2_node");
  TrackingEvaluator eval;
  eval.initialize(node, "eval_p2");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  eval.internal_update(nav_state);

  EXPECT_EQ(eval.call_count, 1);
}

TEST_F(CoreMethodTestCase, EvaluatorUpdateExceptionDoesNotPropagate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_throw_node");
  ThrowingEvaluator eval;
  eval.initialize(node, "eval_throw");

  easynav::NavState nav_state;
  EXPECT_NO_THROW(eval.internal_update(nav_state));
}

TEST_F(CoreMethodTestCase, PublishDiagnosticCreatesGroupAndEntry)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_pub_node");
  PublishingEvaluator eval;
  eval.initialize(node, "my_eval");
  eval.level_to_publish = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  eval.internal_update(nav_state);

  ASSERT_TRUE(nav_state.has("diagnostics.my_eval"));
  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.my_eval").level,
    diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_NE(
    std::find(members.begin(), members.end(), "diagnostics.my_eval"), members.end());
}

TEST_F(CoreMethodTestCase, PublishDiagnosticOverwritesInsteadOfDuplicating)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_pub2_node");
  PublishingEvaluator eval;
  eval.initialize(node, "my_eval2");

  easynav::NavState nav_state;
  eval.level_to_publish = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  eval.internal_update(nav_state);

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  eval.level_to_publish = diagnostic_msgs::msg::DiagnosticStatus::OK;
  eval.internal_update(nav_state);

  EXPECT_EQ(
    nav_state.get<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics.my_eval2").level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);

  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_EQ(
    std::count(members.begin(), members.end(), "diagnostics.my_eval2"), 1);
}

TEST_F(CoreMethodTestCase, PublishDiagnosticFromTwoEvaluatorsBothAppearInGroup)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_eval_pub3_node");
  PublishingEvaluator eval_a;
  eval_a.initialize(node, "eval_a");
  PublishingEvaluator eval_b;
  eval_b.initialize(node, "eval_b");

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  easynav::NavState nav_state;
  eval_a.internal_update(nav_state);
  eval_b.internal_update(nav_state);

  auto members = nav_state.get_group_keys("diagnostics");
  EXPECT_NE(std::find(members.begin(), members.end(), "diagnostics.eval_a"), members.end());
  EXPECT_NE(std::find(members.begin(), members.end(), "diagnostics.eval_b"), members.end());
}

// ─────────────────────────────────────────────────────────────────────────────
// RecoveryMitigationBase: internal_start/internal_cycle/internal_stop (level 1,
// see docs/recoveries_easynav.md).
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(CoreMethodTestCase, MitigationLifecycleCallsAreForwarded)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mit_node");
  TrackingMitigation mit;
  mit.initialize(node, "mit_p");

  easynav::NavState nav_state;
  mit.internal_start(nav_state);
  auto status = mit.internal_cycle(nav_state);
  mit.internal_stop(nav_state);

  EXPECT_EQ(mit.start_calls, 1);
  EXPECT_EQ(mit.cycle_calls, 1);
  EXPECT_EQ(mit.stop_calls, 1);
  EXPECT_EQ(status, easynav::RecoveryStatus::RUNNING);
}

TEST_F(CoreMethodTestCase, MitigationCycleExceptionFailsSafe)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_mit_throw_node");
  ThrowingCycleMitigation mit;
  mit.initialize(node, "mit_throw");

  easynav::NavState nav_state;
  easynav::RecoveryStatus status = easynav::RecoveryStatus::RUNNING;
  EXPECT_NO_THROW(status = mit.internal_cycle(nav_state));

  EXPECT_EQ(status, easynav::RecoveryStatus::FAILED);
  ASSERT_TRUE(nav_state.has("cmd_vel"));
  const auto & applied = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  EXPECT_DOUBLE_EQ(applied.twist.linear.x, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
