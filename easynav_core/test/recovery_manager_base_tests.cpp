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
/// \brief Tests for RecoveryManagerBase: exception safety, velocity slots and SystemActions.

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/RecoveryManagerBase.hpp"
#include "easynav_core/SystemActions.hpp"
#include "easynav_core/VelocityCommand.hpp"

using easynav::VelocitySource;
namespace vc = easynav::velocity_command;

namespace
{

class RecordingSystemActions : public easynav::SystemActions
{
public:
  void abort_mission(const std::string & reason) override {aborted.push_back(reason);}
  void request_shutdown(const std::string & reason) override {shutdowns.push_back(reason);}
  void hold_mission_progress(bool hold) override {holds.push_back(hold);}
  std::vector<std::string> aborted;
  std::vector<std::string> shutdowns;
  std::vector<bool> holds;
};

// Configurable recovery manager: throws, commands or calls SystemActions on demand.
class TestRecoveryManager : public easynav::RecoveryManagerBase
{
public:
  enum class Rt {NOTHING, TAKEOVER, OVERRIDE, THROW_STD, THROW_OTHER};

  Rt rt_action {Rt::NOTHING};
  bool throw_in_update {false};
  int updates {0};
  int updates_rt {0};

  geometry_msgs::msg::TwistStamped twist(double vx, double wz)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.linear.x = vx;
    cmd.twist.angular.z = wz;
    return cmd;
  }

  using RecoveryManagerBase::abort_mission;
  using RecoveryManagerBase::hold_mission_progress;
  using RecoveryManagerBase::request_shutdown;

protected:
  void update(easynav::NavState &) override
  {
    ++updates;
    if (throw_in_update) {throw std::runtime_error("update failed");}
  }

  bool update_rt(easynav::NavState & nav_state) override
  {
    ++updates_rt;
    switch (rt_action) {
      case Rt::TAKEOVER: command_velocity(nav_state, twist(0.2, 0.1)); return true;
      case Rt::OVERRIDE: override_velocity(nav_state, twist(-0.1, 0.0)); return true;
      case Rt::THROW_STD: throw std::runtime_error("update_rt failed");
      case Rt::THROW_OTHER: throw std::logic_error("update_rt logic failed");
      default: return false;
    }
  }
};

}  // namespace

class RecoveryManagerBaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("recovery_base_test_node");
    manager_ = std::make_shared<TestRecoveryManager>();
    manager_->initialize(node_, "recovery_manager");
  }

  void TearDown() override
  {
    manager_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<TestRecoveryManager> manager_;
  easynav::NavState nav_state_;
};

TEST_F(RecoveryManagerBaseTest, NothingProposedByDefault)
{
  EXPECT_FALSE(manager_->internal_update_rt(nav_state_));
  EXPECT_EQ(manager_->updates_rt, 1);
  EXPECT_FALSE(vc::peek(nav_state_, VelocitySource::TAKEOVER).has_value());
  EXPECT_FALSE(vc::peek(nav_state_, VelocitySource::OVERRIDE).has_value());
  EXPECT_FALSE(vc::peek(nav_state_, VelocitySource::CONTROLLER).has_value());
}

TEST_F(RecoveryManagerBaseTest, CommandVelocityGoesToTakeover)
{
  manager_->rt_action = TestRecoveryManager::Rt::TAKEOVER;
  EXPECT_TRUE(manager_->internal_update_rt(nav_state_));

  auto takeover = vc::take(nav_state_, VelocitySource::TAKEOVER);
  ASSERT_TRUE(takeover.has_value());
  EXPECT_DOUBLE_EQ(takeover->twist.linear.x, 0.2);
  EXPECT_DOUBLE_EQ(takeover->twist.angular.z, 0.1);
  EXPECT_FALSE(vc::peek(nav_state_, VelocitySource::OVERRIDE).has_value());
  EXPECT_FALSE(vc::peek(nav_state_, VelocitySource::CONTROLLER).has_value());
}

TEST_F(RecoveryManagerBaseTest, OverrideVelocityGoesToOverride)
{
  manager_->rt_action = TestRecoveryManager::Rt::OVERRIDE;
  EXPECT_TRUE(manager_->internal_update_rt(nav_state_));

  auto override_cmd = vc::take(nav_state_, VelocitySource::OVERRIDE);
  ASSERT_TRUE(override_cmd.has_value());
  EXPECT_DOUBLE_EQ(override_cmd->twist.linear.x, -0.1);
  EXPECT_FALSE(vc::peek(nav_state_, VelocitySource::TAKEOVER).has_value());
}

TEST_F(RecoveryManagerBaseTest, ThrowingUpdateRtStopsTheRobot)
{
  for (auto action : {TestRecoveryManager::Rt::THROW_STD, TestRecoveryManager::Rt::THROW_OTHER}) {
    manager_->rt_action = action;
    bool commanded = false;
    EXPECT_NO_THROW(commanded = manager_->internal_update_rt(nav_state_));
    EXPECT_TRUE(commanded);

    auto stop = vc::take(nav_state_, VelocitySource::OVERRIDE);
    ASSERT_TRUE(stop.has_value());
    EXPECT_DOUBLE_EQ(stop->twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(stop->twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(stop->twist.angular.z, 0.0);
  }
}

TEST_F(RecoveryManagerBaseTest, RecoversAfterAThrowingCycle)
{
  manager_->rt_action = TestRecoveryManager::Rt::THROW_STD;
  manager_->internal_update_rt(nav_state_);
  vc::take(nav_state_, VelocitySource::OVERRIDE);

  manager_->rt_action = TestRecoveryManager::Rt::NOTHING;
  EXPECT_FALSE(manager_->internal_update_rt(nav_state_));
  EXPECT_FALSE(vc::peek(nav_state_, VelocitySource::OVERRIDE).has_value());
}

TEST_F(RecoveryManagerBaseTest, ThrowingUpdateDoesNotPropagate)
{
  manager_->throw_in_update = true;
  EXPECT_NO_THROW(manager_->internal_update(nav_state_));
  EXPECT_NO_THROW(manager_->internal_update(nav_state_));
  EXPECT_EQ(manager_->updates, 2);
  EXPECT_FALSE(vc::peek(nav_state_, VelocitySource::OVERRIDE).has_value())
    << "a non-RT failure does not stop the robot";
}

TEST_F(RecoveryManagerBaseTest, ForwardsSystemActions)
{
  auto actions = std::make_shared<RecordingSystemActions>();
  manager_->set_system_actions(actions);

  manager_->hold_mission_progress(true);
  manager_->abort_mission("lost");
  manager_->hold_mission_progress(false);
  manager_->request_shutdown("broken");

  EXPECT_EQ(actions->holds, std::vector<bool>({true, false}));
  EXPECT_EQ(actions->aborted, std::vector<std::string>({"lost"}));
  EXPECT_EQ(actions->shutdowns, std::vector<std::string>({"broken"}));
}

TEST_F(RecoveryManagerBaseTest, SystemActionsWithoutSystemAreIgnored)
{
  EXPECT_NO_THROW(manager_->abort_mission("lost"));
  EXPECT_NO_THROW(manager_->hold_mission_progress(true));
  EXPECT_NO_THROW(manager_->request_shutdown("broken"));

  // The system went away: the weak reference does not keep it alive.
  auto actions = std::make_shared<RecordingSystemActions>();
  manager_->set_system_actions(actions);
  std::weak_ptr<RecordingSystemActions> weak = actions;
  actions.reset();
  EXPECT_TRUE(weak.expired());
  EXPECT_NO_THROW(manager_->abort_mission("lost"));
  EXPECT_NO_THROW(manager_->request_shutdown("broken"));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
