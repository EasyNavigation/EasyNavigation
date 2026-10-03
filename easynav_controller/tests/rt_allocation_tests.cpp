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
/// \brief The RT velocity path (proposals, mux, smoother) must not allocate memory once warm.

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <new>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_controller/VelocityMux.hpp"
#include "easynav_controller/VelocitySmoother.hpp"
#include "easynav_controller/safety/CommandGuard.hpp"
#include "easynav_core/SafetyChannel.hpp"
#include "easynav_core/VelocityCommand.hpp"

namespace
{
std::atomic<bool> counting {false};
std::atomic<size_t> allocations {0};
}  // namespace

// Counts every allocation made while counting is on (this test binary only).
void * operator new(std::size_t size)
{
  if (counting) {++allocations;}
  if (void * p = std::malloc(size ? size : 1)) {return p;}
  throw std::bad_alloc();
}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}

namespace
{

// One RT cycle of the velocity path, as ControllerNode and a component taking over the motion run it.
void rt_cycle(
  easynav::NavState & nav_state, easynav::VelocityMux & mux, easynav::VelocitySmoother & smoother,
  const geometry_msgs::msg::TwistStamped & controller_cmd,
  const geometry_msgs::msg::TwistStamped & takeover_cmd, bool with_takeover)
{
  easynav::velocity_command::propose(
    nav_state, easynav::VelocitySource::CONTROLLER, controller_cmd);
  if (with_takeover) {
    easynav::velocity_command::propose(
      nav_state, easynav::VelocitySource::TAKEOVER, takeover_cmd);
  }
  // Reading the pending commands without consuming them (as a safety check would).
  (void)easynav::velocity_command::peek(nav_state, easynav::VelocitySource::TAKEOVER);
  (void)easynav::velocity_command::peek(nav_state, easynav::VelocitySource::CONTROLLER);
  const auto selection = mux.select(nav_state);
  (void)smoother.step(selection.cmd.twist, 0.005);
}

}  // namespace

TEST(RtAllocationTest, VelocityPathDoesNotAllocateOnceWarm)
{
  easynav::NavState nav_state;
  nav_state.set("navigation_paused", false);
  easynav::VelocityMux mux;
  easynav::VelocitySmoother smoother;
  smoother.set_limits(easynav::RobotLimits{});

  geometry_msgs::msg::TwistStamped controller_cmd;
  controller_cmd.header.frame_id = "base_footprint";
  controller_cmd.twist.linear.x = 0.4;
  auto takeover_cmd = controller_cmd;
  takeover_cmd.twist.linear.x = -0.1;

  // Warm-up: every NavState slot is created once (allowed, "at the start").
  for (int i = 0; i < 3; ++i) {
    rt_cycle(nav_state, mux, smoother, controller_cmd, takeover_cmd, true);
  }

  allocations = 0;
  counting = true;
  for (int i = 0; i < 1000; ++i) {
    rt_cycle(nav_state, mux, smoother, controller_cmd, takeover_cmd, i % 2 == 0);
  }
  counting = false;

  EXPECT_EQ(allocations.load(), 0u) << "allocations in 1000 RT cycles of the velocity path";
}

TEST(RtAllocationTest, SafetyChannelPathDoesNotAllocateOnceWarm)
{
  easynav::NavState nav_state;
  easynav::VelocityMux mux;
  easynav::VelocitySmoother smoother;
  const easynav::RobotLimits configured;
  smoother.set_limits(configured);

  geometry_msgs::msg::TwistStamped controller_cmd;
  controller_cmd.header.frame_id = "base_footprint";
  controller_cmd.twist.linear.x = 0.4;
  auto takeover_cmd = controller_cmd;

  // What SystemNode writes and ControllerNode applies, every RT cycle: stops and speed limits.
  auto cycle = [&](int i) {
      easynav::SafetyChannelState state;
      state.protective_stop = i % 3 == 0;
      state.max_linear_vel = (i % 2 == 0) ? 0.2 : std::numeric_limits<double>::infinity();
      nav_state.set(easynav::kSafetyStatusKey, state);
      const auto applied = nav_state.get_safe<easynav::SafetyChannelState>(
        easynav::kSafetyStatusKey);
      smoother.set_limits(easynav::limited_by(configured, applied));
      rt_cycle(nav_state, mux, smoother, controller_cmd, takeover_cmd, i % 2 == 0);
    };
  for (int i = 0; i < 3; ++i) {
    cycle(i);
  }

  allocations = 0;
  counting = true;
  for (int i = 0; i < 1000; ++i) {
    cycle(i);
  }
  counting = false;

  EXPECT_EQ(allocations.load(), 0u) << "allocations in 1000 RT cycles with the safety channel";
}

TEST(RtAllocationTest, GuardedVelocityOutputDoesNotAllocateOnceWarm)
{
  // Timed out, discarding NaN, reporting: CommandGuard around the mux, as in ControllerNode.
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "rt_guard_node", rclcpp::NodeOptions().parameter_overrides({{"cmd_timeout", 0.1}}));
  easynav::safety::CommandGuard guard;
  guard.declare_parameters(*node);
  ASSERT_TRUE(guard.configure(*node));

  easynav::NavState nav_state;
  nav_state.set("navigation_paused", false);
  easynav::VelocityMux mux;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = "base_footprint";
  cmd.twist.linear.x = 0.4;
  auto nan_cmd = cmd;
  nan_cmd.twist.linear.x = std::nan("");

  auto cycle = [&](int64_t seconds) {
      const bool discarded = guard.discard_non_finite(nav_state);
      const auto selection = guard.supervise(mux.select(nav_state), rclcpp::Time(seconds, 0));
      guard.report(nav_state, discarded, selection.fresh);
      (void)guard.keepalive_due(rclcpp::Time(seconds, 0));
      (void)guard.is_new(cmd);
    };

  // Warm-up: slots created, a target held, timed out and reported once.
  easynav::velocity_command::propose(nav_state, easynav::VelocitySource::CONTROLLER, cmd);
  cycle(0);
  cycle(1);
  easynav::velocity_command::propose(nav_state, easynav::VelocitySource::CONTROLLER, nan_cmd);
  cycle(2);
  ASSERT_TRUE(guard.timed_out());

  allocations = 0;
  counting = true;
  for (int i = 0; i < 1000; ++i) {
    if (i % 3 == 0) {
      easynav::velocity_command::propose(nav_state, easynav::VelocitySource::CONTROLLER, nan_cmd);
    }
    cycle(3 + i);
  }
  counting = false;

  EXPECT_TRUE(guard.timed_out());
  EXPECT_EQ(allocations.load(), 0u) << "allocations in 1000 timed-out/discarding RT cycles";
  rclcpp::shutdown();
}

TEST(RtAllocationTest, TheCounterSeesAllocations)
{
  // Negative control: a key longer than the small-string buffer allocates, as a literal passed
  // to NavState did before the keys were made static.
  easynav::NavState nav_state;
  nav_state.set("a_key_longer_than_the_small_string_buffer", 1.0);

  allocations = 0;
  counting = true;
  nav_state.set("a_key_longer_than_the_small_string_buffer", 2.0);
  counting = false;

  EXPECT_GT(allocations.load(), 0u);
}
