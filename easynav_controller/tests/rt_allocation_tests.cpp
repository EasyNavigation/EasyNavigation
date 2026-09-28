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

/// \file
/// \brief The RT velocity path (proposals, mux, smoother) must not allocate memory once warm.

#include <atomic>
#include <cstdlib>
#include <new>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_controller/VelocityMux.hpp"
#include "easynav_controller/VelocitySmoother.hpp"
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

// One RT cycle of the velocity path, as ControllerNode and the recovery system run it.
void rt_cycle(
  easynav::NavState & nav_state, easynav::VelocityMux & mux, easynav::VelocitySmoother & smoother,
  const geometry_msgs::msg::TwistStamped & controller_cmd,
  const geometry_msgs::msg::TwistStamped & recovery_cmd, bool with_recovery)
{
  easynav::velocity_command::propose(
    nav_state, easynav::VelocitySource::CONTROLLER, controller_cmd);
  if (with_recovery) {
    easynav::velocity_command::propose(
      nav_state, easynav::VelocitySource::RECOVERY, recovery_cmd);
  }
  // What a reflex checks.
  (void)easynav::velocity_command::peek(nav_state, easynav::VelocitySource::RECOVERY);
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
  auto recovery_cmd = controller_cmd;
  recovery_cmd.twist.linear.x = -0.1;

  // Warm-up: every NavState slot is created once (allowed, "at the start").
  for (int i = 0; i < 3; ++i) {
    rt_cycle(nav_state, mux, smoother, controller_cmd, recovery_cmd, true);
  }

  allocations = 0;
  counting = true;
  for (int i = 0; i < 1000; ++i) {
    rt_cycle(nav_state, mux, smoother, controller_cmd, recovery_cmd, i % 2 == 0);
  }
  counting = false;

  EXPECT_EQ(allocations.load(), 0u) << "allocations in 1000 RT cycles of the velocity path";
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
