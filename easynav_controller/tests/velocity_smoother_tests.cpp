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

#include "gtest/gtest.h"

#include "geometry_msgs/msg/twist.hpp"

#include "easynav_controller/VelocitySmoother.hpp"

namespace
{

easynav::RobotLimits limits()
{
  easynav::RobotLimits l;
  l.max_linear_vel = 1.0;
  l.min_linear_vel = -0.5;
  l.max_angular_vel = 2.0;
  l.max_linear_acc = 1.0;
  l.max_linear_decel = 2.0;
  l.max_angular_acc = 4.0;
  l.max_angular_decel = 8.0;
  return l;
}

geometry_msgs::msg::Twist twist(double vx, double wz = 0.0)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = vx;
  t.angular.z = wz;
  return t;
}

}  // namespace

TEST(VelocitySmootherTest, AcceleratesWithinTheAccelerationLimit)
{
  easynav::VelocitySmoother smoother;
  smoother.set_limits(limits());

  EXPECT_DOUBLE_EQ(smoother.step(twist(1.0, 2.0), 0.1).linear.x, 0.1);   // 1.0 m/s^2
  EXPECT_DOUBLE_EQ(smoother.current().angular.z, 0.4);                   // 4.0 rad/s^2
  EXPECT_FALSE(smoother.reached(twist(1.0, 2.0)));

  for (int i = 0; i < 20; ++i) {
    smoother.step(twist(1.0, 2.0), 0.1);
  }
  EXPECT_TRUE(smoother.reached(twist(1.0, 2.0)));
  EXPECT_DOUBLE_EQ(smoother.current().linear.x, 1.0);
}

TEST(VelocitySmootherTest, BrakesWithinTheDecelerationLimitInsteadOfStoppingDead)
{
  easynav::VelocitySmoother smoother;
  smoother.set_limits(limits());
  smoother.reset(twist(1.0));

  // From 1.0 to 0.0 at 2.0 m/s^2: 0.5 s, not one step.
  EXPECT_DOUBLE_EQ(smoother.step(twist(0.0), 0.1).linear.x, 0.8);
  EXPECT_NEAR(smoother.time_to_stop(), 0.4, 1e-9);
  for (int i = 0; i < 4; ++i) {
    smoother.step(twist(0.0), 0.1);
  }
  EXPECT_NEAR(smoother.current().linear.x, 0.0, 1e-9);
}

TEST(VelocitySmootherTest, ClampsToTheVelocityLimits)
{
  easynav::VelocitySmoother smoother;
  smoother.set_limits(limits());

  for (int i = 0; i < 100; ++i) {
    smoother.step(twist(5.0, -9.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.x, 1.0);
  EXPECT_DOUBLE_EQ(smoother.current().angular.z, -2.0);
  EXPECT_TRUE(smoother.reached(twist(5.0, -9.0)));

  for (int i = 0; i < 100; ++i) {
    smoother.step(twist(-5.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.x, -0.5);  // min_linear_vel
}

TEST(VelocitySmootherTest, ChangingDirectionStopsAtZeroFirst)
{
  easynav::VelocitySmoother smoother;
  smoother.set_limits(limits());
  smoother.reset(twist(0.1));

  // 0.1 m/s forward, target backwards: the step would cross zero, so it stops at zero.
  EXPECT_DOUBLE_EQ(smoother.step(twist(-0.5), 0.1).linear.x, 0.0);
  EXPECT_LT(smoother.step(twist(-0.5), 0.1).linear.x, 0.0);
}
