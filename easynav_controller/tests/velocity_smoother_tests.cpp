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
  l.max_lateral_vel = 0.4;
  l.max_lateral_acc = 0.5;
  l.max_lateral_decel = 1.0;
  return l;
}

geometry_msgs::msg::Twist sideways(double vy)
{
  geometry_msgs::msg::Twist t;
  t.linear.y = vy;
  return t;
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

TEST(VelocitySmootherTest, LateralUsesItsOwnLimits)
{
  easynav::VelocitySmoother smoother;
  smoother.set_limits(limits());

  EXPECT_DOUBLE_EQ(smoother.step(sideways(1.0), 0.1).linear.y, 0.05);  // 0.5 m/s^2, not 1.0
  for (int i = 0; i < 100; ++i) {
    smoother.step(sideways(1.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.y, 0.4) << "max_lateral_vel, not max_linear_vel";
  EXPECT_TRUE(smoother.reached(sideways(1.0)));

  for (int i = 0; i < 100; ++i) {
    smoother.step(sideways(-1.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.y, -0.4) << "symmetric, unlike min_linear_vel";

  // Braking at 1.0 m/s^2 (lateral decel), not 2.0 (linear).
  EXPECT_DOUBLE_EQ(smoother.step(sideways(0.0), 0.1).linear.y, -0.3);
  EXPECT_NEAR(smoother.time_to_stop(), 0.3, 1e-9);
}

TEST(VelocitySmootherTest, TimeToStopIsTheSlowestAxis)
{
  easynav::VelocitySmoother smoother;
  smoother.set_limits(limits());
  auto t = twist(0.4, 0.8);
  t.linear.y = 0.4;
  smoother.reset(t);
  // x: 0.4 / 2.0 = 0.2 s, y: 0.4 / 1.0 = 0.4 s, z: 0.8 / 8.0 = 0.1 s.
  EXPECT_NEAR(smoother.time_to_stop(), 0.4, 1e-9);
}

TEST(VelocitySmootherTest, ZeroLateralLimitKeepsANonHolonomicRobotFromSlipping)
{
  auto l = limits();
  l.max_lateral_vel = 0.0;
  easynav::VelocitySmoother smoother;
  smoother.set_limits(l);
  auto t = twist(0.5);
  t.linear.y = 0.3;
  for (int i = 0; i < 20; ++i) {
    smoother.step(t, 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.y, 0.0);
  EXPECT_DOUBLE_EQ(smoother.current().linear.x, 0.5);
}

geometry_msgs::msg::Twist vertical(double vz)
{
  geometry_msgs::msg::Twist t;
  t.linear.z = vz;
  return t;
}

TEST(VelocitySmootherTest, NoVerticalLimitKeepsAGroundRobotOnTheGround)
{
  easynav::VelocitySmoother smoother;
  smoother.set_limits(limits());  // max_vertical_vel: 0 by default
  for (int i = 0; i < 20; ++i) {
    smoother.step(vertical(1.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.z, 0.0);
  EXPECT_TRUE(smoother.reached(vertical(1.0))) << "the target is clamped to 0";
}

TEST(VelocitySmootherTest, VerticalUsesItsOwnLimits)
{
  auto l = limits();
  l.max_vertical_vel = 0.6;
  l.max_vertical_acc = 0.3;
  l.max_vertical_decel = 1.5;
  easynav::VelocitySmoother smoother;
  smoother.set_limits(l);

  EXPECT_DOUBLE_EQ(smoother.step(vertical(2.0), 0.1).linear.z, 0.03);  // 0.3 m/s^2
  for (int i = 0; i < 100; ++i) {
    smoother.step(vertical(2.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.z, 0.6) << "max_vertical_vel";
  EXPECT_TRUE(smoother.reached(vertical(2.0)));
  EXPECT_NEAR(smoother.time_to_stop(), 0.4, 1e-9) << "0.6 / 1.5";

  // Braking at 1.5 m/s^2, then down: a change of direction stops at zero first
  EXPECT_NEAR(smoother.step(vertical(-2.0), 0.1).linear.z, 0.45, 1e-9);
  for (int i = 0; i < 3; ++i) {
    smoother.step(vertical(-2.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.z, 0.0);
  for (int i = 0; i < 100; ++i) {
    smoother.step(vertical(-2.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.z, -0.6) << "symmetric";
}

TEST(VelocitySmootherTest, ResetKeepsTheVerticalVelocity)
{
  auto l = limits();
  l.max_vertical_vel = 1.0;
  easynav::VelocitySmoother smoother;
  smoother.set_limits(l);
  auto t = twist(0.2);
  t.linear.z = 0.7;
  smoother.reset(t);
  EXPECT_DOUBLE_EQ(smoother.current().linear.z, 0.7);
  EXPECT_FALSE(smoother.reached(twist(0.2)));
}

TEST(VelocitySmootherTest, TurningRadiusBoundsTheTargetCurvature)
{
  auto l = limits();
  l.min_turning_radius = 2.0;
  easynav::VelocitySmoother smoother;
  smoother.set_limits(l);
  // The angular velocity the smoother settles at, from rest, for each target
  auto settled = [&smoother](double vx, double wz) {
      smoother.reset(geometry_msgs::msg::Twist());
      for (int i = 0; i < 100; ++i) {
        smoother.step(twist(vx, wz), 0.1);
      }
      EXPECT_TRUE(smoother.reached(twist(vx, wz)));
      return smoother.current().angular.z;
    };
  // 0.4 m/s at radius 2 m: at most 0.2 rad/s, either way
  EXPECT_DOUBLE_EQ(settled(0.4, 1.0), 0.2);
  EXPECT_DOUBLE_EQ(settled(0.4, -1.0), -0.2);
  EXPECT_DOUBLE_EQ(settled(0.4, 0.1), 0.1) << "within: untouched";
  EXPECT_DOUBLE_EQ(settled(-0.1, 1.0), 0.05) << "reversing too";
  EXPECT_DOUBLE_EQ(settled(0.0, 1.0), 0.0) << "no turning in place";
  // The velocity clamp first: max_linear_vel 1.0 -> 0.5 rad/s
  EXPECT_DOUBLE_EQ(settled(3.0, 1.0), 0.5);
}

TEST(VelocitySmootherTest, TurningRadiusHoldsWhileTheSpeedRamps)
{
  auto l = limits();
  l.min_turning_radius = 2.0;
  easynav::VelocitySmoother smoother;
  smoother.set_limits(l);
  // From rest to (0.5, 0.25): angular accelerates faster (4 rad/s^2) than linear (1 m/s^2), so
  // every command must keep the curvature of the slower axis
  for (int i = 0; i < 20; ++i) {
    const auto & c = smoother.step(twist(0.5, 0.25), 0.05);
    EXPECT_LE(std::abs(c.angular.z), std::abs(c.linear.x) / 2.0 + 1e-12) << "step " << i;
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.x, 0.5);
  EXPECT_DOUBLE_EQ(smoother.current().angular.z, 0.25);

  // Braking to a stop while still asking for the turn: the turn goes down with the speed
  for (int i = 0; i < 20; ++i) {
    const auto & c = smoother.step(twist(0.0, 0.25), 0.05);
    EXPECT_LE(std::abs(c.angular.z), std::abs(c.linear.x) / 2.0 + 1e-12) << "braking " << i;
  }
  EXPECT_DOUBLE_EQ(smoother.current().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(smoother.current().angular.z, 0.0);
}

TEST(VelocitySmootherTest, NoTurningRadiusLetsTheRobotTurnInPlace)
{
  easynav::VelocitySmoother smoother;
  smoother.set_limits(limits());  // min_turning_radius: 0 by default
  for (int i = 0; i < 20; ++i) {
    smoother.step(twist(0.0, 1.0), 0.1);
  }
  EXPECT_DOUBLE_EQ(smoother.current().angular.z, 1.0);
  EXPECT_DOUBLE_EQ(smoother.current().linear.x, 0.0);
}
