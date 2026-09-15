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

#include <cmath>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_core/ObstacleProximity.hpp"

class ObstacleProximityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    easynav::TFInfo tf_info;
    tf_info.robot_frame = "base_link";
    easynav::RTTFBuffer::getInstance()->set_tf_info(tf_info);
  }
};

TEST_F(ObstacleProximityTest, ReturnsInfiniteDistanceWithNoPerceptions)
{
  easynav::NavState nav_state;
  auto result = easynav::compute_nearest_obstacle(nav_state);
  EXPECT_FALSE(std::isfinite(result.distance));
}

TEST_F(ObstacleProximityTest, FindsNearestPointAheadOfTheRobot)
{
  easynav::PointPerception perception;
  perception.frame_id = "base_link";  // same as robot_frame: no TF lookup needed
  perception.stamp = rclcpp::Time(0);
  perception.valid = true;
  perception.data.points.resize(2);
  perception.data.points[0].x = 3.0;
  perception.data.points[0].y = 0.0;
  perception.data.points[0].z = 0.0;
  perception.data.points[1].x = 1.0;   // nearer, straight ahead
  perception.data.points[1].y = 0.0;
  perception.data.points[1].z = 0.0;

  easynav::NavState nav_state;
  nav_state.set("scan", perception);

  auto result = easynav::compute_nearest_obstacle(nav_state);

  ASSERT_TRUE(std::isfinite(result.distance));
  EXPECT_NEAR(result.distance, 1.0, 1e-6);
  EXPECT_NEAR(result.bearing, 0.0, 1e-6);  // straight ahead
}

TEST_F(ObstacleProximityTest, ReportsBearingForAnObstacleBehindTheRobot)
{
  easynav::PointPerception perception;
  perception.frame_id = "base_link";
  perception.stamp = rclcpp::Time(0);
  perception.valid = true;
  perception.data.points.resize(1);
  perception.data.points[0].x = -1.0;  // directly behind
  perception.data.points[0].y = 0.0;
  perception.data.points[0].z = 0.0;

  easynav::NavState nav_state;
  nav_state.set("scan", perception);

  auto result = easynav::compute_nearest_obstacle(nav_state);

  ASSERT_TRUE(std::isfinite(result.distance));
  EXPECT_NEAR(result.distance, 1.0, 1e-6);
  EXPECT_NEAR(std::abs(result.bearing), M_PI, 1e-6);  // behind: bearing near +-pi
}
