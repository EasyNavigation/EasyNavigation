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
/// \brief FaultyMapsManager: each fault does what it says.

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_maps_manager/fault_injection/FaultyMapsManager.hpp"

using namespace std::chrono_literals;

class FaultyMapsManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  std::shared_ptr<easynav::FaultyMapsManager> make(const std::string & fault, int fault_after = 0)
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "faulty_maps_manager_test", rclcpp::NodeOptions().parameter_overrides(
        {{"maps.fault", fault}, {"maps.fault_after", fault_after}, {"maps.hang_time", 0.05}}));
    auto maps = std::make_shared<easynav::FaultyMapsManager>();
    maps->initialize(node_, "maps");
    return maps;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  easynav::NavState nav_state_;
};

TEST_F(FaultyMapsManagerTest, UnknownFaultsFailToInitialize)
{
  EXPECT_THROW(make("explode"), std::invalid_argument);
}

TEST_F(FaultyMapsManagerTest, WithoutFaultItDoesNothing)
{
  auto maps = make("none");
  EXPECT_NO_THROW(maps->update(nav_state_));
  EXPECT_EQ(maps->updates(), 1);
}

TEST_F(FaultyMapsManagerTest, ThrowStartsAfterFaultAfterUpdates)
{
  auto maps = make("throw", 2);
  EXPECT_NO_THROW(maps->update(nav_state_));
  EXPECT_NO_THROW(maps->update(nav_state_));
  EXPECT_THROW(maps->update(nav_state_), std::runtime_error);
}

TEST_F(FaultyMapsManagerTest, HangBlocks)
{
  auto maps = make("hang");
  const auto start = std::chrono::steady_clock::now();
  maps->update(nav_state_);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 50ms);
}
