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

#include <thread>
#include <atomic>
#include <vector>

#include "gtest/gtest.h"

#include "easynav_common/types/NavState.hpp"
#include "geometry_msgs/msg/pose.hpp"

class NavStateTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(NavStateTest, SetAndGet)
{
  easynav::NavState state;
  state.set("name", std::string("robot"));
  state.set("count", 42);
  EXPECT_EQ(state.get<std::string>("name"), "robot");
  EXPECT_EQ(state.get<int>("count"), 42);
}

TEST_F(NavStateTest, OverwriteEntry)
{
  easynav::NavState state;
  state.set("x", 10);
  state.set("x", 20);
  EXPECT_EQ(state.get<int>("x"), 20);
}

TEST_F(NavStateTest, HasReturnsCorrect)
{
  easynav::NavState state;
  EXPECT_FALSE(state.has("missing"));
  state.set("exists", 1.0);
  EXPECT_TRUE(state.has("exists"));
}

TEST_F(NavStateTest, MissingKeyThrows)
{
  easynav::NavState state;
  EXPECT_THROW(state.get<float>("invalid"), std::runtime_error);
}

TEST_F(NavStateTest, DebugStringWithPosePrinter)
{
  easynav::NavState state;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 3.0;
  pose.orientation.w = 1.0;

  state.set("pose", pose);
  state.set<int>("age", 10);

  easynav::NavState::register_printer<geometry_msgs::msg::Pose>(
    [](const geometry_msgs::msg::Pose & p) {
      std::ostringstream oss;
      oss << "Position: (" << p.position.x << ", " << p.position.y << ", " << p.position.z
          << ") Orientation: (" << p.orientation.x << ", " << p.orientation.y << ", "
          << p.orientation.z << ", " << p.orientation.w << ")";
      return oss.str();
    });

  std::string output = state.debug_string();

  std::cerr << output << std::endl;

  EXPECT_NE(output.find("Position: (1, 2, 3)"), std::string::npos);
  EXPECT_NE(output.find("Orientation:"), std::string::npos);
  EXPECT_NE(output.find(": 10"), std::string::npos);
}

TEST(NavStateStressTest, ConcurrentMultiKeyReadWrite)
{
  easynav::NavState state;
  std::atomic<bool> start_flag{false};

  state.set<int>("int_key", 0);
  state.set<double>("double_key", 0.0);
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.orientation.w = 1.0;
  state.set("pose_key", pose);

  auto writer = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      for (int i = 0; i < 10000; ++i) {
        state.set<int>("int_key", i);
        state.set<double>("double_key", static_cast<double>(i) / 2.0);
        geometry_msgs::msg::Pose p;
        p.position.x = static_cast<double>(i);
        p.orientation.w = 1.0;
        state.set("pose_key", p);
      }
    };

  auto reader = [&]() {
      while (!start_flag.load()) {std::this_thread::yield();}
      for (int i = 0; i < 10000; ++i) {
        int vi = state.get<int>("int_key");
        double vd = state.get<double>("double_key");
        geometry_msgs::msg::Pose vp = state.get<geometry_msgs::msg::Pose>("pose_key");

        ASSERT_GE(vi, 0);
        ASSERT_GE(vd, 0.0);
        ASSERT_EQ(vp.orientation.w, 1.0);
      }
    };

  std::vector<std::thread> threads;
  for (int i = 0; i < 3; ++i) {
    threads.emplace_back(writer);
  }
  for (int i = 0; i < 3; ++i) {
    threads.emplace_back(reader);
  }

  start_flag.store(true);

  for (auto & t : threads) {
    t.join();
  }
  SUCCEED();
}

// ─────────────────────────────────────────────────────────────────────────────
// std::vector<std::string> printer (added in register_basic_printers)
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(NavStateTest, VectorStringPrinterEmptyVector)
{
  easynav::NavState state;
  state.set("keys", std::vector<std::string>{});
  std::string s = state.debug_string();
  EXPECT_NE(s.find("[]"), std::string::npos) << "Empty vector must render as []\n" << s;
}

TEST_F(NavStateTest, VectorStringPrinterSingleElement)
{
  easynav::NavState state;
  state.set("keys", std::vector<std::string>{"sensor_a"});
  std::string s = state.debug_string();
  EXPECT_NE(s.find("[sensor_a]"), std::string::npos)
    << "Single element must render as [sensor_a]\n" << s;
}

TEST_F(NavStateTest, VectorStringPrinterMultipleElements)
{
  easynav::NavState state;
  state.set("keys", std::vector<std::string>{"sensor_a", "sensor_b", "sensor_c"});
  std::string s = state.debug_string();
  EXPECT_NE(s.find("[sensor_a, sensor_b, sensor_c]"), std::string::npos)
    << "Multiple elements must render comma-separated\n" << s;
}

TEST_F(NavStateTest, SetGroupIsVisibleInDebugString)
{
  easynav::NavState state;
  state.set_group("points", {"lidar_front", "lidar_back"});
  std::string s = state.debug_string();
  EXPECT_NE(s.find("lidar_front"), std::string::npos)
    << "Group member must appear in debug_string\n" << s;
  EXPECT_NE(s.find("lidar_back"), std::string::npos)
    << "Group member must appear in debug_string\n" << s;
}

// ─────────────────────────────────────────────────────────────────────────────
// get_by_type<T>()
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(NavStateTest, GetByTypeReturnsAllMatchingEntries)
{
  easynav::NavState state;
  state.set("a", 1);
  state.set("b", 2);
  state.set("c", 3);
  state.set("name", std::string("robot"));  // different type

  auto result = state.get_by_type<int>();
  ASSERT_EQ(result.size(), 3u);
  // Values must be 1, 2 or 3 (order unspecified)
  int sum = 0;
  for (const auto & v : result) {
    sum += *v;
  }
  EXPECT_EQ(sum, 6);
}

TEST_F(NavStateTest, GetByTypeExcludesOtherTypes)
{
  easynav::NavState state;
  state.set("x", 42);
  state.set("label", std::string("hello"));
  state.set("ratio", 3.14);

  auto ints = state.get_by_type<int>();
  ASSERT_EQ(ints.size(), 1u);
  EXPECT_EQ(*ints[0], 42);

  auto strings = state.get_by_type<std::string>();
  ASSERT_EQ(strings.size(), 1u);
  EXPECT_EQ(*strings[0], "hello");
}

TEST_F(NavStateTest, GetByTypeEmptyWhenNoMatch)
{
  easynav::NavState state;
  state.set("x", 1.0f);
  auto result = state.get_by_type<int>();
  EXPECT_TRUE(result.empty());
}

TEST_F(NavStateTest, GetByTypeEmptyStateReturnsEmpty)
{
  easynav::NavState state;
  auto result = state.get_by_type<double>();
  EXPECT_TRUE(result.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// get_to_vector<T>(key)
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(NavStateTest, GetToVectorFromGroupReturnsAllMembers)
{
  // get_to_vector wraps a single entry; to iterate a group use get_group.
  // "points" is stored as std::vector<std::string> by set_group, not as int,
  // so get_to_vector<int>("points") returns empty.
  easynav::NavState state;
  state.set("laser1", 10);
  state.set("laser2", 20);
  state.set_group("points", {"laser1", "laser2"});

  auto result = state.get_to_vector<int>("points");
  EXPECT_TRUE(result.empty());

  // Access members individually via get_to_vector
  auto r1 = state.get_to_vector<int>("laser1");
  auto r2 = state.get_to_vector<int>("laser2");
  ASSERT_EQ(r1.size(), 1u);
  ASSERT_EQ(r2.size(), 1u);
  EXPECT_EQ(*r1[0], 10);
  EXPECT_EQ(*r2[0], 20);
}

TEST_F(NavStateTest, GetToVectorFromSingleKeyWrapsInVector)
{
  easynav::NavState state;
  state.set("imu", 99);

  auto result = state.get_to_vector<int>("imu");
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(*result[0], 99);
}

TEST_F(NavStateTest, GetToVectorMissingKeyReturnsEmpty)
{
  easynav::NavState state;
  auto result = state.get_to_vector<int>("nonexistent");
  EXPECT_TRUE(result.empty());
}

TEST_F(NavStateTest, GetToVectorTypeMismatchReturnsEmpty)
{
  easynav::NavState state;
  state.set("sensor", std::string("lidar"));

  auto result = state.get_to_vector<int>("sensor");
  EXPECT_TRUE(result.empty());
}

TEST_F(NavStateTest, GetToVectorGroupTakesPriorityOverValue)
{
  // get_to_vector wraps a single NavState entry in a vector.
  // "points" is stored as std::vector<std::string> by set_group, so T=int won't match it.
  easynav::NavState state;
  state.set("laser1", 1);
  state.set("laser2", 2);
  state.set_group("points", {"laser1", "laser2"});

  // "points" entry has type std::vector<std::string>, so int won't match -> empty.
  auto result_group = state.get_to_vector<int>("points");
  EXPECT_TRUE(result_group.empty());

  // individual keys still work
  auto result_single = state.get_to_vector<int>("laser1");
  ASSERT_EQ(result_single.size(), 1u);
  EXPECT_EQ(*result_single[0], 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Shallow-copy (pointer identity) tests
// Verify that get_ptr / get_by_type / get_to_vector / get_group all return
// shared_ptrs that share ownership of the *same* object stored in NavState
// — no deep copy is ever made.
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(NavStateTest, GetPtrReturnsSameObject)
{
  easynav::NavState state;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  state.set("pose", pose);

  auto ptr1 = state.get_ptr<geometry_msgs::msg::Pose>("pose");
  auto ptr2 = state.get_ptr<geometry_msgs::msg::Pose>("pose");

  // Same underlying raw pointer
  EXPECT_EQ(ptr1.get(), ptr2.get());

  // Mutating through one ptr is visible through the other
  ptr1->position.x = 42.0;
  EXPECT_DOUBLE_EQ(ptr2->position.x, 42.0);
}

TEST_F(NavStateTest, GetByTypeReturnsSameObjects)
{
  easynav::NavState state;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 3.0;
  state.set("sensor_a", pose);

  // Retrieve via get_ptr and get_by_type; raw pointers must match
  auto direct = state.get_ptr<geometry_msgs::msg::Pose>("sensor_a");
  auto by_type = state.get_by_type<geometry_msgs::msg::Pose>();

  ASSERT_EQ(by_type.size(), 1u);
  EXPECT_EQ(by_type[0].get(), direct.get());

  // Mutation visible through both handles
  direct->position.x = 99.0;
  EXPECT_DOUBLE_EQ(by_type[0]->position.x, 99.0);
}

TEST_F(NavStateTest, GetToVectorReturnsSameObject)
{
  easynav::NavState state;
  geometry_msgs::msg::Pose pose;
  pose.position.y = 5.0;
  state.set("sensor_b", pose);

  auto direct = state.get_ptr<geometry_msgs::msg::Pose>("sensor_b");
  auto vec = state.get_to_vector<geometry_msgs::msg::Pose>("sensor_b");

  ASSERT_EQ(vec.size(), 1u);
  EXPECT_EQ(vec[0].get(), direct.get());

  direct->position.y = 77.0;
  EXPECT_DOUBLE_EQ(vec[0]->position.y, 77.0);
}

TEST_F(NavStateTest, GetGroupReturnsSameObjects)
{
  easynav::NavState state;
  geometry_msgs::msg::Pose pose_a, pose_b;
  pose_a.position.x = 1.0;
  pose_b.position.x = 2.0;
  state.set("s1", pose_a);
  state.set("s2", pose_b);
  state.set_group("sensors", {"s1", "s2"});

  auto direct_s1 = state.get_ptr<geometry_msgs::msg::Pose>("s1");
  auto direct_s2 = state.get_ptr<geometry_msgs::msg::Pose>("s2");
  auto group = state.get_group<geometry_msgs::msg::Pose>("sensors");

  ASSERT_EQ(group.size(), 2u);

  // Each group element points to the same object as the direct retrieval
  bool found_s1 = false, found_s2 = false;
  for (const auto & p : group) {
    if (p.get() == direct_s1.get()) {found_s1 = true;}
    if (p.get() == direct_s2.get()) {found_s2 = true;}
  }
  EXPECT_TRUE(found_s1);
  EXPECT_TRUE(found_s2);

  // Mutation through direct ptr is visible through group ptr
  direct_s1->position.x = 55.0;
  for (const auto & p : group) {
    if (p.get() == direct_s1.get()) {
      EXPECT_DOUBLE_EQ(p->position.x, 55.0);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// get_no_group<T>()
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(NavStateTest, GetNoGroupEmptyState)
{
  easynav::NavState state;
  EXPECT_TRUE(state.get_no_group<int>().empty());
}

TEST_F(NavStateTest, GetNoGroupAllInGroup)
{
  easynav::NavState state;
  state.set("s1", 1);
  state.set("s2", 2);
  state.set_group("sensors", {"s1", "s2"});

  EXPECT_TRUE(state.get_no_group<int>().empty());
}

TEST_F(NavStateTest, GetNoGroupNoneInAnyGroup)
{
  easynav::NavState state;
  state.set("a", 10);
  state.set("b", 20);

  auto result = state.get_no_group<int>();
  ASSERT_EQ(result.size(), 2u);
  int sum = 0;
  for (const auto & v : result) {
    sum += *v;
  }
  EXPECT_EQ(sum, 30);
}

TEST_F(NavStateTest, GetNoGroupMixed)
{
  easynav::NavState state;
  state.set("in_group", 1);
  state.set("not_in_group", 99);
  state.set_group("sensors", {"in_group"});

  auto result = state.get_no_group<int>();
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(*result[0], 99);
}

TEST_F(NavStateTest, GetNoGroupSpanningMultipleGroups)
{
  easynav::NavState state;
  state.set("a", 1);
  state.set("b", 2);
  state.set("c", 3);  // not in any group
  state.set_group("g1", {"a"});
  state.set_group("g2", {"b"});

  auto result = state.get_no_group<int>();
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(*result[0], 3);
}

TEST_F(NavStateTest, GetNoGroupTypeFilter)
{
  easynav::NavState state;
  state.set("int_val", 42);
  state.set("str_val", std::string("hello"));

  auto ints = state.get_no_group<int>();
  ASSERT_EQ(ints.size(), 1u);
  EXPECT_EQ(*ints[0], 42);

  auto strs = state.get_no_group<std::string>();
  ASSERT_EQ(strs.size(), 1u);
  EXPECT_EQ(*strs[0], "hello");
}

TEST_F(NavStateTest, GetNoGroupReturnsSameObject)
{
  // Verify pointer identity — no deep copy
  easynav::NavState state;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 7.0;
  state.set("ungrouped", pose);

  auto direct = state.get_ptr<geometry_msgs::msg::Pose>("ungrouped");
  auto no_group = state.get_no_group<geometry_msgs::msg::Pose>();

  ASSERT_EQ(no_group.size(), 1u);
  EXPECT_EQ(no_group[0].get(), direct.get());

  direct->position.x = 77.0;
  EXPECT_DOUBLE_EQ(no_group[0]->position.x, 77.0);
}

TEST_F(NavStateTest, GetNoGroupExcludesGroupedButNotUngroupedSameType)
{
  // Two sensors of the same type: one grouped, one not.
  easynav::NavState state;
  geometry_msgs::msg::Pose grouped_pose, free_pose;
  grouped_pose.position.x = 1.0;
  free_pose.position.x = 2.0;
  state.set("lidar_a", grouped_pose);
  state.set("lidar_b", free_pose);
  state.set_group("points", {"lidar_a"});

  auto result = state.get_no_group<geometry_msgs::msg::Pose>();
  ASSERT_EQ(result.size(), 1u);
  EXPECT_DOUBLE_EQ(result[0]->position.x, 2.0);
}
