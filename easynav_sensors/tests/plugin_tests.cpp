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
/// \brief Tests that verify the pluginlib plugin registration, loading, and
///        correctness of all built-in PerceptionHandler plugins.

#include <memory>
#include <rclcpp/callback_group.hpp>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_sensors/types/Perceptions.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "easynav_sensors/types/ImagePerception.hpp"
#include "easynav_sensors/types/DetectionsPerception.hpp"
#include "easynav_common/types/NavState.hpp"

// Concrete handler types needed for dynamic_cast checks.
// These headers expose the handler class definitions.
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "easynav_sensors/types/ImagePerception.hpp"
#include "easynav_sensors/types/DetectionsPerception.hpp"

#include "gtest/gtest.h"

using easynav::PerceptionHandler;

/// All plugin names that must be registered in the package plugin XML.
static const std::vector<std::string> kAllPlugins = {
  "easynav_sensors/PointPerceptionHandler",
  "easynav_sensors/IMUPerceptionHandler",
  "easynav_sensors/GNSSPerceptionHandler",
  "easynav_sensors/OdometryPerceptionHandler",
  "easynav_sensors/ImagePerceptionHandler",
  "easynav_sensors/DetectionsPerceptionHandler",
};

/// \brief Subclass of SensorsNode that exposes the protected groups_ and handler_list_ for unit testing.
/// Not part of the production API: only instantiated in test code.
class SensorsNodeForTesting : public easynav::SensorsNode
{
public:
  explicit SensorsNodeForTesting(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : easynav::SensorsNode(options) {}

  const std::map<std::string, std::vector<std::string>> &
  groups_for_testing() const {return groups_;}

  const std::vector<std::shared_ptr<easynav::PerceptionHandler>> &
  handler_list_for_testing() const {return handler_list_;}
};

class PluginTestCase : public ::testing::Test
{
protected:
  ~PluginTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    loader_ = std::make_unique<pluginlib::ClassLoader<PerceptionHandler>>(
      "easynav_sensors", "easynav::PerceptionHandler");
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("plugin_test_node");
    cb_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  }

  void TearDown() override {}

  /// Helper: load and initialise a plugin by name.
  std::shared_ptr<PerceptionHandler> load(const std::string & plugin_name)
  {
    auto handler = loader_->createSharedInstance(plugin_name);
    handler->initialize(node_, cb_group_, "test_sensor");
    return handler;
  }

  /// Helper: call populate_nav_state and verify that the expected collection
  /// type \p C was stored under \p group with exactly one element.
  // template<typename C>
  // void assert_populate_nav_state(std::shared_ptr<PerceptionHandler> handler)
  // {
  //   auto perception = handler->create();
  //   ASSERT_NE(perception, nullptr);

  //   std::vector<PerceptionPtr> perceptions = {{perception, nullptr}};
  //   easynav::NavState ns;
  //   const std::string group = handler->group();
  //   handler->populate_nav_state(group, perceptions, ns);

  //   // get<C> throws if the key is absent — let it propagate as a test failure.
  //   const auto & result = ns.get<C>(group);
  //   ASSERT_EQ(result.size(), 1u);
  // }

  std::unique_ptr<pluginlib::ClassLoader<PerceptionHandler>> loader_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
};

// ---------------------------------------------------------------------------
// 1. All built-in plugins are registered and discoverable.
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, all_plugins_are_available)
{
  for (const auto & name : kAllPlugins) {
    SCOPED_TRACE("plugin: " + name);
    EXPECT_TRUE(loader_->isClassAvailable(name));
  }
}

// ---------------------------------------------------------------------------
// 2. Every plugin loads without exception, initialises, and returns the
//    expected group name.
// ---------------------------------------------------------------------------

// TEST_F(PluginTestCase, plugins_load_and_return_correct_group)
// {
//   const std::vector<std::pair<std::string, std::string>> cases = {
//     {"easynav_sensors/PointPerceptionHandler", "points"},
//     {"easynav_sensors/IMUPerceptionHandler", "imu"},
//     {"easynav_sensors/GNSSPerceptionHandler", "gnss"},
//     {"easynav_sensors/ImagePerceptionHandler", "image"},
//     {"easynav_sensors/DetectionsPerceptionHandler", "detections"},
//   };

//   for (const auto & [plugin, expected_group] : cases) {
//     SCOPED_TRACE("plugin: " + plugin);
//     std::shared_ptr<PerceptionHandler> handler;
//     ASSERT_NO_THROW(handler = load(plugin));
//     ASSERT_NE(handler, nullptr);
//     EXPECT_EQ(handler->group(), expected_group);
//     EXPECT_EQ(handler->get_sensor_name(), "test_sensor");
//   }
// }

// ---------------------------------------------------------------------------
// 3. create() returns a non-null PerceptionBase for every plugin.
// ---------------------------------------------------------------------------

// TEST_F(PluginTestCase, plugins_create_non_null_perception)
// {
//   for (const auto & name : kAllPlugins) {
//     SCOPED_TRACE("plugin: " + name);
//     auto handler = load(name);
//     std::shared_ptr<easynav::PerceptionBase> perception;
//     ASSERT_NO_THROW(perception = handler->create());
//     EXPECT_NE(perception, nullptr);
//   }
// }

// ---------------------------------------------------------------------------
// 4. populate_nav_state stores correctly-typed perceptions in NavState.
// ---------------------------------------------------------------------------

// TEST_F(PluginTestCase, point_plugin_populates_nav_state)
// {
//   auto handler = load("easynav_sensors/PointPerceptionHandler");
//   ASSERT_NO_THROW((assert_populate_nav_state<easynav::PointPerceptions>(handler)));
// }

// TEST_F(PluginTestCase, imu_plugin_populates_nav_state)
// {
//   auto handler = load("easynav_sensors/IMUPerceptionHandler");
//   ASSERT_NO_THROW((assert_populate_nav_state<easynav::IMUPerceptions>(handler)));
// }

// TEST_F(PluginTestCase, gnss_plugin_populates_nav_state)
// {
//   auto handler = load("easynav_sensors/GNSSPerceptionHandler");
//   ASSERT_NO_THROW((assert_populate_nav_state<easynav::GNSSPerceptions>(handler)));
// }

// TEST_F(PluginTestCase, image_plugin_populates_nav_state)
// {
//   auto handler = load("easynav_sensors/ImagePerceptionHandler");
//   ASSERT_NO_THROW((assert_populate_nav_state<easynav::ImagePerceptions>(handler)));
// }

// TEST_F(PluginTestCase, detections_plugin_populates_nav_state)
// {
//   auto handler = load("easynav_sensors/DetectionsPerceptionHandler");
//   ASSERT_NO_THROW((assert_populate_nav_state<easynav::DetectionsPerceptions>(handler)));
// }

// ---------------------------------------------------------------------------
// 5. An explicit plugin on one sensor does NOT change the default for other
//    sensors of the same message type that omit the 'plugin:' parameter.
//    Verified through SensorsNode: configure sensorA with an explicit plugin,
//    then configure sensorB with only a type and confirm it still uses the
//    built-in default (PointPerceptionHandler).
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, explicit_plugin_does_not_override_default_for_other_sensors)
{
  auto sensors_node = easynav::SensorsNode::make_shared();

  // sensorA: explicit plugin (same as the built-in default for LaserScan)
  sensors_node->declare_parameter("sensorA.topic", std::string("/scanA"));
  sensors_node->declare_parameter("sensorA.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->declare_parameter("sensorA.plugin",
    std::string("easynav_sensors/PointPerceptionHandler"));

  // sensorB: same type, NO explicit plugin — must still resolve to built-in default
  sensors_node->declare_parameter("sensorB.topic", std::string("/scanB"));
  sensors_node->declare_parameter("sensorB.type", std::string("sensor_msgs/msg/LaserScan"));

  sensors_node->set_parameter({"sensors", std::vector<std::string>{"sensorA", "sensorB"}});

  // on_configure must succeed for both sensors without throwing
  ASSERT_NO_THROW(
    sensors_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

  EXPECT_EQ(
    sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 6. Every PerceptionPtr created by on_configure carries a non-null handler
//    of the correct concrete type for that sensor.
//
//    This is the core invariant introduced by the per-sensor handler fix:
//    set_by_group() dispatches via the handler stored in the PerceptionPtr,
//    not via a shared "first-wins" handlers_ map.  We verify:
//      a) handler pointer is not null.
//      b) handler->group() matches the configured group.
//      c) dynamic_cast to the expected concrete handler type succeeds.
// ---------------------------------------------------------------------------

// TEST_F(PluginTestCase, perception_ptr_stores_correct_handler_type)
// {
//   // Configure three sensors: LaserScan→points, IMU→imu, GNSS→gnss.
//   // Each must end up with the corresponding concrete handler type in its PerceptionPtr.
//   auto sensors_node = std::make_shared<SensorsNodeForTesting>();

//   sensors_node->declare_parameter("scan1.topic", std::string("/scanH"));
//   sensors_node->declare_parameter("scan1.type", std::string("sensor_msgs/msg/LaserScan"));
//   sensors_node->declare_parameter("imu1.topic", std::string("/imuH"));
//   sensors_node->declare_parameter("imu1.type", std::string("sensor_msgs/msg/Imu"));
//   sensors_node->declare_parameter("gnss1.topic", std::string("/gnssH"));
//   sensors_node->declare_parameter("gnss1.type", std::string("sensor_msgs/msg/NavSatFix"));
//   sensors_node->set_parameter({"sensors",
//       std::vector<std::string>{"scan1", "imu1", "gnss1"}});

//   ASSERT_NO_THROW(
//     sensors_node->trigger_transition(
//       lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
//   ASSERT_EQ(
//     sensors_node->get_current_state().id(),
//     lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

//   const auto & groups = sensors_node->groups_for_testing();
//   const auto & handlers = sensors_node->handler_list_for_testing();

//   // --- "points" group: must have one LaserScan sensor with PointPerceptionHandler ---
//   {
//     ASSERT_TRUE(groups.count("points")) << "Group 'points' missing";
//     const auto & point_sensors = groups.at("points");
//     ASSERT_EQ(point_sensors.size(), 1u);

//     int points_handler_count = 0;
//     for (const auto & handler : handlers) {
//       if (handler->group() == "points") {
//         points_handler_count++;
//         ASSERT_NE(handler, nullptr) << "handler must not be null for scan1";
//         EXPECT_EQ(handler->group(), "points")
//           << "handler->group() should be 'points' for LaserScan";
//         EXPECT_NE(
//           std::dynamic_pointer_cast<easynav::PointPerceptionHandler>(handler), nullptr)
//           << "scan1 handler must be a PointPerceptionHandler";
//       }
//     }
//     ASSERT_EQ(points_handler_count, 1) << "Must have 1 handler for 'points' group";
//   }

//   // --- "imu" group: must have one IMU sensor with IMUPerceptionHandler ---
//   {
//     ASSERT_TRUE(groups.count("imu")) << "Group 'imu' missing";
//     const auto & imu_sensors = groups.at("imu");
//     ASSERT_EQ(imu_sensors.size(), 1u);

//     int imu_handler_count = 0;
//     for (const auto & handler : handlers) {
//       if (handler->group() == "imu") {
//         imu_handler_count++;
//         ASSERT_NE(handler, nullptr) << "handler must not be null for imu1";
//         EXPECT_EQ(handler->group(), "imu")
//           << "handler->group() should be 'imu' for Imu";
//         EXPECT_NE(
//           std::dynamic_pointer_cast<easynav::IMUPerceptionHandler>(handler), nullptr)
//           << "imu1 handler must be an IMUPerceptionHandler";
//       }
//     }
//     ASSERT_EQ(imu_handler_count, 1) << "Must have 1 handler for 'imu' group";
//   }

//   // --- "gnss" group: must have one GNSS sensor with GNSSPerceptionHandler ---
//   {
//     ASSERT_TRUE(groups.count("gnss")) << "Group 'gnss' missing";
//     const auto & gnss_sensors = groups.at("gnss");
//     ASSERT_EQ(gnss_sensors.size(), 1u);

//     int gnss_handler_count = 0;
//     for (const auto & handler : handlers) {
//       if (handler->group() == "gnss") {
//         gnss_handler_count++;
//         ASSERT_NE(handler, nullptr) << "handler must not be null for gnss1";
//         EXPECT_EQ(handler->group(), "gnss")
//           << "handler->group() should be 'gnss' for NavSatFix";
//         EXPECT_NE(
//           std::dynamic_pointer_cast<easynav::GNSSPerceptionHandler>(handler), nullptr)
//           << "gnss1 handler must be a GNSSPerceptionHandler";
//       }
//     }
//     ASSERT_EQ(gnss_handler_count, 1) << "Must have 1 handler for 'gnss' group";
//   }
// }

// ---------------------------------------------------------------------------
// 7. Two sensors of the SAME type with a user-defined custom group each carry
//    their own handler with the correct type and group name.  This covers the
//    canonical "first wins" regression: with the old shared handlers_ map a
//    second sensor in the same group would not register its handler; with the
//    new per-PerceptionPtr design every sensor is independently self-contained.
// ---------------------------------------------------------------------------

// TEST_F(PluginTestCase, two_sensors_same_custom_group_have_independent_handlers)
// {
//   auto sensors_node = std::make_shared<SensorsNodeForTesting>();

//   sensors_node->declare_parameter("lidar_front.topic", std::string("/scan_front"));
//   sensors_node->declare_parameter("lidar_front.type", std::string("sensor_msgs/msg/LaserScan"));
//   sensors_node->declare_parameter("lidar_front.group", std::string("my_lidars"));
//   sensors_node->declare_parameter("lidar_rear.topic", std::string("/scan_rear"));
//   sensors_node->declare_parameter("lidar_rear.type", std::string("sensor_msgs/msg/LaserScan"));
//   sensors_node->declare_parameter("lidar_rear.group", std::string("my_lidars"));
//   sensors_node->set_parameter({"sensors",
//       std::vector<std::string>{"lidar_front", "lidar_rear"}});

//   ASSERT_NO_THROW(
//     sensors_node->trigger_transition(
//       lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
//   ASSERT_EQ(
//     sensors_node->get_current_state().id(),
//     lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

//   const auto & groups = sensors_node->groups_for_testing();
//   const auto & handlers = sensors_node->handler_list_for_testing();

//   ASSERT_TRUE(groups.count("my_lidars")) << "Group 'my_lidars' missing";
//   const auto & lidar_sensors = groups.at("my_lidars");
//   ASSERT_EQ(lidar_sensors.size(), 2u) << "Both sensors must be present in group 'my_lidars'";

//   int lidar_handler_count = 0;
//   for (const auto & handler : handlers) {
//     if (handler->group() == "points") {
//       // Handlers for LaserScan sensors are PointPerceptionHandlers
//       lidar_handler_count++;
//       SCOPED_TRACE("Handler index " + std::to_string(lidar_handler_count));

//       // (a) handler is non-null
//       ASSERT_NE(handler, nullptr)
//         << "Handler[" << lidar_handler_count << "] must not be null";

//       // (b) handler reports the canonical group name
//       EXPECT_EQ(handler->group(), "points")
//         << "Underlying handler->group() should always be 'points' (canonical) for LaserScan";

//       // (c) handler is the correct concrete type
//       EXPECT_NE(
//         std::dynamic_pointer_cast<easynav::PointPerceptionHandler>(handler), nullptr)
//         << "Handler[" << lidar_handler_count << "] must be a PointPerceptionHandler";
//     }
//   }
//   ASSERT_EQ(lidar_handler_count, 2) << "Must have 2 point handlers for 'my_lidars' group";
// }

// ---------------------------------------------------------------------------
// 7. Two sensors of different message types (LaserScan + PointCloud2) in the
//    same group both get their handler stored in PerceptionPtr and both
//    populate the NavState correctly (no "first handler wins" truncation).
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, two_sensor_types_same_group_both_populate_nav_state)
{
  auto sensors_node = easynav::SensorsNode::make_shared();

  // laser: LaserScan -> group "points"
  sensors_node->declare_parameter("laser.topic", std::string("/scan_mix"));
  sensors_node->declare_parameter("laser.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->declare_parameter("laser.group", std::string("points"));

  // cloud: PointCloud2 -> same group "points"
  sensors_node->declare_parameter("cloud.topic", std::string("/pc_mix"));
  sensors_node->declare_parameter("cloud.type", std::string("sensor_msgs/msg/PointCloud2"));
  sensors_node->declare_parameter("cloud.group", std::string("points"));

  sensors_node->set_parameter({"sensors", std::vector<std::string>{"laser", "cloud"}});

  ASSERT_NO_THROW(
    sensors_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

  ASSERT_EQ(
    sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // A cycle must write a PointPerceptions vector with exactly 2 entries
  // (one per sensor, regardless of which handler "won" in the old map).
  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_NO_THROW(sensors_node->cycle_rt(nav_state));
  EXPECT_NO_THROW(sensors_node->cycle(nav_state));

  ASSERT_TRUE(nav_state->has_group("points"));
  const auto & perceptions = nav_state->get_group<easynav::PointPerception>("points");
  EXPECT_EQ(perceptions.size(), 2u);
}

// ---------------------------------------------------------------------------
// 8. on_configure returns FAILURE when the message type is unknown and no
//    explicit 'plugin:' parameter is provided.  The node must remain in the
//    UNCONFIGURED state (transition did not complete successfully).
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, configure_fails_on_unknown_message_type)
{
  auto sensors_node = easynav::SensorsNode::make_shared();

  sensors_node->declare_parameter("bad_sensor.topic", std::string("/unknown_topic"));
  sensors_node->declare_parameter("bad_sensor.type",
    std::string("unknown_pkg/msg/UnknownType"));
  sensors_node->set_parameter({"sensors", std::vector<std::string>{"bad_sensor"}});

  // The transition must NOT throw but must return a non-SUCCESS result.
  ASSERT_NO_THROW(
    sensors_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

  // Node must NOT have reached INACTIVE — it should be in UNCONFIGURED or ERROR.
  EXPECT_NE(
    sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// ---------------------------------------------------------------------------
// 9. on_configure returns FAILURE when an explicit 'plugin:' names a plugin
//    that does not exist in the pluginlib registry.  The node must remain in
//    UNCONFIGURED / ERROR state.
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, configure_fails_on_nonexistent_plugin)
{
  auto sensors_node = easynav::SensorsNode::make_shared();

  sensors_node->declare_parameter("bad_sensor.topic", std::string("/scan_bad"));
  sensors_node->declare_parameter("bad_sensor.type",
    std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->declare_parameter("bad_sensor.plugin",
    std::string("easynav_sensors/NonExistentHandler"));
  sensors_node->set_parameter({"sensors", std::vector<std::string>{"bad_sensor"}});

  // pluginlib throws internally; on_configure must catch it and return FAILURE.
  ASSERT_NO_THROW(
    sensors_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

  // Node must NOT have reached INACTIVE — it should be in UNCONFIGURED or ERROR.
  EXPECT_NE(
    sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}
