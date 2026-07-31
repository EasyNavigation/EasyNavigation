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

/// \file
/// \brief Implementation of the SensorsNode class.

#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_sensors/SensorsNode.hpp"

#include "easynav_sensors/types/ImagePerception.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "easynav_sensors/types/DetectionsPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{


SensorsNode::SensorsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("sensors_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  percept_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "sensors_node/perceptions", rclcpp::SensorDataQoS().reliable());

  if (!has_parameter("sensors")) {
    declare_parameter("sensors", std::vector<std::string>());
  }

  if (!has_parameter("forget_time")) {
    declare_parameter("forget_time", 1.0);
  }

  handler_loader_ = std::make_unique<pluginlib::ClassLoader<PerceptionHandler>>(
    "easynav_sensors", "easynav::PerceptionHandler");

  type_to_plugin_ = {
    {"sensor_msgs/msg/PointCloud2", "easynav_sensors/PointPerceptionHandler"},
    {"sensor_msgs/msg/LaserScan", "easynav_sensors/PointPerceptionHandler"},
    {"sensor_msgs/msg/Imu", "easynav_sensors/IMUPerceptionHandler"},
    {"sensor_msgs/msg/NavSatFix", "easynav_sensors/GNSSPerceptionHandler"},
    {"nav_msgs/msg/Odometry", "easynav_sensors/OdometryPerceptionHandler"},
    {"sensor_msgs/msg/Image", "easynav_sensors/ImagePerceptionHandler"},
    {"vision_msgs/msg/Detection3DArray", "easynav_sensors/DetectionsPerceptionHandler"},
  };
}

SensorsNode::~SensorsNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
CallbackReturnT
SensorsNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  get_parameter("forget_time", forget_time_);

  for (const auto & sensor_id : sensors) {
    std::string topic, msg_type, plugin;

    if (!has_parameter(sensor_id + ".topic")) {
      declare_parameter(sensor_id + ".topic", std::string{});
    }
    if (!has_parameter(sensor_id + ".type")) {
      declare_parameter(sensor_id + ".type", std::string{});
    }
    if (!has_parameter(sensor_id + ".plugin")) {
      declare_parameter(sensor_id + ".plugin", std::string{});
    }

    get_parameter(sensor_id + ".topic", topic);
    get_parameter(sensor_id + ".type", msg_type);
    get_parameter(sensor_id + ".plugin", plugin);

    // Auto-detect plugin from the built-in type→plugin table when not explicitly given.
    // An explicit 'plugin:' on one sensor never changes the default for other sensors.
    if (plugin.empty()) {
      auto it = type_to_plugin_.find(msg_type);
      if (it != type_to_plugin_.end()) {
        plugin = it->second;
        RCLCPP_INFO(get_logger(),
          "Auto-detected plugin [%s] for sensor [%s] from type [%s]",
          plugin.c_str(), sensor_id.c_str(), msg_type.c_str());
      }
    }

    if (plugin.empty()) {
      RCLCPP_ERROR(get_logger(),
        "Cannot configure sensor [%s]: no 'plugin' parameter and type [%s] is not recognized. "
        "Add 'plugin: <plugin_name>' to the sensor parameters.",
        sensor_id.c_str(), msg_type.c_str());
      return CallbackReturnT::FAILURE;
    }

    // Load the handler plugin for this sensor
    std::shared_ptr<PerceptionHandler> handler;
    try {
      handler = handler_loader_->createSharedInstance(plugin);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Failed to load perception handler plugin [%s] for sensor [%s]: %s",
        plugin.c_str(), sensor_id.c_str(), ex.what());
      return CallbackReturnT::FAILURE;
    }

    handler->initialize(shared_from_this(), realtime_cbg_, sensor_id);

    std::string group = "";
    if (!has_parameter(sensor_id + ".group")) {
      declare_parameter(sensor_id + ".group", "");
    }
    get_parameter(sensor_id + ".group", group);

    // Store the handler and add sensor to the group
    handler_list_.push_back(handler);
    // Store group only if specified (if param exists)
    if (group != "") {
      // TODO: This assumes that the handler uses the sensor name to write in the
      // NavState and it assumes it sets only one value
      groups_[group].emplace_back(handler->get_sensor_name());
    }

    RCLCPP_INFO(get_logger(),
      "Configured sensor [%s] with plugin [%s] on topic [%s] in group [%s]",
      sensor_id.c_str(), plugin.c_str(), topic.c_str(), group.c_str());
  }

  return CallbackReturnT::SUCCESS;
}


CallbackReturnT
SensorsNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  percept_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  percept_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
SensorsNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

bool
SensorsNode::cycle_rt(
  std::shared_ptr<NavState> nav_state,
  [[maybe_unused]] bool trigger)
{
  bool trigger_perceptions = false;
  // Run handlers' cycle and check if there is new sensor data o trigger perceptions
  for (auto & handler : handler_list_) {
    const bool trigger = handler->cycle_rt(nav_state);
    trigger_perceptions = trigger_perceptions || trigger;
  }

  return trigger_perceptions;
}

void
SensorsNode::cycle([[maybe_unused]] std::shared_ptr<NavState> nav_state)
{
  // Initialize groups in the NavState
  if (!groups_initialized) {
    for (const auto & group : groups_) {
      RCLCPP_INFO(
        get_logger(),
        "Initializing sensor group [%s] in NavState",
        group.first.c_str()
      );
      nav_state->set_group(group.first, group.second);
    }
    groups_initialized = true;
  }

  const auto & points_perceptions = nav_state->get_by_type<PointPerception>();

  if (percept_pub_->get_subscription_count() > 0 && !points_perceptions.empty()) {
    PointPerceptionsOpsView fused_view(std::move(points_perceptions));

    const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
    const std::string & robot_footprint_frame = tf_info.robot_footprint_frame;

    fused_view.fuse(robot_footprint_frame);
    auto fused_points = fused_view.as_points();

    // Skip empty point clouds
    if (fused_points.empty()) {
      return;
    }

    auto msg = points_to_rosmsg(fused_points);
    msg.header.frame_id = robot_footprint_frame;
    const auto & percs = fused_view.get_perceptions();
    if (!percs.empty() && percs[0]) {
      msg.header.stamp = percs[0]->stamp;
    } else {
      msg.header.stamp = now();
    }

    percept_pub_->publish(msg);
  }
}

}  // namespace easynav
