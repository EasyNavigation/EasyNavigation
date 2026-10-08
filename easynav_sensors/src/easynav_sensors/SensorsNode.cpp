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
/// \brief Implementation of the SensorsNode class.

#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "easynav_common/Parameters.hpp"

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

  easynav::declare_parameter_if_absent(*this, "sensors", std::vector<std::string>());

  easynav::declare_parameter_if_absent(*this, "forget_time", 1.0);

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
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }
}

void
SensorsNode::release_handlers()
{
  {
    std::lock_guard<std::mutex> lock(handler_list_mutex_);
    handler_list_.clear();
  }
  groups_.clear();
  groups_initialized = false;
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
CallbackReturnT
SensorsNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  get_parameter("forget_time", forget_time_);
  if (!std::isfinite(forget_time_) || forget_time_ <= 0.0) {
    RCLCPP_ERROR(
      get_logger(), "Invalid parameter: forget_time = %f (> 0)", forget_time_);
    return CallbackReturnT::FAILURE;
  }

  for (const auto & sensor_id : sensors) {
    std::string topic, msg_type, plugin;

    easynav::declare_parameter_if_absent(*this, sensor_id + ".topic", std::string{});
    easynav::declare_parameter_if_absent(*this, sensor_id + ".type", std::string{});
    easynav::declare_parameter_if_absent(*this, sensor_id + ".plugin", std::string{});

    get_parameter(sensor_id + ".topic", topic);
    get_parameter(sensor_id + ".type", msg_type);
    get_parameter(sensor_id + ".plugin", plugin);

    // Auto-detect plugin from the built-in type→plugin table when not explicitly given.
    // An explicit 'plugin:' on one sensor never changes the default for other sensors.
    if (plugin.empty()) {
      auto it = type_to_plugin_.find(msg_type);
      if (it != type_to_plugin_.end()) {
        plugin = it->second;
        RCLCPP_INFO(
          get_logger(),
          "Auto-detected plugin [%s] for sensor [%s] from type [%s]",
          plugin.c_str(), sensor_id.c_str(), msg_type.c_str());
      }
    }

    if (plugin.empty()) {
      RCLCPP_ERROR(
        get_logger(),
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
      RCLCPP_ERROR(
        get_logger(),
        "Failed to load perception handler plugin [%s] for sensor [%s]: %s",
        plugin.c_str(), sensor_id.c_str(), ex.what());
      return CallbackReturnT::FAILURE;
    }

    handler->initialize(shared_from_this(), realtime_cbg_, sensor_id);

    std::string group = "";
    easynav::declare_parameter_if_absent(*this, sensor_id + ".group", "");
    get_parameter(sensor_id + ".group", group);

    // Store the handler and add sensor to the group
    handler_list_.push_back(handler);
    // Store group only if specified (if param exists)
    if (group != "") {
      // TODO: This assumes that the handler uses the sensor name to write in the
      // NavState and it assumes it sets only one value
      groups_[group].emplace_back(handler->get_sensor_name());
    }

    RCLCPP_INFO(
      get_logger(),
      "Configured sensor [%s] with plugin [%s] on topic [%s] in group [%s]",
      sensor_id.c_str(), plugin.c_str(), topic.c_str(), group.c_str());
  }

  // Preallocated: checked every RT cycle.
  data_states_.assign(handler_list_.size(), DataState::NO_DATA);
  data_age_reported_ = false;

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

  release_handlers();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;

  // A shutdown from ACTIVE skips on_deactivate.
  percept_pub_->on_deactivate();
  release_handlers();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;

  percept_pub_->on_deactivate();
  release_handlers();
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
  // Copy the list so handlers stay alive for this call even if on_cleanup()
  // clears handler_list_ right after we release the lock.
  std::vector<std::shared_ptr<PerceptionHandler>> handlers;
  {
    std::lock_guard<std::mutex> lock(handler_list_mutex_);
    handlers = handler_list_;
  }

  bool trigger_perceptions = false;
  // Run handlers' cycle and check if there is new sensor data o trigger perceptions
  for (auto & handler : handlers) {
    const bool trigger = handler->cycle_rt(nav_state);
    trigger_perceptions = trigger_perceptions || trigger;
  }

  check_data_age(handlers, *nav_state);

  return trigger_perceptions;
}

void
SensorsNode::check_data_age(
  const std::vector<std::shared_ptr<PerceptionHandler>> & handlers, NavState & nav_state)
{
  if (data_states_.size() != handlers.size()) {
    return;  // Reconfiguring.
  }
  const auto now = this->now();
  bool changed = !data_age_reported_;
  for (std::size_t i = 0; i < handlers.size(); ++i) {
    const auto perception = handlers[i]->get_perception();
    if (!perception) {
      continue;  // A handler that does not expose its perception is not checked.
    }
    DataState state = DataState::FRESH;
    if (perception->stamp.nanoseconds() == 0 && !perception->valid) {
      state = DataState::NO_DATA;
    } else if (perception->stamp.get_clock_type() == now.get_clock_type() &&
      (now - perception->stamp).seconds() > forget_time_)
    {
      perception->valid = false;  // Nothing uses it until new data arrives.
      state = DataState::STALE;
    } else if (!perception->valid) {
      state = DataState::STALE;
    }
    if (state != data_states_[i]) {
      data_states_[i] = state;
      changed = true;
    }
  }
  if (changed) {
    report_data_age(handlers, nav_state);
  }
}

void
SensorsNode::report_data_age(
  const std::vector<std::shared_ptr<PerceptionHandler>> & handlers, NavState & nav_state)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  data_age_reported_ = true;

  std::string no_data, stale;
  for (std::size_t i = 0; i < handlers.size(); ++i) {
    if (data_states_[i] == DataState::FRESH) {continue;}
    auto & list = data_states_[i] == DataState::NO_DATA ? no_data : stale;
    list += (list.empty() ? "" : ", ") + handlers[i]->get_sensor_name();
  }

  DiagnosticStatus status;
  status.name = "sensors";
  status.hardware_id = get_name();
  if (no_data.empty() && stale.empty()) {
    status.level = DiagnosticStatus::OK;
    status.message = "Sensor data up to date";
  } else {
    status.level = DiagnosticStatus::WARN;
    if (!stale.empty()) {
      status.message = "No data for more than " + std::to_string(forget_time_) + " s from: " +
        stale;
    }
    if (!no_data.empty()) {
      status.message += (status.message.empty() ? "" : "; ") + std::string("No data yet from: ") +
        no_data;
    }
    RCLCPP_WARN(get_logger(), "%s", status.message.c_str());
  }
  diagnostic_msgs::msg::KeyValue stale_kv;
  stale_kv.key = "stale";
  stale_kv.value = stale;
  status.values.push_back(stale_kv);
  diagnostic_msgs::msg::KeyValue no_data_kv;
  no_data_kv.key = "no_data";
  no_data_kv.value = no_data;
  status.values.push_back(no_data_kv);

  nav_state.set("diagnostics.sensors", status);
  nav_state.add_to_group("diagnostics", "diagnostics.sensors");
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
