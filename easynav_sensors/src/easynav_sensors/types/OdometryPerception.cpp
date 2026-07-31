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


#include <string>

#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/time.hpp"

#include "easynav_sensors/types/OdometryPerception.hpp"

namespace easynav
{

void OdometryPerceptionHandler::on_initialize()
{
  // Create the perception data instance
  perception_data_ = std::make_shared<OdometryPerception>();

  // Get sensor parameters
  auto node = get_node();
  std::string topic, msg_type;

  if (!node->has_parameter(get_sensor_name() + ".topic")) {
    node->declare_parameter(get_sensor_name() + ".topic", std::string{});
  }
  if (!node->has_parameter(get_sensor_name() + ".type")) {
    node->declare_parameter(get_sensor_name() + ".type", std::string{});
  }
  if (!node->has_parameter(get_sensor_name() + ".nav_state_key")) {
    node->declare_parameter(get_sensor_name() + ".nav_state_key", get_sensor_name());
  }

  node->get_parameter(get_sensor_name() + ".topic", topic);
  node->get_parameter(get_sensor_name() + ".type", msg_type);
  node->get_parameter(get_sensor_name() + ".nav_state_key", odom_ns_key_);

  // Setup subscription
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = get_realtime_cbg();

  const auto clock_type = node->get_clock()->get_clock_type();

  if (msg_type != "nav_msgs/msg/Odometry") {
    throw std::runtime_error("Unsupported message type for OdometryPerceptionHandler: " + msg_type);
  }

  perception_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::QoS(1),
    [this, clock_type](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      perception_data_->stamp = rclcpp::Time(msg->header.stamp, clock_type);
      perception_data_->frame_id = msg->header.frame_id;
      perception_data_->new_data = true;
      perception_data_->data = *msg;
      perception_data_->valid = true;
    },
    options);
}

bool OdometryPerceptionHandler::cycle_rt(std::shared_ptr<NavState> nav_state)
{
  // Store the data in the NavState
  // NOTE: We store the actual Odometry message, not the Perception object.
  //       This is so this handler can be a replacement of the localizer plugin.
  nav_state->set(odom_ns_key_, perception_data_->data);
  // Check if there was new data to trigger process and reset new_data state
  const bool should_trigger = perception_data_->new_data;
  perception_data_->new_data = false;
  return should_trigger;
}

rclcpp::Time get_latest_odometry_perceptions_stamp(const OdometryPerceptions & perceptions)
{
  auto is_newer = [](const rclcpp::Time & a, const rclcpp::Time & b) {
      if (a.get_clock_type() == b.get_clock_type()) {
        return a > b;
      }
      return a.nanoseconds() > b.nanoseconds();
    };

  rclcpp::Time latest_stamp;
  bool inited = false;

  for (const auto & perception : perceptions) {
    if (!inited || is_newer(perception->stamp, latest_stamp)) {
      latest_stamp = perception->stamp;
      inited = true;
    }
  }

  return latest_stamp;
}

}  // namespace easynav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(easynav::OdometryPerceptionHandler, easynav::PerceptionHandler)
