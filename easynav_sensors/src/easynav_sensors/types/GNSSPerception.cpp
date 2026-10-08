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


#include <string>

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rclcpp/time.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"

namespace easynav
{

void GNSSPerceptionHandler::on_initialize()
{
  // Create the perception data instance
  perception_data_ = std::make_shared<GNSSPerception>();

  // Get sensor parameters
  auto node = get_node();
  std::string topic, msg_type;

  easynav::declare_parameter_if_absent(*node, get_sensor_name() + ".topic", std::string{});
  easynav::declare_parameter_if_absent(*node, get_sensor_name() + ".type", std::string{});

  node->get_parameter(get_sensor_name() + ".topic", topic);
  node->get_parameter(get_sensor_name() + ".type", msg_type);

  // Setup subscription
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = get_realtime_cbg();

  const auto clock_type = node->get_clock()->get_clock_type();

  if (msg_type != "sensor_msgs/msg/NavSatFix") {
    throw std::runtime_error("Unsupported message type for GNSSPerceptionHandler: " + msg_type);
  }

  perception_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    topic, rclcpp::QoS(1),
    [this, clock_type](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
      perception_data_->set_data(
        *msg, rclcpp::Time(msg->header.stamp, clock_type), msg->header.frame_id);
    },
    options);
}

bool GNSSPerceptionHandler::cycle_rt(std::shared_ptr<NavState> nav_state)
{
  // Store the perception in the NavState
  nav_state->set(get_sensor_name(), perception_data_);
  // Check if there was new data to trigger process and reset new_data state
  return perception_data_->consume_new_data();
}

rclcpp::Time get_latest_gnss_perceptions_stamp(const GNSSPerceptions & perceptions)
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
PLUGINLIB_EXPORT_CLASS(easynav::GNSSPerceptionHandler, easynav::PerceptionHandler)
