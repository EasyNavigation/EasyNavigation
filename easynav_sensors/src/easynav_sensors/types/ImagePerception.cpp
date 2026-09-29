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

#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_sensors/types/ImagePerception.hpp"

namespace easynav
{


void ImagePerceptionHandler::on_initialize()
{
  // Create the perception data instance
  perception_data_ = std::make_shared<ImagePerception>();

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

  if (msg_type != "sensor_msgs/msg/Image") {
    throw std::runtime_error("Unsupported message type for ImagePerceptionHandler: " + msg_type);
  }

  perception_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::QoS(1),
    [this, clock_type](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      const auto msg_stamp = rclcpp::Time(msg->header.stamp, clock_type);
      const auto & msg_frame_id = msg->header.frame_id;

      try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        // clone to avoid sharing buffers
        perception_data_->set_data(cv_ptr->image.clone(), msg_stamp, msg_frame_id);
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger("ImagePerceptionHandler"),
          "cv_bridge exception: %s", e.what());
        perception_data_->mark_invalid(msg_stamp, msg_frame_id);
      }
    },
    options);
}

bool ImagePerceptionHandler::cycle_rt(std::shared_ptr<NavState> nav_state)
{
  // Store the perception in the NavState
  nav_state->set(get_sensor_name(), perception_data_);
  // Check if there was new data to trigger process and reset new_data state
  return perception_data_->consume_new_data();
}

rclcpp::Time get_latest_image_perceptions_stamp(const ImagePerceptions & perceptions)
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
PLUGINLIB_EXPORT_CLASS(easynav::ImagePerceptionHandler, easynav::PerceptionHandler)
