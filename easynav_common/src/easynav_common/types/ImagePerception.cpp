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

#include <string>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/ImagePerception.hpp"

namespace easynav
{


rclcpp::SubscriptionBase::SharedPtr
ImagePerceptionHandler::create_subscription(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  const std::string & type,
  std::shared_ptr<PerceptionBase> target,
  rclcpp::CallbackGroup::SharedPtr cb_group)
{
  if (type != "sensor_msgs/msg/Image") {
    throw std::runtime_error("Unsupported message type for ImagePerceptionHandler: " + type);
  }

  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = cb_group;

  return node.create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::QoS(1),
    [target](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto typed_target = std::dynamic_pointer_cast<ImagePerception>(target);

      typed_target->stamp = msg->header.stamp;
      typed_target->frame_id = msg->header.frame_id;
      typed_target->new_data = true;

      try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        typed_target->data = cv_ptr->image.clone();  // se clona para evitar compartir buffers
        typed_target->valid = true;
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger("ImagePerceptionHandler"),
          "cv_bridge exception: %s", e.what());
        typed_target->valid = false;
      }
    },
    options);
}

}  // namespace easynav
