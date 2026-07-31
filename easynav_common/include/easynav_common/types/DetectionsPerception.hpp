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
/// \brief Defines data structures and utilities for representing and processing image perceptions.
///
/// This file contains the definition of the DetectionsPerceptions class
/// and the DetectionsPerceptionsHandler class, which handles subscriptions to image messages and transforms them into
/// DetectionsPerceptions instances. It also defines an alias for a collection of such perceptions.

#ifndef EASYNAV_COMMON_TYPES__DETECTIONSPERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__DETECTIONSPERCEPTIONS_HPP_

#include <string>
#include <vector>

#include "vision_msgs/msg/detection3_d_array.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/// \class DetectionsPerceptions
/// \brief Represents a single image perception from a sensor.
///
/// Inherits from PerceptionBase and stores an OpenCV image (cv::Mat) along with metadata such as timestamp and frame_id.
class DetectionsPerception : public PerceptionBase
{
public:
  /// \brief Group identifier for image perceptions.
  static constexpr std::string_view default_group_ = "detections";

  /// \brief Returns whether the given ROS 2 type name is supported by this perception.
  /// \param t Fully qualified message type name (e.g., "vision_msgs/msg/Detection3DArray").
  /// \return true if \p t equals "vision_msgs/msg/Detection3DArray", otherwise false.
  static inline bool supports_msg_type(std::string_view t)
  {
    return t == "vision_msgs/msg/Detection3DArray";
  }

  /// \brief Detection3DArray data received from the an external processing system.
  vision_msgs::msg::Detection3DArray data;
};

/// \class DetectionsPerceptionsHandler
/// \brief Handles the creation and updating of DetectionsPerceptions instances from sensor_msgs::msg::Image messages.
///
/// This class provides methods to register subscriptions to image topics, decode incoming messages into cv::Mat
/// using cv_bridge, and update target DetectionsPerceptions instances.
class DetectionsPerceptionsHandler : public PerceptionHandler
{
public:
  /// \brief Returns the group managed by this handler.
  /// \return The string literal "image".
  std::string group() const override {return "detections";}

  /// \brief Creates a new empty DetectionsPerceptions instance.
  /// \param sensor_id Name or identifier of the sensor. Currently unused, reserved for future extensions.
  /// \return Shared pointer to a newly created DetectionsPerceptions.
  std::shared_ptr<PerceptionBase> create(const std::string &) override
  {
    return std::make_shared<DetectionsPerception>();
  }

  /// \brief Creates a subscription to an image topic that updates a target DetectionsPerceptions.
  ///
  /// The subscription receives sensor_msgs::msg::Image messages on \p topic, converts them to cv::Mat,
  /// fills DetectionsPerceptions::data, and updates inherited metadata (stamp, frame_id).
  ///
  /// \param node Lifecycle node used to create the subscription.
  /// \param topic Topic name to subscribe to.
  /// \param type ROS message type name. It must be "sensor_msgs/msg/Image".
  /// \param target Shared pointer to the DetectionsPerceptions to be updated.
  /// \param cb_group Callback group for executor-level concurrency control.
  /// \return Shared pointer to the created subscription.
  rclcpp::SubscriptionBase::SharedPtr create_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic,
    const std::string & type,
    std::shared_ptr<PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group) override;
};

/**
 * @typedef DetectionsPerceptionss
 * @brief Alias for a vector of shared pointers to DetectionsPerceptions objects.
 *
 * The container can represent a time-ordered or batched collection, depending on producer logic.
 */
using DetectionsPerceptions =
  std::vector<std::shared_ptr<DetectionsPerception>>;

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__DETECTIONSPERCEPTIONS_HPP_
