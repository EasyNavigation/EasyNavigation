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
/// \brief Defines data structures and utilities for representing and processing image perceptions.
///
/// This file contains the definition of the DetectionsPerceptions class
/// and the DetectionsPerceptionsHandler class, which handles subscriptions to image messages and transforms them into
/// DetectionsPerceptions instances. It also defines an alias for a collection of such perceptions.

#ifndef EASYNAV_SENSORS_TYPES__DETECTIONSPERCEPTIONS_HPP_
#define EASYNAV_SENSORS_TYPES__DETECTIONSPERCEPTIONS_HPP_

#include <string>
#include <vector>

#include "vision_msgs/msg/detection3_d_array.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_sensors/types/Perceptions.hpp"

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

  DetectionsPerception()
  {
    [[maybe_unused]] static const bool _ = [] {
        ::easynav::NavState::register_printer<DetectionsPerception>(
          [](const DetectionsPerception & perception) {
            std::ostringstream ret;
            ret << "{ " << perception.stamp.seconds()
                << " } DetectionsPerception with "
                << perception.data.detections.size()
                << " detections in frame [" << perception.frame_id
                << "] with ts " << perception.stamp.seconds() << "\n";
            return ret.str();
        });
        return true;
      }();
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
  /// \brief Optional post-initialization hook for subclasses.
  /// Here, the handler must reserve memory to store the perception data
  /// and create any Subscription or similar objects to read the data.
  void on_initialize() override;

  /// @brief Run one real-time sensor processing cycle.
  /// This method is called by the SensorsNode before executing its cycle_rt.
  /// Here the handler should update the NavState with the sensor data.
  /// If new data arrived before this call and the state is updated, it must return true.
  ///
  /// @param nav_state Pointer to the NavState to store the sensor data.
  /// @return True if new data was stored (to trigger processing).
  bool cycle_rt([[maybe_unused]] std::shared_ptr<NavState> nav_state) override;

private:
  /// \brief pointer to the perception data
  std::shared_ptr<DetectionsPerception> perception_data_ {nullptr};

  /// \brief pointer to the subscription object
  rclcpp::SubscriptionBase::SharedPtr perception_sub_;
};

/**
 * @typedef DetectionsPerceptionss
 * @brief Alias for a vector of shared pointers to DetectionsPerceptions objects.
 *
 * The container can represent a time-ordered or batched collection, depending on producer logic.
 */
using DetectionsPerceptions =
  std::vector<std::shared_ptr<DetectionsPerception>>;

/// \brief Retrieves the latest timestamp among a set of detection-based perceptions.
/// \param perceptions Container of detection-based perceptions.
/// \return The most recent timestamp found in \p perceptions, or a default-constructed \c rclcpp::Time if \p perceptions is empty.
rclcpp::Time get_latest_detections_perceptions_stamp(const DetectionsPerceptions & perceptions);

}  // namespace easynav

#endif  // EASYNAV_SENSORS_TYPES__DETECTIONSPERCEPTIONS_HPP_
