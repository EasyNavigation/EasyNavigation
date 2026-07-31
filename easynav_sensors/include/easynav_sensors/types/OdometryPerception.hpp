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

/// \file
/// \brief Defines data structures and utilities for representing and processing odometry perceptions.
///
/// This file contains the definition of the OdometryPerception class, which holds odometry sensor data,
/// and the OdometryPerceptionHandler class, which handles subscriptions to odometry messages and
/// transforms them into OdometryPerception instances. It also defines an alias for a collection of such
/// perceptions.

#ifndef EASYNAV_SENSORS_TYPES__ODOMETRYPERCEPTIONS_HPP_
#define EASYNAV_SENSORS_TYPES__ODOMETRYPERCEPTIONS_HPP_

#include <string_view>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_sensors/types/Perceptions.hpp"

namespace easynav
{

/// \class OdometryPerception
/// \brief Represents a single odometry perception from a sensor.
///
/// Inherits from PerceptionBase and stores a nav_msgs::msg::Odometry message.
class OdometryPerception : public PerceptionBase
{
public:
  /// \brief Group identifier for odometry perceptions.
  static constexpr std::string_view default_group_ = "odom";

  /// \brief Returns whether the given ROS 2 type name is supported by this perception.
  /// \param t Fully qualified message type name (e.g., "nav_msgs/msg/Odometry").
  /// \return true if \p t equals "nav_msgs/msg/Odometry", otherwise false.
  static inline bool supports_msg_type(std::string_view t)
  {
    return t == "nav_msgs/msg/Odometry";
  }

  OdometryPerception()
  {
    [[maybe_unused]] static const bool _ = [] {
        ::easynav::NavState::register_printer<OdometryPerception>(
          [](const OdometryPerception & perception) {
            std::ostringstream ret;
            const auto & pose = perception.data.pose.pose;
            const auto & twist = perception.data.twist.twist;
            ret << "{ " << perception.stamp.seconds()
                << " } OdometryPerception pose = ("
                << pose.position.x << ", "
                << pose.position.y << ", "
                << pose.position.z << "), twist = ("
                << twist.linear.x << ", "
                << twist.linear.y << ", "
                << twist.linear.z
                << ") in frame [" << perception.frame_id
                << "] with ts " << perception.stamp.seconds() << "\n";
            return ret.str();
        });
        return true;
      }();
  }

  /// \brief Odometry data received from the sensor.
  nav_msgs::msg::Odometry data;
};

/// \class OdometryPerceptionHandler
/// \brief Handles the creation and updating of OdometryPerception instances from nav_msgs::msg::Odometry messages.
///
/// This class provides methods to register subscriptions to odometry topics and update OdometryPerception objects.
class OdometryPerceptionHandler : public PerceptionHandler
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
  std::shared_ptr<OdometryPerception> perception_data_ {nullptr};

  /// \brief pointer to the subscription object
  rclcpp::SubscriptionBase::SharedPtr perception_sub_;

  /// \brief NavState key to store the odometry value (defaults to sensor name)
  std::string odom_ns_key_;
};

/**
 * @typedef OdometryPerceptions
 * @brief Alias for a vector of shared pointers to OdometryPerception objects.
 *
 * The container can represent a time-ordered or batched collection, depending on producer logic.
 */
using OdometryPerceptions =
  std::vector<std::shared_ptr<OdometryPerception>>;

/// @brief Retrieves the latest timestamp among a set of odometry perceptions.
/// @param perceptions Container of odometry perceptions.
/// @return The most recent timestamp found in \p perceptions, or a default-constructed \c rclcpp::Time if \p perceptions is empty.
rclcpp::Time get_latest_odometry_perceptions_stamp(const OdometryPerceptions & perceptions);

}  // namespace easynav

#endif  // EASYNAV_SENSORS_TYPES__ODOMETRYPERCEPTIONS_HPP_
