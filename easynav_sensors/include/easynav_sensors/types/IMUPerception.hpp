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
/// \brief Defines data structures and utilities for representing and processing IMU perceptions.
///
/// This file contains the definition of the IMUPerception class, which holds IMU sensor data,
/// and the IMUPerceptionHandler class, which handles subscriptions to IMU messages and transforms
/// them into IMUPerception instances. It also defines an alias for a collection of such
/// perceptions.

#ifndef EASYNAV_SENSORS__TYPES__IMUPERCEPTION_HPP_
#define EASYNAV_SENSORS__TYPES__IMUPERCEPTION_HPP_

#include <mutex>
#include <string>
#include <vector>
#include <memory>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_sensors/types/Perceptions.hpp"

namespace easynav
{

/// \class IMUPerception
/// \brief Represents a single IMU perception from a sensor.
///
/// Inherits from PerceptionBase and stores a sensor_msgs::msg::Imu message.
class IMUPerception : public PerceptionBase
{
public:
  /// \brief Group identifier for IMU perceptions.
  static constexpr std::string_view default_group_ = "imu";

  /// \brief Returns whether the given ROS 2 type name is supported by this perception.
  /// \param t Fully qualified message type name (e.g., "sensor_msgs/msg/Imu").
  /// \return true if \p t equals "sensor_msgs/msg/Imu", otherwise false.
  static inline bool supports_msg_type(std::string_view t)
  {
    return t == "sensor_msgs/msg/Imu";
  }

  IMUPerception()
  {
    [[maybe_unused]] static const bool _ = [] {
        ::easynav::NavState::register_printer<IMUPerception>(
          [](const IMUPerception & perception) {
            std::ostringstream ret;
            ret << "{ " << perception.stamp.seconds()
                << " } IMUPerception linear acc = ("
                << perception.data.linear_acceleration.x << ", "
                << perception.data.linear_acceleration.y << ", "
                << perception.data.linear_acceleration.z
                << ") in frame [" << perception.frame_id
                << "] with ts " << perception.stamp.seconds() << "\n";
            return ret.str();
          });
        return true;
      }();
  }

  IMUPerception(const IMUPerception & other)
  {
    std::lock_guard<std::mutex> lock(other.mutex_);
    stamp = other.stamp;
    frame_id = other.frame_id;
    valid = other.valid;
    new_data = other.new_data;
    data = other.data;
  }

  IMUPerception & operator=(const IMUPerception & other)
  {
    if (this == &other) {
      return *this;
    }

    std::scoped_lock lock(mutex_, other.mutex_);
    stamp = other.stamp;
    frame_id = other.frame_id;
    valid = other.valid;
    new_data = other.new_data;
    data = other.data;

    return *this;
  }

  /// \brief IMU data received from the sensor.
  sensor_msgs::msg::Imu data;

  /// \brief Atomically overwrites stamp/frame_id/data/valid from the subscription callback.
  ///
  /// Guards against a concurrent copy (e.g. via \c NavState::get_safe()) observing a
  /// partially-updated object while this handler's RT-thread callback is writing.
  void set_data(
    const sensor_msgs::msg::Imu & msg, const rclcpp::Time & msg_stamp,
    const std::string & msg_frame_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stamp = msg_stamp;
    frame_id = msg_frame_id;
    new_data = true;
    data = msg;
    valid = true;
  }

  /// \brief Atomically reads and clears \ref new_data.
  /// \return The value of \ref new_data before it was cleared.
  bool consume_new_data()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool had_new_data = new_data;
    new_data = false;
    return had_new_data;
  }

protected:
  mutable std::mutex mutex_;
};

/// \class IMUPerceptionHandler
/// \brief Handles the creation and updating of IMUPerception instances from sensor_msgs::msg::Imu
/// messages.
///
/// This class provides methods to register subscriptions to IMU topics and update IMUPerception
/// objects.
class IMUPerceptionHandler : public PerceptionHandler
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

  /// \brief The perception this handler keeps up to date.
  std::shared_ptr<PerceptionBase> get_perception() const override {return perception_data_;}

private:
  /// \brief pointer to the perception data
  std::shared_ptr<IMUPerception> perception_data_ {nullptr};

  /// \brief pointer to the subscription object
  rclcpp::SubscriptionBase::SharedPtr perception_sub_;
};

/**
 * @typedef IMUPerceptions
 * @brief Alias for a vector of shared pointers to IMUPerception objects.
 *
 * The container can represent a time-ordered or batched collection, depending on producer logic.
 */
using IMUPerceptions =
  std::vector<std::shared_ptr<IMUPerception>>;

/// \brief Retrieves the latest timestamp among a set of IMU perceptions.
/// \param perceptions Container of IMU perceptions.
/// \return The most recent timestamp found in \p perceptions, or a default-constructed
/// \c rclcpp::Time if \p perceptions is empty.
rclcpp::Time get_latest_imu_perceptions_stamp(const IMUPerceptions & perceptions);

}  // namespace easynav

#endif  // EASYNAV_SENSORS__TYPES__IMUPERCEPTION_HPP_
