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
/// \brief Defines data structures and utilities for representing and processing sensor perceptions.
///
/// This file provides common interfaces for sensor perception handling:
/// - `PerceptionBase`: base class for sensor data.
/// - `PerceptionPtr`: utility for holding perception state and its subscription.
/// - `get_perceptions`: helper to extract typed collections from a heterogeneous container.
/// - `PerceptionHandler`: abstract base class for group-specific sensor handlers.

#ifndef EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_

#include <string>

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/// \class PerceptionBase
/// \brief Abstract base class for representing a single sensor perception.
///
/// Contains common metadata (timestamp, frame ID, validity flags) that all perception types share.
class PerceptionBase
{
public:
  virtual ~PerceptionBase() = default;

  /// \brief Timestamp of the perception (ROS time).
  rclcpp::Time stamp;

  /// \brief Coordinate frame associated with the perception.
  std::string frame_id;

  /// \brief Whether the perception contains valid data.
  bool valid = false;

  /// \brief Whether the data has changed since the last observation.
  bool new_data = false;
};

/// \typedef PerceptionBasePtr
/// \brief Shared pointer alias to \ref PerceptionBase.
using PerceptionBasePtr = std::shared_ptr<PerceptionBase>;

/// \struct PerceptionPtr
/// \brief Represents a perception entry with its state and ROS subscription.
///
/// Holds a pointer to a perception object (\ref PerceptionBase) and the associated subscription.
/// Used internally by perception managers to update and access sensor data.
struct PerceptionPtr
{
  /// \brief Shared pointer to the current perception object.
  PerceptionBasePtr perception;

  /// \brief ROS 2 subscription to the sensor topic that provides data.
  rclcpp::SubscriptionBase::SharedPtr subscription;
};

/// \brief Extracts a homogeneous collection of perceptions of type \p T from a heterogeneous vector.
///
/// This helper iterates the input vector of \ref PerceptionPtr and:
/// - If \p T is exactly \ref PerceptionBase, returns all stored pointers without casting (heterogeneous view).
/// - Otherwise, attempts a `std::dynamic_pointer_cast<T>` and includes only those perceptions that match (homogeneous view).
///
/// \tparam T Target perception type. Must inherit from \ref PerceptionBase. Defaults to \ref PerceptionBase.
/// \param src Source vector containing heterogeneous perceptions and their subscriptions.
/// \return A vector of `std::shared_ptr<T>` containing the matching perceptions, in the same order as \p src.
template<typename T = PerceptionBase>
inline std::vector<std::shared_ptr<T>>
get_perceptions(const std::vector<PerceptionPtr> & src)
{
  static_assert(
    std::is_base_of_v<PerceptionBase, T>,
    "T must inherit from PerceptionBase");

  std::vector<std::shared_ptr<T>> out;
  out.reserve(src.size());

  for (const auto & h : src) {
    if (!h.perception) {continue;}

    if constexpr (std::is_same_v<T, PerceptionBase>) {
      // Heterogeneous: no cast needed
      out.push_back(h.perception);
    } else {
      // Homogeneous by derived type: include only successful casts
      if (auto p = std::dynamic_pointer_cast<T>(h.perception)) {
        out.push_back(std::move(p));
      }
    }
  }
  return out;
}

/// \class PerceptionHandler
/// \brief Abstract base interface for group-specific perception handlers.
///
/// Each handler is responsible for a sensor group (e.g., "points", "image", "dummy").
/// It creates appropriate \ref PerceptionBase instances and handles the conversion from ROS messages.
class PerceptionHandler
{
public:
  virtual ~PerceptionHandler() = default;

  /// \brief Creates a new instance of a perception object managed by this handler.
  /// \param sensor_id Name or ID of the sensor (optional use by handler).
  /// \return Shared pointer to a new instance of a class derived from \ref PerceptionBase.
  virtual std::shared_ptr<PerceptionBase> create(const std::string & sensor_id) = 0;

  /// \brief Creates a subscription that processes messages into \ref PerceptionBase instances.
  ///
  /// The handler is expected to parse the message received on \p topic of type \p type
  /// and store the result in \p target.
  ///
  /// \param node Reference to the lifecycle node used for creating the subscription.
  /// \param topic Topic name to subscribe to.
  /// \param type ROS message type name (e.g., `"sensor_msgs/msg/LaserScan"`).
  /// \param target Shared pointer where perception results are stored.
  /// \param cb_group Callback group where the subscription callback will be executed.
  /// \return Shared pointer to the created subscription.
  virtual rclcpp::SubscriptionBase::SharedPtr create_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic,
    const std::string & type,
    std::shared_ptr<PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group) = 0;

  /// \brief Returns the group identifier associated with this handler.
  ///
  /// This identifier is used to match sensors to the appropriate handler.
  /// Example: `"points"`, `"image"`, `"dummy"`.
  ///
  /// \return String representing the group name.
  virtual std::string group() const = 0;
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
