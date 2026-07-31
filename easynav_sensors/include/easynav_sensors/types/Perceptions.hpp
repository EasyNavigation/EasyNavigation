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
/// \brief Defines data structures and utilities for representing and processing sensor perceptions.
///
/// This file provides common interfaces for sensor perception handling:
/// - `PerceptionBase`: base class for sensor data.
/// - `PerceptionPtr`: utility for holding perception state and its subscription.
/// - `get_perceptions`: helper to extract typed collections from a heterogeneous container.
/// - `PerceptionHandler`: abstract base class for group-specific sensor handlers (pluginlib plugin).

#ifndef EASYNAV_SENSORS_TYPES__PERCEPTIONS_HPP_
#define EASYNAV_SENSORS_TYPES__PERCEPTIONS_HPP_

#include <string>

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

// Forward declaration needed by PerceptionPtr.
class PerceptionHandler;

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
  // TODO: Not in use
  bool valid = false;

  /// \brief Whether the data has changed since the last observation.
  bool new_data = false;
};

/// \typedef PerceptionBasePtr
/// \brief Shared pointer alias to \ref PerceptionBase.
using PerceptionBasePtr = std::shared_ptr<PerceptionBase>;


/// \brief Extracts a homogeneous collection of perceptions of type \p T from a heterogeneous vector.
///
/// This helper iterates the input vector of \ref PerceptionBasePtr and attempts a `std::dynamic_pointer_cast<T>`
/// and includes only those perceptions that match (homogeneous view).
///
/// \tparam T Target perception type. Must inherit from \ref PerceptionBase. Defaults to \ref PerceptionBase.
/// \param src Source vector containing pointers to heterogeneous perceptions ( \ref PerceptionBasePtr ).
/// \return A vector of `std::shared_ptr<T>` containing the matching perceptions, in the same order as \p src.
template<typename T = PerceptionBase>
inline std::vector<std::shared_ptr<T>>
get_perceptions(const std::vector<PerceptionBasePtr> & src)
{
  static_assert(std::is_base_of_v<PerceptionBase, T>,
                "T must inherit from PerceptionBase");

  std::vector<std::shared_ptr<T>> out;
  out.reserve(src.size());

  for (const auto & perception : src) {
    if (!perception) {continue;}
    // Homogeneous by derived type: include only successful casts
    if (auto p = std::dynamic_pointer_cast<T>(perception)) {
      out.push_back(std::move(p));
    }
  }
  return out;
}


/// \class PerceptionHandler
/// \brief Abstract base class for pluginlib-based sensor perception handlers.
///
/// Each handler is responsible for a single sensor input (e.g., "lidar_center", "image_color", "imu_0").
/// Concrete handlers are registered as pluginlib plugins and loaded at runtime.
/// A user can implement a new sensor input handler by deriving from this class and registering it
/// as a plugin in the corresponding package's plugin XML file.
/// The handler is the owner of the sensor data.
/// It is also responsible for reserving memory to hold the sensor data and
/// must keep the data address consistent during the whole execution.
/// The handler must populate the NavState with the sensor data
class PerceptionHandler
{
public:
  virtual ~PerceptionHandler() = default;

  /// \brief Initializes the handler with the parent node and sensor name.
  ///
  /// Must be called once before any other method. Stores the node and sensor name,
  /// then delegates to \ref on_initialize for subclass-specific setup.
  ///
  /// \param parent_node Shared pointer to the lifecycle node managing this handler.
  /// \param sensor_name Name of the sensor (used as parameter namespace prefix).
  void initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const rclcpp::CallbackGroup::SharedPtr realtime_cbg,
    const std::string & sensor_name)
  {
    parent_node_ = parent_node;
    realtime_cbg_ = realtime_cbg;
    sensor_name_ = sensor_name;
    on_initialize();
  }

  /// \brief Optional post-initialization hook for subclasses.
  /// Here, the handler must reserve memory to store the perception data
  /// and create any Subscription or similar objects to read the data.
  virtual void on_initialize() {}

  /// @brief Run one real-time sensor processing cycle.
  /// This method is called by the SensorsNode before executing its cycle_rt.
  /// Here the handler should update the NavState with the sensor data.
  /// If new data arrived before this call and the state is updated, it must return true.
  ///
  /// @param nav_state Pointer to the NavState to store the sensor data.
  /// @return True if new data was stored (to trigger processing).
  virtual bool cycle_rt([[maybe_unused]] std::shared_ptr<NavState> nav_state) {return false;}


  /// \brief Returns the sensor name provided during \ref initialize.
  const std::string & get_sensor_name() const {return sensor_name_;}

protected:
  /// \brief Returns the parent lifecycle node.
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const {return parent_node_.lock();}

  /// \brief Returns the parent lifecycle node.
  rclcpp::CallbackGroup::SharedPtr get_realtime_cbg() const {return realtime_cbg_;}

  /// \brief Shared pointer to the parent lifecycle node.
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_;

  /// \brief Callback group for real-time operations.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /// \brief Name of the sensor (used as YAML parameter namespace prefix).
  std::string sensor_name_;
};

}  // namespace easynav

#endif  // EASYNAV_SENSORS_TYPES__PERCEPTIONS_HPP_
