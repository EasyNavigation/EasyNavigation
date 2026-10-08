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


#ifndef EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_
#define EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_

#include <mutex>

#include "easynav_common/Singleton.hpp"
#include "easynav_common/types/TFInfo.hpp"

#include "tf2_ros/buffer.hpp"

namespace easynav
{

/**
 * @class RTTFBuffer
 * @brief Provides functionality for RTTFBuffer. It also provides the TFInfo with the
 *  frames convention used across EasyNav.
 */
class RTTFBuffer : public tf2_ros::Buffer, public Singleton<RTTFBuffer>
{
public:
  explicit RTTFBuffer(const rclcpp::Clock::SharedPtr & clock)
  : tf2_ros::Buffer(clock)
  {}

  explicit RTTFBuffer()
  : Buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  {
    RCLCPP_WARN(rclcpp::get_logger("RTTFBuffer"),
      "You should be creating this RTTFBuffer with your clock."
      "Using default clock RCL_ROS_TIME");
  }

  /// @brief Returns a snapshot of the current TFInfo.
  ///
  /// Returned by value (copy taken while `tf_info_mutex_` is held): this is read
  /// continuously from both the RT and non-RT threads while `set_tf_info()` can be
  /// called from either during a reconfigure, so a returned reference into `tf_info_`
  /// would be exposed to a concurrent unsynchronized write (torn/dangling strings).
  TFInfo get_tf_info() const
  {
    std::lock_guard<std::mutex> lock(tf_info_mutex_);
    return tf_info_;
  }

  void set_tf_info(const TFInfo & tf_info)
  {
    std::lock_guard<std::mutex> lock(tf_info_mutex_);
    tf_info_ = tf_info;

    // Apply tf_prefix to all frames
    if (tf_info_.tf_prefix != "") {
      tf_info_.map_frame = tf_info_.tf_prefix + "/" + tf_info_.map_frame;
      tf_info_.odom_frame = tf_info_.tf_prefix + "/" + tf_info_.odom_frame;
      tf_info_.robot_frame = tf_info_.tf_prefix + "/" + tf_info_.robot_frame;
      tf_info_.robot_footprint_frame = tf_info_.tf_prefix + "/" + tf_info_.robot_footprint_frame;
      tf_info_.world_frame = tf_info_.tf_prefix + "/" + tf_info_.world_frame;
    }
  }

private:
  TFInfo tf_info_;
  mutable std::mutex tf_info_mutex_;

  SINGLETON_DEFINITIONS(RTTFBuffer)
};

}  // namespace easynav


#endif  // EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_
