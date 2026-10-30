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


#ifndef EASYNAV_COMMON__RTTFBUFFER_HPP_
#define EASYNAV_COMMON__RTTFBUFFER_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <memory>
#include <string>

#include "easynav_common/Singleton.hpp"
#include "easynav_common/types/TFInfo.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"

namespace easynav
{

/**
 * @class RTTFBuffer
 * @brief Provides functionality for RTTFBuffer. It also provides the TFInfo with the
 *  frames convention used across EasyNav.
 *
 * Optionally extrapolates into the future: a lookup up to `max_future_extrapolation`
 * ahead of the latest available transform is answered by integrating the chain's
 * constant twist estimated over the last `extrapolation_window`. Only runs when tf2 itself
 * fails with a future extrapolation; 50 ms by default, 0 disables it.
 */
class RTTFBuffer : public tf2_ros::Buffer, public Singleton<RTTFBuffer>
{
public:
  explicit RTTFBuffer(const rclcpp::Clock::SharedPtr & clock)
  : tf2_ros::Buffer(clock)
  {}

  RTTFBuffer()
  : Buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("RTTFBuffer"),
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

  /// @brief Max time ahead of the latest TF that lookups may extrapolate (0 disables).
  void set_max_future_extrapolation(const tf2::Duration & max_extrapolation)
  {
    max_future_extrapolation_ns_ = std::max<int64_t>(0, max_extrapolation.count());
  }

  tf2::Duration get_max_future_extrapolation() const
  {
    return tf2::Duration(max_future_extrapolation_ns_.load());
  }

  /// @brief History span used to estimate the twist (must be > 0).
  void set_extrapolation_window(const tf2::Duration & window)
  {
    if (window.count() > 0) {
      extrapolation_window_ns_ = window.count();
    }
  }

  tf2::Duration get_extrapolation_window() const
  {
    return tf2::Duration(extrapolation_window_ns_.load());
  }

  // Keep the remaining tf2_ros::Buffer overloads (rclcpp::Time, etc.) visible.
  using tf2_ros::Buffer::lookupTransform;
  using tf2_ros::Buffer::canTransform;

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time) const override
  {
    try {
      return tf2_ros::Buffer::lookupTransform(target_frame, source_frame, time);
    } catch (const tf2::ExtrapolationException &) {
      geometry_msgs::msg::TransformStamped out;
      if (extrapolate(target_frame, source_frame, time, out)) {
        return out;
      }
      throw;
    }
  }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const tf2::Duration timeout) const override
  {
    // A few ms ahead: answer now instead of blocking for the timeout.
    geometry_msgs::msg::TransformStamped out;
    if (!tf2_ros::Buffer::canTransform(target_frame, source_frame, time) &&
      extrapolate(target_frame, source_frame, time, out))
    {
      return out;
    }
    return tf2_ros::Buffer::lookupTransform(target_frame, source_frame, time, timeout);
  }

  bool canTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, std::string * error_msg = nullptr) const override
  {
    if (tf2_ros::Buffer::canTransform(target_frame, source_frame, time, error_msg)) {
      return true;
    }
    geometry_msgs::msg::TransformStamped out;
    if (extrapolate(target_frame, source_frame, time, out)) {
      if (error_msg) {error_msg->clear();}
      return true;
    }
    return false;
  }

  bool canTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const tf2::Duration timeout,
    std::string * errstr = nullptr) const override
  {
    geometry_msgs::msg::TransformStamped out;
    if (!tf2_ros::Buffer::canTransform(target_frame, source_frame, time) &&
      extrapolate(target_frame, source_frame, time, out))
    {
      return true;
    }
    return tf2_ros::Buffer::canTransform(target_frame, source_frame, time, timeout, errstr);
  }

  // Fixed frame: target at target_time -> fixed -> source at source_time; each leg may
  // extrapolate. Same stamp as tf2: target_time (the latest common time if zero).
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame) const override
  {
    try {
      return tf2_ros::Buffer::lookupTransform(
        target_frame, target_time, source_frame, source_time, fixed_frame);
    } catch (const tf2::ExtrapolationException &) {
      geometry_msgs::msg::TransformStamped out;
      if (extrapolate_fixed(
          target_frame, target_time, source_frame, source_time, fixed_frame, out))
      {
        return out;
      }
      throw;
    }
  }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame, const tf2::Duration timeout) const override
  {
    geometry_msgs::msg::TransformStamped out;
    if (!tf2_ros::Buffer::canTransform(
        target_frame, target_time, source_frame, source_time, fixed_frame) &&
      extrapolate_fixed(
        target_frame, target_time, source_frame, source_time, fixed_frame, out))
    {
      return out;
    }
    return tf2_ros::Buffer::lookupTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
  }

  bool canTransform(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame, std::string * error_msg = nullptr) const override
  {
    if (tf2_ros::Buffer::canTransform(
        target_frame, target_time, source_frame, source_time, fixed_frame, error_msg))
    {
      return true;
    }
    geometry_msgs::msg::TransformStamped out;
    if (extrapolate_fixed(
        target_frame, target_time, source_frame, source_time, fixed_frame, out))
    {
      if (error_msg) {error_msg->clear();}
      return true;
    }
    return false;
  }

  bool canTransform(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame, const tf2::Duration timeout,
    std::string * errstr = nullptr) const override
  {
    geometry_msgs::msg::TransformStamped out;
    if (!tf2_ros::Buffer::canTransform(
        target_frame, target_time, source_frame, source_time, fixed_frame) &&
      extrapolate_fixed(
        target_frame, target_time, source_frame, source_time, fixed_frame, out))
    {
      return true;
    }
    return tf2_ros::Buffer::canTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame, timeout, errstr);
  }

private:
  /// @brief Fixed-frame lookup composed from two (possibly extrapolated) legs.
  bool extrapolate_fixed(
    const std::string & target_frame, const tf2::TimePoint & target_time,
    const std::string & source_frame, const tf2::TimePoint & source_time,
    const std::string & fixed_frame, geometry_msgs::msg::TransformStamped & out) const
  {
    if (max_future_extrapolation_ns_.load() <= 0) {return false;}
    geometry_msgs::msg::TransformStamped target_fixed, fixed_source;
    if (!leg(target_frame, fixed_frame, target_time, target_fixed) ||
      !leg(fixed_frame, source_frame, source_time, fixed_source))
    {
      return false;
    }
    const tf2::Transform T = to_tf(target_fixed) * to_tf(fixed_source);
    out = target_fixed;
    out.child_frame_id = source_frame;
    fill_transform(T, out);
    return true;
  }

  /// @brief One leg of a fixed-frame lookup: tf2 first, extrapolation if it is ahead.
  bool leg(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, geometry_msgs::msg::TransformStamped & out) const
  {
    try {
      out = tf2_ros::Buffer::lookupTransform(target_frame, source_frame, time);
      return true;
    } catch (const tf2::ExtrapolationException &) {
      return extrapolate(target_frame, source_frame, time, out);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  static void fill_transform(
    const tf2::Transform & T, geometry_msgs::msg::TransformStamped & out)
  {
    const auto & p = T.getOrigin();
    const auto q = T.getRotation();
    out.transform.translation.x = p.x();
    out.transform.translation.y = p.y();
    out.transform.translation.z = p.z();
    out.transform.rotation.x = q.x();
    out.transform.rotation.y = q.y();
    out.transform.rotation.z = q.z();
    out.transform.rotation.w = q.w();
  }

  /// @brief Future lookup by integrating the chain's twist; false if not applicable.
  bool extrapolate(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, geometry_msgs::msg::TransformStamped & out) const
  {
    const int64_t max_ns = max_future_extrapolation_ns_.load();
    if (max_ns <= 0) {return false;}

    geometry_msgs::msg::TransformStamped last_msg, prev_msg;
    try {
      last_msg = tf2_ros::Buffer::lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }

    const tf2::TimePoint t_last = stamp_to_time(last_msg);
    if (t_last == tf2::TimePoint()) {return false;}  // Static chain: never extrapolates.
    const int64_t ahead_ns = (time - t_last).count();
    if (ahead_ns <= 0 || ahead_ns > max_ns) {return false;}

    const tf2::Duration window(extrapolation_window_ns_.load());
    try {
      prev_msg = tf2_ros::Buffer::lookupTransform(target_frame, source_frame, t_last - window);
    } catch (const tf2::TransformException &) {
      return false;  // Not enough history to estimate the twist.
    }

    // T(t_last + a*w) = T_last * (T_prev^-1 * T_last)^a, constant twist (exact SE(3) power).
    const tf2::Transform T_last = to_tf(last_msg);
    const tf2::Transform delta = to_tf(prev_msg).inverseTimes(T_last);
    const double alpha = static_cast<double>(ahead_ns) / static_cast<double>(window.count());
    const tf2::Transform T_future = T_last * se3_power(delta, alpha);

    out = last_msg;
    out.header.stamp.sec = static_cast<int32_t>(
      std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count());
    out.header.stamp.nanosec = static_cast<uint32_t>(
      (time.time_since_epoch() - std::chrono::seconds(out.header.stamp.sec)).count());
    fill_transform(T_future, out);
    return true;
  }

  static tf2::TimePoint stamp_to_time(const geometry_msgs::msg::TransformStamped & msg)
  {
    return tf2::TimePoint(
      std::chrono::seconds(msg.header.stamp.sec) +
      std::chrono::nanoseconds(msg.header.stamp.nanosec));
  }

  static tf2::Transform to_tf(const geometry_msgs::msg::TransformStamped & msg)
  {
    const auto & t = msg.transform;
    return tf2::Transform(
      tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
      tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
  }

  /// @brief V(theta, axis) of the SE(3) exponential: translation = V * (rho).
  static tf2::Matrix3x3 se3_v(const tf2::Vector3 & axis, double theta)
  {
    const tf2::Matrix3x3 I(1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (std::abs(theta) < 1e-9) {return I;}
    const tf2::Matrix3x3 K(
      0, -axis.z(), axis.y(),
      axis.z(), 0, -axis.x(),
      -axis.y(), axis.x(), 0);
    const tf2::Matrix3x3 K2 = K * K;
    // Taylor series for small angles avoids cancellation in 1 - cos and theta - sin.
    const double t2 = theta * theta;
    const bool small = std::abs(theta) < 1e-3;
    const double a = small ? theta / 2.0 - theta * t2 / 24.0 : (1.0 - std::cos(theta)) / theta;
    const double b = small ? t2 / 6.0 - t2 * t2 / 120.0 : (theta - std::sin(theta)) / theta;
    return tf2::Matrix3x3(
      I[0][0] + a * K[0][0] + b * K2[0][0], I[0][1] + a * K[0][1] + b * K2[0][1],
      I[0][2] + a * K[0][2] + b * K2[0][2],
      I[1][0] + a * K[1][0] + b * K2[1][0], I[1][1] + a * K[1][1] + b * K2[1][1],
      I[1][2] + a * K[1][2] + b * K2[1][2],
      I[2][0] + a * K[2][0] + b * K2[2][0], I[2][1] + a * K[2][1] + b * K2[2][1],
      I[2][2] + a * K[2][2] + b * K2[2][2]);
  }

  /// @brief delta^alpha in SE(3): exp(alpha * log(delta)).
  static tf2::Transform se3_power(const tf2::Transform & delta, double alpha)
  {
    tf2::Quaternion q = delta.getRotation().normalized();
    if (q.w() < 0.0) {q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());}  // Shortest arc.
    // atan2 keeps tiny angles accurate (acos(w) does not); they get scaled by alpha.
    const tf2::Vector3 v(q.x(), q.y(), q.z());
    const double s = v.length();
    double theta = 2.0 * std::atan2(s, q.w());
    tf2::Vector3 axis(0, 0, 1);
    if (s > 1e-15) {axis = v / s;} else {theta = 0.0;}

    const tf2::Vector3 rho = se3_v(axis, theta).inverse() * delta.getOrigin();
    const double theta_a = alpha * theta;
    const tf2::Quaternion q_a = theta > 0.0 ? tf2::Quaternion(axis, theta_a) :
      tf2::Quaternion::getIdentity();
    return tf2::Transform(q_a, se3_v(axis, theta_a) * (rho * alpha));
  }

  TFInfo tf_info_;
  mutable std::mutex tf_info_mutex_;

  std::atomic<int64_t> max_future_extrapolation_ns_ {50'000'000};  // 50 ms
  std::atomic<int64_t> extrapolation_window_ns_ {50'000'000};  // 50 ms


  SINGLETON_DEFINITIONS(RTTFBuffer)
};

}  // namespace easynav


#endif  // EASYNAV_COMMON__RTTFBUFFER_HPP_
