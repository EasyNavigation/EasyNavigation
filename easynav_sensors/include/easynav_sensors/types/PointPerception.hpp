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
/// \brief Defines data structures and utilities for representing and processing point-based sensor perceptions.
///
/// This header provides:
/// - \c PointPerception: a concrete class for point cloud data.
/// - \c PointPerceptionHandler: a handler for \c sensor_msgs::msg::LaserScan and
///   \c sensor_msgs::msg::PointCloud2 messages.
/// - \c PointPerceptionsOpsView: an efficient view-based operator class for processing multiple perceptions
///   (filtering, downsampling, fusion, and collapsing) without duplicating memory.
/// - Conversion utilities between ROS messages and PCL point clouds.

#ifndef EASYNAV_SENSORS_TYPES__POINTPERCEPTIONS_HPP_
#define EASYNAV_SENSORS_TYPES__POINTPERCEPTIONS_HPP_

#include <string>
#include <vector>
#include <optional>
#include <mutex>

#include "tf2/LinearMath/Transform.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_sensors/types/Perceptions.hpp"
#include "easynav_common/CircularBuffer.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace std
{

/// \brief Custom hash specialization for \c std::tuple<int,int,int>.
template<>
struct hash<std::tuple<int, int, int>>
{
  /// \brief Computes the hash value.
  /// \param key Tuple \c (x,y,z).
  /// \return Hash value for the tuple.
  std::size_t operator()(const std::tuple<int, int, int> & key) const
  {
    std::size_t h1 = std::hash<int>()(std::get<0>(key));
    std::size_t h2 = std::hash<int>()(std::get<1>(key));
    std::size_t h3 = std::hash<int>()(std::get<2>(key));
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

}  // namespace std

namespace easynav
{

struct PointPerceptionBufferType
{
  pcl::PointCloud<pcl::PointXYZ> data;
  std::string frame;
  rclcpp::Time stamp;
};

/// \class PointPerception
/// \brief Concrete perception class for 3D point cloud data.
///
/// Stores a point cloud of type \c pcl::PointCloud<pcl::PointXYZ> and the common metadata inherited
/// from \ref PerceptionBase.
class PointPerception : public PerceptionBase
{
public:
  /// \brief Group identifier for point perceptions.
  static constexpr std::string_view default_group_ = "points";

  /// \brief Checks if a ROS message type is supported by this perception.
  /// \param t Fully-qualified ROS 2 message type name (e.g., \c "sensor_msgs/msg/PointCloud2").
  /// \return \c true if \p t is \c sensor_msgs/msg/LaserScan or \c sensor_msgs/msg/PointCloud2, otherwise \c false.
  static inline bool supports_msg_type(std::string_view t)
  {
    return t == "sensor_msgs/msg/LaserScan" ||
           t == "sensor_msgs/msg/PointCloud2";
  }

  PointPerception()
  {
    [[maybe_unused]] static const bool _ = [] {
        ::easynav::NavState::register_printer<PointPerception>(
          [](const PointPerception & perception) {
            std::ostringstream ret;
            ret << "{ " << perception.stamp.seconds()
                << " } PointPerception with " << perception.data.size()
                << " points in frame [" << perception.frame_id
                << "] with ts " << perception.stamp.seconds() << "\n";
            return ret.str();
        });
        return true;
      }();
  }

  PointPerception(const PointPerception & other)
  {
    std::lock_guard<std::mutex> lock(other.mutex_);
    stamp = other.stamp;
    frame_id = other.frame_id;
    valid = other.valid;
    new_data = other.new_data;

    data = other.data;
    pending_available_ = other.pending_available_;
    pending_cloud_ = other.pending_cloud_;
    pending_frame_ = other.pending_frame_;
    pending_stamp_ = other.pending_stamp_;
    buffer = other.buffer;
  }

  PointPerception & operator=(const PointPerception & other)
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
    pending_available_ = other.pending_available_;
    pending_cloud_ = other.pending_cloud_;
    pending_frame_ = other.pending_frame_;
    pending_stamp_ = other.pending_stamp_;
    buffer = other.buffer;

    return *this;
  }

  /// \brief The 3D point cloud data associated with this perception.
  pcl::PointCloud<pcl::PointXYZ> data;

  bool pending_available_{false};
  pcl::PointCloud<pcl::PointXYZ> pending_cloud_;
  std::string pending_frame_;
  rclcpp::Time pending_stamp_;

  /// \brief Stores a pending cloud atomically for later integration.
  void set_pending_cloud(
    pcl::PointCloud<pcl::PointXYZ> && cloud,
    std::string && frame,
    const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_cloud_ = std::move(cloud);
    pending_frame_ = std::move(frame);
    pending_stamp_ = stamp;
    pending_available_ = true;
  }


  /// \brief Resizes the internal point cloud storage.
  /// \param size Number of points to allocate in \ref data.
  void resize(std::size_t size)
  {
    data.points.resize(size);
  }

  /// \brief Retrieves the most recent buffered perception (independently of it has a valid TF) without removing it from the buffer.
  const PointPerceptionBufferType & get_last_perception() const
  {
    return buffer.latest_ref();
  }

  void integrate_pending_perceptions()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Access TF buffer singleton (already initialized somewhere with a clock)
    auto tf_buffer_ptr = RTTFBuffer::getInstance();
    auto & tf_buffer = *tf_buffer_ptr;
    const auto tf_info = tf_buffer.get_tf_info();
    const std::string & robot_frame = tf_info.robot_frame;

    // ------------------------------------------------------------------
    // 1. Push pending perception into the circular buffer exactly once.
    // ------------------------------------------------------------------
    if (pending_available_) {
      PointPerceptionBufferType pending_item;
      pending_item.data = std::move(pending_cloud_); // avoid deep copy
      pending_item.frame = pending_frame_;
      pending_item.stamp = pending_stamp_;

      buffer.push(std::move(pending_item));
      pending_available_ = false;
    }

    const std::size_t count = buffer.size();
    if (count == 0) {
      // No candidates at all: keep current visible state as is.
      return;
    }

    // ------------------------------------------------------------------
    // 2. Drain the circular buffer into a temporary vector so we can
    //    inspect all items and then rebuild the buffer.
    // ------------------------------------------------------------------
    std::vector<PointPerceptionBufferType> items;
    items.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
      PointPerceptionBufferType item;
      if (!buffer.pop(item)) {
        break; // Defensive guard if pop() fails unexpectedly.
      }
      items.push_back(std::move(item));
    }

    if (items.empty()) {
      // Nothing recovered from buffer: keep visible state untouched.
      return;
    }

    // ------------------------------------------------------------------
    // 3. Find indices:
    //    - newest_idx: newest perception by timestamp (regardless of TF),
    //    - newest_valid_idx: newest perception that has a valid TF.
    //
    //    IMPORTANT: store only indices to avoid copying point clouds
    //    during the scan.
    // ------------------------------------------------------------------
    std::optional<std::size_t> newest_idx;
    std::optional<std::size_t> newest_valid_idx;

    for (std::size_t i = 0; i < items.size(); ++i) {
      const auto & item = items[i];

      // Track newest item overall (used when no TF is valid).
      if (!newest_idx || item.stamp > items[*newest_idx].stamp) {
        newest_idx = i;
      }

      bool has_tf = false;
      try {
        has_tf = tf_buffer.canTransform(
        robot_frame,
        item.frame,
        tf2_ros::fromMsg(item.stamp),
        tf2::durationFromSec(0.0));
      } catch (...) {
      // Any TF exception is treated as "no valid TF" for this item.
        has_tf = false;
      }

      if (has_tf) {
        if (!newest_valid_idx || item.stamp > items[*newest_valid_idx].stamp) {
          newest_valid_idx = i;
        }
      }
    }

    // ------------------------------------------------------------------
    // 4. Update visible state BEFORE moving items back into the buffer.
    //    This guarantees that `data` corresponds to:
    //      - the newest TF-valid item if any exists, otherwise
    //      - the newest item overall.
    // ------------------------------------------------------------------
    if (newest_valid_idx) {
      const auto & sel = items[*newest_valid_idx];
      data = sel.data;        // single deep copy (intentional)
      frame_id = sel.frame;
      stamp = sel.stamp;
      valid = true;           // "valid" means "usable / not too old", not "TF ok"
      new_data = true;
    } else if (newest_idx) {
      const auto & sel = items[*newest_idx];
      data = sel.data;        // single deep copy (intentional)
      frame_id = sel.frame;
      stamp = sel.stamp;
      valid = true;
      new_data = true;
    } else {
      // Defensive: should not happen because items is non-empty.
      return;
    }

    // ------------------------------------------------------------------
    // 5. Rebuild the circular buffer from scratch (move-only, no copies).
    // ------------------------------------------------------------------
    buffer.clear();

    if (newest_valid_idx) {
      // Keep the newest TF-valid item and any newer items (even if TF is not yet available).
      const rclcpp::Time cutoff_stamp = items[*newest_valid_idx].stamp;

      for (auto & item : items) {
        if (item.stamp >= cutoff_stamp) {
          buffer.push(std::move(item));
        }
      }
    } else {
      // No TF-valid items: keep everything in case TF arrives later.
      for (auto & item : items) {
        buffer.push(std::move(item));
      }
    }
  }

protected:
  mutable std::mutex mutex_;
  CircularBuffer<PointPerceptionBufferType> buffer{10};
};

/// \class PointPerceptionHandler
/// \brief \ref PerceptionHandler implementation for sensors producing point-based data.
///
/// Supports both \c sensor_msgs::msg::LaserScan and \c sensor_msgs::msg::PointCloud2, converting
/// incoming messages into \ref PointPerception instances.
class PointPerceptionHandler : public PerceptionHandler
{
public:
  /// \brief Optional post-initialization hook for subclasses.
  /// Here, the handler must reserve memory to store the perception data
  /// and create any Subscription or similar objects to read the data.
  void on_initialize() override;

  /// \brief Run one real-time sensor processing cycle.
  /// This method is called by the SensorsNode before executing its cycle_rt.
  /// Here the handler should update the NavState with the sensor data.
  /// If new data arrived before this call and the state is updated, it must return true.
  ///
  /// \param nav_state Pointer to the NavState to store the sensor data.
  /// \return True if new data was stored (to trigger processing).
  bool cycle_rt([[maybe_unused]] std::shared_ptr<NavState> nav_state) override;

private:
  /// \brief pointer to the perception data
  std::shared_ptr<PointPerception> perception_data_ {nullptr};

  /// \brief pointer to the subscription object
  rclcpp::SubscriptionBase::SharedPtr perception_sub_;
};

/// \brief Converts a \c LaserScan message into a point cloud.
/// \param scan Input laser scan message.
/// \param pc Output point cloud (XYZ).
void convert(const sensor_msgs::msg::LaserScan & scan, pcl::PointCloud<pcl::PointXYZ> & pc);

/// \brief Converts a \ref PointPerception into a \c sensor_msgs::msg::PointCloud2 message.
/// \param perception Perception to convert.
/// \return The resulting ROS \c PointCloud2 message.
sensor_msgs::msg::PointCloud2 perception_to_rosmsg(const PointPerception & perception);

/// \brief Converts a PCL point cloud into a \c sensor_msgs::msg::PointCloud2 message.
/// \param points Input point cloud.
/// \return The resulting ROS \c PointCloud2 message.
sensor_msgs::msg::PointCloud2 points_to_rosmsg(const pcl::PointCloud<pcl::PointXYZ> & points);

/// \typedef PointPerceptions
/// \brief Alias for a vector of shared pointers to \ref PointPerception objects.
using PointPerceptions =
  std::vector<std::shared_ptr<PointPerception>>;

/// \brief Retrieves the latest timestamp among a set of point-based perceptions.
/// \param perceptions Container of point-based perceptions.
/// \return The most recent timestamp found in \p perceptions, or a default-constructed \c rclcpp::Time if \p perceptions is empty.
rclcpp::Time get_latest_point_perceptions_stamp(const PointPerceptions & perceptions);

/// \class PointPerceptionsOpsView
/// \brief Provides efficient, non-destructive, chainable operations over a set of point-based perceptions.
///
/// This view enables filtering, downsampling, fusion, and dimensional collapsing across multiple point clouds
/// without duplicating the underlying perceptions. Most operations work lazily by keeping index-based selections
/// and transformation options, and only materialize a new point cloud when required (for example in as_points()).
class PointPerceptionsOpsView
{
public:
  /// \struct VoxelKey
  /// \brief Discrete 3D voxel index used for downsampling.
  struct VoxelKey
  {
    /// \brief Discrete coordinates (voxel indices).
    int x, y, z;

    /// \brief Equality comparison.
    /// \param other Voxel key to compare against.
    /// \return \c true if all components are equal, otherwise \c false.
    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  /// \struct VoxelKeyHash
  /// \brief Hash functor for \ref VoxelKey.
  struct VoxelKeyHash
  {
    /// \brief Computes the hash value.
    /// \param key Voxel key.
    /// \return Hash value for \p key.
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t h1 = std::hash<int>{}(key.x);
      std::size_t h2 = std::hash<int>{}(key.y);
      std::size_t h3 = std::hash<int>{}(key.z);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  /// \brief Constructs a view over an external container of perceptions.
  /// \param perceptions Constant reference to the source container.
  explicit PointPerceptionsOpsView(const PointPerceptions & perceptions);

  /// \brief Constructs a view from a single PointPerception instance.
  ///
  /// Creates an internal container owning the given perception and
  /// initializes the index set for all its points.
  ///
  /// \param perception The PointPerception to wrap in the view.
  explicit PointPerceptionsOpsView(const PointPerception & perception);

  /// \brief Constructs a view taking ownership of the container.
  /// \param perceptions Rvalue container of perceptions to be owned by the view.
  explicit PointPerceptionsOpsView(PointPerceptions && perceptions);

  PointPerceptionsOpsView(const PointPerceptionsOpsView &) = delete;
  PointPerceptionsOpsView & operator=(const PointPerceptionsOpsView &) = delete;

  PointPerceptionsOpsView(PointPerceptionsOpsView &&) = default;

  /// \brief Filters all point clouds by axis-aligned bounds.
  ///
  /// When the view has no target frame configured (that is, \c fuse() has not been called),
  /// this method always performs an eager filtering step in the original frame of each sensor,
  /// updating the internal index sets in-place.
  ///
  /// When a target frame has been configured via \c fuse(), the behaviour depends on
  /// \p lazy_post_fuse:
  /// - If \p lazy_post_fuse is \c true (default), the bounds are stored as a post-fuse
  ///   filter in the target frame and are applied lazily during materialization
  ///   (for example, in \c as_points()), without modifying the internal indices.
  /// - If \p lazy_post_fuse is \c false, the method applies the transform to the target
  ///   frame immediately and performs an eager filtering step in that frame, updating
  ///   the internal indices accordingly.
  ///
  /// Components set to \c NaN in \p min_bounds or \p max_bounds leave the corresponding
  /// axis unbounded.
  ///
  /// \param min_bounds Minimum \c [x,y,z] values (use \c NaN to disable per axis).
  /// \param max_bounds Maximum \c [x,y,z] values (use \c NaN to disable per axis).
  /// \param lazy_post_fuse If \c true and a target frame is set, configure a lazy
  ///        post-fuse filter in the target frame; if \c false, always filter eagerly
  ///        (updating indices) in the current frame (sensor or target).
  /// \return Reference to \c *this to allow chaining.
  PointPerceptionsOpsView & filter(
    const std::vector<double> & min_bounds,
    const std::vector<double> & max_bounds,
    bool lazy_post_fuse = true);

  /// \brief Downsamples each perception using a voxel grid.
  /// \param resolution Voxel size in meters.
  /// \return Reference to \c *this to allow chaining.
  PointPerceptionsOpsView & downsample(double resolution);

  /// \brief Collapses dimensions to fixed values (for example, projection onto a plane).
  ///
  /// Components set to \c NaN in \p collapse_dims keep the original values.
  ///
  /// The behaviour depends on \p lazy:
  /// - If \p lazy is \c true (default), the collapse configuration is stored in the view
  ///   and applied lazily when materializing point clouds (for example, in \c as_points()),
  ///   without modifying the underlying perception data.
  /// - If \p lazy is \c false and the view owns its internal container, the collapse is
  ///   applied eagerly by updating the coordinates of all stored points, so subsequent
  ///   operations (filters, fusion, etc.) observe the collapsed geometry.
  ///   On non-owning views, the eager mode is ignored to avoid modifying external data.
  ///
  /// \param collapse_dims Fixed values for each axis (use \c NaN to preserve original values).
  /// \param lazy If \c true, configure collapse lazily for output only; if \c false and the
  ///        view is owning, apply the collapse immediately to the internal point data.
  /// \return Reference to \c *this to allow chaining.
  PointPerceptionsOpsView &
  collapse(const std::vector<double> & collapse_dims, bool lazy = true);

  /// \brief Retrieves all selected points across perceptions as a single concatenated cloud.
  /// \return Concatenated point cloud.
  pcl::PointCloud<pcl::PointXYZ> as_points() const;

  /// \brief Retrieves the filtered point cloud for a specific perception.
  /// \param idx Index of the target perception in the underlying container.
  /// \return Const reference to the filtered point cloud.
  const pcl::PointCloud<pcl::PointXYZ> & as_points(int idx) const;

  /// \brief Configures fusion of all perceptions into a common frame.
  ///
  /// This method does not immediately build a fused point cloud. Instead, it stores the target frame
  /// and the required transforms so that subsequent operations (for example filter) and final
  /// materialization (as_points()) work in \p target_frame without duplicating the underlying data.
  ///
  /// \param target_frame Frame ID to which all clouds are conceptually transformed.
  /// \param stamp Effective timestamp for the TF lookup.
  /// \param exact_time If \c true, TF lookups use the exact timestamp of each perception;
  ///        if \c false (default), the most recent available transform is used.
  /// \return Reference to \c *this to allow chaining.
  PointPerceptionsOpsView & fuse(const std::string & target_frame, bool exact_time = false);

  /// \brief Configures fusion of all perceptions into a common frame, returning the effective stamp.
  ///
  /// This method does not immediately build a fused point cloud. Instead, it stores the target frame
  /// and the required transforms so that subsequent operations (for example filter) and final
  /// materialization (as_points()) work in \p target_frame without duplicating the underlying data.
  ///
  /// The effective timestamp used for TF lookups is returned via the \p stamp parameter.
  ///
  /// If \p exact_time is \c true, \p stamp is set to the timestamp of the most recent perception
  /// among those being fused; otherwise, it is set to the time at which the TF lookup is performed.
  /// \param target_frame Frame ID to which all clouds are conceptually transformed.
  /// \param stamp Reference to a \c rclcpp::Time variable where the effective timestamp will be stored.
  /// \param exact_time If \c true, TF lookups use the exact timestamp of each perception;
  ///        if \c false (default), the most recent available transform is used.
  /// \return Reference to \c *this to allow chaining.
  PointPerceptionsOpsView & fuse(
    const std::string & target_frame,
    rclcpp::Time & stamp,
    bool exact_time = false);

  /// \brief Adds a new perception to the current view.
  ///
  /// This method extends the underlying set of perceptions managed by the view.
  /// It does not create a new PointPerceptionsOpsView instance; instead, it
  /// updates the current view in place and returns a reference to \c *this
  /// to allow method chaining.
  ///
  /// The newly added perception becomes part of subsequent operations such as
  /// filtering, fusion, collapsing, and final materialization (as_points()).
  ///
  /// \param points Point cloud to include.
  /// \param frame Frame ID associated with \p points.
  /// \param stamp Timestamp associated with \p points.
  /// \return Reference to \c *this to allow chaining.
  PointPerceptionsOpsView &
  add(
    const pcl::PointCloud<pcl::PointXYZ> points,
    const std::string & frame,
    rclcpp::Time stamp);

  /// \brief Provides a constant reference to the underlying perceptions container.
  /// \return Constant reference to the container.
  const PointPerceptions & get_perceptions() const {return perceptions_;}

  /// \brief Retrieves the latest timestamp across all perceptions.
  /// \return The most recent timestamp.
  rclcpp::Time get_latest_stamp() const;

private:
  std::optional<PointPerceptions> owned_;      ///< Owned container if moved in.
  const PointPerceptions & perceptions_;       ///< Reference to perception container.
  std::vector<pcl::PointIndices> indices_;     ///< Filtered indices per perception.

  // Lazy fusion state
  bool has_target_frame_ {false};              ///< True if a common target frame has been configured.
  std::string target_frame_;                   ///< Target frame configured by fuse().
  std::vector<tf2::Transform> tf_transforms_;  ///< Cached transforms from perception frame to target frame.
  std::vector<bool> tf_valid_;                 ///< True if corresponding transform is valid.

  // Lazy collapse state
  bool collapse_x_ {false};                    ///< Collapse X dimension if true.
  bool collapse_y_ {false};                    ///< Collapse Y dimension if true.
  bool collapse_z_ {false};                    ///< Collapse Z dimension if true.
  float collapse_val_x_ {0.0f};                ///< Value used when collapsing X (if enabled).
  float collapse_val_y_ {0.0f};                ///< Value used when collapsing Y (if enabled).
  float collapse_val_z_ {0.0f};                ///< Value used when collapsing Z (if enabled).

  bool has_post_filter_ {false};
  double post_min_[3] {0.0, 0.0, 0.0};
  double post_max_[3] {0.0, 0.0, 0.0};
  bool use_post_min_[3] {false, false, false};
  bool use_post_max_[3] {false, false, false};

  // Temporary storage for as_points(int)
  mutable pcl::PointCloud<pcl::PointXYZ> tmp_single_cloud_;
};

}  // namespace easynav

#endif  // EASYNAV_SENSORS_TYPES__POINTPERCEPTIONS_HPP_
