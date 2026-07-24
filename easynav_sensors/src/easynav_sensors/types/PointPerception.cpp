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
#include <vector>
#include <optional>

#include <execinfo.h>

#include <cxxabi.h>

#include <cstdlib>
#include <sstream>
#include <chrono>

#include "pcl_conversions/pcl_conversions.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

std::string
backtrace_to_string(std::size_t max_frames = 64, std::size_t skip = 0)
{
  std::ostringstream oss;

  if (max_frames == 0) {
    return {};
  }

  std::vector<void *> frames(max_frames);
  const int count = ::backtrace(frames.data(), static_cast<int>(frames.size()));
  if (count <= 0) {
    return {};
  }

  char ** symbols = ::backtrace_symbols(frames.data(), count);
  if (!symbols) {
    return {};
  }

  const std::size_t start = std::min<std::size_t>(skip, static_cast<std::size_t>(count));
  for (std::size_t i = start; i < static_cast<std::size_t>(count); ++i) {
    // Try to demangle a function name if present in the symbol string.
    // Typical format: <binary>(<mangled>+0x...)[0x...]
    std::string line = symbols[i] ? symbols[i] : "";

    const auto lparen = line.find('(');
    const auto plus = line.find('+', lparen == std::string::npos ? 0 : lparen);
    if (lparen != std::string::npos && plus != std::string::npos && lparen + 1 < plus) {
      const std::string mangled = line.substr(lparen + 1, plus - (lparen + 1));

      int status = 0;
      char * demangled = abi::__cxa_demangle(mangled.c_str(), nullptr, nullptr, &status);
      if (status == 0 && demangled) {
        line.replace(lparen + 1, mangled.size(), demangled);
      }
      std::free(demangled);
    }

    oss << "  [" << (i - start) << "] " << line << "\n";
  }

  std::free(symbols);
  return oss.str();
}

void PointPerceptionHandler::on_initialize()
{
  // Create the perception data instance
  perception_data_ = std::make_shared<PointPerception>();

  // Get sensor parameters
  auto node = get_node();
  std::string topic, msg_type;

  if (!node->has_parameter(get_sensor_name() + ".topic")) {
    node->declare_parameter(get_sensor_name() + ".topic", std::string{});
  }
  if (!node->has_parameter(get_sensor_name() + ".type")) {
    node->declare_parameter(get_sensor_name() + ".type", std::string{});
  }

  node->get_parameter(get_sensor_name() + ".topic", topic);
  node->get_parameter(get_sensor_name() + ".type", msg_type);

  // Setup subscription
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = get_realtime_cbg();

  const auto clock_type = node->get_clock()->get_clock_type();

  if (msg_type == "sensor_msgs/msg/PointCloud2") {
    perception_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::QoS(1),
      [this, clock_type](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        pcl::PointCloud<pcl::PointXYZ> pending_cloud;
        pcl::fromROSMsg(*msg, pending_cloud);
        perception_data_->set_pending_cloud(
          std::move(pending_cloud),
          std::string(msg->header.frame_id),
          rclcpp::Time(msg->header.stamp, clock_type));
        perception_data_->integrate_pending_perceptions();
      },
      options);
  } else if (msg_type == "sensor_msgs/msg/LaserScan") {
    perception_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS().reliable(),
      [this, clock_type](const sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        pcl::PointCloud<pcl::PointXYZ> pending_cloud;
        convert(*msg, pending_cloud);
        perception_data_->set_pending_cloud(
          std::move(pending_cloud),
          std::string(msg->header.frame_id),
          rclcpp::Time(msg->header.stamp, clock_type));
        perception_data_->integrate_pending_perceptions();
      },
      options);
  } else {
    throw std::runtime_error(
    "Unsupported message type for PointPerceptionHandler [" + msg_type + "]");
  }

}

bool PointPerceptionHandler::cycle_rt(std::shared_ptr<NavState> nav_state)
{
  // Store the perception in the NavState
  nav_state->set(get_sensor_name(), perception_data_);
  // Check if there was new data to trigger process and reset new_data state
  const bool should_trigger = perception_data_->new_data;
  perception_data_->new_data = false;
  return should_trigger;
}

void
convert(const sensor_msgs::msg::LaserScan & scan, pcl::PointCloud<pcl::PointXYZ> & pc)
{
  const size_t num_points = scan.ranges.size();

  if (pc.points.size() != num_points) {
    pc.points.resize(num_points);
  }

  pc.header.frame_id = scan.header.frame_id;
  pc.width = static_cast<uint32_t>(pc.points.size());
  pc.height = 1;
  pc.is_dense = false;

  for (size_t i = 0; i < num_points; ++i) {
    float range = scan.ranges[i];

    pcl::PointXYZ & point = pc.points[i];

    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      point.x = std::numeric_limits<float>::quiet_NaN();
      point.y = std::numeric_limits<float>::quiet_NaN();
      point.z = std::numeric_limits<float>::quiet_NaN();
    } else {
      float angle = scan.angle_min + i * scan.angle_increment;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      point.z = 0.0f;
    }
  }
}

sensor_msgs::msg::PointCloud2
perception_to_rosmsg(const PointPerception & perception)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(perception.data, msg);
  msg.header.frame_id = perception.frame_id;
  msg.header.stamp = perception.stamp;
  return msg;
}

sensor_msgs::msg::PointCloud2
points_to_rosmsg(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  return msg;
}


PointPerceptionsOpsView::PointPerceptionsOpsView(const PointPerceptions & perceptions)
: owned_(std::nullopt),
  perceptions_(perceptions)
{
  const std::size_t n = perceptions_.size();
  indices_.resize(n);
  tf_transforms_.resize(n);
  tf_valid_.assign(n, false);

  for (std::size_t i = 0; i < n; ++i) {
    auto & pptr = perceptions_[i];
    if (!pptr || !pptr->valid || pptr->data.empty()) {
      continue;
    }
    auto & idx = indices_[i].indices;
    idx.resize(pptr->data.size());
    std::iota(idx.begin(), idx.end(), 0);
  }
}

PointPerceptionsOpsView::PointPerceptionsOpsView(const PointPerception & perception)
: owned_(PointPerceptions{}),
  perceptions_(*owned_)
{
  owned_->push_back(std::make_shared<PointPerception>(perception));

  const std::size_t n = owned_->size();
  indices_.resize(n);
  tf_transforms_.resize(n);
  tf_valid_.assign(n, false);

  auto & pptr = owned_->front();
  if (pptr && pptr->valid && !pptr->data.empty()) {
    auto & idx = indices_.front().indices;
    idx.resize(pptr->data.size());
    std::iota(idx.begin(), idx.end(), 0);
  }
}

PointPerceptionsOpsView::PointPerceptionsOpsView(PointPerceptions && perceptions)
: owned_(std::move(perceptions)),
  perceptions_(*owned_)
{
  const std::size_t n = perceptions_.size();
  indices_.resize(n);
  tf_transforms_.resize(n);
  tf_valid_.assign(n, false);

  for (std::size_t i = 0; i < n; ++i) {
    auto & pptr = perceptions_[i];
    if (!pptr || !pptr->valid || pptr->data.empty()) {
      continue;
    }
    auto & idx = indices_[i].indices;
    idx.resize(pptr->data.size());
    std::iota(idx.begin(), idx.end(), 0);
  }
}

PointPerceptionsOpsView &
PointPerceptionsOpsView::filter(
  const std::vector<double> & min_bounds,
  const std::vector<double> & max_bounds,
  bool lazy_post_fuse)
{
  if (has_target_frame_ && lazy_post_fuse) {
    has_post_filter_ = true;

    auto fill_bounds = [](const std::vector<double> & src,
      double dst[3],
      bool used[3]) {
        for (int k = 0; k < 3; ++k) {
          if (static_cast<std::size_t>(k) < src.size() && !std::isnan(src[k])) {
            used[k] = true;
            dst[k] = src[k];
          } else {
            used[k] = false;
            dst[k] = 0.0;
          }
        }
      };

    fill_bounds(min_bounds, post_min_, use_post_min_);
    fill_bounds(max_bounds, post_max_, use_post_max_);

    return *this;
  }

  const bool use_x_min = min_bounds.size() > 0 && !std::isnan(min_bounds[0]);
  const bool use_y_min = min_bounds.size() > 1 && !std::isnan(min_bounds[1]);
  const bool use_z_min = min_bounds.size() > 2 && !std::isnan(min_bounds[2]);

  const bool use_x_max = max_bounds.size() > 0 && !std::isnan(max_bounds[0]);
  const bool use_y_max = max_bounds.size() > 1 && !std::isnan(max_bounds[1]);
  const bool use_z_max = max_bounds.size() > 2 && !std::isnan(max_bounds[2]);

  const double xmin = use_x_min ? min_bounds[0] : 0.0;
  const double ymin = use_y_min ? min_bounds[1] : 0.0;
  const double zmin = use_z_min ? min_bounds[2] : 0.0;

  const double xmax = use_x_max ? max_bounds[0] : 0.0;
  const double ymax = use_y_max ? max_bounds[1] : 0.0;
  const double zmax = use_z_max ? max_bounds[2] : 0.0;

  const std::size_t n = perceptions_.size();
  const bool has_tf_array =
    has_target_frame_ && tf_valid_.size() == n && tf_transforms_.size() == n;

  if (!lazy_post_fuse) {
    has_post_filter_ = false;
  }

  for (std::size_t i = 0; i < n; ++i) {
    const auto & pptr = perceptions_[i];
    auto & idx_list = indices_[i].indices;

    if (!pptr || !pptr->valid || pptr->data.empty() || idx_list.empty()) {
      idx_list.clear();
      continue;
    }

    const auto & cloud = pptr->data;

    std::vector<int> new_indices;
    new_indices.reserve(idx_list.size());

    const bool apply_tf = has_tf_array && tf_valid_[i];

    for (int idx : idx_list) {
      if (idx < 0 || static_cast<std::size_t>(idx) >= cloud.size()) {
        continue;
      }

      const auto & pt = cloud[idx];
      double x = pt.x;
      double y = pt.y;
      double z = pt.z;

      if (apply_tf) {
        tf2::Vector3 p(pt.x, pt.y, pt.z);
        p = tf_transforms_[i] * p;
        x = p.x();
        y = p.y();
        z = p.z();
      }

      if (use_x_min && x < xmin) {continue;}
      if (use_y_min && y < ymin) {continue;}
      if (use_z_min && z < zmin) {continue;}

      if (use_x_max && x > xmax) {continue;}
      if (use_y_max && y > ymax) {continue;}
      if (use_z_max && z > zmax) {continue;}

      new_indices.push_back(idx);
    }

    idx_list.swap(new_indices);
  }

  return *this;
}

PointPerceptionsOpsView &
PointPerceptionsOpsView::downsample(double resolution)
{
  if (resolution <= 0.0) {
    return *this;
  }

  const double inv_res = 1.0 / resolution;

  std::unordered_set<VoxelKey, VoxelKeyHash> voxel_set;

  const std::size_t n = perceptions_.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto & pptr = perceptions_[i];
    if (!pptr || !pptr->valid || pptr->data.empty()) {
      continue;
    }

    auto & indices = indices_[i].indices;
    if (indices.size() <= 1) {continue;}

    const auto & cloud = pptr->data;

    voxel_set.clear();
    voxel_set.reserve(indices.size());

    std::size_t write_idx = 0;

    for (std::size_t read_idx = 0; read_idx < indices.size(); ++read_idx) {
      const int idx = indices[read_idx];
      if (idx < 0 || static_cast<std::size_t>(idx) >= cloud.size()) {continue;}

      const auto & pt = cloud[idx];

      const float z_val = collapse_z_ ? collapse_val_z_ : pt.z;

      VoxelKey key{
        static_cast<int>(pt.x * inv_res),
        static_cast<int>(pt.y * inv_res),
        static_cast<int>(z_val * inv_res)};

      if (voxel_set.insert(key).second) {
        indices[write_idx++] = idx;
      }
    }
    indices.resize(write_idx);
  }

  return *this;
}

PointPerceptionsOpsView &
PointPerceptionsOpsView::collapse(const std::vector<double> & collapse_dims, bool lazy)
{
  if (lazy) {
    collapse_x_ = collapse_dims.size() > 0 && !std::isnan(collapse_dims[0]);
    collapse_y_ = collapse_dims.size() > 1 && !std::isnan(collapse_dims[1]);
    collapse_z_ = collapse_dims.size() > 2 && !std::isnan(collapse_dims[2]);

    if (collapse_x_) {
      collapse_val_x_ = static_cast<float>(collapse_dims[0]);
    }
    if (collapse_y_) {
      collapse_val_y_ = static_cast<float>(collapse_dims[1]);
    }
    if (collapse_z_) {
      collapse_val_z_ = static_cast<float>(collapse_dims[2]);
    }

    return *this;
  }

  if (!owned_.has_value()) {
    RCLCPP_WARN(
      rclcpp::get_logger("PointPerceptionsOpsView"),
      "collapse(..., lazy=false) called on a non-owning view. "
      "Operation ignored.");
    return *this;
  }

  auto & container = owned_.value();

  const bool use_x = collapse_dims.size() > 0 && !std::isnan(collapse_dims[0]);
  const bool use_y = collapse_dims.size() > 1 && !std::isnan(collapse_dims[1]);
  const bool use_z = collapse_dims.size() > 2 && !std::isnan(collapse_dims[2]);

  const float vx = use_x ? static_cast<float>(collapse_dims[0]) : 0.0f;
  const float vy = use_y ? static_cast<float>(collapse_dims[1]) : 0.0f;
  const float vz = use_z ? static_cast<float>(collapse_dims[2]) : 0.0f;

  if (!use_x && !use_y && !use_z) {
    return *this;
  }

  for (auto & pptr : container) {
    if (!pptr || !pptr->valid || pptr->data.empty()) {
      continue;
    }
    auto & cloud = pptr->data;

    for (auto & pt : cloud) {
      if (use_x) {pt.x = vx;}
      if (use_y) {pt.y = vy;}
      if (use_z) {pt.z = vz;}
    }
  }

  collapse_x_ = collapse_y_ = collapse_z_ = false;

  return *this;
}

pcl::PointCloud<pcl::PointXYZ>
PointPerceptionsOpsView::as_points() const
{
  pcl::PointCloud<pcl::PointXYZ> out;

  const std::size_t n = perceptions_.size();

  std::size_t total_points = 0;
  for (std::size_t i = 0; i < n; ++i) {
    total_points += indices_[i].indices.size();
  }

  out.points.reserve(total_points);
  out.height = 1;
  out.is_dense = false;

  const bool has_tf_array =
    has_target_frame_ && (tf_valid_.size() == n) && (tf_transforms_.size() == n);

  for (std::size_t i = 0; i < n; ++i) {
    const auto & pptr = perceptions_[i];
    const auto & idx_list = indices_[i].indices;

    if (!pptr || !pptr->valid || pptr->data.empty() || idx_list.empty()) {
      continue;
    }

    const auto & cloud = pptr->data;
    const bool apply_tf = has_tf_array && tf_valid_[i];

    for (int idx : idx_list) {
      if (idx < 0 || static_cast<std::size_t>(idx) >= cloud.size()) {
        continue;
      }

      const auto & src = cloud[idx];
      tf2::Vector3 p(src.x, src.y, src.z);

      if (apply_tf) {
        p = tf_transforms_[i] * p;
      }

      double x = p.x();
      double y = p.y();
      double z = p.z();

      if (has_post_filter_) {
        if (use_post_min_[0] && x < post_min_[0]) {continue;}
        if (use_post_min_[1] && y < post_min_[1]) {continue;}
        if (use_post_min_[2] && z < post_min_[2]) {continue;}

        if (use_post_max_[0] && x > post_max_[0]) {continue;}
        if (use_post_max_[1] && y > post_max_[1]) {continue;}
        if (use_post_max_[2] && z > post_max_[2]) {continue;}
      }

      pcl::PointXYZ dst(
        static_cast<float>(x),
        static_cast<float>(y),
        static_cast<float>(z));

      if (collapse_x_) {dst.x = collapse_val_x_;}
      if (collapse_y_) {dst.y = collapse_val_y_;}
      if (collapse_z_) {dst.z = collapse_val_z_;}

      out.points.push_back(dst);
    }
  }

  out.width = static_cast<uint32_t>(out.points.size());
  return out;
}

const pcl::PointCloud<pcl::PointXYZ> &
PointPerceptionsOpsView::as_points(int idx) const
{
  tmp_single_cloud_.clear();
  tmp_single_cloud_.height = 1;
  tmp_single_cloud_.is_dense = false;

  if (idx < 0 || static_cast<std::size_t>(idx) >= perceptions_.size()) {
    tmp_single_cloud_.width = 0;
    return tmp_single_cloud_;
  }

  const std::size_t i = static_cast<std::size_t>(idx);
  const auto & pptr = perceptions_[i];
  const auto & idx_list = indices_[i].indices;

  if (!pptr || !pptr->valid || pptr->data.empty() || idx_list.empty()) {
    tmp_single_cloud_.width = 0;
    return tmp_single_cloud_;
  }

  const auto & cloud = pptr->data;

  const bool has_tf = has_target_frame_ &&
    tf_valid_.size() == perceptions_.size() &&
    tf_valid_[i];

  for (int id : idx_list) {
    if (id < 0 || static_cast<std::size_t>(id) >= cloud.size()) {
      continue;
    }

    const auto & src = cloud[id];
    tf2::Vector3 p(src.x, src.y, src.z);

    if (has_tf) {
      p = tf_transforms_[i] * p;
    }

    pcl::PointXYZ dst(
      static_cast<float>(p.x()),
      static_cast<float>(p.y()),
      static_cast<float>(p.z()));

    if (collapse_x_) {dst.x = collapse_val_x_;}
    if (collapse_y_) {dst.y = collapse_val_y_;}
    if (collapse_z_) {dst.z = collapse_val_z_;}

    tmp_single_cloud_.points.push_back(dst);
  }

  tmp_single_cloud_.width =
    static_cast<uint32_t>(tmp_single_cloud_.points.size());
  return tmp_single_cloud_;
}


PointPerceptionsOpsView &
PointPerceptionsOpsView::fuse(const std::string & target_frame, bool exact_time)
{
  rclcpp::Time unused_stamp;
  return fuse(target_frame, unused_stamp, exact_time);
}

PointPerceptionsOpsView &
PointPerceptionsOpsView::fuse(
  const std::string & target_frame, rclcpp::Time & stamp,
  bool exact_time)
{
  auto update_latest_stamp = [&](const rclcpp::Time & candidate) {
      if (candidate.nanoseconds() == 0) {
        return;
      }

      if (stamp.nanoseconds() == 0) {
        stamp = rclcpp::Time(candidate.nanoseconds(), candidate.get_clock_type());
        return;
      }

      if (stamp.get_clock_type() == candidate.get_clock_type()) {
        if (candidate > stamp) {
          stamp = candidate;
        }
        return;
      }

      // Mixed clock types: avoid throwing. We cannot strictly order times from
      // different time sources, but we still want a stable “latest” stamp output.
      if (candidate.nanoseconds() > stamp.nanoseconds()) {
        stamp = rclcpp::Time(candidate.nanoseconds(), stamp.get_clock_type());
      }
    };

  auto allow_backtrace_now = []() {
      static std::chrono::steady_clock::time_point last_bt =
        std::chrono::steady_clock::time_point{};

      const auto now = std::chrono::steady_clock::now();
      constexpr auto min_period = std::chrono::seconds(5);

      if (last_bt.time_since_epoch().count() == 0 || (now - last_bt) > min_period) {
        last_bt = now;
        return true;
      }
      return false;
    };

  has_target_frame_ = true;
  target_frame_ = target_frame;

  const std::size_t n = perceptions_.size();
  if (tf_transforms_.size() != n) {
    tf_transforms_.resize(n);
  }
  if (tf_valid_.size() != n) {
    tf_valid_.assign(n, false);
  }

  auto tf_buffer = easynav::RTTFBuffer::getInstance();

  for (std::size_t i = 0; i < n; ++i) {
    auto & pptr = perceptions_[i];
    if (!pptr) {
      tf_valid_[i] = false;
      continue;
    }

    pptr->integrate_pending_perceptions();

    if (!pptr->valid || pptr->data.empty()) {
      tf_valid_[i] = false;
      continue;
    }

    if (pptr->frame_id == target_frame_) {
      tf_valid_[i] = false;
      continue;
    }

    try {
      bool used_fallback_latest_tf = false;
      const auto query_time = exact_time ? tf2_ros::fromMsg(pptr->stamp) : tf2::TimePointZero;

      geometry_msgs::msg::TransformStamped tf_msg;
      try {
        tf_msg = tf_buffer->lookupTransform(
          target_frame_, pptr->frame_id,
          query_time,
          tf2::durationFromSec(0.0));
      } catch (const tf2::TransformException & ex) {
        // Common in RT loops: exact-time request is a few ms ahead of the latest TF.
        // Fall back to latest TF rather than dropping the perception.
        if (exact_time) {
          const std::string what = ex.what();
          if (what.find("extrapolation") != std::string::npos &&
            what.find("future") != std::string::npos)
          {
            tf_msg = tf_buffer->lookupTransform(
              target_frame_, pptr->frame_id,
              tf2::TimePointZero,
              tf2::durationFromSec(0.0));
            used_fallback_latest_tf = true;
          } else {
            throw;
          }
        } else {
          throw;
        }
      }

      if (exact_time && !used_fallback_latest_tf) {
        update_latest_stamp(pptr->stamp);
      } else {
        update_latest_stamp(rclcpp::Time(tf_msg.header.stamp, pptr->stamp.get_clock_type()));
      }

      tf2::fromMsg(tf_msg.transform, tf_transforms_[i]);
      tf_valid_[i] = true;
    } catch (const tf2::TransformException & ex) {
      auto logger = rclcpp::get_logger("PointPerceptionsOpsView");

      if (allow_backtrace_now()) {
        const std::string bt = backtrace_to_string(64, 1);
        if (!bt.empty()) {
          RCLCPP_WARN(
            logger,
            "TF lookup failed in fuse(): %s\nBacktrace:\n%s",
            ex.what(), bt.c_str());
        } else {
          RCLCPP_WARN(logger, "TF lookup failed in fuse(): %s", ex.what());
        }
      } else {
        RCLCPP_WARN(logger, "TF lookup failed in fuse(): %s", ex.what());
      }
      tf_valid_[i] = false;
    }
  }

  return *this;
}

PointPerceptionsOpsView &
PointPerceptionsOpsView::add(
  const pcl::PointCloud<pcl::PointXYZ> points,
  const std::string & frame,
  rclcpp::Time stamp)
{
  if (!owned_.has_value()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PointPerceptionsOpsView"),
      "add() called on a view that does not own the underlying container.");
    return *this;
  }

  auto & container = owned_.value();

  auto p = std::make_shared<PointPerception>();
  p->valid = true;
  p->frame_id = frame;
  p->stamp = stamp;
  p->data = points;

  container.push_back(p);

  indices_.push_back(pcl::PointIndices{});
  auto & idx = indices_.back().indices;
  idx.resize(points.size());
  std::iota(idx.begin(), idx.end(), 0);

  tf_transforms_.push_back(tf2::Transform());
  tf_valid_.push_back(false);

  return *this;
}

rclcpp::Time get_latest_point_perceptions_stamp(const PointPerceptions & perceptions)
{
  auto is_newer = [](const rclcpp::Time & a, const rclcpp::Time & b) {
      if (a.get_clock_type() == b.get_clock_type()) {
        return a > b;
      }
      // Fall back to raw nanoseconds ordering to avoid throwing when clocks differ.
      return a.nanoseconds() > b.nanoseconds();
    };

  rclcpp::Time latest_stamp;
  bool inited = false;

  for (const auto & perception : perceptions) {
    if (!inited || is_newer(perception->stamp, latest_stamp)) {
      latest_stamp = perception->stamp;
      inited = true;
    }
  }
  return latest_stamp;
}

rclcpp::Time
PointPerceptionsOpsView::get_latest_stamp() const
{
  return get_latest_point_perceptions_stamp(perceptions_);
}

}  // namespace easynav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(easynav::PointPerceptionHandler, easynav::PerceptionHandler)
