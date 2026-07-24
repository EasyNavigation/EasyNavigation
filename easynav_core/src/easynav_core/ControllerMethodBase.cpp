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
/// \brief Implementation of the abstract base class ControllerMethodBase.

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/MethodBase.hpp"
#include "easynav_core/ControllerMethodBase.hpp"

#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

void
ControllerMethodBase::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name)
{
  auto node = parent_node;

  collision_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "collision_area", 10);

  node->declare_parameter("colision_checker.active", collision_checker_active_);
  node->declare_parameter("colision_checker.debug_markers", debug_markers_);
  node->declare_parameter("colision_checker.robot_radius", robot_radius_);
  node->declare_parameter("colision_checker.robot_height", robot_height_);
  node->declare_parameter("colision_checker.brake_acc", brake_acc_);
  node->declare_parameter("colision_checker.safety_margin", safety_margin_);
  node->declare_parameter("colision_checker.z_min_filter", z_min_filter_);
  node->declare_parameter("colision_checker.downsample_leaf_size", downsample_leaf_size_);

  node->get_parameter("colision_checker.active", collision_checker_active_);
  node->get_parameter("colision_checker.debug_markers", debug_markers_);
  node->get_parameter("colision_checker.robot_radius", robot_radius_);
  node->get_parameter("colision_checker.robot_height", robot_height_);
  node->get_parameter("colision_checker.brake_acc", brake_acc_);
  node->get_parameter("colision_checker.safety_margin", safety_margin_);
  node->get_parameter("colision_checker.z_min_filter", z_min_filter_);
  node->get_parameter("colision_checker.downsample_leaf_size", downsample_leaf_size_);

  MethodBase::initialize(parent_node, plugin_name);
}

bool
ControllerMethodBase::internal_update_rt(NavState & nav_state, bool trigger)
{
  if (isTime2RunRT() || trigger) {
    EASYNAV_TRACE_EVENT;

    // Save last execution time, even if triggered
    setRunRT();

    update_rt(nav_state);

    if (collision_checker_active_ && is_inminent_collision(nav_state)) {
      on_inminent_collision(nav_state);
    }

    return true;
  } else {
    return false;
  }
}

void
ControllerMethodBase::on_inminent_collision(NavState & nav_state)
{

  RCLCPP_WARN_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "ControllerMethodBase::on_inminent_collision: Inminent collision!! Stopping");

  geometry_msgs::msg::TwistStamped zero_speed;
  zero_speed.header.stamp = collision_stamp_;
  zero_speed.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;

  nav_state.set("cmd_vel", zero_speed);
}

bool
ControllerMethodBase::is_inminent_collision(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;
  bool imminent = false;

  if (!nav_state.has("cmd_vel")) {return false;}

  const auto & perceptions = nav_state.get_by_type<PointPerception>();
  if (perceptions.empty()) {
    return false;
  }

  const auto & twist = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
  const auto & robot_frame = tf_info.robot_frame;

  if (perceptions.empty()) {return false;}

  const double vx = twist.twist.linear.x;
  const double vy = twist.twist.linear.y;
  const double wz = twist.twist.angular.z;
  const double v_norm = std::sqrt(vx * vx + vy * vy);

  const double a_brake = std::max(brake_acc_, 1e-3);
  // const double d_stop = (v_norm * v_norm) / (2.0 * a_brake) + safety_margin_;
  const double t_stop = v_norm / a_brake;

  std::vector<double> min({
      static_cast<double>(-robot_radius_ - safety_margin_),
      static_cast<double>(-robot_radius_ - safety_margin_),
      static_cast<double>(z_min_filter_)});
  std::vector<double> max({
      static_cast<double>(robot_radius_ + safety_margin_ +
      std::max(0.0, v_norm * v_norm / (2.0 * std::max(brake_acc_, 1e-3)))),
      static_cast<double>(robot_radius_ + safety_margin_),
      static_cast<double>(robot_height_)});


  auto view = PointPerceptionsOpsView(perceptions);
  view.downsample(downsample_leaf_size_)
  .filter({-2.0, -2.0, -2.0}, {2.0, 2.0, 2.0}, false)
  .fuse(robot_frame)
  .filter(min, max);

  collision_stamp_ = view.get_latest_stamp();
  const auto & cloud = view.as_points();

  geometry_msgs::msg::Pose base_pose;
  base_pose.orientation.w = 1.0;

  const double r = robot_radius_;
  // const double dx = vx / v_norm;
  // const double dy = vy / v_norm;
  const double r_sq = r * r;
  // const double x_max = d_stop + r;

  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}

    const double px = p.x;
    const double py = p.y;

    const double v_rel_x = -vx + wz * py;
    const double v_rel_y = -vy - wz * px;

    const double v_rel_sq = v_rel_x * v_rel_x + v_rel_y * v_rel_y;
    if (v_rel_sq < 1e-8) {continue;}

    const double dot = px * v_rel_x + py * v_rel_y;
    const double t_star = -dot / v_rel_sq;

    if (t_star < 0.0) {continue;}
    if (t_star > t_stop) {continue;}

    const double cx = px + v_rel_x * t_star;
    const double cy = py + v_rel_y * t_star;
    const double d_min_sq = cx * cx + cy * cy;

    if (d_min_sq <= r_sq) {
      imminent = true;
      publish_collision_zone_marker(min, max, cloud, imminent, collision_stamp_);

      return true;
    }
  }

  publish_collision_zone_marker(min, max, cloud, imminent, collision_stamp_);
  return imminent;
}


void
ControllerMethodBase::publish_collision_zone_marker(
  const std::vector<double> & min,
  const std::vector<double> & max,
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  bool imminent_collision,
  const rclcpp::Time & stamp)
{
  if (!debug_markers_) {return;}
  if (!collision_marker_pub_) {return;}

  visualization_msgs::msg::MarkerArray array;

  const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();
  const auto & robot_frame = tf_info.robot_frame;

  {
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = robot_frame;
    clear.header.stamp = stamp;
    clear.ns = "collision_zone";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
  }

  std_msgs::msg::ColorRGBA color;
  color.r = imminent_collision ? 1.0f : 0.0f;
  color.g = imminent_collision ? 0.0f : 1.0f;
  color.b = 0.0f;
  color.a = 0.25f;

  {
    visualization_msgs::msg::Marker box;
    box.header.frame_id = robot_frame;
    box.header.stamp = stamp;
    box.ns = "collision_zone";
    box.id = 1;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;

    const double cx = 0.5 * (min[0] + max[0]);
    const double cy = 0.5 * (min[1] + max[1]);
    const double cz = 0.5 * (min[2] + max[2]);

    const double sx = (max[0] - min[0]);
    const double sy = (max[1] - min[1]);
    const double sz = (max[2] - min[2]);

    box.pose.position.x = cx;
    box.pose.position.y = cy;
    box.pose.position.z = cz;
    box.pose.orientation.w = 1.0;

    box.scale.x = sx;
    box.scale.y = sy;
    box.scale.z = sz;

    box.color = color;
    box.lifetime = rclcpp::Duration(0, 200 * 1000000);  // 0.2s

    array.markers.push_back(box);
  }

  {
    visualization_msgs::msg::Marker pts;
    pts.header.frame_id = robot_frame;
    pts.header.stamp = stamp;
    pts.ns = "collision_zone";
    pts.id = 2;
    pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::msg::Marker::ADD;

    pts.pose.orientation.w = 1.0;

    const float point_scale = 0.03f;
    pts.scale.x = point_scale;
    pts.scale.y = point_scale;
    pts.scale.z = point_scale;

    pts.color = color;
    pts.lifetime = rclcpp::Duration(0, 200 * 1000000);

    pts.points.reserve(cloud.points.size());
    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      pts.points.push_back(gp);
    }

    array.markers.push_back(pts);
  }

  collision_marker_pub_->publish(array);
}

}  // namespace easynav
