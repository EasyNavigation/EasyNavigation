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
/// \brief Implementation of the DummyLocalizer class.

#include "easynav_localizer/DummyLocalizer.hpp"

#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

void DummyLocalizer::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".cycle_time_rt", 0.0);
  node->declare_parameter<double>(plugin_name + ".cycle_time_nort", 0.0);
  node->get_parameter<double>(plugin_name + ".cycle_time_rt", cycle_time_rt_);
  node->get_parameter<double>(plugin_name + ".cycle_time_nort", cycle_time_nort_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
}

void DummyLocalizer::update_rt([[maybe_unused]] NavState & nav_state)
{
  namespace chr = std::chrono;
  auto start = chr::steady_clock::now();

  const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = get_node()->now();
  tf_msg.header.frame_id = tf_info.map_frame;
  tf_msg.child_frame_id = tf_info.odom_frame;

  RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
  // tf_broadcaster_->sendTransform(tf_msg);

  // Busy wait to simulate processing time
  while (chr::duration<double>(chr::steady_clock::now() - start).count() < cycle_time_rt_) {}
}

void DummyLocalizer::update([[maybe_unused]] NavState & nav_state)
{
  namespace chr = std::chrono;
  auto start = chr::steady_clock::now();

  const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = get_node()->now();
  tf_msg.header.frame_id = tf_info.map_frame;
  tf_msg.child_frame_id = tf_info.odom_frame;

  RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
  // tf_broadcaster_->sendTransform(tf_msg);

  // Busy wait to simulate processing time
  while (chr::duration<double>(chr::steady_clock::now() - start).count() < cycle_time_nort_) {}
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyLocalizer, easynav::LocalizerMethodBase)
