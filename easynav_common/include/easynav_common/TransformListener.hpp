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
/// \brief make_transform_listener(): a node-attached TransformListener on every ROS 2 distro.

#ifndef EASYNAV_COMMON__TRANSFORMLISTENER_HPP_
#define EASYNAV_COMMON__TRANSFORMLISTENER_HPP_

#include <memory>
#include <type_traits>

#include "tf2/buffer_core.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace easynav
{

namespace detail
{

// tf2_ros from Lyrical takes NodeInterfaces (TransformListener::RequiredInterfaces)
template<class T, class = void>
struct has_required_interfaces : std::false_type {};

template<class T>
struct has_required_interfaces<T, std::void_t<typename T::RequiredInterfaces>>
  : std::true_type {};

}  // namespace detail

/**
 * @brief Creates a TransformListener attached to @p node.
 *
 * Up to Kilted, tf2_ros takes the node as a pointer; from Lyrical, as NodeInterfaces
 * (the pointer form is deprecated in Lyrical and removed in Rolling).
 */
template<class NodeT, class ListenerT = tf2_ros::TransformListener>
std::unique_ptr<ListenerT> make_transform_listener(
  tf2::BufferCore & buffer, const std::shared_ptr<NodeT> & node, bool spin_thread = true)
{
  if constexpr (detail::has_required_interfaces<ListenerT>::value) {
    return std::make_unique<ListenerT>(buffer, *node, spin_thread);
  } else {
    return std::make_unique<ListenerT>(buffer, node, spin_thread);
  }
}

}  // namespace easynav

#endif  // EASYNAV_COMMON__TRANSFORMLISTENER_HPP_
