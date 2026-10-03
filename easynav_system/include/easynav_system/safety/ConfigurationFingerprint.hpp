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
/// \brief Fingerprint (dump and SHA-256) of the EasyNav configuration.

#ifndef EASYNAV_SYSTEM__SAFETY__CONFIGURATIONFINGERPRINT_HPP_
#define EASYNAV_SYSTEM__SAFETY__CONFIGURATIONFINGERPRINT_HPP_

#include <map>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav::safety
{

/// @brief EasyNav nodes by name.
using Nodes = std::map<std::string, rclcpp_lifecycle::LifecycleNode::SharedPtr>;

/// @brief Every parameter of \p nodes, one "node/parameter=value" per line, sorted.
std::string configuration_dump(const Nodes & nodes);

/// @brief SHA-256 of \p data, in hex.
std::string sha256_hex(const std::string & data);

/// @brief The loaded plugins: every "<alias>.plugin" listed in a "*_types" parameter, one
/// "\n  node: alias [class]" each.
std::string loaded_plugins(const Nodes & nodes);

/// @brief ROS's log directory ($ROS_LOG_DIR, else $ROS_HOME/log, else ~/.ros/log), or "".
std::string log_directory();

/// @brief "easynav_configuration_[<ns>_]<hash>.txt", for the EasyNav in namespace \p ns.
std::string dump_file_name(const std::string & ns, const std::string & hash);

/// @brief Writes \p dump to \p path, unless it already exists.
/// @return "" on success, otherwise why not.
std::string save_dump(const std::string & dump, const std::string & path);

}  // namespace easynav::safety

#endif  // EASYNAV_SYSTEM__SAFETY__CONFIGURATIONFINGERPRINT_HPP_
