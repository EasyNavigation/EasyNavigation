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
/// \brief get_package_share_path(): a package's share directory on every ROS 2 distro.

#ifndef EASYNAV_COMMON__PACKAGESHARE_HPP_
#define EASYNAV_COMMON__PACKAGESHARE_HPP_

#include <filesystem>
#include <string>

// Humble only has get_package_share_directory(); Rolling only get_package_share_path()
#if __has_include("ament_index_cpp/get_package_share_path.hpp")
#include "ament_index_cpp/get_package_share_path.hpp"
#else
#include "ament_index_cpp/get_package_share_directory.hpp"
#endif

namespace easynav
{

/**
 * @brief Returns the share directory of @p package_name.
 * @throws ament_index_cpp::PackageNotFoundError if the package is not found.
 */
inline std::filesystem::path get_package_share_path(const std::string & package_name)
{
#if __has_include("ament_index_cpp/get_package_share_path.hpp")
  return ament_index_cpp::get_package_share_path(package_name);
#else
  return ament_index_cpp::get_package_share_directory(package_name);
#endif
}

}  // namespace easynav

#endif  // EASYNAV_COMMON__PACKAGESHARE_HPP_
