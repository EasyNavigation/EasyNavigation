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
/// \brief Implementation of the configuration fingerprint.

#include <openssl/evp.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>

#include "rcl_logging_interface/rcl_logging_interface.h"
#include "rcutils/allocator.h"

#include "easynav_system/safety/ConfigurationFingerprint.hpp"

namespace easynav::safety
{

std::string
configuration_dump(const Nodes & nodes)
{
  std::ostringstream dump;
  for (const auto & [name, node] : nodes) {
    auto names = node->list_parameters({}, 0).names;  // 0: any depth
    std::sort(names.begin(), names.end());
    for (const auto & parameter : node->get_parameters(names)) {
      dump << name << "/" << parameter.get_name() << "=" << parameter.value_to_string() << "\n";
    }
  }
  return dump.str();
}

std::string
sha256_hex(const std::string & data)
{
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (EVP_Digest(data.data(), data.size(), digest, &length, EVP_sha256(), nullptr) != 1) {
    return "";
  }
  std::ostringstream hex;
  for (unsigned int i = 0; i < length; ++i) {
    hex << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
  }
  return hex.str();
}

std::string
loaded_plugins(const Nodes & nodes)
{
  std::ostringstream plugins;
  for (const auto & [name, node] : nodes) {
    for (const auto & types : node->list_parameters({}, 0).names) {
      if (types.size() < 6 || types.compare(types.size() - 6, 6, "_types") != 0 ||
        node->get_parameter(types).get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
      {
        continue;
      }
      const auto aliases = node->get_parameter(types).as_string_array();
      for (const auto & alias : aliases) {
        std::string plugin;
        node->get_parameter(alias + ".plugin", plugin);
        plugins << "\n  " << name << ": " << alias << " [" << plugin << "]";
      }
    }
  }
  return plugins.str();
}

std::string
log_directory()
{
  const auto allocator = rcutils_get_default_allocator();
  char * directory = nullptr;
  if (rcl_logging_get_logging_directory(allocator, &directory) != RCL_LOGGING_RET_OK) {
    return "";
  }
  std::string ret(directory);
  allocator.deallocate(directory, allocator.state);
  return ret;
}

std::string
dump_file_name(const std::string & ns, const std::string & hash)
{
  std::string prefix = ns;
  prefix.erase(0, prefix.find_first_not_of('/'));
  std::replace(prefix.begin(), prefix.end(), '/', '_');
  return "easynav_configuration_" + (prefix.empty() ? "" : prefix + "_") + hash + ".txt";
}

std::string
save_dump(const std::string & dump, const std::string & path)
{
  namespace fs = std::filesystem;
  std::error_code error;
  if (fs::exists(path, error)) {
    return "";  // Same hash: same contents.
  }
  fs::create_directories(fs::path(path).parent_path(), error);
  if (error) {
    return error.message();
  }

  // Written aside and renamed: never a half-written file under the final name.
  const auto tmp = path + ".tmp";
  {
    std::ofstream file(tmp);
    file << dump;
    if (!file) {
      return "unable to write " + tmp;
    }
  }
  fs::rename(tmp, path, error);
  if (error) {
    fs::remove(tmp, error);
    return "unable to write " + path;
  }
  return "";
}

}  // namespace easynav::safety
