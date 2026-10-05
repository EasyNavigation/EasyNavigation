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
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "rcl_logging_interface/rcl_logging_interface.h"
#include "rcutils/allocator.h"

#include "easynav_system/safety/ConfigurationFingerprint.hpp"

namespace easynav::safety
{

namespace
{

std::string quote(const std::string & text)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : text) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20 || c == 0x7f) {
          out << "\\x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) <<
            std::dec;
        } else {
          out << c;
        }
    }
  }
  out << '"';
  return out.str();
}

// The shortest text that reads back as the same double: distinct doubles never print the same.
std::string exact(double value)
{
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  return std::string(buffer, result.ptr);
}

template<typename T, typename F>
std::string list(const std::vector<T> & values, F format)
{
  std::string out = "[";
  for (size_t i = 0; i < values.size(); ++i) {
    out += (i > 0 ? ", " : "") + format(values[i]);
  }
  return out + "]";
}

const char * type_name(rclcpp::ParameterType type)
{
  switch (type) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return "bool";
    case rclcpp::ParameterType::PARAMETER_INTEGER: return "integer";
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return "double";
    case rclcpp::ParameterType::PARAMETER_STRING: return "string";
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: return "byte_array";
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: return "bool_array";
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: return "integer_array";
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: return "double_array";
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: return "string_array";
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default: return "not_set";
  }
}

std::string value_of(const rclcpp::Parameter & parameter)
{
  const auto boolean = [](bool b) {return std::string(b ? "true" : "false");};
  const auto integer = [](auto i) {return std::to_string(static_cast<int64_t>(i));};
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return boolean(parameter.as_bool());
    case rclcpp::ParameterType::PARAMETER_INTEGER: return integer(parameter.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return exact(parameter.as_double());
    case rclcpp::ParameterType::PARAMETER_STRING: return quote(parameter.as_string());
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return list(parameter.as_byte_array(), integer);
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return list(parameter.as_bool_array(), boolean);
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return list(parameter.as_integer_array(), integer);
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return list(parameter.as_double_array(), exact);
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return list(parameter.as_string_array(), quote);
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default: return "";
  }
}

}  // namespace

std::string
configuration_dump(const Nodes & nodes)
{
  // Names cannot hold spaces, parentheses, '=' or newlines, and values are typed, exact and
  // quoted: two different configurations never give the same text.
  std::ostringstream dump;
  for (const auto & [name, node] : nodes) {
    auto names = node->list_parameters({}, 0).names;  // 0: any depth
    std::sort(names.begin(), names.end());
    for (const auto & parameter_name : names) {
      // Declared with a type but no value (legitimate): reading it throws.
      rclcpp::Parameter parameter;
      try {
        parameter = node->get_parameter(parameter_name);
      } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
        dump << name << "/" << parameter_name << " (uninitialized)\n";
        continue;
      }
      dump << name << "/" << parameter.get_name() << " (" << type_name(parameter.get_type()) <<
        ") = " << value_of(parameter) << "\n";
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
      if (types.size() < 6 || types.compare(types.size() - 6, 6, "_types") != 0) {
        continue;
      }
      rclcpp::Parameter types_parameter;
      try {
        types_parameter = node->get_parameter(types);
      } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
        continue;
      }
      if (types_parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
        continue;
      }
      const auto aliases = types_parameter.as_string_array();
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
