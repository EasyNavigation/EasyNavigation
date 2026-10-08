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

/// \file
/// \brief Parameter helpers for plugins that may be initialized several times on the same node.

#ifndef EASYNAV_COMMON__PARAMETERS_HPP_
#define EASYNAV_COMMON__PARAMETERS_HPP_

#include <string>

namespace easynav
{

/**
 * @brief Declares \p name with \p default_value unless it already exists.
 *
 * A plugin may be initialized again on the same node (EasyNav reconfigured), possibly after a
 * plugin of another type under the same name that shared some of its parameters: each parameter
 * must be checked on its own, never assumed to be declared along with others.
 */
template<typename T, typename NodeT>
void declare_parameter_if_absent(NodeT & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, default_value);
  }
}

}  // namespace easynav

#endif  // EASYNAV_COMMON__PARAMETERS_HPP_
