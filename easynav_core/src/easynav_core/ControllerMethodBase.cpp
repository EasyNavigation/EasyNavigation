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
/// \brief Implementation of the abstract base class ControllerMethodBase.

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/Parameters.hpp"
#include "easynav_common/types/NavState.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/MethodBase.hpp"
#include "easynav_core/ControllerMethodBase.hpp"

#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

void
ControllerMethodBase::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name)
{
  MethodBase::initialize(parent_node, plugin_name);
}

RobotLimits
ControllerMethodBase::get_robot_limits(const LegacyRobotLimitNames & legacy)
{
  auto * provider = dynamic_cast<RobotLimitsProvider *>(get_node().get());
  RobotLimits limits = provider ? provider->get_robot_limits() : RobotLimits{};

  auto apply = [&](const std::string & name, const std::string & field, double & value) {
      if (name.empty()) {
        return;
      }
      const auto replacement = "controller_node.robot_limits." + field;
      if (provider && provider->is_robot_limit_configured(field)) {
        double ignored = value;
        if (read_deprecated_parameter(name, ignored)) {
          RCLCPP_WARN(
            get_node()->get_logger(), "[%s] '%s.%s' is deprecated and ignored: '%s' takes "
            "precedence", get_plugin_name().c_str(), get_plugin_name().c_str(), name.c_str(),
            replacement.c_str());
        }
        return;
      }
      get_deprecated_parameter(name, replacement, value);
    };
  apply(legacy.max_linear_vel, "max_linear_vel", limits.max_linear_vel);
  apply(legacy.min_linear_vel, "min_linear_vel", limits.min_linear_vel);
  apply(legacy.max_angular_vel, "max_angular_vel", limits.max_angular_vel);
  apply(legacy.max_linear_acc, "max_linear_acc", limits.max_linear_acc);
  apply(legacy.max_linear_decel, "max_linear_decel", limits.max_linear_decel);
  apply(legacy.max_angular_acc, "max_angular_acc", limits.max_angular_acc);
  apply(legacy.max_angular_decel, "max_angular_decel", limits.max_angular_decel);

  if (provider) {
    provider->set_robot_limits(limits);  // What the smoother enforces too.
  }
  return limits;
}

bool
ControllerMethodBase::get_deprecated_parameter(
  const std::string & name, const std::string & replacement, double & value)
{
  if (!read_deprecated_parameter(name, value)) {
    return false;
  }
  RCLCPP_WARN(
    get_node()->get_logger(),
    "[%s] '%s.%s' is deprecated: configure '%s' instead. It will stop working soon.",
    get_plugin_name().c_str(), get_plugin_name().c_str(), name.c_str(), replacement.c_str());
  return true;
}

bool
ControllerMethodBase::read_deprecated_parameter(const std::string & name, double & value)
{
  auto node = get_node();
  const auto full_name = get_plugin_name() + "." + name;
  // Configured: given in the parameters (files/overrides), or declared by a previous instance.
  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.count(full_name) == 0 && !node->has_parameter(full_name)) {
    return false;
  }
  declare_parameter_if_absent(*node, full_name, value);
  node->get_parameter(full_name, value);
  return true;
}

bool
ControllerMethodBase::internal_update_rt(NavState & nav_state, bool trigger)
{
  report_rt_rate(nav_state);
  if (isTime2RunRT() || trigger) {
    EASYNAV_TRACE_EVENT;

    // Save last execution time, even if triggered
    setRunRT();

    bool failed = false;
    try {
      update_rt(nav_state);
    } catch (const std::exception & e) {
      // A faulty plugin must not bring down EasyNav.
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Exception in update_rt() of controller [%s]: %s", get_plugin_name().c_str(), e.what());
      failed = true;
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Unknown exception in update_rt() of controller [%s]", get_plugin_name().c_str());
      failed = true;
    }

    if (failed) {
      // No valid command: stop the robot rather than resend the last one.
      geometry_msgs::msg::TwistStamped zero_speed;
      zero_speed.header.stamp = get_node()->now();
      zero_speed.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
      nav_state.set("cmd_vel", zero_speed);
      return true;
    }

    return true;
  } else {
    return false;
  }
}

}  // namespace easynav
