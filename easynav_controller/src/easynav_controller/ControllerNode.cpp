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
/// \brief Implementation of the ControllerNode class.

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "easynav_controller/ControllerNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

ControllerNode::ControllerNode(
  const rclcpp::NodeOptions & options)
: LifecycleNode("controller_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  controller_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::ControllerMethodBase>>(
    "easynav_core", "easynav::ControllerMethodBase");

  on_set_parameters_callback_handle_ = add_on_set_parameters_callback(std::bind(
      &ControllerNode::on_set_parameters, this, std::placeholders::_1));

  NavState::register_printer<geometry_msgs::msg::TwistStamped>(
    [](const geometry_msgs::msg::TwistStamped & twist) {
      std::ostringstream ret;

      ret << "{ " << rclcpp::Time(twist.header.stamp).seconds() << "} Twist with (" <<
        twist.twist.linear.x << ", " <<
        twist.twist.linear.y << ", " <<
        twist.twist.linear.z << ") (" << twist.twist.angular.x << ", " <<
        twist.twist.angular.y << ", " << twist.twist.angular.z << ")";

      return ret.str();
    });
}

ControllerNode::~ControllerNode()
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }

  controller_method_ = nullptr;
  std::vector<std::string> controller_types;
  get_parameter("controller_types", controller_types);
  for (const auto & controller_type : controller_types) {
    std::string plugin;
    if (has_parameter(controller_type + ".plugin")) {
      get_parameter(controller_type + ".plugin", plugin);
      try {
        controller_loader_->unloadLibraryForClass(plugin);
      } catch (const std::exception &) {
      }
    }
  }
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

std::shared_ptr<ControllerMethodBase>
ControllerNode::create_and_initialize_controller(
  const std::string & controller_type, const std::string & plugin)
{
  try {
    RCLCPP_INFO(get_logger(), "Loading ControllerMethodBase %s [%s]",
                controller_type.c_str(), plugin.c_str());

    auto instance = controller_loader_->createSharedInstance(plugin);
    instance->initialize(shared_from_this(), controller_type);

    RCLCPP_INFO(get_logger(), "Loaded ControllerMethodBase %s [%s]",
                controller_type.c_str(), plugin.c_str());

    return instance;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to load controller [%s] plugin [%s]. Error: %s",
                 controller_type.c_str(), plugin.c_str(), e.what());
    return nullptr;
  }
}

rcl_interfaces::msg::SetParametersResult ControllerNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const rclcpp::Parameter *controller_types_parameter = nullptr;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "controller_types") {
      controller_types_parameter = &parameter;
      break;
    }
  }

  // No controller_types change.
  if (controller_types_parameter == nullptr) {
    return result;
  }

  // Initial declaration during on_configure(): on_configure() itself loads
  // the controller directly, so skip the pending/inactive validation here.
  if (configuring_) {
    return result;
  }

  // Runtime changes are only allowed while inactive.
  if (get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    result.successful = false;
    result.reason =
      "controller_types can only be changed while the node is inactive";
    return result;
  }

  const auto new_types = controller_types_parameter->as_string_array();

  if (new_types.size() > 1) {
    result.successful = false;
    result.reason = "You must instance one controller. [" +
      std::to_string(new_types.size()) + "] requested";
    return result;
  }

  const std::string new_controller_type =
    new_types.empty() ? "" : new_types.front();

  // If a controller was requested, check that its plugin parameter exists.
  if (!new_controller_type.empty()) {
    const std::string plugin_parameter = new_controller_type + ".plugin";

    if (!has_parameter(plugin_parameter)) {
      result.successful = false;
      result.reason =
        "Parameter [" + plugin_parameter +
        "] must be declared before switching controller_types to [" +
        new_controller_type + "]";
      return result;
    }

    std::string plugin;
    get_parameter(plugin_parameter, plugin);

    if (plugin.empty()) {
      result.successful = false;
      result.reason = "Parameter [" + plugin_parameter + "] is empty";
      return result;
    }
  }

  // Only store the requested controller
  // The actual plugin creation is performed in on_activate()
  {
    std::lock_guard<std::mutex> lock(pending_controller_mutex_);

    pending_controller_type_ = new_controller_type;
    controller_change_pending_ = true;
  }

  return result;
}

CallbackReturnT ControllerNode::on_configure(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  configuring_ = true;

  std::vector<std::string> controller_types;

  if (!has_parameter("controller_types")) {
    declare_parameter("controller_types", controller_types);
  }

  get_parameter("controller_types", controller_types);

  if (controller_types.size() > 1) {
    RCLCPP_ERROR(get_logger(), "You must instance one controller. [%lu] found",
                 controller_types.size());

    configuring_ = false;
    return CallbackReturnT::FAILURE;
  }

  const auto parameter_overrides =
    get_node_parameters_interface()->get_parameter_overrides();

  for (const auto &[name, value] : parameter_overrides) {
    static const std::string suffix = ".plugin";

    if (name.size() > suffix.size() &&
      name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0 &&
      !has_parameter(name))
    {
      declare_parameter(name, value);
    }
  }
  for (const auto & controller_type : controller_types) {
    std::string plugin;
    if (!has_parameter(controller_type + ".plugin")) {
      declare_parameter(controller_type + std::string(".plugin"), plugin);
    }
    get_parameter(controller_type + std::string(".plugin"), plugin);

    auto instance = create_and_initialize_controller(controller_type, plugin);
    if (!instance) {
      configuring_ = false;
      return CallbackReturnT::FAILURE;
    }

    {
      std::lock_guard<std::mutex> lock(controller_method_mutex_);
      controller_method_ = instance;
    }
  }

  configuring_ = false;
  return CallbackReturnT::SUCCESS;
}

void ControllerNode::apply_pending_controller()
{
  std::string controller_type;

  {
    std::lock_guard<std::mutex> lock(pending_controller_mutex_);

    if (!controller_change_pending_) {
      return;
    }

    controller_type = pending_controller_type_;
  }

  // No controller selected.
  if (controller_type.empty()) {
    {
      std::lock_guard<std::mutex> lock(controller_method_mutex_);
      controller_method_ = nullptr;
    }

    std::lock_guard<std::mutex> lock(pending_controller_mutex_);
    controller_change_pending_ = false;

    RCLCPP_INFO(get_logger(), "No controller selected");

    return;
  }

  const std::string plugin_parameter = controller_type + ".plugin";

  std::string plugin;
  get_parameter(plugin_parameter, plugin);

  // Plugin creation and initialization happen outside the
  // parameter callback and outside the RT cycle.
  auto instance = create_and_initialize_controller(controller_type, plugin);

  if (!instance) {
    RCLCPP_ERROR(get_logger(), "Unable to switch to controller [%s]",
                 controller_type.c_str());

    return;
  }

  // Only the shared_ptr replacement is protected.
  {
    std::lock_guard<std::mutex> lock(controller_method_mutex_);
    controller_method_ = instance;
  }

  {
    std::lock_guard<std::mutex> lock(pending_controller_mutex_);
    controller_change_pending_ = false;
  }

  RCLCPP_INFO(get_logger(), "Controller switched to [%s]",
              controller_type.c_str());
}

CallbackReturnT ControllerNode::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  apply_pending_controller();

  {
    std::lock_guard<std::mutex> lock(pending_controller_mutex_);

    if (controller_change_pending_) {
      RCLCPP_ERROR(
          get_logger(),
          "Unable to activate: controller change could not be applied");

      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  {
    std::lock_guard<std::mutex> lock(controller_method_mutex_);
    controller_method_ = nullptr;
  }

  std::vector<std::string> controller_types;
  get_parameter("controller_types", controller_types);
  for (const auto & controller_type : controller_types) {
    if (has_parameter(controller_type + ".plugin")) {
      std::string plugin;
      get_parameter(controller_type + ".plugin", plugin);
      try {
        controller_loader_->unloadLibraryForClass(plugin);
      } catch (const std::exception &) {
      }
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_error([[maybe_unused]] const rclcpp_lifecycle::State & state)
{
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
ControllerNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

bool
ControllerNode::cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger)
{
  // Take a local copy so the plugin instance stays alive for this call even
  // if on_cleanup() resets controller_method_ right after we release the lock.
  std::shared_ptr<ControllerMethodBase> controller_method;
  {
    std::lock_guard<std::mutex> lock(controller_method_mutex_);
    controller_method = controller_method_;
  }

  if (controller_method == nullptr) {return false;}

  return controller_method->internal_update_rt(*nav_state, trigger);
}

}  // namespace easynav
