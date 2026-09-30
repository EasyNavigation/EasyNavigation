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
/// \brief Implementation of the SystemNode class.

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_planner/PlannerNode.hpp"
#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_common/YTSession.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_common/Parameters.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "easynav_recovery/RecoveryManagerNode.hpp"
#include "easynav_system/SystemNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

SystemNode::SystemNode(const rclcpp::NodeOptions & options)
: LifecycleNode("system_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  nav_state_ = std::make_shared<NavState>();

  NavState::register_printer<nav_msgs::msg::Goals>(
    [](const nav_msgs::msg::Goals & goals) {
      std::ostringstream ret;
      ret << "{ " << rclcpp::Time(goals.header.stamp).seconds() << " } Goals " <<
        goals.goals.size() << " with :\n";
      for (const auto & goal : goals.goals) {
        ret << "\t--> (" << goal.pose.position.x << ", " << goal.pose.position.y << ")\n";
      }
      return ret.str();
    });

  controller_node_ = ControllerNode::make_shared();
  localizer_node_ = LocalizerNode::make_shared();
  maps_manager_node_ = MapsManagerNode::make_shared();
  planner_node_ = PlannerNode::make_shared();
  sensors_node_ = SensorsNode::make_shared();
  recovery_node_ = RecoveryManagerNode::make_shared();


  TFInfo tf_info;
  declare_parameter<std::string>("tf_prefix", tf_info.tf_prefix);
  declare_parameter<std::string>("robot_frame", tf_info.robot_frame);
  declare_parameter<std::string>("robot_footprint_frame", tf_info.robot_footprint_frame);
  declare_parameter<std::string>("odom_frame", tf_info.odom_frame);
  declare_parameter<std::string>("map_frame", tf_info.map_frame);
  declare_parameter<std::string>("world_frame", tf_info.world_frame);
  // get_logger().set_level(rclcpp::Logger::Level::Debug);
}

SystemNode::~SystemNode()
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
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
SystemNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  forward_deprecated_use_cmd_vel_stamped();

  // What the recovery system may ask of the navigation system: this node (see SystemActions).
  recovery_node_->set_system_actions(
    std::static_pointer_cast<SystemActions>(
      std::static_pointer_cast<SystemNode>(shared_from_this())));

  TFInfo tf_info;
  get_parameter("robot_frame", tf_info.robot_frame);
  get_parameter("robot_footprint_frame", tf_info.robot_footprint_frame);
  get_parameter("odom_frame", tf_info.odom_frame);
  get_parameter("map_frame", tf_info.map_frame);
  get_parameter("world_frame", tf_info.world_frame);

  get_parameter("tf_prefix", tf_info.tf_prefix);

  RTTFBuffer::getInstance()->set_tf_info(tf_info);
  RCLCPP_INFO(
    get_logger(),
      "EasyNav configured with TFInfo: prefix='%s', map='%s', odom='%s', robot='%s', footprint='%s', world='%s'",
    tf_info.tf_prefix.c_str(), tf_info.map_frame.c_str(),
    tf_info.odom_frame.c_str(), tf_info.robot_frame.c_str(),
    tf_info.robot_footprint_frame.c_str(), tf_info.world_frame.c_str());

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Configuring [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to configure [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  // Kept across cleanup/configure: a reconfiguration does not lose the mission.
  if (!goal_manager_) {
    goal_manager_ = GoalManager::make_shared(*nav_state_, shared_from_this());
  } else {
    goal_manager_->read_parameters(*nav_state_);
  }

  navstate_pub_ = create_publisher<std_msgs::msg::String>(
    "easynav_navstate", 100);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Activating [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to activate [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  {
    std::lock_guard<std::mutex> lock(rt_mutex_);
    active_ = true;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  {
    // Once no RT cycle is in flight, none will publish again: ControllerNode's stop, on its
    // deactivation below, is the last command.
    std::lock_guard<std::mutex> lock(rt_mutex_);
    active_ = false;
    clear_cmd_vel();
  }

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Deactivating [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to deactivate [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  if (is_shutdown_requested()) {
    // Unrecoverable error while Active: ErrorProcessing (on_error()), not back to Inactive.
    return CallbackReturnT::ERROR;
  }

  return CallbackReturnT::SUCCESS;
}

void
SystemNode::clear_cmd_vel()
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = now();
  stop.header.frame_id = RTTFBuffer::getInstance()->get_tf_info().robot_frame;
  nav_state_->set("cmd_vel", stop);
}

void
SystemNode::forward_deprecated_use_cmd_vel_stamped()
{
  const std::string name = "use_cmd_vel_stamped";
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.count(name) == 0 && !has_parameter(name)) {
    return;
  }

  bool stamped = false;
  declare_parameter_if_absent(*this, name, stamped);
  get_parameter(name, stamped);

  const auto & controller_overrides =
    controller_node_->get_node_parameters_interface()->get_parameter_overrides();
  if (controller_overrides.count(name) > 0) {
    RCLCPP_WARN(
      get_logger(), "'system_node.%s' is deprecated and ignored: 'controller_node.%s' takes "
      "precedence", name.c_str(), name.c_str());
    return;
  }
  RCLCPP_WARN(
    get_logger(), "'system_node.%s' is deprecated: configure 'controller_node.%s' instead. "
    "It will stop working soon.", name.c_str(), name.c_str());
  controller_node_->set_parameter(rclcpp::Parameter(name, stamped));
}

CallbackReturnT
SystemNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Cleaning up [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      RCLCPP_ERROR(get_logger(), "Unable to clean up [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  // goal_manager_ is kept (see on_configure()).
  navstate_pub_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;

  if (!is_shutdown_requested()) {
    return CallbackReturnT::SUCCESS;
  }

  // Unrecoverable (see request_shutdown()): shut every EasyNav node down and fail, so this node
  // ends in Finalized.
  RCLCPP_FATAL(get_logger(), "Unrecoverable error: finalizing EasyNav");
  for (auto & system_node : get_system_nodes()) {
    auto & node = system_node.second.node_ptr;
    switch (node->get_current_state().id()) {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
        break;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
        break;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        node->trigger_transition(
          lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
        break;
      default:
        break;
    }
  }
  return CallbackReturnT::FAILURE;
}

std::string
SystemNode::get_shutdown_reason() const
{
  std::lock_guard<std::mutex> lock(shutdown_reason_mutex_);
  return shutdown_reason_;
}

void
SystemNode::abort_mission(const std::string & reason)
{
  if (goal_manager_ && goal_manager_->get_state() == GoalManager::State::ACTIVE) {
    RCLCPP_ERROR(get_logger(), "Mission aborted by recovery: %s", reason.c_str());
    goal_manager_->set_error(reason);
  }
}

void
SystemNode::hold_mission_progress(bool hold)
{
  if (goal_manager_) {
    goal_manager_->set_progress_held(hold);
  }
}

void
SystemNode::request_shutdown(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(shutdown_reason_mutex_);
  if (shutdown_requested_) {
    return;  // Latched: the first reason is kept
  }
  shutdown_reason_ = reason;
  shutdown_requested_ = true;
  RCLCPP_FATAL(get_logger(), "Shutdown requested by recovery: %s", reason.c_str());
}

rclcpp::CallbackGroup::SharedPtr
SystemNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

void
SystemNode::system_cycle_rt()
{
  EASYNAV_TRACE_EVENT;

  std::lock_guard<std::mutex> lock(rt_mutex_);
  if (!active_) {
    return;
  }

  RCLCPP_DEBUG(get_logger(), "SystemNode::system_cycle_rt\n%s", nav_state_->debug_string().c_str());

  bool trigger_perceptions = sensors_node_->cycle_rt(nav_state_);
  bool trigger_localization = localizer_node_->cycle_rt(nav_state_, trigger_perceptions);

  const bool trigger = trigger_perceptions || trigger_localization;
  controller_node_->cycle_rt(nav_state_, trigger);
  // The recovery system may take over or override the command before it is published.
  recovery_node_->cycle_rt(nav_state_);

  // Selected, smoothed within the robot limits, and published.
  controller_node_->publish_cmd_vel_rt(nav_state_);
}

void
SystemNode::system_cycle()
{
  EASYNAV_TRACE_EVENT;

  RCLCPP_DEBUG(get_logger(), "SystemNode::system_cycle\n%s", nav_state_->debug_string().c_str());

  sensors_node_->cycle(nav_state_);
  localizer_node_->cycle(nav_state_);
  maps_manager_node_->cycle(nav_state_);
  goal_manager_->update(*nav_state_);

  rclcpp::Time planner_ts = planner_node_->get_last_execution_ts();
  rclcpp::Time goals_ts(goal_manager_->get_goals().header.stamp, planner_ts.get_clock_type());

  planner_node_->cycle(nav_state_, planner_ts < goals_ts);

  // Last: the recovery system diagnoses what the cycle above just produced.
  recovery_node_->cycle(nav_state_);

  if (navstate_pub_->get_subscription_count() > 0) {
    std_msgs::msg::String msg;
    msg.data = nav_state_->debug_string();
    navstate_pub_->publish(msg);
  }
}

std::map<std::string, SystemNodeInfo>
SystemNode::get_system_nodes()
{
  std::map<std::string, SystemNodeInfo> ret;

  ret[controller_node_->get_name()] = {controller_node_, controller_node_->get_real_time_cbg()};
  ret[localizer_node_->get_name()] = {localizer_node_, localizer_node_->get_real_time_cbg()};
  ret[maps_manager_node_->get_name()] = {maps_manager_node_, nullptr};
  ret[planner_node_->get_name()] = {planner_node_, nullptr};
  ret[sensors_node_->get_name()] = {sensors_node_, sensors_node_->get_real_time_cbg()};
  ret[recovery_node_->get_name()] = {recovery_node_, nullptr};

  return ret;
}

}  // namespace easynav
