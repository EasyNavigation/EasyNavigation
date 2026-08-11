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
#include "easynav_common/RTTFBuffer.hpp"

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

  declare_parameter<bool>("use_cmd_vel_stamped", use_cmd_vel_stamped_);

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
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
SystemNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  TFInfo tf_info;
  get_parameter<bool>("use_cmd_vel_stamped", use_cmd_vel_stamped_);
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

  goal_manager_ = GoalManager::make_shared(*nav_state_, shared_from_this());

  navstate_pub_ = create_publisher<std_msgs::msg::String>(
    "easynav_navstate", 100);

  if (use_cmd_vel_stamped_) {
    vel_pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 100);
  } else {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
  }

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

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

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

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
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
  return CallbackReturnT::SUCCESS;
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

  RCLCPP_DEBUG(get_logger(), "SystemNode::system_cycle_rt\n%s", nav_state_->debug_string().c_str());

  bool trigger_perceptions = sensors_node_->cycle_rt(nav_state_);
  bool trigger_localization = localizer_node_->cycle_rt(nav_state_, trigger_perceptions);

  bool trigger_controller = false;

  bool trigger = trigger_perceptions || trigger_localization;
  trigger_controller = controller_node_->cycle_rt(nav_state_, trigger);

  if (nav_state_->has("cmd_vel")) {
    geometry_msgs::msg::TwistStamped current_cmd_vel;
    current_cmd_vel = nav_state_->get<geometry_msgs::msg::TwistStamped>("cmd_vel");

    if (trigger_controller) {
      if (use_cmd_vel_stamped_ && vel_pub_stamped_->get_subscription_count()) {
        vel_pub_stamped_->publish(current_cmd_vel);
      }
      if (!use_cmd_vel_stamped_ && vel_pub_->get_subscription_count()) {
        vel_pub_->publish(current_cmd_vel.twist);
      }
    }
  }
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

  return ret;
}

}  // namespace easynav
