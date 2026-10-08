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

#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

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
#include "easynav_common/RobotGeometry.hpp"
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

  const RobotGeometry geometry;
  declare_parameter("robot_geometry.radius", geometry.radius);
  declare_parameter("robot_geometry.inscribed_radius", geometry.inscribed_radius);
  declare_parameter("robot_geometry.height", geometry.height);

  safety_.declare_parameters(*this);

  // Read by system_main. The cycles check whether each component has to run (at its own
  // "<plugin>.rt_freq" / "<plugin>.freq"): they must be at least as fast as any of them.
  declare_parameter("use_real_time", true);
  declare_parameter("rt_freq", 200.0);
  declare_parameter("freq", 200.0);
  declare_parameter("spin_time_rt", 0.001);
  declare_parameter("spin_time_nort", 0.001);
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

  // Both, to report every error.
  const bool system_valid = check_system_parameters();
  const bool safety_valid = safety_.check_system(*this, realtime_cbg_);
  if (!system_valid || !safety_valid) {
    return CallbackReturnT::FAILURE;
  }

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

  configure_robot_geometry();

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Configuring [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to configure [%s]", system_node.first.c_str());
      cleanup_subnodes();
      return CallbackReturnT::FAILURE;
    }
  }

  // Both, to report every error.
  const bool frequencies_valid = check_component_frequencies();
  if (!safety_.check_controller(*controller_node_) || !frequencies_valid) {
    cleanup_subnodes();
    return CallbackReturnT::FAILURE;
  }

  // Kept across cleanup/configure: a reconfiguration does not lose the mission.
  if (!goal_manager_) {
    goal_manager_ = GoalManager::make_shared(*nav_state_, shared_from_this());
  } else {
    goal_manager_->read_parameters(*nav_state_);
  }

  navstate_pub_ = create_publisher<std_msgs::msg::String>(
    "easynav_navstate", 100);

  safety_.on_configured(get_all_nodes(), *nav_state_);

  return CallbackReturnT::SUCCESS;
}

bool
SystemNode::check_system_parameters()
{
  std::vector<std::string> errors;
  auto check = [&](const std::string & name, bool valid, const std::string & expected) {
      const double value = get_parameter(name).as_double();
      if (!std::isfinite(value) || !valid) {
        errors.push_back(name + " = " + std::to_string(value) + " (" + expected + ")");
      }
    };
  auto value = [this](const std::string & name) {return get_parameter(name).as_double();};

  check("rt_freq", value("rt_freq") > 0.0, "> 0");
  check("freq", value("freq") > 0.0, "> 0");
  check("spin_time_rt", value("spin_time_rt") >= 0.0, ">= 0");
  check("spin_time_nort", value("spin_time_nort") >= 0.0, ">= 0");
  check("robot_geometry.radius", value("robot_geometry.radius") >= 0.0, ">= 0");
  check(
    "robot_geometry.inscribed_radius", value("robot_geometry.inscribed_radius") >= 0.0, ">= 0");
  check("robot_geometry.height", value("robot_geometry.height") >= 0.0, ">= 0");

  for (const auto & error : errors) {
    RCLCPP_ERROR(get_logger(), "Invalid parameter: %s", error.c_str());
  }
  return errors.empty();
}

bool
SystemNode::check_component_frequencies()
{
  // The system cycles only check whether each component has to run: they must keep up.
  const double rt_freq = get_parameter("rt_freq").as_double();
  const double freq = get_parameter("freq").as_double();
  auto ends_with = [](const std::string & name, const std::string & suffix) {
      return name.size() > suffix.size() &&
             name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
    };

  bool valid = true;
  for (const auto & [node_name, info] : get_system_nodes()) {
    for (const auto & name : info.node_ptr->list_parameters({}, 0).names) {
      const bool rt = ends_with(name, ".rt_freq");
      if (!rt && !ends_with(name, ".freq")) {continue;}
      rclcpp::Parameter param;
      try {
        param = info.node_ptr->get_parameter(name);
      } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
        continue;  // Declared without a value (e.g. dynamically typed): not a frequency yet
      }
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {continue;}
      const double system_freq = rt ? rt_freq : freq;
      if (param.as_double() > system_freq) {
        RCLCPP_ERROR(
          get_logger(), "Invalid parameter: %s.%s = %.1f Hz: above %s = %.1f Hz, the system "
          "cycle that runs it", node_name.c_str(), param.get_name().c_str(), param.as_double(),
          rt ? "rt_freq" : "freq", system_freq);
        valid = false;
      }
    }
  }
  return valid;
}

void
SystemNode::cleanup_subnodes()
{
  for (auto & [name, info] : get_system_nodes()) {
    if (info.node_ptr->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      info.node_ptr->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
  }
}

std::map<std::string, rclcpp_lifecycle::LifecycleNode::SharedPtr>
SystemNode::get_all_nodes()
{
  std::map<std::string, rclcpp_lifecycle::LifecycleNode::SharedPtr> nodes;
  nodes[get_name()] = std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(
    shared_from_this());
  for (const auto & [name, info] : get_system_nodes()) {
    nodes[name] = info.node_ptr;
  }
  return nodes;
}

std::string
SystemNode::get_configuration_dump()
{
  return safety::configuration_dump(get_all_nodes());
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
    safety_.on_activate();
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
SystemNode::configure_robot_geometry()
{
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  const RobotGeometry defaults;
  RobotGeometry geometry;
  std::set<std::string> configured;
  for (const auto & [field, member] : std::vector<std::pair<std::string, double RobotGeometry::*>>{
      {"radius", &RobotGeometry::radius},
      {"inscribed_radius", &RobotGeometry::inscribed_radius},
      {"height", &RobotGeometry::height}})
  {
    const auto name = "robot_geometry." + field;
    get_parameter(name, geometry.*member);
    // Explicitly configured: in the parameter files/overrides, or changed at runtime.
    if (overrides.count(name) > 0 || geometry.*member != defaults.*member) {
      configured.insert(field);
    }
  }
  if (configured.count("inscribed_radius") == 0) {
    geometry.inscribed_radius = geometry.radius;  // A round robot, unless told otherwise
  }

  RobotGeometryRegistry::getInstance()->set_geometry(geometry, configured);
  RCLCPP_INFO(
    get_logger(), "Robot geometry: radius=%.3f, inscribed_radius=%.3f, height=%.3f",
    geometry.radius, geometry.inscribed_radius, geometry.height);
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
  RCLCPP_FATAL(get_logger(), "Shutdown requested: %s", reason.c_str());
}

bool
SystemNode::request_reconfigure(
  const std::vector<ParameterChange> & changes, const std::string & reason)
{
  if (!safety_.allows_reconfiguration(reason)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(reconfigure_mutex_);
  pending_reconfigure_ = ReconfigureRequest{changes, false, reason};
  return true;
}

bool
SystemNode::request_restore_parameters(const std::string & reason)
{
  if (!safety_.allows_reconfiguration(reason)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(reconfigure_mutex_);
  pending_reconfigure_ = ReconfigureRequest{{}, true, reason};
  return true;
}

bool
SystemNode::is_reconfigure_pending() const
{
  std::lock_guard<std::mutex> lock(reconfigure_mutex_);
  return pending_reconfigure_.has_value();
}

rclcpp_lifecycle::LifecycleNode::SharedPtr
SystemNode::find_node(const std::string & name)
{
  if (name == get_name()) {
    return std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(shared_from_this());
  }
  auto nodes = get_system_nodes();
  auto it = nodes.find(name);
  return it == nodes.end() ? nullptr : it->second.node_ptr;
}

bool
SystemNode::apply_pending_reconfigure()
{
  using lifecycle_msgs::msg::State;

  ReconfigureRequest request;
  {
    std::lock_guard<std::mutex> lock(reconfigure_mutex_);
    if (!pending_reconfigure_ || is_shutdown_requested() ||
      get_current_state().id() != State::PRIMARY_STATE_ACTIVE)
    {
      return false;
    }
    request = std::move(*pending_reconfigure_);
    pending_reconfigure_.reset();
  }

  std::vector<ParameterChange> changes = request.changes;
  if (request.restore) {
    changes.clear();
    for (const auto & [key, original] : original_parameters_) {
      changes.push_back(original);
    }
  }
  if (changes.empty()) {
    return false;
  }

  // Current values: the originals of first changes, and what to go back to if these fail.
  std::vector<ParameterChange> previous;
  for (const auto & change : changes) {
    auto node = find_node(change.node);
    if (!node || !node->has_parameter(change.parameter.get_name())) {
      RCLCPP_ERROR(
        get_logger(), "Reconfiguration rejected (%s): no parameter [%s] in [%s]",
        request.reason.c_str(), change.parameter.get_name().c_str(), change.node.c_str());
      return false;
    }
    previous.push_back({change.node, node->get_parameter(change.parameter.get_name())});
  }

  RCLCPP_WARN(get_logger(), "Reconfiguring EasyNav: %s", request.reason.c_str());
  if (!restart_with(changes)) {
    RCLCPP_ERROR(get_logger(), "Reconfiguration failed, restoring the previous values");
    if (!restart_with(previous)) {
      request_shutdown("unable to reconfigure (" + request.reason + ") or restore");
    }
    return true;
  }

  if (request.restore) {
    original_parameters_.clear();
  } else {
    for (const auto & value : previous) {
      original_parameters_.emplace(value.node + "/" + value.parameter.get_name(), value);
    }
  }

  std::vector<std::string> changed;
  for (const auto & [key, original] : original_parameters_) {
    changed.push_back(key);
  }
  nav_state_->set("reconfigured_parameters", changed);
  return true;
}

bool
SystemNode::restart_with(const std::vector<ParameterChange> & changes)
{
  using lifecycle_msgs::msg::State;
  using lifecycle_msgs::msg::Transition;

  if (get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(Transition::TRANSITION_DEACTIVATE);
  }
  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(Transition::TRANSITION_CLEANUP);
  }
  if (get_current_state().id() != State::PRIMARY_STATE_UNCONFIGURED) {
    return false;
  }
  // A failed configure may leave some subnodes configured.
  for (auto & [name, info] : get_system_nodes()) {
    if (info.node_ptr->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
      info.node_ptr->trigger_transition(Transition::TRANSITION_CLEANUP);
    }
  }

  bool all_set = true;
  for (const auto & change : changes) {
    const auto result = find_node(change.node)->set_parameter(change.parameter);
    if (!result.successful) {
      RCLCPP_ERROR(
        get_logger(), "Unable to set [%s/%s]: %s", change.node.c_str(),
        change.parameter.get_name().c_str(), result.reason.c_str());
      all_set = false;
    }
  }

  return trigger_transition(Transition::TRANSITION_CONFIGURE).id() ==
         State::PRIMARY_STATE_INACTIVE &&
         trigger_transition(Transition::TRANSITION_ACTIVATE).id() == State::PRIMARY_STATE_ACTIVE &&
         all_set;
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

  // First: a late cycle is seen even if what follows throws.
  if (!safety_.cycle_rt(*nav_state_, std::chrono::steady_clock::now())) {
    request_shutdown(safety_.failure());
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
