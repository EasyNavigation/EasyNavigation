// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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
/// \brief Implementation of the GoalManager class.

#include <cmath>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_system/GoalManager.hpp"


namespace easynav
{

/** Internal utility functions */
/**
  * @brief Get the x/y distance between two poses
  *
  * @param pose1 First pose
  * @param pose2 Second pose
  */
double calculate_distance_xy(
  const geometry_msgs::msg::Pose & pose1,
  const geometry_msgs::msg::Pose & pose2)
{
  const double dx = pose1.position.x - pose2.position.x;
  const double dy = pose1.position.y - pose2.position.y;
  return std::hypot(dx, dy);
}

/** Angle normalization to [-pi, pi) range (in radians) */
constexpr double norm_angle(const double angle)
{
  double out_angle = std::fmod(angle + M_PI, 2 * M_PI);
  if (out_angle < 0.0) {
    out_angle += 2 * M_PI;
  }
  return out_angle - M_PI;
}

/**
  * @brief Get the (absolute) yaw angle difference between two poses
  *
  * @param pose1 First pose
  * @param pose2 Second pose
  */
double calculate_angle(
  const geometry_msgs::msg::Pose & pose1,
  const geometry_msgs::msg::Pose & pose2)
{
  const double yaw1 = tf2::getYaw(pose1.orientation);
  const double yaw2 = tf2::getYaw(pose2.orientation);
  return norm_angle(yaw1 - yaw2);
}


GoalManager::GoalManager(
  NavState & nav_state,
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node)
: parent_node_(parent_node)
{
  nav_state.set("navigation_state", state_);

  parent_node_->declare_parameter("allow_preempt_goal", allow_preempt_goal_);
  parent_node_->declare_parameter("position_tolerance", goal_tolerance_.position);
  parent_node_->declare_parameter("height_tolerance", goal_tolerance_.height);
  parent_node_->declare_parameter("angle_tolerance", goal_tolerance_.yaw);
  parent_node_->declare_parameter("update_frequency", update_frequency_);
  parent_node_->get_parameter("allow_preempt_goal", allow_preempt_goal_);
  parent_node_->get_parameter("position_tolerance", goal_tolerance_.position);
  parent_node_->get_parameter("height_tolerance", goal_tolerance_.height);
  parent_node_->get_parameter("angle_tolerance", goal_tolerance_.yaw);
  parent_node_->get_parameter("update_frequency", update_frequency_);
  if (update_frequency_ <= 0.0) {
    RCLCPP_WARN(
      parent_node_->get_logger(),
      "Parameter 'update_frequency' must be > 0.0 (got %.3f); falling back to 20.0",
      update_frequency_);
    update_frequency_ = 20.0;
  }

  update_period_ = rclcpp::Duration::from_seconds(1.0 / update_frequency_);
  // Expose initial goal tolerances in NavState so controllers can reuse them
  nav_state.set("goal_tolerance.position", goal_tolerance_.position);
  nav_state.set("goal_tolerance.height", goal_tolerance_.height);
  nav_state.set("goal_tolerance.yaw", goal_tolerance_.yaw);

  control_sub_ = parent_node_->create_subscription<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100,
    std::bind(&GoalManager::control_callback, this, std::placeholders::_1)
  );

  comanded_pose_sub_ = parent_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 100,
    std::bind(&GoalManager::comanded_pose_callback, this, std::placeholders::_1)
  );

  control_pub_ = parent_node_->create_publisher<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100);

  info_pub_ = parent_node_->create_publisher<easynav_interfaces::msg::GoalManagerInfo>(
    "easynav_manager_info", 100);

  id_ = "easynav_system";
  last_control_ = std::make_unique<easynav_interfaces::msg::NavigationControl>();

  NavState::register_printer<State>(
    [](const State & state) {
      std::ostringstream ret;
      if (state == State::IDLE) {
        ret << "State IDLE\n";
      } else {
        ret << "State ACTIVE\n";
      }
      return ret.str();
    });

  // parent_node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
}

void
GoalManager::accept_request(
  const easynav_interfaces::msg::NavigationControl & msg,
  easynav_interfaces::msg::NavigationControl & response)
{
  nav_start_time_ = parent_node_->now();

  RCLCPP_DEBUG(parent_node_->get_logger(), "Accepted navigation request");

  goals_ = msg.goals;

  current_client_id_ = msg.user_id;
  response.status_message = "Goal accepted";
  response.type = easynav_interfaces::msg::NavigationControl::ACCEPT;
  response.nav_current_user_id = current_client_id_;
  state_ = State::ACTIVE;
}


void
GoalManager::control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg)
{
  if (msg->user_id == id_) {return;}  // Avoid self messages

  RCLCPP_DEBUG(parent_node_->get_logger(), "Managing navigation control message received");

  easynav_interfaces::msg::NavigationControl response;
  response = *msg;
  response.header.stamp = parent_node_->now();
  response.seq = msg->seq + 1;
  response.user_id = id_;

  switch (msg->type) {
    case easynav_interfaces::msg::NavigationControl::REQUEST:
      RCLCPP_DEBUG(parent_node_->get_logger(), "Navigation request");
      if (msg->goals.goals.empty()) {
        RCLCPP_DEBUG(parent_node_->get_logger(), "Rejected navigation request (empty goals)");

        response.status_message = "Goals are empty";
        response.type = easynav_interfaces::msg::NavigationControl::REJECT;
        response.nav_current_user_id = msg->user_id;
      } else {
        if (state_ == State::IDLE) {
          accept_request(*msg, response);
        } else {
          if (allow_preempt_goal_) {
            if (msg->user_id != current_client_id_) {
              set_preempted();
            }
            accept_request(*msg, response);
          } else {
            RCLCPP_DEBUG(parent_node_->get_logger(),
              "Rejected navigation request (unable to preempt)");

            response.status_message = "Goal rejected; unable to preemp current active goal";
            response.type = easynav_interfaces::msg::NavigationControl::REJECT;
            response.nav_current_user_id = msg->user_id;
          }
        }
      }
      break;
    case easynav_interfaces::msg::NavigationControl::CANCEL:
      RCLCPP_DEBUG(parent_node_->get_logger(), "Navigation cancelation requested");

      if (current_client_id_ != msg->user_id) {
        RCLCPP_DEBUG(parent_node_->get_logger(), "Navigation cancelation rejected (not yours)");
        response.status_message = "Trying to cancel a navigation not commanded by you";
        response.type = easynav_interfaces::msg::NavigationControl::REJECT;
        response.nav_current_user_id = msg->user_id;
      } else {
        if (state_ == State::IDLE) {
          RCLCPP_DEBUG(parent_node_->get_logger(),
            "Navigation cancelation rejected (not navigating)");
          response.status_message = "Nothing to cancel; easynav is idle";
          response.type = easynav_interfaces::msg::NavigationControl::ERROR;
          response.nav_current_user_id = msg->user_id;
        } else {
          RCLCPP_DEBUG(parent_node_->get_logger(), "Navigation cancelation accepted");
          goals_.goals.clear();
          response.status_message = "Goal cancelled";
          response.type = easynav_interfaces::msg::NavigationControl::CANCELLED;
          response.nav_current_user_id = current_client_id_;
          state_ = State::IDLE;
        }
      }
      break;
    default:
      RCLCPP_WARN(parent_node_->get_logger(), "Received erroneous control message %d", msg->type);
      response.status_message = "Unable to process message";
      response.type = easynav_interfaces::msg::NavigationControl::ERROR;
      response.nav_current_user_id = msg->user_id;
      break;
  }

  control_pub_->publish(response);
  last_control_ = std::move(msg);
}

void
GoalManager::set_preempted()
{
  goals_.goals.clear();

  easynav_interfaces::msg::NavigationControl response;
  response = *last_control_;
  response.header.stamp = parent_node_->now();
  response.seq = last_control_->seq + 1;
  response.user_id = id_;
  response.type = easynav_interfaces::msg::NavigationControl::CANCELLED;
  response.nav_current_user_id = current_client_id_;
  response.status_message = "Navigation preempted by others";

  control_pub_->publish(response);
}

void
GoalManager::set_finished()
{
  state_ = State::IDLE;
  goals_.goals.clear();

  easynav_interfaces::msg::NavigationControl response;
  response = *last_control_;
  response.header.stamp = parent_node_->now();
  response.seq = last_control_->seq + 1;
  response.user_id = id_;
  response.type = easynav_interfaces::msg::NavigationControl::FINISHED;
  response.nav_current_user_id = current_client_id_;
  response.status_message = "Navigation succesfully finished";

  control_pub_->publish(response);
}

void
GoalManager::set_error(const std::string & reason)
{
  state_ = State::IDLE;
  goals_.goals.clear();

  easynav_interfaces::msg::NavigationControl response;
  response = *last_control_;
  response.header.stamp = parent_node_->now();
  response.seq = last_control_->seq + 1;
  response.user_id = id_;
  response.type = easynav_interfaces::msg::NavigationControl::ERROR;
  response.nav_current_user_id = current_client_id_;
  response.status_message = reason;

  control_pub_->publish(response);
}

void
GoalManager::set_failed(const std::string & reason)
{
  state_ = State::IDLE;
  goals_.goals.clear();

  easynav_interfaces::msg::NavigationControl response;
  response = *last_control_;
  response.header.stamp = parent_node_->now();
  response.seq = last_control_->seq + 1;
  response.user_id = id_;
  response.type = easynav_interfaces::msg::NavigationControl::FAILED;
  response.nav_current_user_id = current_client_id_;
  response.status_message = reason;

  control_pub_->publish(response);
}

void
GoalManager::comanded_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
  auto command = std::make_unique<easynav_interfaces::msg::NavigationControl>();
  command->header = msg->header;
  command->seq = last_control_->seq + 1;
  command->user_id = id_ + "_initpose";
  command->type = easynav_interfaces::msg::NavigationControl::REQUEST;
  command->goals.header = msg->header;
  command->goals.goals.push_back(*msg);

  control_callback(std::move(command));
}

void
GoalManager::update(NavState & nav_state)
{
  if (nav_state.get<State>("navigation_state") != state_) {
    nav_state.set("navigation_state", state_);
  }

   // Keep published tolerances in sync with current parameters
  nav_state.set("goal_tolerance.position", goal_tolerance_.position);
  nav_state.set("goal_tolerance.height", goal_tolerance_.height);
  nav_state.set("goal_tolerance.yaw", goal_tolerance_.yaw);

  if (state_ == State::IDLE) {
    goals_ = nav_msgs::msg::Goals();
    nav_state.set("goals", goals_);
    return;
  }

  if (!nav_state.has("robot_pose")) {
    RCLCPP_WARN(parent_node_->get_logger(), "No robot pose at GoalManager::Update");
    return;
  }

  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose").pose.pose;

  easynav_interfaces::msg::NavigationControl feedback;
  feedback.type = easynav_interfaces::msg::NavigationControl::FEEDBACK;
  feedback.header.stamp = parent_node_->now();
  feedback.seq = last_control_->seq + 1;
  feedback.user_id = id_;
  feedback.nav_current_user_id = current_client_id_;

  const auto & odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");

  feedback.goals = goals_;
  feedback.current_pose.header = odom.header;
  feedback.current_pose.pose = odom.pose.pose;
  feedback.navigation_time = parent_node_->now() - nav_start_time_;

  // Copy (not reference): check_goals() below may erase the front goal, which would
  // otherwise leave this dangling.
  const auto first_goal = goals_.goals.front().pose;
  feedback.distance_to_goal = calculate_distance_xy(robot_pose, first_goal);

  // ToDo[@fmrico]: Complete feedback info: estimated_time_remaining and distance_covered

  // Throttle only the periodic FEEDBACK / GoalManagerInfo publishing so these topics
  // aren't saturated. The nav_state bookkeeping below (goals, navigation_state) must run
  // every cycle regardless, since the planner reacts to it as soon as a new goal's
  // timestamp is newer than its last planned one (see SystemNode::system_cycle()); delaying
  // that sync here would make the planner compute a path from stale/empty goals.
  const auto now = parent_node_->now();
  const bool should_publish = first_update_ || (now - last_update_time_) >= update_period_;

  if (should_publish) {
    RCLCPP_DEBUG(parent_node_->get_logger(), "Sending navigation feedback");
    control_pub_->publish(feedback);
    *last_control_ = feedback;
    first_update_ = false;
    last_update_time_ = now;
  }

  check_goals(robot_pose, goal_tolerance_);

  if (!nav_state.has("goals")) {
    nav_state.set("goals", goals_);
  }

  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");

  if (goals != goals_) {
    nav_state.set("goals", goals_);
  }

  if (goals_.goals.empty()) {
    set_finished();
  }

  if (nav_state.get<State>("navigation_state") != state_) {
    nav_state.set("navigation_state", state_);
  }

  if (should_publish && info_pub_->get_subscription_count() > 0) {
    easynav_interfaces::msg::GoalManagerInfo msg;
    msg.status = static_cast<int>(get_state());
    msg.goals = get_goals();
    msg.position_tolerance = goal_tolerance_.position;
    msg.angle_tolerance = goal_tolerance_.yaw;
    msg.position_distance = feedback.distance_to_goal;
    msg.angle_distance = calculate_angle(robot_pose, first_goal);
    info_pub_->publish(msg);
  }
}

void
GoalManager::check_goals(
  const geometry_msgs::msg::Pose & current_pose,
  const GoalTolerance & goal_tolerance)
{
  if (goals_.goals.empty()) {return;}

  const auto & first_goal = goals_.goals.front().pose;

  const double distance_xy = calculate_distance_xy(current_pose, first_goal);
  const double distance_z = std::abs(current_pose.position.z - first_goal.position.z);

  if (distance_xy > goal_tolerance.position || distance_z > goal_tolerance.height) {
    return;
  }

  const double angle_diff = calculate_angle(current_pose, first_goal);

  if (std::fabs(angle_diff) <= goal_tolerance.yaw) {
    goals_.goals.erase(goals_.goals.begin());
  }
}

}  // namespace easynav
