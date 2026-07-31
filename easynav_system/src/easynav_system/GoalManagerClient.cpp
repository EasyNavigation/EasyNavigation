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


#include "easynav_interfaces/msg/navigation_control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"

#include "easynav_system/GoalManagerClient.hpp"

namespace easynav
{

using namespace  std::placeholders;


GoalManagerClient::GoalManagerClient(rclcpp::Node::SharedPtr node)
: node_(node)
{
  control_sub_ = node_->create_subscription<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100,
    std::bind(&GoalManagerClient::control_callback, this, _1)
  );

  control_pub_ = node_->create_publisher<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100);

  id_ = std::string(node_->get_name()) + "_goal_manager_client";
  state_ = State::IDLE;
}

void
GoalManagerClient::control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg)
{
  if (msg->user_id == id_) {return;}  // Avoid self messages
  if (msg->nav_current_user_id != id_) {return;}  // Avoid messages to others

  RCLCPP_DEBUG(
    node_->get_logger(), "Received a navigation %d msg with user_id %s",
    msg->type, msg->user_id.c_str());

  switch (state_) {
    case State::IDLE:
    case State::NAVIGATION_FINISHED:
    case State::NAVIGATION_FAILED:
    case State::NAVIGATION_CANCELLED:
    case State::ERROR:
      break;
    case State::SENT_GOAL:
      switch (msg->type) {
        case easynav_interfaces::msg::NavigationControl::FEEDBACK:
          RCLCPP_DEBUG(
            node_->get_logger(),
            "Getting navigation feedback while waiting for acceptance");
          last_feedback_ = *msg;
          break;
        case easynav_interfaces::msg::NavigationControl::ACCEPT:
          RCLCPP_DEBUG(node_->get_logger(), "Goal accepted. Navigating");
          state_ = State::ACCEPTED_AND_NAVIGATING;
          break;
        case easynav_interfaces::msg::NavigationControl::REJECT:
          RCLCPP_ERROR(node_->get_logger(), "Rejected navigation goal");
          state_ = State::NAVIGATION_FAILED;
          break;
        case easynav_interfaces::msg::NavigationControl::ERROR:
          RCLCPP_ERROR(node_->get_logger(), "Error in navigation");
          state_ = State::ERROR;
          break;
        default:
          RCLCPP_ERROR(
            node_->get_logger(), "State SENT_GOAL; Unexpected message : %d: %s",
            msg->type, msg->status_message.c_str());
          state_ = State::ERROR;
      }
      break;
    case State::SENT_PREEMPT:
      switch (msg->type) {
        case easynav_interfaces::msg::NavigationControl::FEEDBACK:
          RCLCPP_DEBUG(node_->get_logger(), "Getting navigation feedback");
          last_feedback_ = *msg;
          break;
        case easynav_interfaces::msg::NavigationControl::ACCEPT:
          RCLCPP_DEBUG(node_->get_logger(), "Goal preemption accepted. Navigating");
          state_ = State::ACCEPTED_AND_NAVIGATING;
          break;
        case easynav_interfaces::msg::NavigationControl::REJECT:
          RCLCPP_ERROR(node_->get_logger(), "Rejected navigation preempt goal");
          state_ = State::ACCEPTED_AND_NAVIGATING;
          break;
        case easynav_interfaces::msg::NavigationControl::ERROR:
          RCLCPP_ERROR(node_->get_logger(), "Error in preempting navigation");
          state_ = State::ERROR;
          break;
        default:
          RCLCPP_ERROR(
            node_->get_logger(), "State SENT_PREEMPT; Unexpected message: %d: %s",
            msg->type, msg->status_message.c_str());
          state_ = State::ERROR;
      }
      break;
    case State::ACCEPTED_AND_NAVIGATING:
      switch (msg->type) {
        case easynav_interfaces::msg::NavigationControl::FEEDBACK:
          RCLCPP_DEBUG(node_->get_logger(), "Getting navigation feedback");
          last_feedback_ = *msg;
          break;
        case easynav_interfaces::msg::NavigationControl::FINISHED:
          RCLCPP_INFO(node_->get_logger(), "Navigation succesfully finished");
          last_result_ = *msg;
          state_ = State::NAVIGATION_FINISHED;
          break;
        case easynav_interfaces::msg::NavigationControl::FAILED:
          RCLCPP_ERROR(
            node_->get_logger(), "Navigation with error finished: %s",
            msg->status_message.c_str());
          last_result_ = *msg;
          state_ = State::NAVIGATION_FAILED;
          break;
        case easynav_interfaces::msg::NavigationControl::CANCELLED:
          RCLCPP_INFO(node_->get_logger(), "Navigation cancelled");
          last_result_ = *msg;
          state_ = State::NAVIGATION_CANCELLED;
          break;
        default:
          RCLCPP_ERROR(
            node_->get_logger(), "State ACCEPTED_AND_NAVIGATING; Unexpected message: %d: %s",
            msg->type, msg->status_message.c_str());
          last_result_ = *msg;
          state_ = State::ERROR;
      }
      break;
    default:
      RCLCPP_ERROR(
        node_->get_logger(), "State not managed: %d:",
        static_cast<int>(state_));
      state_ = State::ERROR;
  }

  last_control_ = std::move(msg);
}

const easynav_interfaces::msg::NavigationControl &
GoalManagerClient::get_last_control() const
{
  return *last_control_;
}

const easynav_interfaces::msg::NavigationControl &
GoalManagerClient::get_feedback() const
{
  return last_feedback_;
}

const easynav_interfaces::msg::NavigationControl &
GoalManagerClient::get_result() const
{
  return last_result_;
}

void
GoalManagerClient::send_goal(const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_DEBUG(node_->get_logger(), "Sending navigation goal");

  nav_msgs::msg::Goals msg;
  msg.header = goal.header;
  msg.goals.push_back(goal);

  send_goals(msg);
}

void
GoalManagerClient::send_goals(const nav_msgs::msg::Goals & goals)
{
  if (last_control_ == nullptr) {
    last_control_ = std::make_unique<easynav_interfaces::msg::NavigationControl>();
  }

  if (goals.goals.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Trying to command empty goals");
    return;
  }

  easynav_interfaces::msg::NavigationControl msg;

  switch (state_) {
    case State::IDLE:
      state_ = State::SENT_GOAL;
      msg.type = easynav_interfaces::msg::NavigationControl::REQUEST;
      break;
    case State::ACCEPTED_AND_NAVIGATING:
      state_ = State::SENT_PREEMPT;
      msg.type = easynav_interfaces::msg::NavigationControl::REQUEST;
      break;
    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Trying to send new goals in state %d. Ignoring",
        static_cast<int>(state_));
      return;
  }

  msg.header = goals.header;
  msg.user_id = id_;
  msg.seq = last_control_->seq + 1;
  msg.goals = goals;

  control_pub_->publish(msg);
}

void
GoalManagerClient::cancel()
{
  RCLCPP_DEBUG(node_->get_logger(), "Sending nevigation cancelation");

  if (state_ != State::ACCEPTED_AND_NAVIGATING) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Triying to cancel a non-active navigation (state %d)",
      static_cast<int>(state_));
    return;
  }

  easynav_interfaces::msg::NavigationControl msg;
  msg.type = easynav_interfaces::msg::NavigationControl::CANCEL;
  msg.header = last_control_->header;
  msg.user_id = id_;
  msg.seq = last_control_->seq + 1;


  RCLCPP_DEBUG(node_->get_logger(), "Navigation cancelation sent");
  control_pub_->publish(msg);
}

void
GoalManagerClient::reset()
{
  if (state_ == State::NAVIGATION_FINISHED || state_ == State::NAVIGATION_REJECTED ||
    state_ == State::NAVIGATION_FAILED || state_ == State::NAVIGATION_CANCELLED ||
    state_ == State::ERROR)
  {
    state_ = State::IDLE;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Triying to reset navigation in a a non-finished navigation state %d",
      static_cast<int>(state_));
  }
}

}  // namespace easynav
