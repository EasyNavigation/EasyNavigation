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
/// \brief Declaration of the GoalManager class.

#ifndef EASYNAV_SYSTEM__GOALMANAGER_HPP_
#define EASYNAV_SYSTEM__GOALMANAGER_HPP_

#include "rclcpp/subscription.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_interfaces/msg/navigation_control.hpp"
#include "easynav_interfaces/msg/goal_manager_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"

#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class GoalManager
 * @brief Handles navigation goals, their lifecycle, and command interface.
 *
 * Manages goal state, handles external control messages, and interacts with navigation components.
 */
class GoalManager
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GoalManager)

  /**
   * @enum State
   * @brief Current internal goal state.
   */
  enum class State
  {
    IDLE,   ///< No active goal.
    ACTIVE  ///< A goal is currently being pursued.
  };

  /**
   * @struct GoalTolerance
   * @brief A structure to represent the goal tolerances.
   */
  struct GoalTolerance
  {
    /// Positional tolerance for x/y in meters.
    double position {0.03};
    /// Positional tolerance for z axis in meters. Very big number as default.
    double height {std::numeric_limits<double>::max()};
    /// Angular tolerance in radians for the yaw angle.
    double yaw {0.01};
  };

  /**
   * @brief Constructor.
   * @param nav_state Shared pointer to navigation state.
   * @param parent_node Lifecycle node for parameter and interface management.
   */
  GoalManager(
    NavState & nav_state,
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node);

  /**
   * @brief Get current goals.
   * @return Goals message.
   */
  [[nodiscard]] inline nav_msgs::msg::Goals get_goals() const {return goals_;}

  /**
   * @brief Get current internal goal state.
   * @return GoalManager::State value.
   */
  [[nodiscard]] inline State get_state() const {return state_;}

  /**
   * @brief Mark the current goal as successfully completed.
   */
  void set_finished();

  /**
   * @brief Mark the current goal as failed.
   * @param reason Textual explanation.
   */
  void set_failed(const std::string & reason);

  /**
   * @brief Mark the system as in error state.
   * @param reason Textual explanation.
   */
  void set_error(const std::string & reason);

  /**
   * @brief Update internal logic, including preemption and timeout checks.
   */
  void update(NavState & nav_state);

  /**
   * @brief Check if the robot is currently at the first goal.
   *
   * Compares the current pose against the first target in the list using
   * positional and angular tolerances.
   *
   * @param current_pose Current pose of the robot.
   * @param goal_tolerance Maximum allowed error for the goal position and orientation.
   */
  void check_goals(
    const geometry_msgs::msg::Pose & current_pose,
    const GoalTolerance & goal_tolerance
  );

private:
  /// @brief Lifecycle node.
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_;

  /// @brief Goal tolerance (translation and rotation).
  GoalTolerance goal_tolerance_ {};

  /// @brief Currently active goals.
  nav_msgs::msg::Goals goals_;

  /// @brief Publisher for goal control responses.
  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr control_pub_;

  /// @brief Subscription to external goal control commands.
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr control_sub_;

  /// @brief Subscription to pose-stamped goals (GUI or RViz).
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr comanded_pose_sub_;

  /// @brief Publisher for publishing internal info.
  rclcpp::Publisher<easynav_interfaces::msg::GoalManagerInfo>::SharedPtr info_pub_;

  /// @brief Last received navigation control message.
  easynav_interfaces::msg::NavigationControl::UniquePtr last_control_;

  /// @brief My ID.
  std::string id_;

  /// @brief ID of the client who sent the current goal.
  std::string current_client_id_;

  /// @brief Whether goal preemption is allowed.
  bool allow_preempt_goal_ {true};

  /// @brief Timestamp when the current navigation started.
  rclcpp::Time nav_start_time_;

  /// @brief Maximum frequency (Hz) at which FEEDBACK / GoalManagerInfo is published.
  double update_frequency_ {20.0};

  /// @brief Minimum period between consecutive FEEDBACK / GoalManagerInfo publications.
  rclcpp::Duration update_period_ {0, 0};

  /// @brief Timestamp of the last FEEDBACK / GoalManagerInfo publication.
  rclcpp::Time last_update_time_;

  /// @brief Whether feedback/info has been published at least once.
  bool first_update_ {true};

  /// @brief Handle new goal request and populate the response.
  void accept_request(
    const easynav_interfaces::msg::NavigationControl & msg,
    easynav_interfaces::msg::NavigationControl & response);

  /// @brief Handle incoming control messages.
  void control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg);

  /// @brief Handle goal poses received via PoseStamped messages.
  void comanded_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg);

  /// @brief Mark current goal as preempted.
  void set_preempted();

  /// @brief Internal goal state.
  State state_ {State::IDLE};
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__GOALMANAGER_HPP_
