# Copyright 2025 Intelligent Robotics Lab
#
# This file is part of the project Easy Navigation (EasyNav in short)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from enum import Enum

# Interfaces
from easynav_interfaces.msg import NavigationControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Goals

from rclpy.node import Node
from rclpy.qos import QoSProfile


class ClientState(Enum):
    IDLE = 0
    SENT_GOAL = 1
    SENT_PREEMPT = 2
    ACCEPTED_AND_NAVIGATING = 3
    NAVIGATION_FINISHED = 4
    NAVIGATION_REJECTED = 5
    NAVIGATION_FAILED = 6
    NAVIGATION_CANCELLED = 7
    ERROR = 8


class GoalManagerClient:
    """Python client compatible with C++ GoalManagerClient."""

    def __init__(
        self,
        node: Node
    ) -> None:
        self.node = node

        self.id = self.node.get_name() + '_goal_manager_client'
        self.id = self.node.get_name() + '_goal_manager_client'

        self.control_topic = 'easynav_control'
        self.goal_topic = 'goal_pose'

        self.state = ClientState.IDLE

        self.last_control = NavigationControl()
        self.last_feedback = NavigationControl()
        self.last_result = NavigationControl()

        qos = QoSProfile(depth=100)
        self._control_pub = self.node.create_publisher(NavigationControl, self.control_topic, qos)
        self._goal_pub = self.node.create_publisher(PoseStamped, self.goal_topic, qos)
        self._control_sub = self.node.create_subscription(
            NavigationControl, self.control_topic, self._on_control, qos
        )

    def send_goal(self, goal: PoseStamped) -> None:
        """Send a single goal to the GoalManager."""
        self.node.get_logger().info('Sending navigation goal')

        goals = Goals()
        goals.header = goal.header
        goals.goals.append(goal)

        self.send_goals(goals)

    def send_goals(self, goals: Goals) -> None:
        """Send a list of goals to the GoalManager."""
        if len(goals.goals) == 0:
            self.node.get_logger().error('Trying to command empty goals')
            return

        msg = NavigationControl()

        match self.state:
            case ClientState.IDLE:
                self.state = ClientState.SENT_GOAL
                msg.type = NavigationControl.REQUEST
            case ClientState.ACCEPTED_AND_NAVIGATING:
                self.state = ClientState.SENT_PREEMPT
                msg.type = NavigationControl.REQUEST
            case _:
                self.node.get_logger().error(
                    f'Trying to send new goals in state {self.state.name}. Ignoring')
                return

        msg.header = goals.header
        msg.user_id = self.id
        msg.seq = self.last_control.seq + 1
        msg.goals = goals

        self._control_pub.publish(msg)

    def cancel(self) -> None:
        """Cancel the current goal."""
        self.node.get_logger().info('Sending nevigation cancelation')

        if self.state != ClientState.ACCEPTED_AND_NAVIGATING:
            self.node.get_logger().error(
                f'Triying to cancel a non-active navigation (state {self.state.name})')
            return

        msg = NavigationControl()
        msg.type = NavigationControl.CANCEL
        msg.header = self.last_control.header
        msg.user_id = self.id
        msg.seq = self.last_control.seq + 1

        self.node.get_logger().debug('Navigation cancelation sent')

        self._control_pub.publish(msg)

    def reset(self) -> None:
        """Reset internal client state."""
        if (
            self.state == ClientState.NAVIGATION_FINISHED or
            self.state == ClientState.NAVIGATION_REJECTED or
            self.state == ClientState.NAVIGATION_FAILED or
            self.state == ClientState.NAVIGATION_CANCELLED or
            self.state == ClientState.ERROR
        ):
            self.state = ClientState.IDLE
        else:
            self.node.get_logger().error(
                f'Triying to reset navigation in a a non-finished nav state {self.state.name}')

    def get_state(self) -> ClientState:
        """Get the current internal state."""
        return self.state

    def get_last_control(self) -> NavigationControl:
        """Get the last control message sent or received."""
        return self.last_control

    def get_feedback(self) -> NavigationControl:
        """Get the most recent feedback received."""
        return self.last_feedback

    def get_result(self) -> NavigationControl:
        """Get the last result message received."""
        return self.last_result

    def _on_control(self, msg: NavigationControl) -> None:
        if msg.user_id == self.id:  # Avoid self messages
            return
        if msg.nav_current_user_id != self.id:  # Avoid messages to others
            return

        self.node.get_logger().debug(
            f'Received a navigation {msg.type} msg with user_id {msg.user_id}')

        if (
            self.state != ClientState.IDLE and
            self.state != ClientState.NAVIGATION_FINISHED and
            self.state != ClientState.NAVIGATION_FAILED and
            self.state != ClientState.NAVIGATION_CANCELLED and
            self.state != ClientState.ERROR
        ):
            match self.state:
                case ClientState.SENT_GOAL:
                    match msg.type:
                        case NavigationControl.FEEDBACK:
                            self.node.get_logger().debug(
                                'Getting navigation feedback while waiting for acceptance')
                            self.last_feedback = msg
                        case NavigationControl.ACCEPT:
                            self.node.get_logger().debug('Goal accepted. Navigating')
                            self.state = ClientState.ACCEPTED_AND_NAVIGATING
                        case NavigationControl.REJECT:
                            self.node.get_logger().error('Rejected navigation goal')
                            self.state = ClientState.NAVIGATION_FAILED
                        case NavigationControl.ERROR:
                            self.node.get_logger().error('Error in navigation')
                            self.state = ClientState.ERROR
                        case _:
                            self.node.get_logger().error(
                                'State SENT_GOAL; Unexpected message: "%d": "%s"' %
                                (msg.type, msg.status_message))
                            self.state = ClientState.ERROR
                case ClientState.SENT_PREEMPT:
                    match msg.type:
                        case NavigationControl.FEEDBACK:
                            self.node.get_logger().debug('Getting navigation feedback')
                            self.last_feedback = msg
                        case NavigationControl.ACCEPT:
                            self.node.get_logger().debug('Goal preemption accepted. Navigating')
                            self.state = ClientState.ACCEPTED_AND_NAVIGATING
                        case NavigationControl.REJECT:
                            self.node.get_logger().error('Rejected navigation preempt goal')
                            self.state = ClientState.ACCEPTED_AND_NAVIGATING
                        case NavigationControl.ERROR:
                            self.node.get_logger().error('Error in preempting navigation')
                            self.state = ClientState.ERROR
                        case _:
                            self.node.get_logger().error(
                                'State SENT_PREEMPT; Unexpected message: "%d": "%s"' %
                                (msg.type, msg.status_message))
                            self.state = ClientState.ERROR
                case ClientState.ACCEPTED_AND_NAVIGATING:
                    match msg.type:
                        case NavigationControl.FEEDBACK:
                            self.node.get_logger().debug('Getting navigation feedback')
                            self.last_feedback = msg
                        case NavigationControl.FINISHED:
                            self.node.get_logger().info('Navigation succesfully finished')
                            self.last_result = msg
                            self.state = ClientState.NAVIGATION_FINISHED
                        case NavigationControl.FAILED:
                            self.node.get_logger().error(
                                'Navigation with error finished: %s"' % msg.status_message)
                            self.last_result = msg
                            self.state = ClientState.NAVIGATION_FAILED
                        case NavigationControl.CANCELLED:
                            self.node.get_logger().error('Navigation cancelled')
                            self.last_result = msg
                            self.state = ClientState.NAVIGATION_CANCELLED
                        case _:
                            self.node.get_logger().error(
                                'State ACCEPTED_AND_NAVIGATING; Unexpected message: "%d": "%s"' %
                                msg.type, msg.status_message)
                            self.last_result = msg
                            self.state = ClientState.ERROR
                case _:
                    self.node.get_logger().error(f'State not managed: {self.state.name}')
                    self.state = ClientState.ERROR

        self.last_control = msg
