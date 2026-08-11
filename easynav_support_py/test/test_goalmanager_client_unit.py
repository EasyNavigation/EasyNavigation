# Copyright 2025 Intelligent Robotics Lab
#
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

import time
import unittest

from easynav_goalmanager_py.goal_manager_client import ClientState, GoalManagerClient
from easynav_interfaces.msg import NavigationControl

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Goals

import rclpy
from rclpy.node import Node


def spin_wait(node: Node, predicate, timeout=3.0, step=0.01):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=step)
    return predicate()


class TestGoalManagerClientUnit(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('gm_client_unit_tester')
        self.client = GoalManagerClient(self.node)

        self.server_pub = self.node.create_publisher(
            NavigationControl, self.client.control_topic, 10)

        self.goals = Goals()
        self.goals.header.frame_id = 'map'
        self.goals.header.stamp = self.node.get_clock().now().to_msg()
        g = PoseStamped()
        g.header = self.goals.header
        g.pose.orientation.w = 1.0
        self.goals.goals.append(g)

        self.single_goal = g

    def tearDown(self):
        self.node.destroy_node()

    def _srv_msg(self, t, status=''):
        msg = NavigationControl()
        msg.type = t
        msg.user_id = 'server'

        try:
            msg.nav_current_user_id = self.client.id
        except AttributeError:
            pass
        msg.status_message = status
        return msg

    def _publish_and_wait(self, msg, cond, timeout=1.0):
        self.server_pub.publish(msg)
        return spin_wait(self.node, cond, timeout=timeout)

    # --------------- Tests ---------------

    def test_send_goals_and_finished(self):
        self.client.send_goals(self.goals)
        self.assertEqual(self.client.get_state(), ClientState.SENT_GOAL)

        self._publish_and_wait(
            self._srv_msg(NavigationControl.ACCEPT),
            lambda: self.client.get_state() == ClientState.ACCEPTED_AND_NAVIGATING)
        self._publish_and_wait(
            self._srv_msg(NavigationControl.FEEDBACK, 'moving'),
            lambda: self.client.get_feedback() is not None)

        ok = self._publish_and_wait(
            self._srv_msg(NavigationControl.FINISHED, 'done'),
            lambda: self.client.get_state() == ClientState.NAVIGATION_FINISHED)

        self.assertTrue(ok)
        self.assertEqual(self.client.get_result().type, NavigationControl.FINISHED)

        self.client.reset()
        self.assertEqual(self.client.get_state(), ClientState.IDLE)

    def test_send_goal_path(self):
        # Now explicitly exercise send_goal()
        self.client.send_goal(self.single_goal)
        self.assertIn(self.client.get_state(), (ClientState.SENT_GOAL, ClientState.SENT_PREEMPT))

        self._publish_and_wait(
            self._srv_msg(NavigationControl.ACCEPT),
            lambda: self.client.get_state() == ClientState.ACCEPTED_AND_NAVIGATING)
        self._publish_and_wait(
            self._srv_msg(NavigationControl.CANCELLED, 'preempted'),
            lambda: self.client.get_state() == ClientState.NAVIGATION_CANCELLED)

    def test_reject(self):
        self.client.send_goals(self.goals)
        ok = self._publish_and_wait(
            self._srv_msg(NavigationControl.REJECT, 'invalid'),
            lambda: self.client.get_state() in (ClientState.NAVIGATION_REJECTED,
                                                ClientState.NAVIGATION_FAILED))

        self.assertTrue(ok)
        # Prefer REJECTED; allow FAILED to expose divergence
        self.assertIn(self.client.get_state(), (ClientState.NAVIGATION_REJECTED,
                                                ClientState.NAVIGATION_FAILED))
        self.client.reset()
        self.assertEqual(self.client.get_state(), ClientState.IDLE)

    def test_failed(self):
        self.client.send_goals(self.goals)
        self._publish_and_wait(
            self._srv_msg(NavigationControl.ACCEPT),
            lambda: self.client.get_state() == ClientState.ACCEPTED_AND_NAVIGATING)
        ok = self._publish_and_wait(
            self._srv_msg(NavigationControl.FAILED, 'collision'),
            lambda: self.client.get_state() == ClientState.NAVIGATION_FAILED, timeout=2.0)
        self.assertTrue(ok, 'Expected NAVIGATION_FAILED after FAILED')

    def test_unexpected_message_while_accepted_and_navigating(self):
        # Regression test: _on_control()'s ACCEPTED_AND_NAVIGATING branch used to build
        # its error log with `'...%d...%s' % msg.type, msg.status_message` — a missing
        # tuple, so the string formatting itself raised TypeError (not enough arguments
        # for format string) inside the subscription callback the moment an
        # unrecognized message.type arrived in this state. ERROR is not handled by the
        # ACCEPTED_AND_NAVIGATING branch (only FEEDBACK/FINISHED/FAILED/CANCELLED are),
        # so it exercises the previously-broken `case _:` path.
        self.client.send_goals(self.goals)
        self._publish_and_wait(
            self._srv_msg(NavigationControl.ACCEPT),
            lambda: self.client.get_state() == ClientState.ACCEPTED_AND_NAVIGATING)

        try:
            ok = self._publish_and_wait(
                self._srv_msg(NavigationControl.ERROR, 'unexpected'),
                lambda: self.client.get_state() == ClientState.ERROR)
        except TypeError as e:
            self.fail(f'_on_control() raised {e!r} handling an unexpected message type')

        self.assertTrue(ok, 'Expected ERROR after an unrecognized message type')
        self.assertEqual(self.client.get_result().status_message, 'unexpected')

    def test_preempt_local(self):
        self.client.send_goals(self.goals)
        self._publish_and_wait(
            self._srv_msg(NavigationControl.ACCEPT),
            lambda: self.client.get_state() == ClientState.ACCEPTED_AND_NAVIGATING)

        # New goal -> SENT_PREEMPT
        g2 = Goals()
        g2.header = self.goals.header
        g2.goals.append(self.single_goal)
        self.client.send_goals(g2)
        self.assertEqual(self.client.get_state(), ClientState.SENT_PREEMPT)

        # Server accepts preemption -> ACTIVE
        self._publish_and_wait(
            self._srv_msg(NavigationControl.ACCEPT),
            lambda: self.client.get_state() == ClientState.ACCEPTED_AND_NAVIGATING)

    def test_cancel_api(self):
        # cancel before ACTIVE should be tolerated (depending on your checks)
        try:
            self.client.cancel()
        except Exception as e:
            self.fail(f'cancel() raised before ACTIVE: {e}')

        # Now ACTIVE
        self.client.send_goals(self.goals)
        self._publish_and_wait(
            self._srv_msg(NavigationControl.ACCEPT),
            lambda: self.client.get_state() == ClientState.ACCEPTED_AND_NAVIGATING)
        # cancel should not throw
        try:
            self.client.cancel()
        except Exception as e:
            self.fail(f'cancel() raised in ACTIVE: {e}')

    def test_ignores_self_and_other_users(self):
        # Self message
        msg_self = self._srv_msg(NavigationControl.ACCEPT)
        msg_self.user_id = self.client.id
        self.server_pub.publish(msg_self)
        time.sleep(0.05)
        self.assertEqual(self.client.get_state(), ClientState.IDLE)

        # Message targeted to someone else
        msg_other = self._srv_msg(NavigationControl.ACCEPT)
        msg_other.nav_current_user_id = 'someone_else'
        self.server_pub.publish(msg_other)
        time.sleep(0.05)
        self.assertEqual(self.client.get_state(), ClientState.IDLE)
