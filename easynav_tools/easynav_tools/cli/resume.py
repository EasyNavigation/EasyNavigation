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

import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from ros2cli.node.strategy import add_arguments, NodeStrategy
from ros2cli.verb import VerbExtension

from easynav_goalmanager_py.goal_manager_client import GoalManagerClient

from easynav_interfaces.msg import NavigationControl


class ResumeVerb(VerbExtension):
    """Resume a previously paused EasyNav navigation."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '--timeout', type=float, default=3.0,
            help='Seconds to wait for a confirmation from EasyNav')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            client = GoalManagerClient(node)

            try:
                if not client.wait_for_server(timeout_sec=min(args.timeout, 2.0)):
                    print(
                        'Timed out waiting to discover a running EasyNav instance.',
                        file=sys.stderr)
                    return 1

                # Retry the request every 0.5s until a reply arrives: even
                # after wait_for_server() reports a match, a very-short-lived
                # client can still lose the first message to a residual
                # discovery race.
                t_end = time.time() + args.timeout
                response = NavigationControl()
                next_send = 0.0
                while time.time() < t_end:
                    if time.time() >= next_send:
                        client.resume()
                        next_send = time.time() + 0.5
                    rclpy.spin_once(node, timeout_sec=0.1)
                    response = client.get_last_control()
                    if response.type in (NavigationControl.RESUMED, NavigationControl.REJECT):
                        break
            except (KeyboardInterrupt, ExternalShutdownException):
                return 1

            if response.type == NavigationControl.RESUMED:
                print('Navigation resumed.')
                return 0
            elif response.type == NavigationControl.REJECT:
                print(f'Resume rejected: {response.status_message}', file=sys.stderr)
                return 1
            else:
                print(
                    'No confirmation received from EasyNav (is it running?).',
                    file=sys.stderr)
                return 1
