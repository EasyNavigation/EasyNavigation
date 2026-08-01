# Copyright 2025 Intelligent Robotics Lab
# SPDX-License-Identifier: Apache-2.0
import unittest

from .goal_manager_client import ClientState, GoalManagerClient  # noqa: F401


def load_tests(loader, tests, pattern):
    return unittest.TestSuite()
