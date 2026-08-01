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

"""Unit tests for easynav_tools utility functions (no ROS node required)."""

import math

from builtin_interfaces.msg import Duration
from easynav_interfaces.msg import GoalManagerInfo, NavigationControl
from easynav_tools.cli.verb_utils import bbcode_to_ansi
from easynav_tools.controller.ros_controllers import (
    _fmt_duration,
    _fmt_pose,
    _quat_to_yaw,
    _shorten_name,
    _sort_key_suffix,
    EasyNavControlProcessor,
    GoalManagerInfoProcessor,
    LogReader,
    RunningStats,
    TwistProcessor,
    TwistStampedProcessor,
)
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist, TwistStamped
from nav_msgs.msg import Goals


# ─────────────────────────────────────────────────────────────────────
# bbcode_to_ansi
# ─────────────────────────────────────────────────────────────────────

def test_bbcode_color_disabled_strips_known_tag():
    result = bbcode_to_ansi('[red]hello[/red]', enable_color=False)
    assert result == 'hello'


def test_bbcode_color_disabled_strips_nested_tags():
    result = bbcode_to_ansi('[bold][green]text[/green][/bold]', enable_color=False)
    assert result == 'text'


def test_bbcode_color_disabled_no_tags_unchanged():
    result = bbcode_to_ansi('plain text', enable_color=False)
    assert result == 'plain text'


def test_bbcode_color_disabled_strips_unknown_tag():
    result = bbcode_to_ansi('[unknown]val[/unknown]', enable_color=False)
    assert result == 'val'


def test_bbcode_color_enabled_red():
    result = bbcode_to_ansi('[red]hello[/red]', enable_color=True)
    assert '\033[31m' in result
    assert 'hello' in result
    assert '\033[0m' in result


def test_bbcode_color_enabled_green():
    result = bbcode_to_ansi('[green]ok[/green]', enable_color=True)
    assert '\033[32m' in result


def test_bbcode_color_enabled_bold():
    result = bbcode_to_ansi('[bold]text[/bold]', enable_color=True)
    assert '\033[1m' in result


def test_bbcode_color_enabled_unknown_tag_stripped():
    result = bbcode_to_ansi('[unknown]val[/unknown]', enable_color=True)
    assert result == 'val'


# ─────────────────────────────────────────────────────────────────────
# TwistProcessor.msg2text
# ─────────────────────────────────────────────────────────────────────

def test_twist_msg2text_contains_header():
    msg = Twist()
    text = TwistProcessor.msg2text(msg)
    assert 'Twist' in text


def test_twist_msg2text_linear_values():
    msg = Twist()
    msg.linear.x = 1.0
    msg.linear.y = 0.5
    msg.linear.z = 0.0
    text = TwistProcessor.msg2text(msg)
    assert '1.000' in text
    assert '0.500' in text


def test_twist_msg2text_angular_values():
    msg = Twist()
    msg.angular.z = 0.3
    text = TwistProcessor.msg2text(msg)
    assert '0.300' in text


def test_twist_msg2text_zero_message():
    msg = Twist()
    text = TwistProcessor.msg2text(msg)
    assert '0.000' in text


# ─────────────────────────────────────────────────────────────────────
# TwistStampedProcessor.msg2text
# ─────────────────────────────────────────────────────────────────────

def test_twiststamped_msg2text_contains_header():
    msg = TwistStamped()
    text = TwistStampedProcessor.msg2text(msg)
    assert 'TwistStamped' in text


def test_twiststamped_msg2text_linear_values():
    msg = TwistStamped()
    msg.twist.linear.x = 2.0
    msg.twist.angular.z = -0.5
    text = TwistStampedProcessor.msg2text(msg)
    assert '2.000' in text
    assert '-0.500' in text


def test_twiststamped_msg2text_zero():
    msg = TwistStamped()
    text = TwistStampedProcessor.msg2text(msg)
    assert '0.000' in text


# ─────────────────────────────────────────────────────────────────────
# EasyNavControlProcessor.msg2text
# ─────────────────────────────────────────────────────────────────────

def test_easynavcontrol_all_known_types():
    for type_val in range(9):  # REQUEST .. ERROR
        msg = NavigationControl()
        msg.type = type_val
        msg.status_message = 'test'
        text = EasyNavControlProcessor.msg2text(msg)
        assert 'Type:' in text


def test_easynavcontrol_unknown_type():
    msg = NavigationControl()
    msg.type = 99
    msg.status_message = ''
    text = EasyNavControlProcessor.msg2text(msg)
    assert 'Type:' in text
    assert '99' in text


def test_easynavcontrol_with_current_pose():
    msg = NavigationControl()
    msg.type = 3  # FEEDBACK
    msg.status_message = 'moving'
    msg.current_pose = PoseStamped()
    msg.current_pose.pose.position.x = 3.0
    msg.current_pose.pose.position.y = 4.0
    msg.current_pose.pose.orientation.w = 1.0
    text = EasyNavControlProcessor.msg2text(msg)
    assert '3.000' in text


def test_easynavcontrol_distance_fields():
    msg = NavigationControl()
    msg.type = 4  # FINISHED
    msg.status_message = 'done'
    msg.distance_covered = 5.5
    msg.distance_to_goal = 2.2
    text = EasyNavControlProcessor.msg2text(msg)
    assert '5.500' in text
    assert '2.200' in text


# ─────────────────────────────────────────────────────────────────────
# GoalManagerInfoProcessor.msg2text
# ─────────────────────────────────────────────────────────────────────

def test_goalmanagerinfo_idle_no_goals():
    msg = GoalManagerInfo()
    msg.status = 0  # IDLE
    msg.position_distance = 0.1
    msg.position_tolerance = 0.5
    msg.angle_distance = 0.01
    msg.angle_tolerance = 0.1
    msg.goals = Goals()
    msg.goals.goals = []
    text = GoalManagerInfoProcessor.msg2text(msg)
    assert 'IDLE' in text
    assert 'First goal: —' in text


def test_goalmanagerinfo_active_with_goal():
    msg = GoalManagerInfo()
    msg.status = 1  # ACTIVE
    msg.position_distance = 0.1
    msg.position_tolerance = 0.5
    msg.angle_distance = 0.0
    msg.angle_tolerance = 0.2
    ps = PoseStamped()
    ps.pose.position.x = 1.0
    ps.pose.position.y = 2.0
    ps.pose.position.z = 0.0
    ps.pose.orientation.w = 1.0
    goals = Goals()
    goals.goals = [ps]
    msg.goals = goals
    text = GoalManagerInfoProcessor.msg2text(msg)
    assert 'ACTIVE' in text
    assert 'x=1.000' in text


def test_goalmanagerinfo_position_within_tolerance():
    msg = GoalManagerInfo()
    msg.status = 1
    msg.position_distance = 0.05
    msg.position_tolerance = 0.2
    msg.angle_distance = 0.01
    msg.angle_tolerance = 0.1
    msg.goals = Goals()
    msg.goals.goals = []
    text = GoalManagerInfoProcessor.msg2text(msg)
    assert '[green]Position:' in text


def test_goalmanagerinfo_position_outside_tolerance():
    msg = GoalManagerInfo()
    msg.status = 0
    msg.position_distance = 1.0
    msg.position_tolerance = 0.2
    msg.angle_distance = 0.01
    msg.angle_tolerance = 0.1
    msg.goals = Goals()
    msg.goals.goals = []
    text = GoalManagerInfoProcessor.msg2text(msg)
    # Position line should NOT be green when outside tolerance
    assert '[green]Position:' not in text


def test_goalmanagerinfo_all_within_tolerance_angle_green():
    msg = GoalManagerInfo()
    msg.status = 1
    msg.position_distance = 0.05
    msg.position_tolerance = 0.2
    msg.angle_distance = 0.01
    msg.angle_tolerance = 0.1
    msg.goals = Goals()
    msg.goals.goals = []
    text = GoalManagerInfoProcessor.msg2text(msg)
    assert '[green]Angle:' in text


# ─────────────────────────────────────────────────────────────────────
# _quat_to_yaw
# ─────────────────────────────────────────────────────────────────────

def test_quat_to_yaw_identity():
    q = Quaternion()
    q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
    assert abs(_quat_to_yaw(q)) < 1e-9


def test_quat_to_yaw_90_degrees():
    q = Quaternion()
    q.x, q.y = 0.0, 0.0
    q.z = math.sin(math.pi / 4)
    q.w = math.cos(math.pi / 4)
    yaw = _quat_to_yaw(q)
    assert abs(yaw - math.pi / 2) < 1e-6


def test_quat_to_yaw_180_degrees():
    q = Quaternion()
    q.x, q.y, q.z, q.w = 0.0, 0.0, 1.0, 0.0
    yaw = _quat_to_yaw(q)
    assert abs(abs(yaw) - math.pi) < 1e-6


def test_quat_to_yaw_minus_90_degrees():
    q = Quaternion()
    q.x, q.y = 0.0, 0.0
    q.z = math.sin(-math.pi / 4)
    q.w = math.cos(-math.pi / 4)
    yaw = _quat_to_yaw(q)
    assert abs(yaw + math.pi / 2) < 1e-6


# ─────────────────────────────────────────────────────────────────────
# _fmt_duration
# ─────────────────────────────────────────────────────────────────────

def test_fmt_duration_zero():
    d = Duration()
    d.sec, d.nanosec = 0, 0
    assert _fmt_duration(d) == '0.000s'


def test_fmt_duration_half_second():
    d = Duration()
    d.sec, d.nanosec = 0, 500_000_000
    assert _fmt_duration(d) == '0.500s'


def test_fmt_duration_one_second():
    d = Duration()
    d.sec, d.nanosec = 1, 0
    assert _fmt_duration(d) == '1.000s'


def test_fmt_duration_one_and_a_quarter():
    d = Duration()
    d.sec, d.nanosec = 1, 250_000_000
    assert _fmt_duration(d) == '1.250s'


def test_fmt_duration_fallback_on_bad_input():
    result = _fmt_duration('not_a_duration')
    assert isinstance(result, str)


# ─────────────────────────────────────────────────────────────────────
# _fmt_pose
# ─────────────────────────────────────────────────────────────────────

def test_fmt_pose_with_posestamped():
    ps = PoseStamped()
    ps.pose.position.x = 1.0
    ps.pose.position.y = 2.0
    ps.pose.position.z = 0.5
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = 0.0
    ps.pose.orientation.w = 1.0
    result = _fmt_pose(ps)
    assert 'pos=(1.000, 2.000, 0.500)' in result


def test_fmt_pose_with_pose_object():
    p = Pose()
    p.position.x = 3.0
    p.position.y = 4.0
    p.position.z = 0.0
    p.orientation.w = 1.0
    result = _fmt_pose(p)
    assert 'pos=(3.000, 4.000, 0.000)' in result


def test_fmt_pose_fallback_on_bad_input():
    result = _fmt_pose('bad_pose')
    assert isinstance(result, str)


# ─────────────────────────────────────────────────────────────────────
# _shorten_name
# ─────────────────────────────────────────────────────────────────────

def test_shorten_name_removes_easynav_prefix():
    assert _shorten_name('easynav::MyClass::method') == 'MyClass::method'


def test_shorten_name_no_prefix_unchanged():
    assert _shorten_name('OtherNamespace::func') == 'OtherNamespace::func'


def test_shorten_name_plain_string():
    assert _shorten_name('plain') == 'plain'


def test_shorten_name_only_prefix():
    assert _shorten_name('easynav::') == ''


# ─────────────────────────────────────────────────────────────────────
# _sort_key_suffix
# ─────────────────────────────────────────────────────────────────────

def test_sort_key_suffix_with_easynav_prefix():
    key = _sort_key_suffix('easynav::NS::MyCls')
    assert key == ('MyCls', 'NS::MyCls')


def test_sort_key_suffix_no_prefix():
    key = _sort_key_suffix('NS::Func')
    assert key == ('Func', 'NS::Func')


def test_sort_key_suffix_plain():
    key = _sort_key_suffix('plain')
    assert key == ('plain', 'plain')


def test_sort_key_suffix_ordering():
    names = ['easynav::B::Zeta', 'easynav::A::Alpha', 'easynav::C::Zeta']
    sorted_names = sorted(names, key=_sort_key_suffix)
    # Primary sort by suffix: Alpha < Zeta; within same suffix sort by short name
    assert sorted_names[0] == 'easynav::A::Alpha'
    assert sorted_names[1] in ('easynav::B::Zeta', 'easynav::C::Zeta')


# ─────────────────────────────────────────────────────────────────────
# RunningStats (Welford online algorithm)
# ─────────────────────────────────────────────────────────────────────

def test_running_stats_empty_returns_zero():
    rs = RunningStats()
    mean, std = rs.as_tuple()
    assert mean == 0.0
    assert std == 0.0


def test_running_stats_single_sample():
    rs = RunningStats()
    rs.update(5.0)
    mean, std = rs.as_tuple()
    assert mean == 5.0
    assert std == 0.0


def test_running_stats_two_samples():
    rs = RunningStats()
    rs.update(2.0)
    rs.update(4.0)
    mean, std = rs.as_tuple()
    assert abs(mean - 3.0) < 1e-9
    assert abs(std - math.sqrt(2.0)) < 1e-6


def test_running_stats_five_samples():
    rs = RunningStats()
    values = [1.0, 2.0, 3.0, 4.0, 5.0]
    for v in values:
        rs.update(v)
    mean, std = rs.as_tuple()
    expected_mean = 3.0
    expected_std = math.sqrt(2.5)  # sample std: sqrt(10/4)
    assert abs(mean - expected_mean) < 1e-9
    assert abs(std - expected_std) < 1e-6


def test_running_stats_constant_sequence():
    rs = RunningStats()
    for _ in range(5):
        rs.update(7.0)
    mean, std = rs.as_tuple()
    assert abs(mean - 7.0) < 1e-9
    assert abs(std) < 1e-9


# ─────────────────────────────────────────────────────────────────────
# LogReader.rows2text
# ─────────────────────────────────────────────────────────────────────

def test_logReader_rows2text_empty_list():
    text = LogReader.rows2text([])
    assert 'function name' in text
    assert 'execution time' in text


def test_logReader_rows2text_none_input():
    text = LogReader.rows2text(None)
    assert 'function name' in text


def test_logReader_rows2text_contains_data():
    rows = [
        ('my_func', (1.234, 0.012), (100.0, 5.0), (10.0, 0.5)),
        ('other',   (0.5,   0.01),  (50.0,  1.0), (20.0, 0.1)),
    ]
    text = LogReader.rows2text(rows)
    assert 'my_func' in text
    assert 'other' in text
    assert '±' in text


def test_logReader_rows2text_separator_present():
    rows = [('func', (1.0, 0.1), (10.0, 1.0), (5.0, 0.5))]
    text = LogReader.rows2text(rows)
    # Should contain separator line
    assert '-' in text


def test_logReader_rows2text_column_widths_align():
    rows = [
        ('short', (0.1, 0.01), (1.0, 0.1), (100.0, 5.0)),
        ('a_very_long_function_name', (0.1, 0.01), (1.0, 0.1), (100.0, 5.0)),
    ]
    text = LogReader.rows2text(rows)
    lines = text.split('\n')
    # All lines should have the same length (padded columns)
    lengths = [len(line) for line in lines]
    assert len(set(lengths)) == 1
