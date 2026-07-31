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

import math
import os
import re

from easynav_interfaces.msg import GoalManagerInfo, NavigationControl
from geometry_msgs.msg import Twist, TwistStamped

from std_msgs.msg import String


class TwistProcessor():

    def __init__(self, node, callback):
        self.twist_sub = node.create_subscription(
            Twist,
            'cmd_vel',
            callback,
            10)

        self.twist_sub

    @staticmethod
    def msg2text(msg: Twist) -> str:
        return (
            'Twist:\n'
            f'  linear : x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}\n'
            f'  angular: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}'
        )


class TwistStampedProcessor():

    def __init__(self, node, callback):
        self.twist_sub = node.create_subscription(
            TwistStamped,
            'cmd_vel_stamped',
            callback,
            10)

        self.twist_sub

    @staticmethod
    def msg2text(msg: TwistStamped) -> str:
        tw = msg.twist
        return (
            'TwistStamped:\n'
            f'  linear : x={tw.linear.x:.3f}, y={tw.linear.y:.3f}, z={tw.linear.z:.3f}\n'
            f'  angular: x={tw.angular.x:.3f}, y={tw.angular.y:.3f}, z={tw.angular.z:.3f}'
        )


# Mapping of NavigationControl.type (uint8) to (label, color)
NC_TYPE_MAP: dict[int, tuple[str, str]] = {
    0: ('REQUEST',   'green'),
    1: ('REJECT',    'red'),
    2: ('ACCEPT',    'green'),
    3: ('FEEDBACK',  'green'),
    4: ('FINISHED',  'green'),
    5: ('FAILED',    'red'),
    6: ('CANCEL',    'yellow'),
    7: ('CANCELLED', 'yellow'),
    8: ('ERROR',     'red'),
}


# ---------- Formatting helpers ----------

def _fmt_duration(dur) -> str:
    try:
        sec = dur.sec
        nsec = dur.nanosec
        return f'{sec + nsec/1e9:.3f}s'
    except Exception:
        return str(dur)


def _fmt_pose(pose) -> str:
    try:
        p = pose.pose if hasattr(pose, 'pose') else pose
        px, py, pz = p.position.x, p.position.y, p.position.z
        ox, oy, oz, ow = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        return (
            f'pos=({px:.3f}, {py:.3f}, {pz:.3f}), '
            f'quat=({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})'
        )
    except Exception:
        return str(pose)


class EasyNavControlProcessor():

    def __init__(self, node, callback):
        self.control_sub = node.create_subscription(
            NavigationControl,
            'easynav_control',
            callback,
            10)

        self.control_sub

    @staticmethod
    def msg2text(msg: NavigationControl) -> str:
        t_val: int = msg.type
        label, color = NC_TYPE_MAP.get(t_val, (str(t_val), 'white'))
        type_line = f'[{color}]{label}[/{color}]'

        pose_txt = _fmt_pose(msg.current_pose) if msg.current_pose else '—'
        nav_time_txt = _fmt_duration(msg.navigation_time) if msg.navigation_time else '—'
        eta_txt = _fmt_duration(
            msg.estimated_time_remaining) if msg.estimated_time_remaining else '—'
        dist_cov_txt = f'{msg.distance_covered:.3f} m'if isinstance(
            msg.distance_covered, (int, float)) else '—'
        dist_goal_txt = f'{msg.distance_to_goal:.3f} m' if isinstance(
            msg.distance_to_goal, (int, float)) else '—'

        text = (
            f'Type: {type_line}\n'
            f'Message: {msg.status_message}\n'
            f'Current pose: {pose_txt}\n'
            f'Navigation time: {nav_time_txt}\n'
            f'ETA: {eta_txt}\n'
            f'Distance covered: {dist_cov_txt}\n'
            f'Distance to goal: {dist_goal_txt}'
        )
        return text


# Mapping for GoalManagerInfo.status (uint8)
GM_STATUS_MAP: dict[int, tuple[str, str]] = {
    0: ('IDLE',   'yellow'),
    1: ('ACTIVE', 'green'),
}


def _quat_to_yaw(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalManagerInfoProcessor():

    def __init__(self, node, callback):
        self.gm_info_sub = node.create_subscription(
            GoalManagerInfo,
            'easynav_manager_info',
            callback,
            10)

        self.gm_info_sub

    @staticmethod
    def msg2text(msg: GoalManagerInfo) -> str:

        s_val: int = msg.status
        s_label, s_color = GM_STATUS_MAP.get(s_val, (str(s_val), 'white'))
        status_line = f'Status: [{s_color}]{s_label}[/{s_color}]'

        pos_ok = (msg.position_distance <= msg.position_tolerance)
        ang_ok = (abs(msg.angle_distance) <= msg.angle_tolerance)

        if pos_ok:
            pos_line = (
                f'[green]Position: distance={msg.position_distance:.3f} m '
                f'/ tol={msg.position_tolerance:.3f} m[/green]'
            )
        else:
            pos_line = (
                f'Position: distance={msg.position_distance:.3f} m '
                f'/ tol={msg.position_tolerance:.3f} m'
            )

        if pos_ok and ang_ok:
            ang_line = (
                f'[green]Angle: distance={msg.angle_distance:.3f} rad '
                f'/ tol={msg.angle_tolerance:.3f} rad[/green]'
            )
        else:
            ang_line = (
                f'Angle: distance={msg.angle_distance:.3f} rad '
                f'/ tol={msg.angle_tolerance:.3f} rad'
            )

        goals_count = len(msg.goals.goals)
        goals_line = f'Goals remaining: {goals_count}'

        if goals_count > 0:
            g0 = msg.goals.goals[0]
            pose0 = g0.pose if hasattr(g0, 'pose') else g0
            x = pose0.position.x
            y = pose0.position.y
            z = pose0.position.z
            yaw = _quat_to_yaw(pose0.orientation)
            first_goal_line = f'First goal: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f} rad'
        else:
            first_goal_line = 'First goal: —'

        return '\n'.join([status_line, pos_line, ang_line, goals_line, first_goal_line])


class NavStateProcessor():

    def __init__(self, node, callback):
        self.node = node
        self.navstate_sub = node.create_subscription(
            String,
            'easynav_navstate',
            callback,
            10)

        self.navstate_sub

    @staticmethod
    def msg2text(msg: String) -> str:
        return msg.data

    def destroy(self):
        self.node.destroy_subscription(self.navstate_sub)


# ---------- Time stats config ----------
_LOG_PATH = '/tmp/easynav.log'
_LOG_RE = re.compile(r'^(?P<name>\S+)\s+(?P<start>\d+)\s+(?P<end>\d+)\s*$')


# -------- Running stats (Welford) --------
class RunningStats:

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.M2 = 0.0

    def update(self, x: float) -> None:
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.M2 += delta * delta2

    def as_tuple(self) -> tuple[float, float]:
        if self.n < 2:
            return (self.mean, 0.0)
        return (self.mean, (self.M2 / (self.n - 1)) ** 0.5)


def _shorten_name(full: str) -> str:
    # drop 'easynav::' prefix if present
    if full.startswith('easynav::'):
        return full[len('easynav::'):]
    return full


def _sort_key_suffix(full: str) -> tuple[str, str]:
    # sort by suffix after last '::', then by full short name
    short = _shorten_name(full)
    parts = short.split('::')
    suffix = parts[-1] if parts else short
    return (suffix, short)


class LogReader:

    def __init__(self):
        # ---- Time stats state (tailing the log) ----
        self._log_fh = None
        self._log_inode = None
        self._log_pos = 0
        self._ts_stats: dict[str, dict] = {}

    def _open_log_if_needed(self) -> None:
        """Open the log file if available, preserving position; handle rotation/truncation."""
        try:
            st = os.stat(_LOG_PATH)
        except FileNotFoundError:
            # file missing: close if we had it
            if self._log_fh:
                try:
                    self._log_fh.close()
                except Exception:
                    pass
            self._log_fh = None
            self._log_inode = None
            self._log_pos = 0
            return

        if self._log_fh is None:
            # first open: read from start to accumulate history
            self._log_fh = open(_LOG_PATH, 'r', encoding='utf-8', errors='ignore')
            self._log_inode = st.st_ino
            self._log_pos = 0
            return

        # if inode changed or file shrank: reopen and start from 0
        try:
            same_inode = (self._log_inode == st.st_ino)
            curr_size = st.st_size
            if (not same_inode) or (curr_size < self._log_pos):
                try:
                    self._log_fh.close()
                except Exception:
                    pass
                self._log_fh = open(_LOG_PATH, 'r', encoding='utf-8', errors='ignore')
                self._log_inode = st.st_ino
                self._log_pos = 0
        except Exception:
            pass

    def _accum_sample(self, name: str, start_ns: int, end_ns: int) -> None:
        d = self._ts_stats.get(name)
        if d is None:
            d = {
                'exec': RunningStats(),     # ms
                'elapsed': RunningStats(),  # ms
                'freq': RunningStats(),     # Hz
                'last_start': None,         # ns
            }
            self._ts_stats[name] = d

        # Execution time in ms (ns -> ms)
        exec_ns = max(0, end_ns - start_ns)
        exec_ms = exec_ns / 1_000_000.0
        d['exec'].update(exec_ms)

        last_start = d['last_start']
        d['last_start'] = start_ns

        # Elapsed between consecutive starts (period) in ms; frequency in Hz
        if last_start is not None:
            elapsed_ns = max(0, start_ns - last_start)
            elapsed_ms = elapsed_ns / 1_000_000.0
            d['elapsed'].update(elapsed_ms)
            if elapsed_ns > 0:
                freq_hz = 1_000_000_000.0 / elapsed_ns
                d['freq'].update(freq_hz)

    def poll_time_stats_log(self):
        """Read new lines from the log and update the Time stats table."""
        self._open_log_if_needed()
        if self._log_fh is None:
            # No file; render empty (or keep previous). Here we keep previous.
            return

        # seek to last known position and read what’s new
        try:
            self._log_fh.seek(self._log_pos)
            for line in self._log_fh:
                m = _LOG_RE.match(line)
                if not m:
                    continue
                name = m.group('name')
                start_ns = int(m.group('start'))
                end_ns = int(m.group('end'))
                self._accum_sample(name, start_ns, end_ns)
            self._log_pos = self._log_fh.tell()
        except Exception:
            # On any IO/parsing error, do not crash the UI
            return

        # Build rows from accumulators
        rows = []
        for full_name, d in sorted(self._ts_stats.items(), key=lambda kv: _sort_key_suffix(kv[0])):
            short = _shorten_name(full_name)
            exec_mean, exec_std = d['exec'].as_tuple()            # μs
            elap_mean, elap_std = d['elapsed'].as_tuple()         # ms
            freq_mean, freq_std = d['freq'].as_tuple()            # Hz
            rows.append((short, (exec_mean, exec_std),
                         (elap_mean, elap_std), (freq_mean, freq_std)))

        return rows

    @staticmethod
    def rows2text(rows) -> str:
        """Build a plain-text table."""
        headers = [
            'function name',
            'execution time (ms)',
            'elapsed (ms)',
            'frequency (Hz)',
        ]

        # Normalize input (handle None)
        rows = rows or []

        # Format data rows
        data = []
        for name, exec_t, elapsed, freq in rows:
            e_mean, e_std = exec_t      # ms
            l_mean, l_std = elapsed     # ms
            f_mean, f_std = freq        # Hz
            data.append([
                f'{name}',
                f'{e_mean:.3f} ± {e_std:.3f}',
                f'{l_mean:.3f} ± {l_std:.3f}',
                f'{f_mean:.2f} ± {f_std:.2f}',
            ])

        # Compute column widths based on headers + data
        matrix = [headers] + data if data else [headers]
        cols = list(zip(*matrix))
        widths = [max(len(str(cell)) for cell in col) for col in cols]

        def fmt_row(cells):
            return ' | '.join(str(c).ljust(w) for c, w in zip(cells, widths))

        sep = '-+-'.join('-' * w for w in widths)

        lines = [fmt_row(headers), sep]
        for row in data:
            lines.append(fmt_row(row))

        # If there is no data, still return header and separator
        return '\n'.join(lines)
