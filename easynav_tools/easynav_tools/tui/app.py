#!/usr/bin/env python3

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


import atexit
import math
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException

from rich.text import Text

from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal, Vertical
from textual.widgets import Footer, Label, Static, Switch, Tab, Tabs

from ..controller.ros_controllers import (
    EasyNavControlProcessor,
    GoalManagerInfoProcessor,
    LogReader,
    NavStateProcessor,
    TwistProcessor,
    TwistStampedProcessor,
)


# Prefer vendored textual package inside easynav_tools/vendor.
_vendor_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
if os.path.isdir(_vendor_dir) and _vendor_dir not in sys.path:
    sys.path.insert(0, _vendor_dir)


class EasyNavTabbedApp(App):
    """TUI with Tabs: 'Status' (Navigation Status + NavState/Time stats) and 'Commanding'."""

    CSS = """
    Screen {
        layout: vertical;
    }

    #tabs {
        dock: top;
    }

    #pages {
        height: 1fr;
        width: 100%;
    }

    #status_root {
        layout: horizontal;
        width: 100%;
        height: 100%;
    }

    /* Left 35%: Navigation Status (sub-boxes) */
    #left_col {
        width: 35%;
        height: 100%;
        layout: vertical;
    }

    /* Right 65%: NavState (top), Time stats (bottom) */
    #right_col {
        width: 65%;
        height: 100%;
        layout: vertical;
        margin-left: 1;
    }

    .titled {
        layout: vertical;
        width: 100%;
        height: auto;
        margin-bottom: 1;
    }

    .hdr {
        layout: horizontal;
        height: auto;
        width: 100%;
    }

    .spacer {
        width: 1fr;
    }

    .title {
        padding: 0 1;
        height: auto;
    }

    .box {
        border: round;
        padding: 0 1;
        content-align: left top;
        width: 100%;
        height: auto;
        overflow: auto;
    }

    #navstate_block { height: 1fr; }
    #timestats_block { height: 1fr; margin-top: 1; }

    #navstate_wrap { height: 100%; }
    #timestats_wrap { height: 100%; }

    #navstatus_block { height: 100%; }
    #navstatus_box   { height: 100%; }

    .navstatus_item {}

    #page_commanding {
        border: round;
        padding: 1 2;
        content-align: left top;
        width: 100%;
        height: 100%;
        overflow: auto;
    }
    """

    BINDINGS = [
        ('q', 'quit', 'Salir'),
        ('ctrl+c', 'quit', 'Salir'),
        ('1', 'show_status', 'Tab Status'),
        ('2', 'show_commanding', 'Tab Commanding'),
    ]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # ROS 2 init
        rclpy.init(args=None)
        atexit.register(self._ros_shutdown)
        self.node = rclpy.create_node('easynav_tui_status_commanding')

        # ROS 2 subscribers
        self.subs = {}
        self.subs['twist_subscriber'] = TwistProcessor(self.node, self.twist_callback)
        self.subs['twist_stamped_subscriber'] = TwistStampedProcessor(
            self.node, self.twist_stamped_callback
        )
        self.subs['easynav_control'] = EasyNavControlProcessor(self.node, self.control_callback)
        self.subs['goal_info'] = GoalManagerInfoProcessor(self.node, self.goal_info_callback)

        # Widget refs
        self.st_navstate: Static | None = None
        self.st_timestats: Static | None = None
        self.page_commanding: Static | None = None

        # Sub-boxes inside 'Navigation Status'
        self.box_nav_control: Static | None = None
        self.box_goal_info: Static | None = None
        self.box_twist: Static | None = None

        # Switch state and buffers
        self.navstate_enabled = True
        self.timestats_enabled = True
        self._last_navstate_text: Text | str = ''
        self._last_timestats_text: Text | str = ''

        # Cached last twist texts
        self._last_twist_text = '—'
        self._last_twiststamped_text = '—'

        # self.node's namespace already reflects any "-r __ns:=..." remap the
        # TUI was launched with, so the trace log for that same EasyNav
        # instance is found without a separate --namespace flag.
        self.log_reader = LogReader(namespace=self.node.get_namespace())

    def compose(self) -> ComposeResult:
        yield Tabs(
            Tab('Status', id='tab_status'),
            Tab('Commanding', id='tab_commanding'),
            id='tabs',
        )

        with Container(id='pages'):
            # Status page
            with Container(id='page_status'):
                with Horizontal(id='status_root'):
                    # LEFT column: Navigation Status
                    with Vertical(id='left_col'):
                        with Vertical(id='navstatus_block', classes='titled'):
                            yield Label('Navigation Status', classes='title')
                            with Vertical(id='navstatus_box', classes='box'):
                                # 1) Navigation Control
                                with Vertical(classes='titled'):
                                    yield Label('Navigation Control', classes='title')
                                    self.box_nav_control = Static(
                                        'Esperando NavigationControl…',
                                        classes='box navstatus_item'
                                    )
                                    yield self.box_nav_control
                                # 2) Goal Info
                                with Vertical(classes='titled'):
                                    yield Label('Goal Info', classes='title')
                                    self.box_goal_info = Static(
                                        'Esperando GoalManagerInfo…',
                                        classes='box navstatus_item'
                                    )
                                    yield self.box_goal_info
                                # 3) Twist
                                with Vertical(classes='titled'):
                                    yield Label('Twist', classes='title')
                                    self.box_twist = Static(
                                        'Esperando Twist/TwistStamped…',
                                        classes='box navstatus_item'
                                    )
                                    yield self.box_twist

                    # RIGHT column: NavState + Time stats
                    with Vertical(id='right_col'):
                        # NavState (switch inside border)
                        with Vertical(id='navstate_block', classes='titled'):
                            with Vertical(id='navstate_wrap', classes='box'):
                                with Horizontal(classes='hdr'):
                                    yield Label('NavState', classes='title')
                                    yield Static('', classes='spacer')
                                    yield Switch(value=True, id='sw_navstate')
                                self.st_navstate = Static('NavState: esperando…')
                                yield self.st_navstate

                        # Time stats (switch inside border)
                        with Vertical(id='timestats_block', classes='titled'):
                            with Vertical(id='timestats_wrap', classes='box'):
                                with Horizontal(classes='hdr'):
                                    yield Label('Time stats', classes='title')
                                    yield Static('', classes='spacer')
                                    yield Switch(value=True, id='sw_timestats')
                                # start with placeholder; will be replaced by log data
                                initial_table = self._render_time_stats_table([])
                                self._last_timestats_text = initial_table
                                self.st_timestats = Static(initial_table)
                                yield self.st_timestats

            # Commanding page
            self.page_commanding = Static(
                'Esperando mensajes TwistStamped…\n\nSalir: "q" o "Ctrl+C"',
                id='page_commanding',
            )
            yield self.page_commanding

        yield Footer()

    def on_mount(self) -> None:
        self._show_page('status')
        # ROS polling
        self.set_interval(0.05, self._ros_spin_once)
        # create NavState sub if switch is ON
        if self.query_one('#sw_navstate', Switch).value:
            self.subs['navstate'] = NavStateProcessor(self.node, self.navstate_callback)
        # Time stats: poll the log periodically (10 Hz is overkill; use ~2 Hz)
        self.set_interval(0.5, self._poll_time_stats_log)

    # ---------- Tabs <-> Pages ----------
    def on_tabs_tab_activated(self, event: Tabs.TabActivated) -> None:
        if event.tab.id == 'tab_status':
            self._show_page('status')
        elif event.tab.id == 'tab_commanding':
            self._show_page('commanding')

    def action_show_status(self) -> None:
        self.query_one(Tabs).active = 'tab_status'
        self._show_page('status')

    def action_show_commanding(self) -> None:
        self.query_one(Tabs).active = 'tab_commanding'
        self._show_page('commanding')

    def _show_page(self, which: str) -> None:
        page_status = self.query_one('#page_status', Container)
        page_command = self.query_one('#page_commanding', Static)
        if which == 'status':
            page_status.display = True
            page_command.display = False
        else:
            page_status.display = False
            page_command.display = True

    # ---------- Switch handling ----------
    def on_switch_changed(self, event: Switch.Changed) -> None:
        if event.switch.id == 'sw_navstate':
            self.navstate_enabled = event.value
            if event.value:
                # ON: (re)create subscriber and restore last content
                self.subs['navstate'] = NavStateProcessor(self.node, self.navstate_callback)
                if self.st_navstate:
                    self.st_navstate.update(self._last_navstate_text)
            else:
                # OFF: clear UI and destroy subscriber to free resources
                if self.st_navstate:
                    self.st_navstate.update('')
                # try to destroy wrapper and remove
                sub = self.subs.pop('navstate', None)
                if sub is not None and hasattr(sub, 'destroy'):
                    try:
                        sub.destroy()
                    except Exception:
                        pass

        elif event.switch.id == 'sw_timestats':
            self.timestats_enabled = event.value
            if not event.value and self.st_timestats:
                self.st_timestats.update('')
            elif event.value and self.st_timestats:
                self.st_timestats.update(self._last_timestats_text)

    # ---------- ROS polling ----------
    def _ros_spin_once(self) -> None:
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.0)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass

    def set_navstate_text(self, text: str) -> None:
        self._last_navstate_text = text
        if self.st_navstate is not None:
            if self.navstate_enabled:
                self.st_navstate.update(text)
            else:
                self.st_navstate.update('')

    def set_time_stats_rows(self, rows) -> None:
        table = self._render_time_stats_table(rows)
        self._last_timestats_text = table
        if self.st_timestats is not None:
            if self.timestats_enabled:
                self.st_timestats.update(table)
            else:
                self.st_timestats.update('')

    # ---------- Formatting helpers ----------

    @staticmethod
    def _render_time_stats_table(rows) -> Text:
        headers = [
            'function name',
            'execution time (ms)',
            'elapsed (ms)',
            'frequency (Hz)',
        ]
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
        cols = list(zip(*([headers] + data))) if data else [headers]
        widths = [max(len(str(x)) for x in col) for col in cols]

        def fmt_row(cells):
            return ' | '.join(str(c).ljust(w) for c, w in zip(cells, widths))

        sep = '-+-'.join('-' * w for w in widths)
        lines = [fmt_row(headers), sep]
        for row in data:
            lines.append(fmt_row(row))
        table_str = '\n'.join(lines)
        return Text(table_str, no_wrap=True)

    @staticmethod
    def _quat_to_yaw(q) -> float:
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ---------- ROS callbacks ----------

    def control_callback(self, msg) -> None:
        if self.box_nav_control is None:
            return

        text = EasyNavControlProcessor.msg2text(msg)
        self.box_nav_control.update(text)

    def goal_info_callback(self, msg) -> None:
        if self.box_goal_info is None:
            return

        text = GoalManagerInfoProcessor.msg2text(msg)
        self.box_goal_info.update(text)

    def twist_callback(self, msg) -> None:
        self._last_twist_text = TwistProcessor.msg2text(msg)
        self._update_twist_box()

    def twist_stamped_callback(self, msg) -> None:
        self._last_twiststamped_text = TwistStampedProcessor.msg2text(msg)
        self._update_twist_box()

    def navstate_callback(self, msg) -> None:
        text = NavStateProcessor.msg2text(msg)
        self._last_navstate_text = text
        if self.navstate_enabled and self.st_navstate is not None:
            self.st_navstate.update(text)

    def _update_twist_box(self) -> None:
        if self.box_twist is None:
            return
        parts = []
        if self._last_twist_text != '—':
            parts.append(self._last_twist_text)
        if self._last_twiststamped_text != '—':
            parts.append(self._last_twiststamped_text)
        self.box_twist.update('\n\n'.join(parts) if parts else 'Esperando Twist/TwistStamped…')

    def _poll_time_stats_log(self) -> None:
        """Read new lines from the log and update the Time stats table."""
        rows = self.log_reader.poll_time_stats_log()
        if rows is not None:
            self.set_time_stats_rows(rows)

    def _ros_shutdown(self) -> None:
        if rclpy.ok():
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()


if __name__ == '__main__':
    EasyNavTabbedApp().run()


def run_app() -> None:
    EasyNavTabbedApp().run()
